from collections import Counter

import time

from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.common import add_facts, add_certified, SolutionStore, UNKNOWN_EVALUATION
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem
from pddlstream.algorithms.instantiate_task import sas_from_pddl, instantiate_task
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.search import abstrips_solve_from_task
from pddlstream.language.constants import is_plan
from pddlstream.language.conversion import obj_from_pddl_plan
from pddlstream.language.attachments import has_attachments, compile_fluents_as_attachments, solve_pyplanners
from pddlstream.language.statistics import load_stream_statistics, write_stream_statistics
from pddlstream.language.temporal import solve_tfd, SimplifiedDomain
from pddlstream.language.write_pddl import get_problem_pddl
from pddlstream.utils import INF, Verbose, str_from_object, elapsed_time

UPDATE_STATISTICS = False

def solve_temporal(evaluations, goal_exp, domain, debug=False, **kwargs):
    assert isinstance(domain, SimplifiedDomain)
    problem = get_problem_pddl(evaluations, goal_exp, domain.pddl)
    return solve_tfd(domain.pddl, problem, debug=debug)

def solve_sequential(evaluations, goal_exp, domain, unit_costs=False, debug=False, **search_args):
    problem = get_problem(evaluations, goal_exp, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    if has_attachments(domain):
        with Verbose(debug):
            instantiated = instantiate_task(task)
        return solve_pyplanners(instantiated, **search_args)
    sas_task = sas_from_pddl(task, debug=debug)
    return abstrips_solve_from_task(sas_task, debug=debug, **search_args)

def solve_finite(evaluations, goal_exp, domain, **kwargs):
    if isinstance(domain, SimplifiedDomain):
        pddl_plan, cost = solve_temporal(evaluations, goal_exp, domain, **kwargs)
    else:
        pddl_plan, cost = solve_sequential(evaluations, goal_exp, domain, **kwargs)
    plan = obj_from_pddl_plan(pddl_plan)
    return plan, cost

##################################################

def process_instance(instantiator, store, instance, verbose=False): #, **complexity_args):
    if instance.enumerated:
        return []
    start_time = time.time()
    new_results, new_facts = instance.next_results(verbose=verbose)
    store.sample_time += elapsed_time(start_time)

    evaluations = store.evaluations
    #remove_blocked(evaluations, instance, new_results)
    for result in new_results:
        complexity = result.compute_complexity(evaluations)
        #complexity = instantiator.compute_complexity(instance)
        for evaluation in add_certified(evaluations, result):
            instantiator.add_atom(evaluation, complexity)
    fact_complexity = 0 # TODO: record the instance or treat as initial?
    for evaluation in add_facts(evaluations, new_facts, result=UNKNOWN_EVALUATION, complexity=fact_complexity):
        instantiator.add_atom(evaluation, fact_complexity)
    if not instance.enumerated:
        instantiator.push_instance(instance)
    return new_results

def process_stream_queue(instantiator, store, complexity_limit=INF, verbose=False):
    instances = []
    results = []
    num_successes = 0
    while not store.is_terminated() and instantiator and (instantiator.min_complexity() <= complexity_limit):
        instance = instantiator.pop_stream()
        if instance.enumerated:
            continue
        instances.append(instance)
        new_results = process_instance(instantiator, store, instance, verbose=verbose)
        results.extend(new_results)
        num_successes += bool(new_results) # TODO: max_results?
    if verbose:
        print('Eager Calls: {} | Successes: {} | Results: {} | Counts: {}'.format(
            len(instances), num_successes, len(results),
            str_from_object(Counter(instance.external.name for instance in instances))))
    return len(instances)

# def retrace_stream_plan(store, domain, goal_expression):
#     # TODO: retrace the stream plan that supports the plan to find the certificate
#     if store.best_plan is None:
#         return None
#     assert not domain.axioms
#     from pddlstream.algorithms.downward import plan_preimage
#     print(goal_expression)
#     plan_preimage(store.best_plan, goal_expression)
#     raise NotImplementedError()

##################################################

def solve_incremental(problem, constraints=PlanConstraints(),
                      unit_costs=False, success_cost=INF,
                      max_iterations=INF, max_time=INF, max_memory=INF,
                      initial_complexity=0, complexity_step=1, max_complexity=INF,
                      verbose=False, **search_kwargs):
    """
    Solves a PDDLStream problem by alternating between applying all possible streams and searching
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the set of legal solutions

    :param unit_costs: use unit action costs rather than numeric costs
    :param success_cost: the exclusive (strict) upper bound on plan cost to successfully terminate

    :param max_time: the maximum runtime
    :param max_iterations: the maximum number of search iterations
    :param max_memory: the maximum amount of memory

    :param initial_complexity: the initial stream complexity limit
    :param complexity_step: the increase in the stream complexity limit per iteration
    :param max_complexity: the maximum stream complexity limit

    :param verbose: if True, print the result of each stream application
    :param search_kwargs: keyword args for the search subroutine

    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan (INF if no plan), and evaluations is init expanded
        using stream applications
    """
    # max_complexity = 0 => current
    # complexity_step = INF => exhaustive
    # success_cost = terminate_cost = decision_cost
    # TODO: warning if optimizers are present
    evaluations, goal_expression, domain, externals = parse_problem(
        problem, constraints=constraints, unit_costs=unit_costs)
    store = SolutionStore(evaluations, max_time, success_cost, verbose, max_memory=max_memory) # TODO: include other info here?
    if UPDATE_STATISTICS:
        load_stream_statistics(externals)
    static_externals = compile_fluents_as_attachments(domain, externals)
    num_iterations = num_calls = 0
    complexity_limit = initial_complexity
    instantiator = Instantiator(static_externals, evaluations)
    num_calls += process_stream_queue(instantiator, store, complexity_limit, verbose=verbose)
    while not store.is_terminated() and (num_iterations < max_iterations) and (complexity_limit <= max_complexity):
        num_iterations += 1
        print('Iteration: {} | Complexity: {} | Calls: {} | Evaluations: {} | Solved: {} | Cost: {:.3f} | '
              'Search Time: {:.3f} | Sample Time: {:.3f} | Time: {:.3f}'.format(
            num_iterations, complexity_limit, num_calls, len(evaluations),
            store.has_solution(), store.best_cost, store.search_time, store.sample_time, store.elapsed_time()))
        plan, cost = solve_finite(evaluations, goal_expression, domain,
                                  max_cost=min(store.best_cost, constraints.max_cost), **search_kwargs)
        if is_plan(plan):
            store.add_plan(plan, cost)
        if not instantiator:
            break
        if complexity_step is None:
            # TODO: option to select the next k-smallest complexities
            complexity_limit = instantiator.min_complexity()
        else:
            complexity_limit += complexity_step
        num_calls += process_stream_queue(instantiator, store, complexity_limit, verbose=verbose)
    #retrace_stream_plan(store, domain, goal_expression)
    #print('Final queue size: {}'.format(len(instantiator)))

    summary = store.export_summary()
    summary.update({
        'iterations': num_iterations,
        'complexity': complexity_limit,
    })
    print('Summary: {}'.format(str_from_object(summary, ndigits=3))) # TODO: return the summary

    if UPDATE_STATISTICS:
        write_stream_statistics(externals, verbose)
    return store.extract_solution()

##################################################

def solve_immediate(problem, **kwargs):
    """
    Solves a PDDLStream problem by searching only
    INCOMPLETENESS WARNING: only use if no stream evaluations are necessarily (otherwise terminates early)
    :param problem: a PDDLStream problem
    :param kwargs: keyword args for solve_incremental
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    return solve_incremental(problem, start_complexity=0, complexity_step=0, max_complexity=0, **kwargs)

def solve_exhaustive(problem, **kwargs):
    """
    Solves a PDDLStream problem by applying all possible streams and searching once
    INCOMPLETENESS WARNING: only use if a finite set of instantiable stream instances (otherwise infinite loop)
    :param problem: a PDDLStream problem
    :param kwargs: keyword args for solve_incremental
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    return solve_incremental(problem, start_complexity=INF, complexity_step=INF, max_complexity=INF, **kwargs)
