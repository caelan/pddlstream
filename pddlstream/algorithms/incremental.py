from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.common import add_facts, add_certified, SolutionStore, UNKNOWN_EVALUATION
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem
from pddlstream.algorithms.instantiate_task import sas_from_pddl, instantiate_task, convert_instantiated
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.search import abstrips_solve_from_task
from pddlstream.language.constants import is_plan, PDDLProblem
from pddlstream.language.conversion import obj_from_pddl_plan
from pddlstream.language.attachments import has_attachments, compile_fluents_as_attachments, solve_pyplanners
from pddlstream.language.statistics import load_stream_statistics, write_stream_statistics
from pddlstream.language.stream import DEBUG
from pddlstream.language.temporal import solve_tfd, SimplifiedDomain
from pddlstream.language.write_pddl import get_problem_pddl
from pddlstream.utils import INF, Verbose, str_from_object

UPDATE_STATISTICS = False

def process_instance(instantiator, evaluations, instance, verbose=False): #, **complexity_args):
    if instance.enumerated:
        return False
    new_results, new_facts = instance.next_results(verbose=verbose)
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
    return True

def solve_finite(evaluations, goal_exp, domain, unit_costs=False, debug=False, **search_args):
    if isinstance(domain, SimplifiedDomain):
        problem = get_problem_pddl(evaluations, goal_exp, domain.pddl)
        pddl_plan, cost = solve_tfd(domain.pddl, problem, debug=debug)
    else:
        problem = get_problem(evaluations, goal_exp, domain, unit_costs)
        task = task_from_domain_problem(domain, problem)
        if has_attachments(domain):
            with Verbose(debug):
                instantiated = instantiate_task(task)
            pddl_plan, cost = solve_pyplanners(instantiated, **search_args)
        else:
            sas_task = sas_from_pddl(task, debug=debug)
            pddl_plan, cost = abstrips_solve_from_task(sas_task, debug=debug, **search_args)
    plan = obj_from_pddl_plan(pddl_plan)
    return plan, cost

##################################################

def process_stream_queue(instantiator, store, complexity_limit, **kwargs):
    num_calls = 0
    while not store.is_terminated() and instantiator and (instantiator.min_complexity() <= complexity_limit):
        num_calls += process_instance(instantiator, store.evaluations, instantiator.pop_stream(), **kwargs)
    return num_calls

# def retrace_stream_plan(store, domain, goal_expression):
#     # TODO: retrace the stream plan used
#     if store.best_plan is None:
#         return None
#     assert not domain.axioms
#     from pddlstream.algorithms.downward import plan_preimage
#     print(goal_expression)
#     plan_preimage(store.best_plan, goal_expression)
#     raise NotImplementedError()

def solve_incremental(problem, constraints=PlanConstraints(),
                      unit_costs=False, success_cost=INF,
                      max_iterations=INF, max_time=INF, max_memory=INF,
                      initial_complexity=0, complexity_step=1, max_complexity=INF,
                      verbose=False, **search_args):
    """
    Solves a PDDLStream problem by alternating between applying all possible streams and searching
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the set of legal solutions
    :param max_time: the maximum amount of time to apply streams
    :param max_iterations: the maximum amount of search iterations
    :param initial_complexity: the stream complexity on the first iteration
    :param complexity_step: the increase in the complexity limit after each iteration
    :param max_complexity: the maximum stream complexity
    :param unit_costs: use unit action costs rather than numeric costs
    :param success_cost: an exclusive (strict) upper bound on plan cost to terminate
    :param verbose: if True, this prints the result of each stream application
    :param search_args: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    # max_complexity = 0 => current
    # complexity_step = INF => exhaustive
    # success_cost = terminate_cost = decision_cost
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
    while not store.is_terminated() and (num_iterations <= max_iterations) and (complexity_limit <= max_complexity):
        num_iterations += 1
        print('Iteration: {} | Complexity: {} | Calls: {} | Evaluations: {} | Solved: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, complexity_limit, num_calls, len(evaluations),
            store.has_solution(), store.best_cost, store.elapsed_time()))
        plan, cost = solve_finite(evaluations, goal_expression, domain,
                                  max_cost=min(store.best_cost, constraints.max_cost), **search_args)
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
        # TODO: integrate into store
        'iterations': num_iterations,
        'complexity': complexity_limit,
        # TODO: optimal, infeasible, etc...
    })
    print('Summary: {}'.format(str_from_object(summary))) # TODO: return the summary

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

##################################################

def examine_instantiated(problem, constraints=PlanConstraints(), unit_costs=False, max_time=INF, verbose=False, **search_args):
    # TODO: refactor to an analysis file
    domain_pddl, constant_map, stream_pddl, _, init, goal = problem
    stream_map = DEBUG
    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    evaluations, goal_exp, domain, externals = parse_problem(
        problem, constraints=constraints, unit_costs=unit_costs)
    store = SolutionStore(evaluations, max_time, success_cost=INF, verbose=verbose)
    #externals = compile_fluents_as_attachments(domain, externals) #

    #from pddlstream.algorithms.refinement import iterative_plan_streams, optimistic_process_streams
    #results, exhausted = optimistic_process_streams(complexity_evals, externals, complexity_limit=INF) #, **effort_args)

    instantiator = Instantiator(externals, evaluations)
    process_stream_queue(instantiator, store, complexity_limit=INF, verbose=verbose)

    #plan, cost = solve_finite(evaluations, goal_exp, domain, max_cost=max_cost, **search_args)
    debug = False
    assert not isinstance(domain, SimplifiedDomain)
    problem = get_problem(evaluations, goal_exp, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    with Verbose(debug):
        instantiated = instantiate_task(task)
        if instantiated is None:
            return None
        instantiated = convert_instantiated(instantiated)
    return instantiated

    #max_cost = min(store.best_cost, constraints.max_cost)
    # sas_task = sas_from_pddl(task, debug=debug)
    # pddl_plan, cost = abstrips_solve_from_task(sas_task, max_cost=max_cost, debug=debug, **search_args)
    # plan = obj_from_pddl_plan(pddl_plan)
    # if is_plan(plan):
    #     store.add_plan(plan, cost)
    # return store.extract_solution()