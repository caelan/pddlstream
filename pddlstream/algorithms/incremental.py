import time

from pddlstream.language.statistics import load_stream_statistics, write_stream_statistics
from pddlstream.algorithms.algorithm import parse_problem, SolutionStore, add_facts, add_certified, solve_finite
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.language.conversion import revert_solution
from pddlstream.language.function import FunctionInstance
from pddlstream.language.stream import Stream
from pddlstream.utils import INF
from pddlstream.utils import elapsed_time

DEFAULT_VERBOSE = True
UPDATE_STATISTICS = False

def ensure_no_fluent_streams(streams):
    for stream in streams:
        if isinstance(stream, Stream) and stream.is_fluent():
            raise NotImplementedError('Algorithm does not support fluent stream: {}'.format(stream.name))

def process_stream_queue(instantiator, evaluations, **kwargs):
    instance = instantiator.pop()
    if instance.enumerated:
        return
    new_results, new_facts = instance.next_results(**kwargs)
    #if new_results and isinstance(instance, StreamInstance):
    #    evaluations.pop(evaluation_from_fact(instance.get_blocked_fact()), None)
    for result in new_results:
        for evaluation in add_certified(evaluations, result):
            instantiator.add_atom(evaluation)
    for evaluation in add_facts(evaluations, new_facts, result=None): # TODO: record the instance?
        instantiator.add_atom(evaluation)
    if not instance.enumerated:
        instantiator.push(instance)

def function_process_stream_queue(instantiator, evaluations, **kwargs):
    for _ in range(len(instantiator)):
        if isinstance(instantiator.head(), FunctionInstance):
            process_stream_queue(instantiator, evaluations, **kwargs)
        else:
            instantiator.poppush()

##################################################

def solve_current(problem, constraints=PlanConstraints(),
                  unit_costs=False, verbose=DEFAULT_VERBOSE, **search_kwargs):
    """
    Solves a PDDLStream problem without applying any streams
    Will fail if the problem requires stream applications
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the available solutions
    :param unit_costs: use unit action costs rather than numeric costs
    :param verbose: if True, this prints the result of each stream application
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    evaluations, goal_expression, domain, externals = parse_problem(
        problem, constraints=constraints, unit_costs=unit_costs)
    instantiator = Instantiator(evaluations, externals)
    function_process_stream_queue(instantiator, evaluations, verbose=verbose)
    plan, cost = solve_finite(evaluations, goal_expression, domain,
                              max_cost=constraints.max_cost, **search_kwargs)
    return revert_solution(plan, cost, evaluations)

##################################################

def solve_exhaustive(problem, constraints=PlanConstraints(),
                     unit_costs=False, max_time=300, verbose=DEFAULT_VERBOSE, **search_kwargs):
    """
    Solves a PDDLStream problem by applying all possible streams and searching once
    Requires a finite max_time when infinitely many stream instances
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the available solutions
    :param unit_costs: use unit action costs rather than numeric costs
    :param max_time: the maximum amount of time to apply streams
    :param verbose: if True, this prints the result of each stream application
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    start_time = time.time()
    evaluations, goal_expression, domain, externals = parse_problem(
        problem, constraints=constraints, unit_costs=unit_costs)
    ensure_no_fluent_streams(externals)
    if UPDATE_STATISTICS:
        load_stream_statistics(externals)
    instantiator = Instantiator(evaluations, externals)
    while instantiator and (elapsed_time(start_time) < max_time):
        process_stream_queue(instantiator, evaluations, verbose=verbose)
    function_process_stream_queue(instantiator, evaluations, verbose=verbose)
    plan, cost = solve_finite(evaluations, goal_expression, domain,
                              max_cost=constraints.max_cost, **search_kwargs)
    if UPDATE_STATISTICS:
        write_stream_statistics(externals, verbose)
    return revert_solution(plan, cost, evaluations)

##################################################

def layered_process_stream_queue(instantiator, evaluations, store, num_layers, **kwargs):
    # TODO: priority queue and iteratively increase max stream max or add effort
    num_calls = 0
    for layer in range(num_layers):
        for _ in range(len(instantiator)):
            if store.is_terminated():
                return num_calls
            process_stream_queue(instantiator, evaluations, **kwargs)
            num_calls += 1
    return num_calls

def solve_incremental(problem, constraints=PlanConstraints(),
                      unit_costs=False, success_cost=INF,
                      max_iterations=INF, layers_per_iteration=1,
                      max_time=INF, verbose=DEFAULT_VERBOSE,
                      **search_kwargs):
    """
    Solves a PDDLStream problem by alternating between applying all possible streams and searching
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the set of legal solutions
    :param layers_per_iteration: the number of stream application layers per iteration
    :param max_time: the maximum amount of time to apply streams
    :param max_iterations: the maximum amount of search iterations
    :param unit_costs: use unit action costs rather than numeric costs
    :param success_cost: an exclusive (strict) upper bound on plan cost to terminate
    :param verbose: if True, this prints the result of each stream application
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    # success_cost = terminate_cost = decision_cost
    store = SolutionStore(max_time, success_cost, verbose) # TODO: include other info here?
    evaluations, goal_expression, domain, externals = parse_problem(
        problem, constraints=constraints, unit_costs=unit_costs)
    ensure_no_fluent_streams(externals)
    if UPDATE_STATISTICS:
        load_stream_statistics(externals)
    num_iterations = 0
    num_calls = 0
    instantiator = Instantiator(evaluations, externals)
    while not store.is_terminated() and (num_iterations < max_iterations):
        num_iterations += 1
        print('Iteration: {} | Calls: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, num_calls, len(evaluations), store.best_cost, store.elapsed_time()))
        function_process_stream_queue(instantiator, evaluations, verbose=verbose)
        max_cost = min(store.best_cost, constraints.max_cost)
        plan, cost = solve_finite(evaluations, goal_expression, domain, max_cost=max_cost, **search_kwargs)
        if plan is not None:
            store.add_plan(plan, cost)
        if store.is_terminated() or not instantiator:
            break
        num_calls += layered_process_stream_queue(instantiator, evaluations, store,
                                                  layers_per_iteration, verbose=verbose)
    if UPDATE_STATISTICS:
        write_stream_statistics(externals, verbose)
    return revert_solution(store.best_plan, store.best_cost, evaluations)
