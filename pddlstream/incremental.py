import time

from pddlstream.algorithm import parse_problem, solve_finite, SolutionStore, add_certified
from pddlstream.conversion import revert_solution
from pddlstream.instantiation import Instantiator
from pddlstream.function import FunctionInstance
from pddlstream.utils import INF, elapsed_time

def process_stream_queue(instantiator, evaluations, verbose=True):
    stream_instance = instantiator.stream_queue.popleft()
    if stream_instance.enumerated:
        return
    for result in stream_instance.next_results(verbose=verbose):
        for evaluation in add_certified(evaluations, result):
            instantiator.add_atom(evaluation)
    if not stream_instance.enumerated:
        instantiator.stream_queue.append(stream_instance)

##################################################

def solve_current(problem, **search_kwargs):
    """
    Solves a PDDLStream problem without applying any streams
    Will fail if the problem requires stream applications
    :param problem: a PDDLStream problem
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    evaluations, goal_expression, domain, stream_name, streams = parse_problem(problem)
    plan, cost = solve_finite(evaluations, goal_expression, domain, **search_kwargs)
    return revert_solution(plan, cost, evaluations)

##################################################

def solve_exhaustive(problem, max_time=300, verbose=True, **search_kwargs):
    """
    Solves a PDDLStream problem by applying all possible streams and searching once
    Requires a finite max_time when infinitely many stream instances
    :param problem: a PDDLStream problem
    :param max_time: the maximum amount of time to apply streams
    :param verbose: if True, this prints the result of each stream application
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    start_time = time.time()
    evaluations, goal_expression, domain, stream_name, streams = parse_problem(problem)
    instantiator = Instantiator(evaluations, streams)
    while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
        process_stream_queue(instantiator, evaluations, verbose=verbose)
    plan, cost = solve_finite(evaluations, goal_expression, domain, **search_kwargs)
    return revert_solution(plan, cost, evaluations)

##################################################

def function_process_stream_queue(instantiator, evaluations, store):
    for _ in range(len(instantiator.stream_queue)):
        if isinstance(instantiator.stream_queue[0], FunctionInstance):
            process_stream_queue(instantiator, evaluations, verbose=store.verbose)
        else:
            instantiator.stream_queue.rotate(-1)

def layered_process_stream_queue(instantiator, evaluations, store, num_layers):
    for _ in range(num_layers):
        for _ in range(len(instantiator.stream_queue)):
            if store.is_terminated():
                return
            process_stream_queue(instantiator, evaluations, verbose=store.verbose)

def solve_incremental(problem, max_time=INF, max_cost=INF, layers=1, verbose=True, **search_kwargs):
    """
    Solves a PDDLStream problem by alternating between applying all possible streams and searching
    :param problem: a PDDLStream problem
    :param max_time: the maximum amount of time to apply streams
    :param max_cost: a strict upper bound on plan cost
    :param layers: the number of stream application layers per iteration
    :param verbose: if True, this prints the result of each stream application
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    store = SolutionStore(max_time, max_cost, verbose) # TODO: include other info here?
    evaluations, goal_expression, domain, stream_name, streams = parse_problem(problem)
    instantiator = Instantiator(evaluations, streams)
    num_iterations = 0
    while not store.is_terminated():
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), store.best_cost, store.elapsed_time()))
        function_process_stream_queue(instantiator, evaluations, store)
        plan, cost = solve_finite(evaluations, goal_expression, domain, **search_kwargs)
        store.add_plan(plan, cost)
        if not instantiator.stream_queue:
            break
        layered_process_stream_queue(instantiator, evaluations, store, layers)
    return revert_solution(store.best_plan, store.best_cost, evaluations)
