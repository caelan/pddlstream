import time

from pddlstream.algorithm import parse_problem, solve_finite
from pddlstream.conversion import revert_solution, evaluation_from_fact
from pddlstream.instantiation import Instantiator
from pddlstream.function import Function, FunctionInstance
from pddlstream.utils import INF, elapsed_time

def process_stream_queue(instantiator, evaluations, verbose=True):
    stream_instance = instantiator.stream_queue.popleft()
    if stream_instance.enumerated:
        return
    for result in stream_instance.next_results(verbose=verbose):
        for fact in result.get_certified():
            evaluation = evaluation_from_fact(fact)
            if evaluation not in evaluations:
                evaluations[evaluation] = result
                instantiator.add_atom(evaluation)
    if not stream_instance.enumerated:
        instantiator.queue_stream_instance(stream_instance)

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
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, stream_name, streams = parse_problem(problem)
    instantiator = Instantiator(evaluations, streams)
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        for _ in range(len(instantiator.stream_queue)):
            if isinstance(instantiator.stream_queue[0], FunctionInstance):
                process_stream_queue(instantiator, evaluations, verbose=verbose)
            else:
                instantiator.stream_queue.rotate(-1)
        plan, cost = solve_finite(evaluations, goal_expression, domain, **search_kwargs)
        if cost < best_cost:
            best_plan = plan; best_cost = cost
        if (best_cost < max_cost) or not instantiator.stream_queue:
            break
        for _ in range(layers):
            for _ in range(len(instantiator.stream_queue)):
                if max_time <= elapsed_time(start_time):
                    break
                process_stream_queue(instantiator, evaluations, verbose=verbose)
    return revert_solution(best_plan, best_cost, evaluations)
