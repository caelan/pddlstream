import time

from pddlstream.algorithm import parse_problem, solve_finite, process_stream_queue
from pddlstream.conversion import revert_solution
from pddlstream.instantiation import Instantiator
from pddlstream.stream import Function
from pddlstream.utils import INF, elapsed_time


def solve_current(problem, **kwargs):
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    return revert_solution(plan, cost, evaluations)

def solve_exhaustive(problem, max_time=INF, verbose=True, **kwargs):
    start_time = time.time()
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    instantiator = Instantiator(evaluations, streams)
    while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
        process_stream_queue(instantiator, evaluations, prioritized=False, optimistic=False, verbose=verbose)
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    return revert_solution(plan, cost, evaluations)

def prioritize_functions(streams):
    for stream in streams:
        if isinstance(stream, Function):
            stream.prioritized = True

def solve_incremental(problem, max_time=INF, max_cost=INF, verbose=True, **kwargs):
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    prioritize_functions(streams)
    instantiator = Instantiator(evaluations, streams)
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        while instantiator.prioritized_stream_queue and (elapsed_time(start_time) < max_time):
            process_stream_queue(instantiator, evaluations, prioritized=True, verbose=verbose)
        plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
        if cost < best_cost:
            best_plan = plan; best_cost = cost
        if (best_cost < max_cost) or not instantiator.stream_queue:
            break
        for _ in range(len(instantiator.stream_queue)):
            if max_time <= elapsed_time(start_time):
                break
            process_stream_queue(instantiator, evaluations, prioritized=False, verbose=verbose)
    return revert_solution(best_plan, best_cost, evaluations)
