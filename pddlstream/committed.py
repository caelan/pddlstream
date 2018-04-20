from pddlstream.conversion import evaluation_from_fact
from pddlstream.incremental import parse_problem, solve_finite, revert_solution, \
    process_stream_queue, print_output_values_list
from pddlstream.instantiation import Instantiator
from pddlstream.stream import StreamInstance, StreamResult
from pddlstream.stream_scheduling import sequential_stream_plan, simultaneous_stream_plan
from pddlstream.utils import INF, elapsed_time
from focused import reset_disabled, process_stream_plan

import time

def solve_committed(problem, max_time=INF, effort_weight=None, verbose=False, **kwargs):
    # TODO: constrain plan skeleton
    # TODO: constrain ususable samples
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    disabled = []

    # TOD:
    instantiator = Instantiator(evaluations, streams)
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        stream_results = []
        while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
            stream_results += process_stream_queue(instantiator, None,
                                                   StreamInstance.next_optimistic,
                                                   revisit=False, verbose=False)

        solve_stream_plan = sequential_stream_plan if effort_weight is None else simultaneous_stream_plan
        stream_plan, action_plan = solve_stream_plan(evaluations, goal_expression,
                                                     domain, stream_results, **kwargs)
        print('Stream plan: {}\n'
              'Action plan: {}'.format(stream_plan, action_plan))
        if stream_plan is None:
            if not disabled:
                break
            reset_disabled(disabled)
        elif len(stream_plan) == 0:
            best_plan = action_plan
            break
        else:
            # TODO: break if no new evaluations?
            for evaluation in process_stream_plan(evaluations, stream_plan, disabled, verbose):
                instantiator.add_atom(evaluation)

    return revert_solution(best_plan, best_cost, evaluations)