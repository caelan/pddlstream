import time

from pddlstream.conversion import evaluation_from_fact
from pddlstream.incremental import parse_problem, solve_finite, revert_solution, \
    process_stream_queue, print_output_values_list
from pddlstream.instantiation import Instantiator
from pddlstream.stream import StreamInstance, StreamResult
from pddlstream.stream_scheduling import sequential_stream_plan, simultaneous_stream_plan
from pddlstream.utils import INF, elapsed_time
from collections import defaultdict

def exhaustive_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    if stream_results:
        return stream_results, []
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    if plan is None:
        return None, plan
    return [], plan

def incremental_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    if plan is not None:
        return [], plan
    if stream_results:
        return stream_results, plan
    return None, plan

def disable_stream_instance(stream_instance, disabled):
    disabled.append(stream_instance)
    stream_instance.disabled = True

def reset_disabled(disabled):
    for stream_instance in disabled:
        stream_instance.disabled = False
    disabled[:] = []

def process_stream_plan(evaluations, stream_plan, disabled, verbose, quick_fail=True):
    new_evaluations = []
    opt_bindings = defaultdict(list)
    # TODO: bindings
    for opt_stream_result in stream_plan:
        stream_instance = opt_stream_result.stream_instance
        domain = set(map(evaluation_from_fact, stream_instance.get_domain()))
        if not (domain <= evaluations):
            continue
        disable_stream_instance(stream_instance, disabled)
        # TODO: assert something about the arguments
        output_values_list = stream_instance.next_outputs()
        if verbose:
            print_output_values_list(stream_instance, output_values_list)
        if quick_fail and (not output_values_list):
            break
        for output_values in output_values_list:
            stream_result = StreamResult(stream_instance, output_values)
            for opt, val in zip(opt_stream_result.output_values, stream_result.output_values):
                opt_bindings[opt].append(val)
            for fact in stream_result.get_certified():
                evaluation = evaluation_from_fact(fact)
                evaluations.add(evaluation) # To be used on next iteration
                new_evaluations.append(evaluation)
    return new_evaluations

def solve_focused(problem, max_time=INF, effort_weight=None, verbose=False, **kwargs):
    # TODO: eager, negative, context, costs, bindings
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    disabled = []
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        # TODO: version that just calls one of the incremental algorithms
        instantiator = Instantiator(evaluations, streams)
        stream_results = []
        while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
            stream_results += process_stream_queue(instantiator, None,
                                                   StreamInstance.next_optimistic,
                                                   revisit=False, verbose=False)
        # exhaustive_stream_plan | incremental_stream_plan | simultaneous_stream_plan | sequential_stream_plan
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
            process_stream_plan(evaluations, stream_plan, disabled, verbose)

    return revert_solution(best_plan, best_cost, evaluations)