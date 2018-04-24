import time

from pddlstream.conversion import evaluation_from_fact, revert_solution, substitute_expression
from pddlstream.algorithm import parse_problem, solve_finite, print_output_values_list, process_stream_queue, \
    get_optimistic_constraints
from pddlstream.instantiation import Instantiator
from pddlstream.stream import StreamInstance, StreamResult
from pddlstream.relaxed_scheduling import relaxed_stream_plan
from pddlstream.sequential_scheduling import sequential_stream_plan
from pddlstream.simultaneous_scheduling import simultaneous_stream_plan
from pddlstream.utils import INF, elapsed_time, clear_dir
from pddlstream.object import Object
from pddlstream.visualization import visualize_stream_plan, visualize_stream_plan_bipartite, \
    visualize_constraints
from collections import defaultdict
from itertools import product
import os

CONSTRAINT_NETWORK_DIR = 'constraint_networks/'
STREAM_PLAN_DIR = 'stream_plans/'
ITERATION_TEMPLATE = 'iteration_{}.pdf'

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

##################################################

def disable_stream_instance(stream_instance, disabled):
    disabled.append(stream_instance)
    stream_instance.disabled = True

def reset_disabled(disabled):
    for stream_instance in disabled:
        stream_instance.disabled = False
    disabled[:] = []

##################################################

def ground_stream_instances(stream_instance, bindings, evaluations):
    # TODO: combination for domain predicates
    input_values = [[i] if isinstance(i, Object) else bindings[i]
                    for i in stream_instance.input_values]
    for combo in product(*input_values):
        mapping = dict(zip(stream_instance.input_values, combo))
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping)))
        if domain <= evaluations:
            yield stream_instance.stream.get_instance(combo)

def process_stream_plan(evaluations, stream_plan, disabled, verbose, quick_fail=True, max_values=1):
    # TODO: return instance for the committed algorithm
    new_evaluations = []
    opt_bindings = defaultdict(list)
    unexplored_stream_instances = []
    failure = False
    for opt_stream_result in stream_plan:
        stream_instances = list(ground_stream_instances(opt_stream_result.stream_instance,
                                                        opt_bindings, evaluations))
        unexplored_stream_instances += stream_instances[max_values:]
        for stream_instance in stream_instances[:max_values]:
            disable_stream_instance(stream_instance, disabled)
            output_values_list = stream_instance.next_outputs() if not stream_instance.enumerated else []
            if verbose:
                print_output_values_list(stream_instance, output_values_list)
            if not output_values_list:
                failure = True
                if quick_fail:
                    break
            for output_values in output_values_list:
                stream_result = StreamResult(stream_instance, output_values)
                for opt, val in zip(opt_stream_result.output_values, stream_result.output_values):
                    opt_bindings[opt].append(val)
                for fact in stream_result.get_certified():
                    evaluation = evaluation_from_fact(fact)
                    evaluations.add(evaluation) # To be used on next iteration
                    new_evaluations.append(evaluation)
    # TODO: return unexplored_stream_instances
    # TODO: retrace successful argument path upon success
    return new_evaluations

##################################################

def solve_focused(problem, max_time=INF, effort_weight=None, visualize=False, verbose=False, **kwargs):
    # TODO: eager, negative, context, costs, bindings
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    disabled = []
    if visualize:
        clear_dir(CONSTRAINT_NETWORK_DIR)
        clear_dir(STREAM_PLAN_DIR)
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
        #solve_stream_plan = sequential_stream_plan if effort_weight is None else simultaneous_stream_plan
        solve_stream_plan = relaxed_stream_plan
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
            if visualize:
                # TODO: place it in the temp_dir?
                filename = ITERATION_TEMPLATE.format(num_iterations)
                #visualize_stream_plan(stream_plan, path)
                visualize_constraints(get_optimistic_constraints(evaluations, stream_plan),
                                      os.path.join(CONSTRAINT_NETWORK_DIR, filename))
                visualize_stream_plan_bipartite(stream_plan,
                                                os.path.join(STREAM_PLAN_DIR, filename))
            process_stream_plan(evaluations, stream_plan, disabled, verbose)

    return revert_solution(best_plan, best_cost, evaluations)