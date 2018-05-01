import os
import time
from collections import defaultdict
from itertools import product

from pddlstream.algorithm import parse_problem, optimistic_process_stream_queue, \
    get_optimistic_constraints
from pddlstream.conversion import evaluation_from_fact, revert_solution, substitute_expression
from pddlstream.instantiation import Instantiator
from pddlstream.object import Object
from pddlstream.scheduling.sequential import sequential_stream_plan
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan
from pddlstream.scheduling.incremental import exhaustive_stream_plan, incremental_stream_plan
from pddlstream.scheduling.relaxed import relaxed_stream_plan
from pddlstream.stream import StreamResult
from pddlstream.utils import INF, elapsed_time, clear_dir
from pddlstream.visualization import visualize_stream_plan_bipartite, \
    visualize_constraints

CONSTRAINT_NETWORK_DIR = 'constraint_networks/'
STREAM_PLAN_DIR = 'stream_plans/'
ITERATION_TEMPLATE = 'iteration_{}.pdf'

##################################################

class StreamOptions(object):
    # TODO: make bound, effort, etc meta-parameters of the algorithms or part of the problem?
    def __init__(self, bound_fn, effort_fn, prioritized=False):
        # TODO: could change frequency/priority for the incremental algorithm
        self.bound_fn = bound_fn
        self.effort_fn = effort_fn
        self.prioritized = prioritized
        # TODO: context?

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
    input_objects = [[i] if isinstance(i, Object) else bindings[i]
                    for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = dict(zip(stream_instance.input_objects, combo))
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping)))
        if domain <= evaluations:
            yield stream_instance.stream.get_instance(combo)

def query_stream(stream_instance, verbose):
    output_objects_list = stream_instance.next_outputs() if not stream_instance.enumerated else []
    if verbose:
        stream_instance.dump_output_list(output_objects_list)
    return [StreamResult(stream_instance, output_objects) for output_objects in output_objects_list]

def process_stream_plan(evaluations, stream_plan, disabled, verbose, quick_fail=True, max_values=1):
    # TODO: return instance for the committed algorithm
    new_evaluations = []
    opt_bindings = defaultdict(list)
    unexplored_stream_instances = []
    failure = False
    for opt_stream_result in stream_plan:
        # TODO: could bind by just using new_evaluations
        stream_instances = list(ground_stream_instances(opt_stream_result.instance,
                                                        opt_bindings, evaluations))
        unexplored_stream_instances += stream_instances[max_values:]
        for stream_instance in stream_instances[:max_values]:
            disable_stream_instance(stream_instance, disabled)
            stream_results = query_stream(stream_instance, verbose)
            for stream_result in stream_results:
                for opt, val in zip(opt_stream_result.output_objects, stream_result.output_objects):
                    opt_bindings[opt].append(val)
                for fact in stream_result.get_certified():
                    evaluation = evaluation_from_fact(fact)
                    evaluations.add(evaluation) # To be used on next iteration
                    new_evaluations.append(evaluation)
            if not stream_results:
                failure = True
                if quick_fail:
                    break
    # TODO: return unexplored_stream_instances
    # TODO: retrace successful argument path upon success
    # TODO: identify subset of the initial state that support the plan
    return new_evaluations


def process_immediate_stream_plan(evaluations, stream_plan, disabled, verbose):
    new_evaluations = []
    for opt_result in stream_plan:
        instance = opt_result.instance
        if set(map(evaluation_from_fact, instance.get_domain())) <= evaluations:
            disable_stream_instance(instance, disabled)
            for result in instance.next_results(verbose=verbose):
                for fact in result.get_certified():
                    evaluation = evaluation_from_fact(fact)
                    #evaluations.add(evaluation) # To be used on next iteration
                    new_evaluations.append(evaluation)
    evaluations.update(new_evaluations)
    return new_evaluations

##################################################

def solve_focused(problem, max_time=INF, effort_weight=None, num_incr_iters=0,
                  visualize=False, verbose=True, **kwargs):
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
        # TODO: evaluate once at the beginning?
        num_iterations += 1
        print('\nIteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        # TODO: version that just calls one of the incremental algorithms
        instantiator = Instantiator(evaluations, streams)
        stream_results = []
        # TODO: apply incremetnal algorithm for sum number of iterations
        while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
            # TODO: could handle costs here
            stream_results += optimistic_process_stream_queue(instantiator, prioritized=False)
        # exhaustive_stream_plan | incremental_stream_plan | simultaneous_stream_plan | sequential_stream_plan | relaxed_stream_plan
        solve_stream_plan = sequential_stream_plan if effort_weight is None else simultaneous_stream_plan
        #solve_stream_plan = simultaneous_stream_plan
        stream_plan, action_plan, cost = solve_stream_plan(evaluations, goal_expression,
                                                     domain, stream_results, **kwargs)
        print('Stream plan: {}\n'
              'Action plan: {}'.format(stream_plan, action_plan))
        if stream_plan is None:
            if not disabled:
                break
            reset_disabled(disabled)
        elif (len(stream_plan) == 0) and (cost < best_cost):
            best_plan = action_plan; best_cost = cost
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
            #process_stream_plan(evaluations, stream_plan, disabled, verbose)
            process_immediate_stream_plan(evaluations, stream_plan, disabled, verbose)

    return revert_solution(best_plan, best_cost, evaluations)