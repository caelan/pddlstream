import time

from pddlstream.scheduling.sequential import sequential_stream_plan
from pddlstream.scheduling.relaxed import relaxed_stream_plan

from pddlstream.algorithm import parse_problem, optimistic_process_stream_queue
from pddlstream.conversion import revert_solution, evaluation_from_fact
from pddlstream.focused import reset_disabled, process_immediate_stream_plan, \
    get_optimistic_constraints, disable_stream_instance, ground_stream_instances
from pddlstream.context import ConstraintSolver
from pddlstream.instantiation import Instantiator
from pddlstream.object import OptimisticObject, Object
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan
from pddlstream.utils import INF, elapsed_time
from pddlstream.visualization import clear_visualizations, create_visualizations
from collections import defaultdict
from pddlstream.stream import StreamResult

def populate_results(evaluations, streams, max_time):
    start_time = time.time()
    instantiator = Instantiator(evaluations, streams)
    stream_results = []
    while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
        stream_results += optimistic_process_stream_queue(instantiator, prioritized=False)
    return stream_results

def process_stream_plan(evaluations, stream_plan, disabled, verbose,
                        quick_fail=True, layers=True, max_values=INF):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    # TODO: return instance for the committed algorithm
    opt_bindings = defaultdict(list)
    next_results = []
    for opt_result in stream_plan:
        # Could check opt_bindings to see if new bindings
        num_instances = max_values if (layers or all(isinstance(o, Object)
                                                     for o in opt_result.instance.input_objects)) else 0
        for i, instance in enumerate(ground_stream_instances(opt_result.instance, opt_bindings, evaluations)):
            if i < num_instances:
                results = instance.next_results(verbose=verbose)
                disable_stream_instance(instance, disabled)
            else:
                results = instance.next_optimistic()
                next_results += results
            for result in results:
                if i < num_instances:
                    evaluations.update(map(evaluation_from_fact, result.get_certified()))
                if isinstance(result, StreamResult): # Could not add if same value
                    for opt, obj in zip(opt_result.output_objects, result.output_objects):
                        opt_bindings[opt].append(obj)
            if quick_fail and not results: # TODO: check if satisfies target certified
                return [] # TODO: return None to prevent reattempt
    return next_results

def solve_committed(problem, max_time=INF, effort_weight=None, visualize=False, verbose=True, **kwargs):
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    #constraint_solver = ConstraintSolver(problem[3])
    disabled = []
    if visualize:
        clear_visualizations()
    stream_results = populate_results(evaluations, streams, max_time-elapsed_time(start_time))
    base_problem = True
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('\nIteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        solve_stream_plan = sequential_stream_plan if effort_weight is None else simultaneous_stream_plan
        #solve_stream_plan = relaxed_stream_plan
        # TODO: constrain to use previous plan to some degree
        stream_plan, action_plan, cost = solve_stream_plan(evaluations, goal_expression,
                                                     domain, stream_results, **kwargs)
        print('Stream plan: {}\n'
              'Action plan: {}'.format(stream_plan, action_plan))
        if stream_plan is None:
            if not base_problem or disabled:
                if not base_problem:
                    reset_disabled(disabled)
                stream_results = populate_results(evaluations, streams, max_time - elapsed_time(start_time))
                base_problem = True
            else:
                break
        elif (len(stream_plan) == 0) and (cost < best_cost):
            best_plan = action_plan; best_cost = cost
            break
        else:
            if visualize:
                create_visualizations(evaluations, stream_plan, num_iterations)
            stream_results = process_stream_plan(evaluations, stream_plan, disabled, verbose)
            base_problem = False
    return revert_solution(best_plan, best_cost, evaluations)