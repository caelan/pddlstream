import time

from pddlstream.instantiation import Instantiator
from pddlstream.skeleton import optimistic_process_stream_queue, eagerly_evaluate

from experimental.context import ConstraintSolver
from experimental.stream_plan import reset_disabled, disable_stream_instance
from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.scheduling import sequential_stream_plan
from pddlstream.algorithms.scheduling import simultaneous_stream_plan
from pddlstream.algorithms.visualization import clear_visualizations, create_visualizations, get_optimistic_constraints
from pddlstream.language.conversion import evaluation_from_fact, revert_solution
from pddlstream.language.statistics import update_stream_info
from pddlstream.utils import INF, elapsed_time


#def query_stream(stream_instance, verbose):
#    output_objects_list = stream_instance.next_outputs() if not stream_instance.enumerated else []
#    if verbose:
#        stream_instance.dump_output_list(output_objects_list)
#    return [StreamResult(stream_instance, output_objects) for output_objects in output_objects_list]
#
# def process_stream_plan(evaluations, stream_plan, disabled, verbose, quick_fail=True, max_values=1):
#     # TODO: return instance for the committed algorithm
#     new_evaluations = []
#     opt_bindings = defaultdict(list)
#     unexplored_stream_instances = []
#     failure = False
#     for opt_stream_result in stream_plan:
#         # TODO: could bind by just using new_evaluations
#         stream_instances = list(ground_stream_instances(opt_stream_result.instance,
#                                                         opt_bindings, evaluations))
#         unexplored_stream_instances += stream_instances[max_values:]
#         for stream_instance in stream_instances[:max_values]:
#             disable_stream_instance(stream_instance, disabled)
#             stream_results = query_stream(stream_instance, verbose)
#             for stream_result in stream_results:
#                 for opt, val in zip(opt_stream_result.output_objects, stream_result.output_objects):
#                     opt_bindings[opt].append(val)
#                 for fact in stream_result.get_certified():
#                     evaluation = evaluation_from_fact(fact)
#                     evaluations.add(evaluation) # To be used on next iteration
#                     new_evaluations.append(evaluation)
#             if not stream_results:
#                 failure = True
#                 if quick_fail:
#                     break
#     # TODO: return unexplored_stream_instances
#     # TODO: retrace successful argument path upon success
#     # TODO: identify subset of the initial state that support the plan
#     return new_evaluations


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

def solve_focused(problem, max_time=INF, stream_info={}, effort_weight=None, eager_iterations=1,
                  visualize=False, verbose=True, **kwargs):
    # TODO: eager, negative, context, costs, bindings
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, externals = parse_problem(problem)
    update_stream_info(externals, stream_info)
    eager_externals = filter(lambda e: e.info.eager, externals)
    constraint_solver = ConstraintSolver(problem[3])
    disabled = []
    if visualize:
        clear_visualizations()
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('\nIteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        eagerly_evaluate(evaluations, eager_externals, eager_iterations, max_time - elapsed_time(start_time), verbose)
        # TODO: version that just calls one of the incremental algorithms
        instantiator = Instantiator(evaluations, externals)
        stream_results = []
        while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
            stream_results += optimistic_process_stream_queue(instantiator)
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
                create_visualizations(evaluations, stream_plan, num_iterations)
            constraint_facts = constraint_solver.solve(get_optimistic_constraints(evaluations, stream_plan), verbose=verbose)
            if constraint_facts:
                evaluations.update(map(evaluation_from_fact, constraint_facts))
            else:
                #process_stream_plan(evaluations, stream_plan, disabled, verbose)
                process_immediate_stream_plan(evaluations, stream_plan, disabled, verbose)

    return revert_solution(best_plan, best_cost, evaluations)
