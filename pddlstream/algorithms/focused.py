from __future__ import print_function

import time

from pddlstream.algorithms.algorithm import parse_problem, SolutionStore, has_costs, dump_plans, \
    partition_externals
from pddlstream.algorithms.disabled import process_disabled
from pddlstream.algorithms.incremental import layered_process_stream_queue
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.postprocess import locally_optimize
from pddlstream.algorithms.refine_shared import iterative_solve_stream_plan
from pddlstream.algorithms.reorder import separate_plan, reorder_combined_plan, reorder_stream_plan
from pddlstream.algorithms.scheduling.relaxed import relaxed_stream_plan
from pddlstream.algorithms.skeleton import SkeletonQueue, process_skeleton_queue
# from pddlstream.algorithms.scheduling.sequential import sequential_stream_plan
# from pddlstream.algorithms.scheduling.incremental import incremental_stream_plan, exhaustive_stream_plan
from pddlstream.algorithms.visualization import reset_visualizations, create_visualizations, \
    has_pygraphviz, log_plans
from pddlstream.language.conversion import revert_solution
from pddlstream.language.execution import get_action_info
from pddlstream.language.optimizer import combine_optimizers, replan_with_optimizers
from pddlstream.language.statistics import load_stream_statistics, \
    write_stream_statistics
from pddlstream.utils import INF, elapsed_time


# TODO: make stream_info just a dict
# TODO: implement the holdout of evaluations strategy

def solve_focused(problem, stream_info={}, action_info={}, synthesizers=[],
                  max_time=INF, max_cost=INF, unit_costs=False,
                  unit_efforts=False, effort_weight=None, eager_layers=1,
                  search_sampling_ratio=1, use_skeleton=True,
                  visualize=False, verbose=True,
                  reorder=True, postprocess=False, **search_kwargs):
    """
    Solves a PDDLStream problem by first hypothesizing stream outputs and then determining whether they exist
    :param problem: a PDDLStream problem
    :param action_info: a dictionary from stream name to ActionInfo for planning and execution
    :param stream_info: a dictionary from stream name to StreamInfo altering how individual streams are handled
    :param synthesizers: a list of StreamSynthesizer objects
    :param max_time: the maximum amount of time to apply streams
    :param max_cost: a strict upper bound on plan cost
    :param unit_costs: use unit action costs rather than numeric costs
    :param unit_efforts: use unit stream efforts rather than estimated numeric efforts
    :param effort_weight: a multiplier for stream effort compared to action costs
    :param eager_layers: the number of eager stream application layers per iteration
    :param search_sampling_ratio: the desired ratio of search time / sample time
    :param visualize: if True, it draws the constraint network and stream plan as a graphviz file
    :param verbose: if True, this prints the result of each stream application
    :param postprocess: postprocess the stream plan to find a better solution
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    # TODO: return to just using the highest level samplers at the start
    # TODO: select whether to search or sample based on expected success rates
    solve_stream_plan_fn = relaxed_stream_plan
    #solve_stream_plan_fn = relaxed_stream_plan if effort_weight is None else simultaneous_stream_plan
    #solve_stream_plan_fn = sequential_stream_plan # simultaneous_stream_plan | sequential_stream_plan
    #solve_stream_plan_fn = incremental_stream_plan # incremental_stream_plan | exhaustive_stream_plan
    # TODO: warning check if using simultaneous_stream_plan or sequential_stream_plan with non-eager functions
    # TODO: no optimizers during search with relaxed_stream_plan
    num_iterations = 0
    search_time = sample_time = 0
    store = SolutionStore(max_time, max_cost, verbose) # TODO: include other info here?
    evaluations, goal_expression, domain, externals = parse_problem(problem, stream_info)
    unit_costs |= not has_costs(domain)
    full_action_info = get_action_info(action_info)
    load_stream_statistics(externals + synthesizers)
    if visualize and not has_pygraphviz():
        visualize = False
        print('Warning, visualize=True requires pygraphviz. Setting visualize=False')
    if visualize:
        reset_visualizations()
    eager_externals = list(filter(lambda e: e.info.eager, externals))
    streams, functions, negative = partition_externals(externals)
    if verbose:
        print('Streams: {}\nFunctions: {}\nNegated: {}'.format(streams, functions, negative))
    queue = SkeletonQueue(store, evaluations, goal_expression, domain)
    disabled = set()
    while not store.is_terminated():
        start_time = time.time()
        num_iterations += 1
        print('\nIteration: {} | Queue: {} | Evaluations: {} | Cost: {} '
              '| Search Time: {:.3f} | Sample Time: {:.3f} | Total Time: {:.3f}'.format(
            num_iterations, len(queue), len(evaluations), store.best_cost,
            search_time, sample_time, store.elapsed_time()))

        layered_process_stream_queue(Instantiator(evaluations, eager_externals), evaluations, store, eager_layers)
        solve_stream_plan = lambda sr: solve_stream_plan_fn(evaluations, goal_expression, domain, sr, negative,
                                                            max_cost=store.best_cost, #max_cost=min(store.best_cost, max_cost),
                                                            unit_costs=unit_costs,
                                                            unit_efforts=unit_efforts, effort_weight=effort_weight,
                                                            **search_kwargs)
        #combined_plan, cost = solve_stream_plan(optimistic_process_streams(evaluations, streams + functions))
        combined_plan, cost = iterative_solve_stream_plan(evaluations, streams, functions, solve_stream_plan)
        if action_info:
            combined_plan = reorder_combined_plan(evaluations, combined_plan, full_action_info, domain)
            print('Combined plan: {}'.format(combined_plan))
        stream_plan, action_plan = separate_plan(combined_plan, full_action_info)
        #stream_plan = replan_with_optimizers(evaluations, stream_plan, domain, externals)
        stream_plan = combine_optimizers(evaluations, stream_plan)
        #stream_plan = get_synthetic_stream_plan(stream_plan, # evaluations
        #                                        [s for s in synthesizers if not s.post_only])
        if reorder:
            stream_plan = reorder_stream_plan(stream_plan) # TODO: is this redundant when combined_plan?
        dump_plans(stream_plan, action_plan, cost)
        if (stream_plan is not None) and visualize:
            log_plans(stream_plan, action_plan, num_iterations)
            create_visualizations(evaluations, stream_plan, num_iterations)
        search_time += elapsed_time(start_time)

        # TODO: more generally just add the original plan skeleton to the plan
        # TODO: cutoff search exploration time at a certain point
        start_time = time.time()
        allocated_sample_time = search_sampling_ratio*search_time - sample_time
        if use_skeleton:
            terminate = not process_skeleton_queue(store, queue, stream_plan, action_plan, cost, allocated_sample_time)
        else:
            reenable = effort_weight is not None # No point if not stream effort considerations
            terminate = not process_disabled(store, evaluations, domain, disabled, stream_plan, action_plan, cost,
                                             allocated_sample_time, reenable)
        sample_time += elapsed_time(start_time)
        if terminate:
            break

    if postprocess and (not unit_costs): # and synthesizers
        locally_optimize(evaluations, store, goal_expression, domain,
                         functions, negative, synthesizers, visualize)
    write_stream_statistics(externals + synthesizers, verbose)
    return revert_solution(store.best_plan, store.best_cost, evaluations)