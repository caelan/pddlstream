from __future__ import print_function

import time

from pddlstream.algorithms.algorithm import parse_problem, SolutionStore, dump_plans, partition_externals
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.disabled import process_disabled
from pddlstream.algorithms.incremental import process_stream_queue
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.refine_shared import iterative_solve_stream_plan, get_optimistic_solve_fn
from pddlstream.algorithms.reorder import separate_plan, reorder_combined_plan, reorder_stream_plan
from pddlstream.algorithms.skeleton import SkeletonQueue
# from pddlstream.algorithms.scheduling.sequential import sequential_stream_plan
# from pddlstream.algorithms.scheduling.incremental import incremental_stream_plan, exhaustive_stream_plan
from pddlstream.algorithms.visualization import reset_visualizations, create_visualizations, \
    has_pygraphviz, log_plans
from pddlstream.language.constants import is_plan
from pddlstream.language.execution import get_action_info
from pddlstream.language.optimizer import combine_optimizers
from pddlstream.language.statistics import load_stream_statistics, \
    write_stream_statistics
from pddlstream.utils import INF, elapsed_time

def solve_focused(problem, constraints=PlanConstraints(),
                  stream_info={}, action_info={}, synthesizers=[],
                  max_time=INF, max_iterations=INF, unit_costs=False, success_cost=INF,
                  unit_efforts=False, effort_step=1, max_effort=INF, effort_weight=None,
                  reorder=True, use_skeleton=True, search_sample_ratio=0,
                  visualize=False, verbose=True, **search_kwargs):
    """
    Solves a PDDLStream problem by first hypothesizing stream outputs and then determining whether they exist
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the set of legal solutions
    :param stream_info: a dictionary from stream name to StreamInfo altering how individual streams are handled
    :param action_info: a dictionary from stream name to ActionInfo for planning and execution
    :param synthesizers: a list of StreamSynthesizer objects
    :param max_time: the maximum amount of time to apply streams
    :param max_iterations: the maximum number of search iterations
    :param unit_costs: use unit action costs rather than numeric costs
    :param success_cost: an exclusive (strict) upper bound on plan cost to terminate
    :param unit_efforts: use unit stream efforts rather than estimated numeric efforts
    :param effort_step: the increase in the effort limit after each failure
    :param max_effort: the maximum amount of effort to consider for streams
    :param effort_weight: a multiplier for stream effort compared to action costs
    :param reorder: if True, stream plans are reordered to minimize the expected sampling overhead
    :param use_skeleton: maintains a set of plan skeletons to sample from
    :param search_sample_ratio: the desired ratio of search time / sample time
    :param visualize: if True, it draws the constraint network and stream plan as a graphviz file
    :param verbose: if True, this prints the result of each stream application
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    # TODO: select whether to search or sample based on expected success rates
    # TODO: no optimizers during search with relaxed_stream_plan
    num_iterations = search_time = sample_time = calls = effort_limit = 0
    evaluations, goal_exp, domain, externals = parse_problem(
        problem, stream_info=stream_info, constraints=constraints,
        unit_costs=unit_costs, unit_efforts=unit_efforts)
    store = SolutionStore(evaluations, max_time, success_cost, verbose)
    full_action_info = get_action_info(action_info)
    load_stream_statistics(externals + synthesizers)
    if visualize and not has_pygraphviz():
        visualize = False
        print('Warning, visualize=True requires pygraphviz. Setting visualize=False')
    if visualize:
        reset_visualizations()
    streams, functions, negative = partition_externals(externals, verbose=verbose)
    eager_externals = list(filter(lambda e: e.info.eager, externals))
    queue = SkeletonQueue(store, goal_exp, domain)
    disabled = set()
    terminate = False
    while (not store.is_terminated()) and (not terminate) and (num_iterations < max_iterations):
        start_time = time.time()
        num_iterations += 1
        # TODO: maintain a single Instantiator instance. Use effort_limit in this as well
        calls += process_stream_queue(Instantiator(evaluations, eager_externals, max_effort=None),
                                                   store, effort_limit=1, verbose=False)
        print('\nIteration: {} | Limit: {} | Skeletons: {} | Queue: {} | Disabled: {} | Evaluations: {} | Eager calls: {} '
              '| Cost: {:.3f} | Search Time: {:.3f} | Sample Time: {:.3f} | Total Time: {:.3f}'.format(
            num_iterations, effort_limit, len(queue.skeletons), len(queue), len(disabled), len(evaluations), calls,
            store.best_cost, search_time, sample_time, store.elapsed_time()))
        optimistic_solve_fn = get_optimistic_solve_fn(goal_exp, domain, negative,
                                                      max_cost=min(store.best_cost, constraints.max_cost),
                                                      unit_efforts=unit_efforts, effort_weight=effort_weight, **search_kwargs)
        combined_plan, cost = iterative_solve_stream_plan(evaluations, externals, optimistic_solve_fn,
                                                          effort_limit=effort_limit, max_effort=max_effort)
        if action_info:
            combined_plan = reorder_combined_plan(evaluations, combined_plan, full_action_info, domain)
            print('Combined plan: {}'.format(combined_plan))
        stream_plan, action_plan = separate_plan(combined_plan, full_action_info)
        #stream_plan = replan_with_optimizers(evaluations, stream_plan, domain, externals)
        stream_plan = combine_optimizers(evaluations, stream_plan)
        #stream_plan = get_synthetic_stream_plan(stream_plan, # evaluations
        #                                       [s for s in synthesizers if not s.post_only])
        if reorder:
            stream_plan = reorder_stream_plan(stream_plan) # This may be redundant when using reorder_combined_plan
        dump_plans(stream_plan, action_plan, cost)
        if is_plan(stream_plan) and visualize:
            log_plans(stream_plan, action_plan, num_iterations)
            create_visualizations(evaluations, stream_plan, num_iterations)
        search_time += elapsed_time(start_time)

        start_time = time.time()
        if not is_plan(stream_plan):
            effort_limit += effort_step
        if use_skeleton:
            allocated_sample_time = (search_sample_ratio * search_time) - sample_time
            if max_iterations <= num_iterations:
                allocated_sample_time = INF
            terminate = not queue.process(stream_plan, action_plan, cost, allocated_sample_time)
        else:
            reenable = effort_weight is not None # No point if no stream effort biasing
            terminate = not process_disabled(store, domain, disabled, stream_plan, action_plan, cost, reenable)
        sample_time += elapsed_time(start_time)

    write_stream_statistics(externals + synthesizers, verbose)
    return store.extract_solution()