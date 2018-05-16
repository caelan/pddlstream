from __future__ import print_function

from heapq import heappush

from pddlstream.algorithm import parse_problem, SolutionStore, has_costs
from pddlstream.instantiation import Instantiator
from pddlstream.conversion import revert_solution
from pddlstream.function import Function, Predicate
from pddlstream.macro_stream import get_macro_stream_plan
from pddlstream.postprocess import locally_optimize
from pddlstream.reorder import separate_plan, reorder_combined_plan, reorder_stream_plan
from pddlstream.scheduling.relaxed import relaxed_stream_plan
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan
from pddlstream.statistics import get_action_info, update_stream_info, load_stream_statistics, \
    write_stream_statistics
from pddlstream.stream_plan import populate_results, instantiate_first, \
    Skeleton, SkeletonKey, greedily_process_queue, fairly_process_queue
from pddlstream.utils import INF
from pddlstream.visualization import clear_visualizations, create_visualizations
from pddlstream.incremental import layered_process_stream_queue


# TODO: compute total stream plan p_success and overhead
# TODO: ensure search and sampling have equal time
# TODO: select whether to search or sample based on expected success rates
# TODO: estimate the success rate for a stream_plan given past outcomes
# TODO: make a subroutine that does commit

def partition_externals(externals):
    functions = filter(lambda s: type(s) is Function, externals)
    negative = filter(lambda s: (type(s) is Predicate) and s.is_negative(), externals)
    streams = filter(lambda s: s not in (functions + negative), externals)
    return streams, functions, negative


##################################################

def solve_focused(problem, stream_info={}, action_info={}, dynamic_streams=[],
                  max_time=INF, max_cost=INF, unit_costs=None, sampling_time=0,
                  commit=True, effort_weight=None, eager_layers=1,
                  visualize=False, verbose=True, postprocess=False, **search_kwargs):
    """
    Solves a PDDLStream problem by first hypothesizing stream outputs and then determining whether they exist
    :param problem: a PDDLStream problem
    :param action_info: a dictionary from stream name to ActionInfo for planning and execution
    :param stream_info: a dictionary from stream name to StreamInfo altering how individual streams are handled
    :param max_time: the maximum amount of time to apply streams
    :param max_cost: a strict upper bound on plan cost
    :param commit: if True, it commits to instantiating a particular partial plan-skeleton.
    :param effort_weight: a multiplier for stream effort compared to action costs
    :param eager_layers: the number of eager stream application layers per iteration
    :param visualize: if True, it draws the constraint network and stream plan as a graphviz file
    :param verbose: if True, this prints the result of each stream application
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    # TODO: return to just using the highest level samplers at the start
    num_iterations = 0
    store = SolutionStore(max_time, max_cost, verbose) # TODO: include other info here?
    evaluations, goal_expression, domain, stream_name, externals = parse_problem(problem)
    if unit_costs is None:
        unit_costs = not has_costs(domain)
    full_action_info = get_action_info(action_info)
    update_stream_info(externals, stream_info)
    load_stream_statistics(stream_name, externals)
    if visualize:
        clear_visualizations()
    eager_externals = filter(lambda e: e.info.eager, externals)
    streams, functions, negative = partition_externals(externals)
    queue = []
    # TODO: switch to searching if believe chance of search better than sampling
    while not store.is_terminated():
        num_iterations += 1
        # TODO: decide max_sampling_time based on total search_time or likelihood estimates
        print('\nIteration: {} | Queue: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(queue), len(evaluations), store.best_cost, store.elapsed_time()))
        # TODO: constrain to use previous plan to some degree
        layered_process_stream_queue(Instantiator(evaluations, eager_externals), evaluations, store, eager_layers)
        stream_results = populate_results(evaluations, streams + functions)
        # TODO: warning check if using simultaneous_stream_plan or sequential_stream_plan with non-eager functions
        solve_stream_plan = relaxed_stream_plan if effort_weight is None else simultaneous_stream_plan
        combined_plan, cost = solve_stream_plan(evaluations, goal_expression, domain, stream_results,
                                                negative, max_cost=store.best_cost,
                                                unit_costs=unit_costs, **search_kwargs)
        if action_info:
            combined_plan = reorder_combined_plan(evaluations, combined_plan, full_action_info, domain)
        #print('Combined plan: {}'.format(combined_plan))
        stream_plan, action_plan = separate_plan(combined_plan, full_action_info)
        stream_plan = reorder_stream_plan(stream_plan) # TODO: is this strictly redundant?
        stream_plan = get_macro_stream_plan(stream_plan, dynamic_streams)
        print('Stream plan: {}\n'
              'Action plan: {}'.format(stream_plan, action_plan))

        if stream_plan is None:
            if queue:
                 fairly_process_queue(queue, evaluations, store)
            else:
                break
        else:
            if visualize:
                create_visualizations(evaluations, stream_plan, num_iterations)
            sampling_key = SkeletonKey(0, len(stream_plan))
            bindings = {}
            sampling_problem = Skeleton(instantiate_first(bindings, stream_plan), 0,
                                        bindings, stream_plan, action_plan, cost)
            heappush(queue, (sampling_key, sampling_problem))
            greedily_process_queue(queue, evaluations, store, sampling_time)

    if postprocess and (not unit_costs) and (store.best_plan is not None):
        store.best_plan = locally_optimize(evaluations, store.best_plan, goal_expression, domain,
                                     functions, negative, dynamic_streams, verbose)
    write_stream_statistics(stream_name, externals)
    return revert_solution(store.best_plan, store.best_cost, evaluations)