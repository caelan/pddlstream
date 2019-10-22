from __future__ import print_function

import time

from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.common import SolutionStore
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.disabled import push_disabled, reenable_disabled, process_stream_plan
from pddlstream.algorithms.disable_skeleton import create_disabled_axioms
from pddlstream.algorithms.incremental import process_stream_queue
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.refinement import iterative_plan_streams, get_optimistic_solve_fn
from pddlstream.algorithms.reorder import reorder_stream_plan
from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.algorithms.visualization import reset_visualizations, create_visualizations, \
    has_pygraphviz, log_plans
from pddlstream.language.constants import is_plan, get_length, str_from_plan, INFEASIBLE
from pddlstream.language.function import Function, Predicate
from pddlstream.language.optimizer import ComponentStream
from pddlstream.algorithms.recover_optimizers import combine_optimizers
from pddlstream.language.statistics import load_stream_statistics, \
    write_stream_statistics, compute_plan_effort
from pddlstream.language.stream import Stream
from pddlstream.utils import INF, elapsed_time, implies, user_input, check_memory

def get_negative_externals(externals):
    negative_predicates = list(filter(lambda s: type(s) is Predicate, externals)) # and s.is_negative()
    negated_streams = list(filter(lambda s: isinstance(s, Stream) and s.is_negated(), externals))
    return negative_predicates + negated_streams

def partition_externals(externals, verbose=False):
    functions = list(filter(lambda s: type(s) is Function, externals))
    negative = get_negative_externals(externals)
    optimizers = list(filter(lambda s: isinstance(s, ComponentStream) and (s not in negative), externals))
    streams = list(filter(lambda s: s not in (functions + negative + optimizers), externals))
    if verbose:
        print('Streams: {}\nFunctions: {}\nNegated: {}\nOptimizers: {}'.format(
            streams, functions, negative, optimizers))
    return streams, functions, negative, optimizers

def solve_focused(problem, constraints=PlanConstraints(), stream_info={}, replan_actions=set(),
                  max_time=INF, max_iterations=INF, max_memory=INF,
                  initial_complexity=0, complexity_step=1,
                  max_skeletons=INF, bind=True, max_failures=0,
                  unit_costs=False, success_cost=INF,
                  unit_efforts=False, max_effort=INF, effort_weight=None,
                  reorder=True, search_sample_ratio=0,
                  visualize=False, verbose=True, **search_kwargs):
    """
    Solves a PDDLStream problem by first hypothesizing stream outputs and then determining whether they exist
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the set of legal solutions
    :param stream_info: a dictionary from stream name to StreamInfo altering how individual streams are handled
    :param max_time: the maximum amount of time to apply streams
    :param max_iterations: the maximum number of search iterations
    :param max_skeletons: the maximum number of plan skeletons to consider
    :param unit_costs: use unit action costs rather than numeric costs
    :param success_cost: an exclusive (strict) upper bound on plan cost to terminate
    :param unit_efforts: use unit stream efforts rather than estimated numeric efforts
    :param initial_complexity: the initial effort limit
    :param complexity_step: the increase in the effort limit after each failure
    :param max_effort: the maximum amount of effort to consider for streams
    :param effort_weight: a multiplier for stream effort compared to action costs
    :param reorder: if True, stream plans are reordered to minimize the expected sampling overhead
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
    # TODO: locally optimize only after a solution is identified
    # TODO: replan with a better search algorithm after feasible
    num_iterations = search_time = sample_time = eager_calls = 0
    complexity_limit = initial_complexity
    # TODO: make effort_weight be a function of the current cost
    # TODO: change the search algorithm and unit costs based on the best cost
    eager_disabled = effort_weight is None  # No point if no stream effort biasing
    evaluations, goal_exp, domain, externals = parse_problem(
        problem, stream_info=stream_info, constraints=constraints,
        unit_costs=unit_costs, unit_efforts=unit_efforts)
    store = SolutionStore(evaluations, max_time, success_cost, verbose, max_memory=max_memory)
    load_stream_statistics(externals)
    if visualize and not has_pygraphviz():
        visualize = False
        print('Warning, visualize=True requires pygraphviz. Setting visualize=False')
    if visualize:
        reset_visualizations()
    streams, functions, negative, optimizers = partition_externals(externals, verbose=verbose)
    eager_externals = list(filter(lambda e: e.info.eager, externals))
    positive_externals = streams + functions + optimizers
    use_skeletons = max_skeletons is not None
    has_optimizers = bool(optimizers)
    assert implies(has_optimizers, use_skeletons)
    skeleton_queue = SkeletonQueue(store, domain, disable=not has_optimizers)
    disabled = set() # Max skeletons after a solution
    while (not store.is_terminated()) and (num_iterations < max_iterations):
        start_time = time.time()
        num_iterations += 1
        eager_instantiator = Instantiator(eager_externals, evaluations) # Only update after an increase?
        if eager_disabled:
            push_disabled(eager_instantiator, disabled)
        eager_calls += process_stream_queue(eager_instantiator, store,
                                            complexity_limit=complexity_limit, verbose=verbose)

        print('\nIteration: {} | Complexity: {} | Skeletons: {} | Skeleton Queue: {} | Disabled: {} | Evaluations: {} | '
              'Eager Calls: {} | Cost: {:.3f} | Search Time: {:.3f} | Sample Time: {:.3f} | Total Time: {:.3f}'.format(
            num_iterations, complexity_limit, len(skeleton_queue.skeletons), len(skeleton_queue), len(disabled),
            len(evaluations), eager_calls, store.best_cost, search_time, sample_time, store.elapsed_time()))
        optimistic_solve_fn = get_optimistic_solve_fn(goal_exp, domain, negative,
                                                      replan_actions=replan_actions, reachieve=use_skeletons,
                                                      max_cost=min(store.best_cost, constraints.max_cost),
                                                      max_effort=max_effort, effort_weight=effort_weight, **search_kwargs)
        # TODO: just set unit effort for each stream beforehand
        if (max_skeletons is None) or (len(skeleton_queue.skeletons) < max_skeletons):
            disabled_axioms = create_disabled_axioms(skeleton_queue) if has_optimizers else []
            if disabled_axioms:
                domain.axioms.extend(disabled_axioms)
            stream_plan, opt_plan, cost = iterative_plan_streams(evaluations, positive_externals,
                optimistic_solve_fn, complexity_limit, max_effort=max_effort)
            for axiom in disabled_axioms:
                domain.axioms.remove(axiom)
        else:
            stream_plan, opt_plan, cost = INFEASIBLE, INFEASIBLE, INF
        #stream_plan = replan_with_optimizers(evaluations, stream_plan, domain, externals) or stream_plan
        stream_plan = combine_optimizers(evaluations, stream_plan)
        #stream_plan = get_synthetic_stream_plan(stream_plan, # evaluations
        #                                       [s for s in synthesizers if not s.post_only])
        if reorder:
            # TODO: this blows up memory wise for long stream plans
            stream_plan = reorder_stream_plan(store, stream_plan)

        num_optimistic = sum(r.optimistic for r in stream_plan) if stream_plan else 0
        action_plan = opt_plan.action_plan if is_plan(opt_plan) else opt_plan
        print('Stream plan ({}, {}, {:.3f}): {}\nAction plan ({}, {:.3f}): {}'.format(
            get_length(stream_plan), num_optimistic, compute_plan_effort(stream_plan), stream_plan,
            get_length(action_plan), cost, str_from_plan(action_plan)))
        if is_plan(stream_plan) and visualize:
            log_plans(stream_plan, action_plan, num_iterations)
            create_visualizations(evaluations, stream_plan, num_iterations)
        search_time += elapsed_time(start_time)

        if (stream_plan is INFEASIBLE) and (not eager_instantiator) and (not skeleton_queue) and (not disabled):
            break
        start_time = time.time()
        if not is_plan(stream_plan):
            complexity_limit += complexity_step
            if not eager_disabled:
                reenable_disabled(evaluations, domain, disabled)

        #print(stream_plan_complexity(evaluations, stream_plan))
        if use_skeletons:
            #optimizer_plan = replan_with_optimizers(evaluations, stream_plan, domain, optimizers)
            optimizer_plan = None
            if optimizer_plan is not None:
                # TODO: post process a bound plan
                print('Optimizer plan ({}, {:.3f}): {}'.format(
                    get_length(optimizer_plan), compute_plan_effort(optimizer_plan), optimizer_plan))
                skeleton_queue.new_skeleton(optimizer_plan, opt_plan, cost)
            allocated_sample_time = (search_sample_ratio * search_time) - sample_time \
                if len(skeleton_queue.skeletons) <= max_skeletons else INF
            if not skeleton_queue.process(stream_plan, opt_plan, cost, complexity_limit, allocated_sample_time):
                break
        else:
            process_stream_plan(store, domain, disabled, stream_plan, opt_plan, cost,
                                bind=bind, max_failures=max_failures)
        sample_time += elapsed_time(start_time)

    write_stream_statistics(externals, verbose)
    return store.extract_solution()
