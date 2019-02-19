from __future__ import print_function

from pddlstream.algorithms.algorithm import parse_stream_pddl, evaluations_from_init
from pddlstream.algorithms.common import SolutionStore
from pddlstream.algorithms.downward import make_domain, make_predicate, add_predicate, make_axiom
from pddlstream.algorithms.recover_optimizers import retrace_instantiation, combine_optimizers
from pddlstream.algorithms.reorder import reorder_stream_plan
from pddlstream.algorithms.scheduling.postprocess import reschedule_stream_plan
#from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.algorithms.skeleton2 import SkeletonQueue
from pddlstream.algorithms.scheduling.utils import partition_external_plan
from pddlstream.language.constants import is_parameter, get_length, partition_facts
from pddlstream.language.conversion import revert_solution, \
    evaluation_from_fact, replace_expression, get_prefix, get_args
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.language.optimizer import UNSATISFIABLE
from pddlstream.language.stream import Stream
from pddlstream.language.function import Function, Predicate
from pddlstream.language.statistics import write_stream_statistics, compute_plan_effort
from pddlstream.utils import INF, get_mapping, elapsed_time, str_from_object, safe_zip
from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.utils import get_connected_components, grow_component, adjacent_from_edges, incoming_from_edges

import time

BIND_ACTION = 'bindings'

def obj_from_existential_expression(parent): # obj_from_value_expression
    return replace_expression(parent, lambda o: OptimisticObject
                              .from_opt(o, o) if is_parameter(o) else Object.from_value(o))


def create_domain(goal_facts):
    domain = make_domain()
    for fact in goal_facts: # TODO: consider removing this annoying check
        name = get_prefix(fact)
        parameters = ['?x{}'.format(i) for i in range(len(get_args(fact)))]
        add_predicate(domain, make_predicate(name, parameters))
    return domain

def plan_functions(functions, externals):
    external_from_function = {}
    for external in filter(lambda e: isinstance(e, Function), externals):
        assert external.function not in external_from_function
        external_from_function[external.function] = external
    function_plan = set()
    for term in functions:
        if get_prefix(term) not in external_from_function:
            raise ValueError('{} is not implemented'.format(get_prefix(term)))
        external = external_from_function[get_prefix(term)]
        instance = external.get_instance(get_args(term))
        [result] = instance.next_optimistic()
        function_plan.add(result)
    print('Function plan:', str_from_object(function_plan))
    return function_plan

def get_parameters(goal_facts):
    return {o for f in goal_facts for o in get_args(f) if isinstance(o, OptimisticObject)}

def extract_streams(evaluations, externals, goal_facts):
    streams = list(filter(lambda e: isinstance(e, Stream), externals))
    free_parameters = get_parameters(goal_facts)
    visited_facts = set()
    stream_results = []
    for fact in goal_facts:
        # TODO: prune results that already exceed effort limit
        retrace_instantiation(fact, streams, evaluations, free_parameters, visited_facts, stream_results)
    print('Streams:', stream_results)
    # TODO: express some of this pruning using effort (e.g. unlikely to sample bound value)
    return stream_results

def get_optimistic_cost(function_plan):
    return sum([0.] + [result.value for result in function_plan
                       if type(result.external) == Function])

def bindings_from_plan(plan_skeleton, action_plan):
    if action_plan is None:
        return None
    bindings = {}
    for (name1, args1), (name2, args2) in safe_zip(plan_skeleton, action_plan):
        assert name1 == name2
        parameter_names = [o.value for o in args1]
        bindings.update(get_mapping(parameter_names, args2))
    return bindings

##################################################

def create_disable_axiom(external_plan):
    # TODO: express constraint mutexes upfront
    stream_plan, _  = partition_external_plan(external_plan)
    #print(stream_plan)
    parameters = []
    preconditions = [result.stream_fact for result in stream_plan]
    derived = (UNSATISFIABLE,)
    # TODO: add parameters in the event that the same skeleton can be blocked twice
    return make_axiom(parameters, preconditions, derived)

def compute_failed_indices(skeleton):
    failed_indices = set()
    for binding in skeleton.root.post_order():
        result = binding.next_result
        if (result is not None) and result.instance.num_calls and (not result.instance.successes):
            failed_indices.add(binding.stream_indices[0])
            #assert not binding.children
    return sorted(failed_indices)

def current_failed_cluster(binding):
    failed_index = binding.stream_indices[0]
    assert 1 <= binding.stream_attempts[0]
    failed_result = binding.skeleton.stream_plan[failed_index]
    successful_results = [result for i, result in enumerate(binding.skeleton.stream_plan)
                          if i not in binding.stream_indices]
    stream_plan = successful_results + [failed_result]
    partial_orders = get_partial_orders(stream_plan)
    # All connected components
    #return get_connected_components(stream_plan, partial_orders)
    # Only the failed connected component
    return [grow_component([failed_result], adjacent_from_edges(partial_orders))]

def current_failure_contributors(binding):
    # Alternatively, find unsuccessful streams in cluster and add ancestors
    failed_index = binding.stream_indices[0]
    assert 1 <= binding.stream_attempts[0]
    failed_result = binding.skeleton.stream_plan[failed_index]
    failed_indices = compute_failed_indices(binding.skeleton)  # Use last index?
    partial_orders = get_partial_orders(binding.skeleton.stream_plan)
    incoming = incoming_from_edges(partial_orders)
    failed_ancestors = grow_component([failed_result], incoming)
    for index in reversed(failed_indices):
        if index == failed_index:
            continue
        result = binding.skeleton.stream_plan[index]
        ancestors = grow_component([result], incoming)
        if ancestors & failed_ancestors:
            failed_ancestors.update(ancestors)
    return [failed_ancestors]

def extract_disabled_clusters(queue):
    clusters = set()
    for skeleton in queue.skeletons:
        # TODO: include costs within clustering?
        # What is goal is to be below a cost threshold?
        # In satisfaction, no need because costs are fixed
        # Make stream_facts for externals to prevent use of the same ones
        # This ordering is why it's better to put likely to fail first
        # Branch on the different possible binding outcomes
        # TODO: consider a nonlinear version of this that evaluates out of order
        # Need extra sampling effort to identify infeasible subsets
        # Treat unevaluated optimistically, as in always satisfiable
        # Need to keep streams with outputs to connect if downstream is infeasible
        # TODO: prune streams that always have at least one success
        # TODO: CSP identification of irreducible unsatisfiable subsets
        # TODO: take into consideration if a stream is enumerated to mark as a hard failure
        # Decompose down optimizers

        #cluster_plans = [skeleton.stream_plan]
        partial_orders = get_partial_orders(skeleton.stream_plan)
        cluster_plans = get_connected_components(skeleton.stream_plan, partial_orders)
        binding = skeleton.best_binding
        if not binding.is_bound():
            # TODO: block if cost sensitive to possibly get cheaper solutions
            #cluster_plans = current_failed_cluster(binding)
            cluster_plans = current_failure_contributors(binding)
        for cluster_plan in cluster_plans:
            clusters.add(frozenset(cluster_plan))
    return clusters

def are_domainated(clusters1, clusters2):
    return all(any(c1 <= c2 for c2 in clusters2) for c1 in clusters1)

##################################################

def constraint_satisfaction(stream_pddl, stream_map, init, terms, stream_info={},
                            costs=True, max_cost=INF, success_cost=INF, max_time=INF, max_effort=INF,
                            max_skeletons=INF, search_sample_ratio=1, verbose=True, **search_args):
    # Approaches
    # 1) Existential quantification of bindings in goal conditions
    # 2) Backtrack useful streams and then schedule. Create arbitrary outputs for not mentioned.
    # 3) Construct all useful streams and then associate outputs with bindings
    #    Useful stream must satisfy at least one fact. How should these assignments be propagated though?
    #    Make an action that maps each stream result to unbound values?
    # TODO: include functions again for cost-sensitive satisfaction
    # TODO: convert init into streams to bind certain facts
    # TODO: investigate constraint satisfaction techniques for binding instead
    # TODO: could also instantiate all possible free parameters even if not useful
    # TODO: effort that is a function of the number of output parameters (degrees of freedom)
    # TODO: use a CSP solver instead of a planner internally
    # TODO: max_iterations?
    if not terms:
        return {}, 0, init
    constraints, negated, functions = partition_facts(set(map(obj_from_existential_expression, terms)))
    if not costs:
        functions = []
    evaluations = evaluations_from_init(init)
    goal_facts = set(filter(lambda f: evaluation_from_fact(f) not in evaluations, constraints))
    free_parameters = sorted(get_parameters(goal_facts))

    externals = parse_stream_pddl(stream_pddl, stream_map, stream_info)
    stream_results = extract_streams(evaluations, externals, goal_facts)
    function_plan = plan_functions(negated + functions, externals)
    plan_skeleton = [(BIND_ACTION, free_parameters)]
    cost = get_optimistic_cost(function_plan)
    if max_cost < cost:
        return None, INF, init
    # TODO: detect connected components
    # TODO: eagerly evaluate fully bound constraints

    # TODO: consider other results if this fails
    domain = create_domain(goal_facts)
    init_evaluations = evaluations.copy()
    store = SolutionStore(evaluations, max_time=max_time, success_cost=success_cost, verbose=verbose)
    queue = SkeletonQueue(store, domain, disable=False)
    num_iterations = search_time = sample_time = 0
    last_clusters = set()
    last_success = True
    while not store.is_terminated():
        num_iterations += 1
        start_time = time.time()
        print('\nIteration: {} | Skeletons: {} | Skeleton Queue: {} | Evaluations: {} | '
              'Cost: {:.3f} | Search Time: {:.3f} | Sample Time: {:.3f} | Total Time: {:.3f}'.format(
            num_iterations, len(queue.skeletons), len(queue),
            len(evaluations), store.best_cost, search_time, sample_time, store.elapsed_time()))
        external_plan = None
        if len(queue.skeletons) < max_skeletons:
            clusters = extract_disabled_clusters(queue)
            domain.axioms[:] = [create_disable_axiom(cluster) for cluster in clusters]
            dominated = are_domainated(last_clusters, clusters)
            last_clusters = clusters
            planner = 'ff-astar' # TODO: toggle within reschedule_stream_plan
            #if last_success or not dominated: # Could also keep a history of results
            stream_plan = reschedule_stream_plan(init_evaluations, goal_facts, domain, stream_results,
                                                 unique_binding=True, unsatisfiable=True,
                                                 max_effort=max_effort, planner=planner, **search_args)
            if stream_plan is not None:
                external_plan = reorder_stream_plan(combine_optimizers(
                    init_evaluations, stream_plan + list(function_plan)))
        print('Stream plan ({}, {:.3f}): {}'.format(
            get_length(external_plan), compute_plan_effort(external_plan), external_plan))
        last_success = (external_plan is not None)
        search_time += elapsed_time(start_time)

        # Once a constraint added for a skeleton, it should only be relaxed
        start_time = time.time()
        if last_success: # Only works if create_disable_axioms never changes
            allocated_sample_time = (search_sample_ratio * search_time) - sample_time
        else:
            allocated_sample_time = INF
        queue.process(external_plan, plan_skeleton, cost=cost,
                      complexity_limit=INF,  max_time=allocated_sample_time)
        sample_time += elapsed_time(start_time)
        if not last_success and not queue:
            break
        # TODO: exhaustively compute all plan skeletons and add to queue within the focused algorithm

    write_stream_statistics(externals, verbose)
    action_plan, cost, facts = revert_solution(store.best_plan, store.best_cost, evaluations)
    bindings = bindings_from_plan(plan_skeleton, action_plan)
    return bindings, cost, facts
