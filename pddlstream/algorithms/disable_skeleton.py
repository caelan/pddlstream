from pddlstream.algorithms.downward import make_axiom
from pddlstream.algorithms.reorder import get_partial_orders, get_stream_plan_components
from pddlstream.algorithms.scheduling.utils import partition_external_plan
from pddlstream.language.optimizer import UNSATISFIABLE
from pddlstream.utils import grow_component, adjacent_from_edges, incoming_from_edges


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
        result = binding.result
        if (result is not None) and result.instance.num_calls and (not result.instance.successes):
            failed_indices.add(binding.index)
            #assert not binding.children
    return sorted(failed_indices)


def current_failed_cluster(binding):
    assert 1 <= binding.attempts
    failed_result = binding.skeleton.stream_plan[binding.index]
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
    assert (1 <= binding.attempts) or binding.is_dominated()
    failed_result = binding.skeleton.stream_plan[binding.index]
    failed_indices = compute_failed_indices(binding.skeleton)  # Use last index?
    partial_orders = get_partial_orders(binding.skeleton.stream_plan)
    incoming = incoming_from_edges(partial_orders)
    failed_ancestors = grow_component([failed_result], incoming)
    for index in reversed(failed_indices):
        if index == binding.index:
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
        cluster_plans = get_stream_plan_components(skeleton.stream_plan)
        binding = skeleton.best_binding
        if not binding.is_bound():
            # TODO: block if cost sensitive to possibly get cheaper solutions
            #cluster_plans = current_failed_cluster(binding)
            cluster_plans = current_failure_contributors(binding)
        for cluster_plan in cluster_plans:
            clusters.add(frozenset(cluster_plan))
    return clusters

def create_disabled_axioms(queue, last_clusters=None):
    clusters = extract_disabled_clusters(queue)
    return [create_disable_axiom(cluster) for cluster in clusters]
