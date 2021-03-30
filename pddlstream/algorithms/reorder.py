from collections import namedtuple, deque
from itertools import combinations

from pddlstream.language.constants import is_plan
from pddlstream.language.external import Result
from pddlstream.language.stream import StreamResult
from pddlstream.utils import INF, implies, neighbors_from_orders, topological_sort, get_connected_components, \
    sample_topological_sort, dijkstra, is_acyclic, layer_sort

# TODO: should I use the product of all future probabilities?

def get_partial_orders(stream_plan, init_facts=set()):
    achieved_facts = set(init_facts) # TODO: achieved objects
    partial_orders = set()
    for i, stream1 in enumerate(stream_plan):
        new_facts = set(stream1.get_certified()) - achieved_facts
        achieved_facts.update(new_facts)
        for stream2 in stream_plan[i+1:]: # Prevents circular
            if new_facts & set(stream2.instance.get_domain()):
                partial_orders.add((stream1, stream2))
            if isinstance(stream1, StreamResult) and \
                    (set(stream1.output_objects) & stream2.instance.get_objects()):
                partial_orders.add((stream1, stream2))
    assert is_acyclic(stream_plan, partial_orders)
    return partial_orders

def get_stream_plan_components(external_plan):
    partial_orders = get_partial_orders(external_plan)
    return get_connected_components(external_plan, partial_orders)

##################################################

Stats = namedtuple('Stats', ['p_success', 'overhead'])

# Extract streams required to do one action
# Compute streams that strongly depend on these. Evaluate these.
# Execute the full prefix of the plan
# Make the first action cheaper if uses something that doesn't need to re-expand
# How to do this with shared objects?
# Just do the same thing but make the cost 1 if a shared object

def get_future_p_successes(stream_plan):
    # TODO: should I use this instead of p_success in some places?
    # TODO: learn this instead. Can estimate conditional probabilities of certain sequences
    orders = get_partial_orders(stream_plan)
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)
    descendants_map = {}
    for s1 in reversed(stream_plan):
        descendants_map[s1] = s1.instance.get_p_success()
        for s2 in outgoing_edges[s1]:
            descendants_map[s1] *= descendants_map[s2]
    return descendants_map

def get_stream_stats(result, negate=False):
    # TODO: can just do on the infos themselves
    sign = -1 if negate else +1
    return Stats(
        #p_success=result.instance.get_p_success(),
        #overhead=sign*result.instance.get_overhead(),
        p_success=result.instance.external.get_p_success(),
        overhead=sign*result.instance.external.get_overhead(),
    )

def compute_expected_cost(stream_plan, stats_fn=get_stream_stats):
    # TODO: prioritize cost functions as they can prune when we have a better plan
    if not is_plan(stream_plan):
        return INF
    expected_cost = 0
    for result in reversed(stream_plan):
        p_success, overhead = stats_fn(result)
        expected_cost = overhead + p_success * expected_cost
    return expected_cost

##################################################

Subproblem = namedtuple('Subproblem', ['cost', 'head', 'subset'])

def compute_pruning_orders(vertices, stats_fn):
    # TODO: reason about pairs that don't have a (transitive) ordering
    dominates = lambda v1, v2: all(s1 <= s2 for s1, s2 in zip(stats_fn(v1), stats_fn(v2)))
    effort_orders = set()
    for v1, v2 in combinations(vertices, r=2):
        if dominates(v1, v2):
            effort_orders.add((v1, v2))  # Includes equality
        elif dominates(v2, v1):
            effort_orders.add((v2, v1))
    return effort_orders

def dynamic_programming(store, vertices, valid_head_fn, stats_fn, prune=True, greedy=False):
    # TODO: include context here as a weak constraint
    # TODO: works in the absence of partial orders
    # TODO: can also more manually reorder
    # 2^N rather than N!
    effort_orders = set() # 1 cheaper than 2
    if prune:
        effort_orders.update(compute_pruning_orders(vertices, stats_fn))
    _, out_priority_orders = neighbors_from_orders(effort_orders) # more expensive
    priority_ordering = topological_sort(vertices, effort_orders)[::-1] # most expensive to cheapest
    # TODO: can break ties with index on action plan to prioritize doing the temporally first things

    # TODO: could the greedy strategy lead to premature choices
    # TODO: this starts to blow up - group together similar streams (e.g. collision streams) to decrease size
    # TODO: key grouping concern are partial orders and ensuring feasibility (isomorphism)
    # TODO: flood-fill cheapest as soon as something that has no future dependencies has been found
    # TODO: do the forward version to take advantage of sink vertices
    subset = frozenset()
    queue = deque([subset]) # Acyclic because subsets
    subproblems = {subset: Subproblem(cost=0, head=None, subset=None)}
    while queue: # searches backward from last to first
        if store.is_terminated():
            return vertices
        subset = queue.popleft() # TODO: greedy/weighted A* version of this (heuristic is next cheapest stream)
        applied = set()
        # TODO: roll-out more than one step to cut the horizon
        # TODO: compute a heuristic that's the best case affordances from subsequent streams
        for v in priority_ordering: # most expensive first
            if greedy and applied:
                break
            if (v not in subset) and valid_head_fn(v, subset) and not (out_priority_orders[v] & applied):
                applied.add(v)
                new_subset = frozenset([v]) | subset
                p_success, overhead = stats_fn(v)
                new_cost = overhead + p_success*subproblems[subset].cost
                subproblem = Subproblem(cost=new_cost, head=v, subset=subset)  # Adds new element to the front
                if new_subset not in subproblems:
                    queue.append(new_subset)
                    subproblems[new_subset] = subproblem
                elif new_cost < subproblems[new_subset].cost:
                    subproblems[new_subset] = subproblem

    ordering = []
    subset = frozenset(vertices)
    while True:
        if subset not in subproblems:
            print(vertices)
            # TODO: some sort of bug where the problem isn't solved?
        subproblem = subproblems[subset]
        if subproblem.head is None:
            break
        ordering.append(subproblem.head)
        subset = subproblem.subset
    return ordering

##################################################

# TODO: replan flag to toggle different behaviors

def dummy_reorder_stream_plan(store, stream_plan, **kwargs):
    return stream_plan

def random_reorder_stream_plan(store, stream_plan, **kwargs):
    if not is_plan(stream_plan):
        return stream_plan
    return sample_topological_sort(stream_plan, get_partial_orders(stream_plan))

def greedy_reorder_stream_plan(store, stream_plan, **kwargs):
    if not is_plan(stream_plan):
        return stream_plan
    return topological_sort(stream_plan, get_partial_orders(stream_plan),
                            priority_fn=lambda stream: get_stream_stats(stream).overhead)

def dump_layers(distances):
    streams_from_layer = {}
    for stream, layer in distances.items():
        streams_from_layer.setdefault(layer, []).append(stream)
    for layer, streams in streams_from_layer.items():
        print(layer, sorted(streams, key=lambda s: s.external.tiebreaker, reverse=True))
    return streams_from_layer

def layer_reorder_stream_plan(store, stream_plan, **kwargs):
    if not is_plan(stream_plan):
        return stream_plan
    stream_orders = get_partial_orders(stream_plan)
    reversed_orders = {(s2, s1) for s1, s2 in stream_orders}
    in_stream_orders, out_stream_orders = neighbors_from_orders(reversed_orders)
    sources = {stream for stream in stream_plan if not in_stream_orders[stream]} # In the reversed DAG
    output_sources = {stream for stream in sources if stream.external.has_outputs}
    test_sources = sources - output_sources
    #visited = dijkstra(output_sources, reversed_orders)
    #distances = {stream: node.g for stream, node in visited.items()}
    distances = layer_sort(set(stream_plan) - test_sources, reversed_orders)

    # TODO: take into account argument overlap
    for stream in stream_plan:
        if stream not in distances:
            distances[stream] = min([distances[s] - 1 for s in out_stream_orders[stream]], default=INF)

    #priority_fn = lambda s: distances[s] if s in distances else INF
    #priority_fn = lambda s: s.external.tiebreaker # Need to reverse
    sorted_streams = sorted(stream_plan, key=lambda s: s.external.tiebreaker, reverse=True)
    #priority_fn = sorted_streams.index
    priority_fn = lambda s: (not s.external.has_outputs, distances[s], sorted_streams.index(s))

    #dump_layers(distances)
    reverse_order = topological_sort(stream_plan, reversed_orders, priority_fn=priority_fn)
    return reverse_order[::-1]

def optimal_reorder_stream_plan(store, stream_plan, **kwargs):
    if not is_plan(stream_plan):
        return stream_plan
    # TODO: use the negative output (or overhead) as a bound
    indices = range(len(stream_plan))
    index_from_stream = dict(zip(stream_plan, indices))
    stream_orders = get_partial_orders(stream_plan)
    stream_orders = {(index_from_stream[s1], index_from_stream[s2]) for s1, s2 in stream_orders}
    #nodes = stream_plan
    nodes = indices

    in_stream_orders, out_stream_orders = neighbors_from_orders(stream_orders)
    valid_combine = lambda v, subset: out_stream_orders[v] <= subset
    #valid_combine = lambda v, subset: in_stream_orders[v] & subset

    # TODO: these are special because they don't enable any downstream access to another stream
    #sources = {stream_plan[index] for index in indices if not in_stream_orders[index]}
    #sinks = {stream_plan[index] for index in indices if not out_stream_orders[index]} # Contains collision checks
    #print(dijkstra(sources, get_partial_orders(stream_plan)))

    #stats_fn = get_stream_stats
    stats_fn = lambda idx: get_stream_stats(stream_plan[idx])
    ordering = dynamic_programming(store, nodes, valid_combine, stats_fn, **kwargs)
    #import gc
    #gc.collect()
    return [stream_plan[index] for index in ordering]

# TODO: toggle based on p_success and overhead
reorder_stream_plan = layer_reorder_stream_plan # optimal_reorder_stream_plan