import time

from collections import namedtuple, deque, Counter
from itertools import combinations

from pddlstream.language.constants import is_plan
from pddlstream.language.external import Result
from pddlstream.language.statistics import Stats, Performance, EPSILON
from pddlstream.language.stream import StreamResult
from pddlstream.utils import INF, neighbors_from_orders, topological_sort, get_connected_components, \
    sample_topological_sort, is_acyclic, layer_sort, Score, safe_zip


def get_output_objects(result):
    if isinstance(result, StreamResult):
        return result.output_objects
    return tuple()

def get_object_orders(stream_plan):
    # TODO: check that only one result per output object
    partial_orders = set()
    for i, stream1 in enumerate(stream_plan):
        for stream2 in stream_plan[i+1:]:
            if set(get_output_objects(stream1)) & stream2.instance.get_all_input_objects():
                partial_orders.add((stream1, stream2))
    return partial_orders

def get_initial_orders(init_facts, stream_plan):
    return {(fact, stream) for stream in stream_plan for fact in stream.get_domain() if fact in init_facts}

def get_fact_orders(stream_plan, init_facts=set()):
    # TODO: explicitly recover this from plan_streams
    # TODO: init_facts isn't used in practice
    achieved_facts = set(init_facts)
    partial_orders = set()
    for i, stream1 in enumerate(stream_plan):
        new_facts = set(stream1.get_certified()) - achieved_facts
        for stream2 in stream_plan[i+1:]: # Prevents circular
            if new_facts & set(stream2.get_domain()):
                partial_orders.add((stream1, stream2))
        achieved_facts.update(new_facts)
    return partial_orders

def get_partial_orders(stream_plan, use_facts=True, **kwargs):
    partial_orders = get_object_orders(stream_plan)
    if use_facts:
        partial_orders.update(get_fact_orders(stream_plan, **kwargs))
    assert is_acyclic(stream_plan, partial_orders)
    return partial_orders

##################################################

def get_stream_plan_components(external_plan, **kwargs):
    partial_orders = get_partial_orders(external_plan, **kwargs)
    return get_connected_components(external_plan, partial_orders)

def dump_components(stream_plan):
    for i, result in enumerate(stream_plan):
        components = get_stream_plan_components(stream_plan[:i+1])
        print(i, len(components), components)

##################################################

def get_future_p_successes(stream_plan):
    # TODO: should I use this instead of p_success in some places?
    # TODO: learn this instead by estimating conditional probabilities of certain sequence
    # TODO: propagate stats_heuristic
    orders = get_partial_orders(stream_plan)
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)
    descendants_map = {}
    for s1 in reversed(stream_plan):
        descendants_map[s1] = s1.instance.get_p_success()
        for s2 in outgoing_edges[s1]:
            descendants_map[s1] *= descendants_map[s2]
    return descendants_map

def compute_expected_cost(stream_plan, stats_fn=Performance.get_statistics):
    if not is_plan(stream_plan):
        return INF
    expected_cost = 0.
    for result in reversed(stream_plan):
        p_success, overhead = stats_fn(result)
        expected_cost = overhead + p_success * expected_cost
    return expected_cost

##################################################

Subproblem = namedtuple('Subproblem', ['cost', 'head', 'subset'])

def compute_pruning_orders(results, stats_fn=Performance.get_statistics, tiebreaker_fn=lambda v: None):
    # TODO: reason about pairs that don't have a (transitive) ordering
    # TODO: partial orders make this heuristic not optimal
    # TODO: use result.external.name to cluster?
    dominates = lambda v1, v2: all(s1 <= s2 for s1, s2 in safe_zip(stats_fn(v1), stats_fn(v2))) \
                               and tiebreaker_fn(v1) <= tiebreaker_fn(v2)
    effort_orders = set()
    for v1, v2 in combinations(results, r=2): # randomize
        if dominates(v1, v2):
            effort_orders.add((v1, v2))  # Includes equality
        elif dominates(v2, v1):
            effort_orders.add((v2, v1))
    return effort_orders

def dynamic_programming(store, vertices, valid_head_fn, stats_fn=Performance.get_statistics, prune=True, greedy=False, **kwargs):
    # TODO: include context here as a weak constraint
    # TODO: works in the absence of partial orders
    # TODO: can also more manually reorder
    # 2^N rather than N!
    start_time = time.time()
    effort_orders = set() # 1 cheaper than 2
    if prune:
        effort_orders.update(compute_pruning_orders(vertices, stats_fn=stats_fn, **kwargs))
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
    #print('Streams: {} | Expected cost: {:.3f} | Time: {:.3f}'.format(
    #    len(ordering), compute_expected_cost(ordering, stats_fn=stats_fn), elapsed_time(start_time)))
    return ordering

##################################################

def dummy_reorder_stream_plan(stream_plan, **kwargs):
    return stream_plan

def random_reorder_stream_plan(stream_plan, **kwargs):
    if not stream_plan:
        return stream_plan
    return sample_topological_sort(stream_plan, get_partial_orders(stream_plan))

def greedy_reorder_stream_plan(stream_plan, **kwargs):
    if not stream_plan:
        return stream_plan
    return topological_sort(stream_plan, get_partial_orders(stream_plan),
                            priority_fn=lambda s: s.get_statistics().overhead)

##################################################

def dump_layers(distances):
    streams_from_layer = {}
    for stream, layer in distances.items():
        streams_from_layer.setdefault(layer, []).append(stream)
    for layer, streams in streams_from_layer.items():
        print(layer, sorted(streams, key=Result.stats_heuristic, reverse=True))
    return streams_from_layer

def compute_distances(stream_plan):
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
    max_distance = max([0] + list(distances.values()))
    for stream in stream_plan:
        if stream not in distances:
            distances[stream] = min([max_distance] + [distances[s] - 1 for s in out_stream_orders[stream]])
    #dump_layers(distances)
    return distances

def layer_reorder_stream_plan(stream_plan, **kwargs):
    if not stream_plan:
        return stream_plan
    stream_orders = get_partial_orders(stream_plan)
    reversed_orders = {(s2, s1) for s1, s2 in stream_orders}
    distances = compute_distances(stream_plan)
    priority_fn = lambda s: Score(not s.external.has_outputs, distances[s], -s.stats_heuristic())
    reverse_order = topological_sort(stream_plan, reversed_orders, priority_fn=priority_fn)
    return reverse_order[::-1]

def compute_statistics(stream_plan, bias=True):
    stats_from_stream = {result: result.external.get_statistics() for result in stream_plan}
    if not bias:
        return stats_from_stream
    distances = compute_distances(stream_plan)
    max_distance = max(distances.values())
    for result in stream_plan:
        p_success, overhead = stats_from_stream[result]
        if result.external.has_outputs:
            # TODO: is_function, number of free inputs, etc.
            # TODO: decrease p_success if fewer free inputs (or input streams)
            # TODO: dynamic_programming seems to automatically order streams with fewer free ahead anyways
            overhead += EPSILON*(max_distance - distances[result] + 1)
        else:
            p_success *= EPSILON
        stats_from_stream[result] = Stats(p_success, overhead)
    return stats_from_stream

##################################################

def optimal_reorder_stream_plan(store, stream_plan, stats_from_stream=None, **kwargs):
    if not stream_plan:
        return stream_plan
    if stats_from_stream is None:
        stats_from_stream = compute_statistics(stream_plan)

    # TODO: use the negative output (or overhead) as a bound
    indices = range(len(stream_plan))
    index_from_stream = dict(zip(stream_plan, indices))
    stream_orders = get_partial_orders(stream_plan)
    stream_orders = {(index_from_stream[s1], index_from_stream[s2]) for s1, s2 in stream_orders}
    #nodes = stream_plan
    nodes = indices # TODO: are indices actually much faster?

    in_stream_orders, out_stream_orders = neighbors_from_orders(stream_orders)
    valid_combine = lambda v, subset: out_stream_orders[v] <= subset
    #valid_combine = lambda v, subset: in_stream_orders[v] & subset

    # TODO: these are special because they don't enable any downstream access to another stream
    #sources = {stream_plan[index] for index in indices if not in_stream_orders[index]}
    #sinks = {stream_plan[index] for index in indices if not out_stream_orders[index]} # Contains collision checks
    #print(dijkstra(sources, get_partial_orders(stream_plan)))

    stats_fn = lambda idx: stats_from_stream[stream_plan[idx]]
    #tiebreaker_fn = lambda *args: 0
    #tiebreaker_fn = lambda *args: random.random() # TODO: introduces cycles
    tiebreaker_fn = lambda idx: stream_plan[idx].stats_heuristic()
    ordering = dynamic_programming(store, nodes, valid_combine, stats_fn=stats_fn, tiebreaker_fn=tiebreaker_fn, **kwargs)
    #import gc
    #gc.collect()
    return [stream_plan[index] for index in ordering]

##################################################

def reorder_stream_plan(store, stream_plan, algorithm=None, **kwargs):
    if not stream_plan:
        return stream_plan

    stats_from_stream = compute_statistics(stream_plan)
    stats = Counter(stats_from_stream.values())
    if algorithm is None:
        algorithm = 'layer' if len(stats) <= 1 else 'optimal'

    if algorithm == 'dummy':
        return dummy_reorder_stream_plan(stream_plan, **kwargs)
    if algorithm == 'random':
        return random_reorder_stream_plan(stream_plan, **kwargs)
    if algorithm == 'greedy':
        return greedy_reorder_stream_plan(stream_plan, **kwargs)
    if algorithm == 'layer':
        #print('Heuristic reordering:', stats)
        return layer_reorder_stream_plan(stream_plan, **kwargs)
    if algorithm == 'optimal':
        #print('Optimal reordering:', stats)
        return optimal_reorder_stream_plan(store, stream_plan, stats_from_stream, **kwargs)
    raise NotImplementedError(algorithm)
