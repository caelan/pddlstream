from collections import defaultdict, namedtuple, deque
from functools import cmp_to_key
from heapq import heappush, heappop

from pddlstream.utils import INF
from pddlstream.visualization import get_partial_orders


def cmp(v1, v2):
    i1 = v1.instance.external.info
    i2 = v2.instance.external.info
    if ((i1.p_success <= i2.p_success) and (i1.overhead < i2.overhead)) or \
            ((i1.p_success < i2.p_success) and (i1.overhead <= i2.overhead)):
        return -1
    if (i1.p_success <= i2.p_success) and (i1.overhead <= i2.overhead):
        return -1
    if ((i2.p_success <= i1.p_success) and (i2.overhead < i1.overhead)) or \
            ((i2.p_success < i1.p_success) and (i2.overhead <= i1.overhead)):
        return +1
    # TODO: include context here as a weak constraint
    # TODO: actions as a weak constraint
    # TODO: works in the absence of parital orders
    # TODO: actions are extremely unlikely to work
    return 0


def get_stream_dag(stream_plan):
    orders = get_partial_orders(stream_plan)
    incoming_edges = defaultdict(set)
    outgoing_edges = defaultdict(set)
    for v1, v2 in orders:
        incoming_edges[v2].add(v1)
        outgoing_edges[v1].add(v2)
    return incoming_edges, outgoing_edges


def forward_topological_sort(stream_plan):
    if stream_plan is None:
        return None
    incoming_edges, outgoing_edges = get_stream_dag(stream_plan)
    # TODO: factor other streams depending on these (i.e. if eval enables something that is likely to fail)
    key = cmp_to_key(cmp)
    ordering = []
    queue = []
    for v in stream_plan:
        if not incoming_edges[v]:
            heappush(queue, (key(v), v))
    while queue:
        _, v1 = heappop(queue)
        ordering.append(v1)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1)
            if not incoming_edges[v2]:
                heappush(queue, (key(v2), v2))
    return ordering


def backward_topological_sort(stream_plan):
    # Works well because DP reduces to this when no choices
    if stream_plan is None:
        return None
    incoming_edges, outgoing_edges = get_stream_dag(stream_plan)
    queue = []
    reversed_ordering = []
    visited = set()
    for v in stream_plan:
        if not outgoing_edges[v]:
            info = v.instance.external.info
            key = (-info.p_success, -info.overhead)
            heappush(queue, (key, v))
    while queue:
        _, v1 = heappop(queue)
        reversed_ordering.append(v1)
        visited.add(v1)
        for v2 in incoming_edges[v1]:
            if outgoing_edges[v2] <= visited:
                info = v2.instance.external.info
                key = (-info.p_success, -info.overhead)
                # TODO: could factor in outgoing_edges[v2]
                heappush(queue, (key, v2))
    return list(reversed(reversed_ordering))


def compute_expected_cost(stream_plan):
    if stream_plan is None:
        return INF
    expected_cost = 0
    for result in reversed(stream_plan):
        info = result.instance.external.info
        expected_cost = info.overhead + info.p_success * expected_cost
    return expected_cost


Subproblem = namedtuple('Subproblem', ['cost', 'head', 'subset'])


def dynamic_programming(stream_plan):
    # 2^N rather than N!
    if stream_plan is None:
        return None

    incoming_edges, outgoing_edges = get_stream_dag(stream_plan)
    def valid_combine(v, subset):
        return (v not in subset) and (outgoing_edges[v] <= subset)
        #return (v not in subset) and not (incoming_edges[v] & subset) # These are equivalent

    # Used to prune dominated choices. Equality is pruned as well.
    key_fn = cmp_to_key(cmp)
    layers = []
    layer = []
    for v in sorted(stream_plan, key=key_fn):
        if not layer or (key_fn(layer[-1]) == key_fn(v)):
            layer.append(v)
        else:
            layers.append(layer)
            layer = [v]
    layers.append(layer)

    subset = frozenset()
    queue = deque([subset]) # Acyclic because subsets
    expected_cost = {subset: Subproblem(0, None, None)}
    iterations = 0
    while queue:
        iterations += 1
        subset = queue.popleft()
        for layer in reversed(layers):
            valid = filter(lambda v: valid_combine(v, subset), layer)
            for v in valid:
                new_subset = frozenset([v]) | subset
                info = v.instance.external.info
                new_cost = info.overhead + info.p_success*expected_cost[subset].cost # Add new element to front
                subproblem = Subproblem(new_cost, v, subset)
                if new_subset not in expected_cost:
                    queue.append(new_subset)
                    expected_cost[new_subset] = subproblem
                elif new_cost < expected_cost[new_subset].cost:
                    expected_cost[new_subset] = subproblem
            if valid:
                break

    ordering = []
    subset = frozenset(stream_plan)
    while True:
        subproblem = expected_cost[subset]
        if subproblem.head is None:
            break
        ordering.append(subproblem.head)
        subset = subproblem.subset
    return ordering