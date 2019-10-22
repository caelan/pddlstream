from collections import namedtuple, deque

from pddlstream.language.constants import is_plan
from pddlstream.language.external import Result
from pddlstream.language.stream import StreamResult
from pddlstream.utils import INF, implies, neighbors_from_orders, topological_sort, get_connected_components

import gc

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
    return partial_orders

def get_stream_plan_components(external_plan):
    partial_orders = get_partial_orders(external_plan)
    return get_connected_components(external_plan, partial_orders)

##################################################

# Extract streams required to do one action
# Compute streams that strongly depend on these. Evaluate these.
# Execute the full prefix of the plan
# Make the first action cheaper if uses something that doesn't need to rexpand
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

# def get_all_descendants(stream_plan):
#     orders = get_partial_orders(stream_plan)
#     incoming_edges, outgoing_edges = neighbors_from_orders(orders)
#     descendants_map = {}
#     for s1 in reversed(stream_plan):
#         descendants_map[s1] = set(outgoing_edges[s1])
#         for s2 in outgoing_edges[s1]:
#             descendants_map[s1].update(descendants_map[s2])
#     return descendants_map

# def get_ancestors(stream_result, stream_plan):
#     orders = get_partial_orders(stream_plan)
#     incoming_edges, _ = neighbors_from_orders(orders)
#     ancestors = {stream_result}
#     queue = deque([stream_result])
#     while queue:
#         v1 = queue.popleft()
#         for v2 in incoming_edges[v1]:
#             if v2 not in ancestors:
#                 ancestors.add(v2)
#                 queue.append(v1)
#     return ancestors

##################################################

def get_stream_stats(result):
    #return result.instance.get_p_success(), result.instance.get_overhead()
    return result.instance.external.get_p_success(), result.instance.external.get_overhead()

def compute_expected_cost(stream_plan, stats_fn=get_stream_stats):
    # TODO: prioritize cost functions as they can prune when we have a better plan
    if not is_plan(stream_plan):
        return INF
    expected_cost = 0
    for result in reversed(stream_plan):
        p_success, overhead = stats_fn(result)
        expected_cost = overhead + p_success * expected_cost
    return expected_cost

# TODO: include context here as a weak constraint
# TODO: actions as a weak constraint
# TODO: works in the absence of partial orders
# TODO: actions are extremely unlikely to work
# TODO: can give actions extreme priority
# TODO: can also more manually reorder

Subproblem = namedtuple('Subproblem', ['cost', 'head', 'subset'])

def dynamic_programming(store, vertices, valid_head_fn, stats_fn, prune=True, greedy=False):
    # 2^N rather than N!
    # TODO: can just do on the infos themselves
    dominates = lambda v1, v2: all(s1 <= s2 for s1, s2 in zip(stats_fn(v1), stats_fn(v2)))
    effort_orders = set()
    if prune:
        for i, v1 in enumerate(vertices):
            for v2 in vertices[i+1:]:
                if dominates(v1, v2):
                    effort_orders.add((v1, v2)) # Includes equality
                elif dominates(v2, v1):
                    effort_orders.add((v2, v1))
    _, out_priority_orders = neighbors_from_orders(effort_orders)
    priority_ordering = topological_sort(vertices, effort_orders)[::-1]

    # TODO: could the greedy strategy lead to premature choices
    # TODO: this starts to blow up
    subset = frozenset()
    queue = deque([subset]) # Acyclic because subsets
    subproblems = {subset: Subproblem(0, None, None)}
    while queue:
        if store.is_terminated():
            return vertices
        subset = queue.popleft() # TODO: greedy version of this
        applied = set()
        for v in priority_ordering:
            if greedy and applied:
                break
            if (v not in subset) and valid_head_fn(v, subset) and not (out_priority_orders[v] & applied):
                applied.add(v)
                new_subset = frozenset([v]) | subset
                p_success, overhead = stats_fn(v)
                new_cost = overhead + p_success*subproblems[subset].cost
                subproblem = Subproblem(new_cost, v, subset)  # Adds new element to the front
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

def reorder_stream_plan(store, stream_plan, **kwargs):
    if not is_plan(stream_plan):
        return stream_plan
    indices = range(len(stream_plan))
    index_from_stream = dict(zip(stream_plan, indices))
    stream_orders = get_partial_orders(stream_plan)
    stream_orders = {(index_from_stream[s1], index_from_stream[s2]) for s1, s2 in stream_orders}
    #nodes = stream_plan
    nodes = indices

    in_stream_orders, out_stream_orders = neighbors_from_orders(stream_orders)
    valid_combine = lambda v, subset: out_stream_orders[v] <= subset
    #valid_combine = lambda v, subset: in_stream_orders[v] & subset
    #stats_fn = get_stream_stats
    stats_fn = lambda idx: get_stream_stats(stream_plan[idx])
    ordering = dynamic_programming(store, nodes, valid_combine, stats_fn, **kwargs)
    #gc.collect()
    ordering = [stream_plan[index] for index in ordering]
    return ordering
