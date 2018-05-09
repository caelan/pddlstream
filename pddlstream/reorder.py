from collections import defaultdict, namedtuple, deque
from heapq import heappush, heappop

from pddlstream.utils import INF
from pddlstream.visualization import get_partial_orders

def neighbors_from_orders(orders):
    incoming_edges = defaultdict(set)
    outgoing_edges = defaultdict(set)
    for v1, v2 in orders:
        incoming_edges[v2].add(v1)
        outgoing_edges[v1].add(v2)
    return incoming_edges, outgoing_edges

def topological_sort(vertices, orders, priority_fn=lambda v: 0):
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)
    ordering = []
    queue = []
    for v in vertices:
        if not incoming_edges[v]:
            heappush(queue, (priority_fn(v), v))
    while queue:
        _, v1 = heappop(queue)
        ordering.append(v1)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1)
            if not incoming_edges[v2]:
                heappush(queue, (priority_fn(v2), v2))
    return ordering

##################################################

def get_stream_stats(result):
    info = result.instance.external.info
    return info.p_success, info.overhead

def get_deterministic_action_stats(action):
    p_success = 1
    overhead = INF
    return p_success, overhead

def get_replan_action_stats(action):
    p_success = 1e-3 # Should never do zero...
    overhead = 1
    return p_success, overhead

def compute_expected_cost(stream_plan):
    if stream_plan is None:
        return INF
    expected_cost = 0
    for result in reversed(stream_plan):
        p_success, overhead = get_stream_stats(result)
        expected_cost = overhead + p_success * expected_cost
    return expected_cost

Subproblem = namedtuple('Subproblem', ['cost', 'head', 'subset'])

# TODO: include context here as a weak constraint
# TODO: actions as a weak constraint
# TODO: works in the absence of parital orders
# TODO: actions are extremely unlikely to work

def dynamic_programming(stream_plan, prune=True, greedy=False):
    # 2^N rather than N!
    if stream_plan is None:
        return None

    def dominates(v1, v2):
        p_success1, overhead1 = get_stream_stats(v1)
        p_success2, overhead2 = get_stream_stats(v2)
        return (p_success1 <= p_success2) and (overhead1 <= overhead2)

    # TODO: can just do on the infos themselves
    effort_orders = set()
    if prune:
        for i, v1 in enumerate(stream_plan):
            for v2 in stream_plan[i+1:]:
                if dominates(v1, v2):
                    effort_orders.add((v1, v2)) # Includes equality
                elif dominates(v2, v1):
                    effort_orders.add((v2, v1))
    _, out_effort_orders = neighbors_from_orders(effort_orders)
    effort_ordering = topological_sort(stream_plan, effort_orders)[::-1]

    in_stream_orders, out_stream_orders = neighbors_from_orders(get_partial_orders(stream_plan))
    def valid_combine(v, subset):
        return (v not in subset) and (out_stream_orders[v] <= subset)
        #return (v not in subset) and not (in_stream_orders[v] & subset) # These are equivalent

    subset = frozenset()
    queue = deque([subset]) # Acyclic because subsets
    subproblems = {subset: Subproblem(0, None, None)}
    iterations = 0
    while queue:
        iterations += 1
        subset = queue.popleft()
        applied = set()
        for v in effort_ordering:
            if greedy and applied:
                break
            if valid_combine(v, subset) and not (out_effort_orders[v] & applied):
                applied.add(v)
                new_subset = frozenset([v]) | subset
                p_success, overhead = get_stream_stats(v)
                new_cost = overhead + p_success*subproblems[subset].cost
                subproblem = Subproblem(new_cost, v, subset)  # Adds new element to the front
                if new_subset not in subproblems:
                    queue.append(new_subset)
                    subproblems[new_subset] = subproblem
                elif new_cost < subproblems[new_subset].cost:
                    subproblems[new_subset] = subproblem

    ordering = []
    subset = frozenset(stream_plan)
    while True:
        subproblem = subproblems[subset]
        if subproblem.head is None:
            break
        ordering.append(subproblem.head)
        subset = subproblem.subset
    return ordering

##################################################

def partial_ordered(plan):
    # https://www.aaai.org/ocs/index.php/ICAPS/ICAPS10/paper/viewFile/1420/1539
    # http://repository.cmu.edu/cgi/viewcontent.cgi?article=1349&context=compsci
    # https://arxiv.org/pdf/1105.5441.pdf
    # https://pdfs.semanticscholar.org/e057/e330249f447c2f065cf50db9dfaddad16aaa.pdf
    # https://github.mit.edu/caelan/PAL/blob/master/src/search/post_processing.cc

    instances = instantiate_plan(plan)
    orders = set()
    primary_effects = set() # TODO: start and goal operators here?
    for i in reversed(xrange(len(instances))):
        for pre in instances[i].preconditions:
            for j in reversed(xrange(i)):
                #if pre in instances[j].effects:
                if any(eff == pre for eff in instances[j].effects):
                    orders.add((j, i))
                    primary_effects.add((j, pre))
                    break
        for eff in instances[i].effects:
            for j in xrange(i):
                if any((pre.head == eff.head) and (pre.value != eff.value) for pre in instances[j].preconditions):
                    orders.add((j, i))
            if (i, eff) in primary_effects:
                for j in xrange(i):
                    if any((eff2.head == eff.head) and (eff2.value != eff.value) for eff2 in instances[j].effects):
                        orders.add((j, i))
    # TODO: could remove transitive
    # TODO: this isn't so helpful because it will choose arbitrary streams until an action is feasible (i.e. not intelligent ones)
    for i, (action, args) in enumerate(plan):
        print i, action, args #, instances[i].preconditions, instances[i].effects
    print orders
    print primary_effects
    print topological_sort(range(len(plan)), orders, lambda v: hasattr(plan[v][0], 'stream'))