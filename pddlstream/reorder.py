from collections import defaultdict, namedtuple, deque
from heapq import heappush, heappop

from pddlstream.conversion import evaluation_from_fact, get_prefix, EQ, pddl_from_object
from pddlstream.fast_downward import get_init, task_from_domain_problem, get_problem, fd_from_fact, is_applicable, \
    apply_action
from pddlstream.function import PredicateResult, Result
from pddlstream.scheduling.relaxed import instantiate_axioms, get_achieving_axioms, extract_axioms
from pddlstream.scheduling.simultaneous import evaluations_from_stream_plan
from pddlstream.utils import INF, Verbose, MockSet, find
from pddlstream.visualization import get_partial_orders

def neighbors_from_orders(orders):
    incoming_edges = defaultdict(set)
    outgoing_edges = defaultdict(set)
    for v1, v2 in orders:
        incoming_edges[v2].add(v1)
        outgoing_edges[v1].add(v2)
    return incoming_edges, outgoing_edges

def topological_sort(vertices, orders, priority_fn=lambda v: 0):
    # Can also do a DFS version
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

# TODO: include context here as a weak constraint
# TODO: actions as a weak constraint
# TODO: works in the absence of partial orders
# TODO: actions are extremely unlikely to work
# TODO: can give actions extreme priority
# TODO: can also more manually reorder

def dominates(v1, v2):
    p_success1, overhead1 = get_stream_stats(v1)
    p_success2, overhead2 = get_stream_stats(v2)
    return (p_success1 <= p_success2) and (overhead1 <= overhead2)

Subproblem = namedtuple('Subproblem', ['cost', 'head', 'subset'])

def dynamic_programming(vertices, valid_head, effort_orders=set(), prune=True, greedy=False):
    # 2^N rather than N!
    _, out_effort_orders = neighbors_from_orders(effort_orders)
    effort_ordering = topological_sort(vertices, effort_orders)[::-1]

    subset = frozenset()
    queue = deque([subset]) # Acyclic because subsets
    subproblems = {subset: Subproblem(0, None, None)}
    while queue:
        subset = queue.popleft()
        applied = set()
        for v in effort_ordering:
            if greedy and applied:
                break
            if (v not in subset) and valid_head(v, subset) and not (out_effort_orders[v] & applied):
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
    subset = frozenset(vertices)
    while True:
        subproblem = subproblems[subset]
        if subproblem.head is None:
            break
        ordering.append(subproblem.head)
        subset = subproblem.subset
    return ordering

def reorder_streams(stream_plan, prune=True, **kwargs):
    if stream_plan is None:
        return None
    in_stream_orders, out_stream_orders = neighbors_from_orders(get_partial_orders(stream_plan))
    def valid_combine(v, subset):
        return out_stream_orders[v] <= subset
        #return in_stream_orders[v] & subset # These are equivalent

    # TODO: can just do on the infos themselves
    effort_orders = set()
    if prune:
        for i, v1 in enumerate(stream_plan):
            for v2 in stream_plan[i+1:]:
                if dominates(v1, v2):
                    effort_orders.add((v1, v2)) # Includes equality
                elif dominates(v2, v1):
                    effort_orders.add((v2, v1))

    return dynamic_programming(stream_plan, valid_combine, effort_orders, **kwargs)

##################################################

"""
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
"""

##################################################

def stuff_programming(evaluations, combined_plan, domain):
    stream_plan, action_plan = separate_plan(combined_plan)
    goal_expression = ('and',)
    orders = instantiate_plan(evaluations, stream_plan, action_plan, goal_expression, domain)
    print(orders)

    return dynamic_programming(combined_plan)

def instantiate_plan(evaluations, stream_plan, action_plan, goal_expression, domain):
    if action_plan is None:
        return None
    # TODO: could just do this within relaxed
    # TODO: propositional stream instances
    # TODO: do I want to strip the fluents and just do the total ordering?
    negative_results = filter(lambda r: isinstance(r, PredicateResult) and (r.value == False), stream_plan)
    negative_init = set(get_init((evaluation_from_fact(f) for r in negative_results
                                  for f in r.get_certified()), negated=True))
    #negated_from_name = {r.instance.external.name for r in negative_results}
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_plan)

    import pddl_to_prolog
    import build_model
    import axiom_rules
    import pddl
    import instantiate

    task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs=False))
    actions = task.actions
    task.actions = []
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    function_assignments = {f.fluent: f.expression for f in task.init
                            if isinstance(f, pddl.f_expression.FunctionAssignment)}
    task.init = (set(task.init) | {a.negate() for a in negative_init}) - set(function_assignments)

    # TODO: something that inverts the negative items
    stream_instances = [] # TODO: could even apply these to the state directly
    for result in stream_plan:
        name = result.instance.external.name
        precondition = list(map(fd_from_fact, result.instance.get_domain()))
        effects = [([], fd_from_fact(fact)) for fact in result.get_certified() if not get_prefix(fact) == EQ]
        cost = None # TODO: effort?
        instance = pddl.PropositionalAction(name, precondition, effects, cost)
        stream_instances.append(instance)

    action_instances = []
    for name, objects in action_plan:
        # TODO: just instantiate task?
        with Verbose(False):
            model = build_model.compute_model(pddl_to_prolog.translate(task)) # Changes based on init
        #fluent_facts = instantiate.get_fluent_facts(task, model)
        fluent_facts = MockSet()
        init_facts = task.init
        instantiated_axioms = instantiate_axioms(task, model, init_facts, fluent_facts)

        # TODO: what if more than one action of the same name due to normalization?
        action = find(lambda a: a.name == name, actions)
        args = map(pddl_from_object, objects)
        assert(len(action.parameters) == len(args))
        variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        instance = action.instantiate(variable_mapping, init_facts,
                           fluent_facts, type_to_objects,
                           task.use_min_cost_metric, function_assignments)
        assert(instance is not None)

        goal_list = [] # TODO: include the goal?
        with Verbose(False): # TODO: helpful_axioms prunes axioms that are already true (e.g. not Unsafe)
            helpful_axioms, axiom_init, _ = axiom_rules.handle_axioms([instance], instantiated_axioms, goal_list)
        axiom_from_atom = get_achieving_axioms(task.init | negative_init,
                                               helpful_axioms, axiom_init)
                                               #negated_from_name=negated_from_name)

        axiom_plan = []
        extract_axioms(axiom_from_atom, instance.precondition, axiom_plan)
        # TODO: what the propositional axiom has conditional derived
        axiom_pre = {p for ax in axiom_plan for p in ax.condition}
        axiom_eff = {ax.effect for ax in axiom_plan}
        instance.precondition = list((set(instance.precondition) | axiom_pre) - axiom_eff)

        assert(is_applicable(task.init, instance))
        apply_action(task.init, instance)
        action_instances.append(instance)

    #combined_instances = stream_instances + action_instances
    orders = set()
    for i, a1 in enumerate(action_plan):
        for a2 in action_plan[i+1:]:
            orders.add((a1, a2))
    # TODO: just store first achiever here
    for i, instance1 in enumerate(stream_instances):
        for j in range(i+1, len(stream_instances)):
            effects = {e for _, e in  instance1.add_effects}
            if effects & set(stream_instances[j].precondition):
                orders.add((stream_plan[i], stream_plan[j]))
    for i, instance1 in enumerate(stream_instances):
        for j, instance2 in enumerate(action_instances):
            effects = {e for _, e in  instance1.add_effects}
            if effects & set(instance2.precondition):
                orders.add((stream_plan[i], action_plan[j]))
    return orders

##################################################

def separate_plan(combined_plan, action_info=None):
    if combined_plan is None:
        return None, None
    stream_plan = []
    action_plan = []
    terminated = False
    for operator in combined_plan:
        if not terminated and isinstance(operator, Result):
            stream_plan.append(operator)
        else:
            action_plan.append(operator)
            if action_info is not None:
                name, _ = operator
                terminated |= action_info[name].terminal
    return stream_plan, action_plan