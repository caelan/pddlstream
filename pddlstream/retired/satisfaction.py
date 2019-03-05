from __future__ import print_function

from collections import namedtuple

from pddlstream.algorithms.constraints import to_constant, ORDER_PREDICATE, ASSIGNED_PREDICATE, \
    get_internal_prefix
from pddlstream.algorithms.downward import make_action, make_domain, make_predicate
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import is_parameter, Not, PDDLProblem, MINIMIZE, NOT, partition_facts, get_costs, \
    get_constraints
from pddlstream.language.conversion import get_prefix, get_args, obj_from_value_expression
from pddlstream.utils import safe_zip

Cluster = namedtuple('Cluster', ['constraints', 'parameters'])


def get_parameters(expression):
    head = get_prefix(expression)
    if head in [NOT, MINIMIZE]:
        return get_parameters(get_args(expression)[0])
    return list(filter(is_parameter, get_args(expression)))


def update_cluster(cluster1, cluster2):
    assert cluster2.parameters <= cluster1.parameters
    cluster1.constraints.extend(cluster2.constraints)
    cluster1.parameters.update(cluster2.parameters)


def cluster_constraints(terms):
    # Can always combine clusters but leads to inefficient grounding
    # The extreme case of this making a single goal
    # Alternatively, can just keep each cluster separate (shouldn't slow down search much)
    clusters = sorted([Cluster([constraint], set(get_parameters(constraint)))
                       for constraint in get_constraints(terms)],
                      key=lambda c: len(c.parameters), reverse=True)
    cost_clusters = sorted([Cluster([cost], set(get_parameters(cost)))
                            for cost in get_costs(terms)],
                           key=lambda c: len(c.parameters))
    for c1 in cost_clusters:
        for c2 in reversed(clusters):
            if 1 < len(get_costs(c1.constraints)) + len(get_costs(c2.constraints)):
                continue
            if c1.parameters <= c2.parameters:
                update_cluster(c2, c1)
                break
        else:
            # TODO: extend this to allow the intersection to cover the cluster
            raise RuntimeError('Unable to find a cluster for cost term:', c1.constraints[0])

    for i in reversed(range(len(clusters))):
        c1 = clusters[i]
        for j in reversed(range(i)):
            c2 = clusters[j]
            if 1 < len(get_costs(c1.constraints)) + len(get_costs(c2.constraints)):
                continue
            if c1.parameters <= c2.parameters:
                update_cluster(c2, c1)
                clusters.pop(i)
                break
    return clusters

##################################################

def planning_from_satisfaction(init, constraints):
    clusters = cluster_constraints(constraints)
    prefix = get_internal_prefix(internal=False)
    assigned_predicate = ASSIGNED_PREDICATE.format(prefix)
    order_predicate = ORDER_PREDICATE.format(prefix)
    #order_value_facts = make_order_facts(order_predicate, 0, len(clusters)+1)
    order_value_facts = [(order_predicate, '_t{}'.format(i)) for i in range(len(clusters)+1)]
    init.append(order_value_facts[0])
    goal_expression = order_value_facts[-1]
    order_facts = list(map(obj_from_value_expression, order_value_facts))
    bound_parameters = set()
    actions = []
    #constants = {}
    for i, cluster in enumerate(clusters):
        objectives = list(map(obj_from_value_expression, cluster.constraints))
        constraints, negated, costs = partition_facts(objectives)
        if negated:
            raise NotImplementedError(negated)
        #free_parameters = cluster.parameters - bound_parameters
        existing_parameters = cluster.parameters & bound_parameters
        # TODO: confirm that negated predicates work as intended

        name = 'cluster-{}'.format(i)
        parameters = list(sorted(cluster.parameters))
        preconditions = [(assigned_predicate, to_constant(p), p) for p in sorted(existing_parameters)] + \
                        constraints + [order_facts[i]]
        effects = [(assigned_predicate, to_constant(p), p) for p in parameters] + \
                  [order_facts[i+1], Not(order_facts[i])]

        if costs:
            assert len(costs) == 1
            [cost] = costs
        else:
            cost = None
        actions.append(make_action(name, parameters, preconditions, effects, cost))
        #actions[-1].dump()
        bound_parameters.update(cluster.parameters)

    predicates = [make_predicate(order_predicate, ['?step'])] # '?num',
    domain = make_domain(predicates=predicates, actions=actions)
    return domain, goal_expression

##################################################

def pddl_from_csp(stream_pddl, stream_map, init, constraints):
    domain, goal = planning_from_satisfaction(init, constraints)
    constant_map = {}
    return PDDLProblem(domain, constant_map, stream_pddl, stream_map, init, goal)

def bindings_from_plan(problem, plan):
    if plan is None:
        return None
    domain = problem[0]
    bindings = {}
    for action, (name, args) in safe_zip(domain.actions, plan):
        assert action.name == name
        for param, arg in safe_zip(action.parameters, args):
            name = param.name
            assert bindings.get(name, arg) is arg
            bindings[name] = arg
    return bindings

##################################################

def solve_pddlstream_satisfaction(stream_pddl, stream_map, init, constraints, incremental=False, **kwargs):
    # TODO: prune set of streams based on constraints
    # TODO: investigate constraint satisfaction techniques for search instead
    # TODO: optimistic objects based on free parameters that prevent cycles
    # TODO: disallow creation of new parameters / certifying new facts
    problem = pddl_from_csp(stream_pddl, stream_map, init, constraints)
    if incremental:
        plan, cost, facts = solve_incremental(problem, **kwargs)
    else:
        plan, cost, facts = solve_focused(problem, **kwargs)
    bindings = bindings_from_plan(problem, plan)
    return bindings, cost, facts
