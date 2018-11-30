from __future__ import print_function

from collections import namedtuple

from pddlstream.algorithms.algorithm import SolutionStore
from pddlstream.algorithms.algorithm import parse_stream_pddl, evaluations_from_init
from pddlstream.algorithms.downward import make_action, make_domain, make_predicate
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.reorder import reorder_stream_plan
from pddlstream.algorithms.scheduling.postprocess import reschedule_stream_plan
from pddlstream.algorithms.skeleton import SkeletonQueue, process_skeleton_queue
from pddlstream.language.constants import is_parameter, Not, PDDLProblem
from pddlstream.language.conversion import revert_solution, \
    evaluation_from_fact, replace_expression, get_prefix, get_args, obj_from_value_expression
from pddlstream.language.external import get_plan_effort
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.language.optimizer import retrace_instantiation
from pddlstream.language.stream import Stream
from pddlstream.utils import INF, get_mapping, str_from_object, get_length, safe_zip


# TODO: version of this where I pass in a plan skeleton instead
# Might be easiest to do this by performing the search normally but constraining the actions used
# Can add effects that bind future parameters when going along

# TODO: investigate constraint satisfaction techniques for binding instead

def dump_assignment(solution):
    # TODO: version of incremetal algorithm focused on constraint satisfaction
    bindings, cost, evaluations = solution
    print()
    print('Solved: {}'.format(bindings is not False))
    print('Cost: {}'.format(cost))
    print('Evaluations: {}'.format(len(evaluations)))
    if bindings is None:
        return
    print('Assignments:')
    for param in sorted(bindings):
        print('{} = {}'.format(param, str_from_object(bindings[param])))

##################################################

def obj_from_parameterized_expression(parent): # obj_from_value_expression
    return replace_expression(parent, lambda o: OptimisticObject
                              .from_opt(o, o) if is_parameter(o) else Object.from_value(o))


def create_domain(constraints_list):
    predicate_dict = {}
    for fact in constraints_list: # TODO: consider removing this annoying check
        name = get_prefix(fact)
        if name not in predicate_dict:
            parameters = ['?x{}'.format(i) for i in range(len(get_args(fact)))]
            predicate_dict[name] = make_predicate(name, parameters)
    return make_domain(predicates=list(predicate_dict.values()))

def constraint_satisfaction(stream_pddl, stream_map, init, constraints, stream_info={},
                            max_sample_time=INF, **kwargs):
    # Approaches
    # 1) Existential quantification of bindings in goal conditions
    # 2) Backtrack useful streams and then schedule. Create arbitrary outputs for not mentioned.
    # 3) Construct all useful streams and then associate outputs with bindings
    #    Useful stream must satisfy at least one fact. How should these assignments be propagated though?
    #    Make an action that maps each stream result to unbound values?
    # TODO: connect with the focused/incremental algorithms themselves
    # TODO: include functions again for cost-sensitive satisfaction
    if not constraints:
        return {}, 0, init
    externals = parse_stream_pddl(stream_pddl, stream_map, stream_info)
    streams = list(filter(lambda e: isinstance(e, Stream), externals))
    #print('Streams:', streams)
    evaluations = evaluations_from_init(init)
    goal_facts = set(filter(lambda f: evaluation_from_fact(f) not in evaluations,
                            map(obj_from_parameterized_expression, constraints)))
    store = SolutionStore(max_time=INF, max_cost=INF, verbose=True) # TODO: include other info here?

    visited_facts = set()
    stream_results = []
    for fact in goal_facts:
        retrace_instantiation(fact, streams, evaluations, visited_facts, stream_results)

    # TODO: consider other results if this fails
    stream_plan = reschedule_stream_plan(evaluations, goal_facts, create_domain(goal_facts),
                                         stream_results, unique_binding=True, **kwargs)
    if stream_plan is None:
        return revert_solution(store.best_plan, store.best_cost, evaluations)
    stream_plan = reorder_stream_plan(stream_plan)
    print('Stream plan ({}, {:.1f}): {}'.format(
        get_length(stream_plan), get_plan_effort(stream_plan), stream_plan))

    objects = set()
    for fact in goal_facts:
        objects.update(get_args(fact))
    free_parameters = list(filter(lambda o: isinstance(o, OptimisticObject), objects))
    action_plan = [('bindings', free_parameters)]

    queue = SkeletonQueue(store, evaluations, None, None)
    terminate = not process_skeleton_queue(store, queue, stream_plan, action_plan,
                                           cost=0, max_sample_time=max_sample_time)

    #write_stream_statistics(externals + synthesizers, verbose)
    plan, cost, init = revert_solution(store.best_plan, store.best_cost, evaluations)
    if plan is None:
        return plan, cost, init
    parameter_names = [o.value for o in free_parameters]
    [(_, parameter_values)] = plan
    bindings = get_mapping(parameter_names, parameter_values)
    return bindings, cost, init

##################################################

ASSIGNED_PREDICATE = 'assigned'
BOUND_PREDICATE = 'bound'
#UNBOUND_PREDICATE = 'unbound'
ORDER_PREDICATE = 'order'

Cluster = namedtuple('Cluster', ['constraints', 'parameters'])

def to_constant(parameter):
    name = parameter[1:]
    return '@{}'.format(name)

def cluster_constraints(constraints):
    # Can always combine clusters but leads to inefficient grounding
    # The extreme case of this making a single goal
    # Alternatively, can just keep each cluster separate (shouldn't slow down search much)
    # Can apply constants either as actual
    clusters = [Cluster([constraint], set(filter(is_parameter, get_args(constraint))))
                for constraint in constraints]
    clusters = sorted(clusters, key=lambda pair: len(pair[1]), reverse=True)
    for i in reversed(range(len(clusters))):
        for j in reversed(range(i)):
            if clusters[i].parameters <= clusters[j].parameters:
                clusters[j].constraints.extend(clusters[i].constraints)
                clusters[j].parameters.update(clusters[i].parameters)
                clusters.pop(i)
                break
    return clusters


def planning_from_satisfaction(constraints):
    clusters = cluster_constraints(constraints)
    order_value_facts = [(ORDER_PREDICATE, 't{}'.format(i)) for i in range(len(clusters))]
    goal_expression = order_value_facts[-1]
    order_facts = list(map(obj_from_value_expression, order_value_facts))
    bound_parameters = set()
    actions = []
    #constants = {}
    for i, cluster in enumerate(clusters):
        constraints = list(map(obj_from_value_expression, cluster.constraints))
        free_parameters = cluster.parameters - bound_parameters
        existing_parameters = cluster.parameters & bound_parameters
        # TODO: handle cost here. Can combine multiple costs after instantiation as well
        name = 'cluster-{}'.format(i)
        parameters = list(sorted(cluster.parameters))
        preconditions = [(ASSIGNED_PREDICATE, to_constant(p), p) for p in sorted(existing_parameters)] + \
                        [Not((BOUND_PREDICATE, to_constant(p))) for p in free_parameters] + \
                        constraints
        # [Not(UNBOUND_PREDICATE, constant_from_parameter(p)) for p in free_parameters]
        effects = [(ASSIGNED_PREDICATE, to_constant(p), p) for p in parameters] + \
                  [(BOUND_PREDICATE, to_constant(p)) for p in parameters] + \
                  [order_facts[i]]
        if i != 0:
            preconditions.append(order_facts[i-1])
            effects.append(Not(order_facts[i-1]))
        actions.append(make_action(name, parameters, preconditions, effects))
        bound_parameters.update(cluster.parameters)

    predicates = [make_predicate(ORDER_PREDICATE, ['?x'])]
    domain = make_domain(predicates=predicates, actions=actions)
    return domain, goal_expression


def solve_pddlstream_satisfaction(stream_pddl, stream_map, init, constraints, incremental=False, **kwargs):
    # TODO: support predicates and functions
    # I should be able to just add these to the end of the produced stream plan
    domain, goal = planning_from_satisfaction(constraints)
    constant_map = {}
    problem = PDDLProblem(domain, constant_map, stream_pddl, stream_map, init, goal)

    if incremental:
        plan, cost, facts = solve_incremental(problem, **kwargs)
    else:
        plan, cost, facts = solve_focused(problem, **kwargs)
    if plan is None:
        return None, cost, facts
    assert len(plan) == len(domain.actions)

    bindings = {}
    for action, (name, args) in safe_zip(domain.actions, plan):
        assert action.name == name
        for param, arg in safe_zip(action.parameters, args):
            name = param.name
            assert bindings.get(name, arg) is arg
            bindings[name] = arg
    return bindings, cost, facts
