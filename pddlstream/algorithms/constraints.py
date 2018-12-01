from copy import deepcopy

from pddlstream.algorithms.downward import make_predicate, make_preconditions, make_effects
from pddlstream.language.constants import And, is_parameter, Equal, Not
from pddlstream.language.conversion import obj_from_value_expression
from pddlstream.language.object import Object
from pddlstream.utils import find_unique, safe_zip

ANY = '*'
ASSIGNED_PREDICATE = 'assigned'
ORDER_PREDICATE = 'order'


def to_constant(parameter):
    name = parameter[1:]
    return '@{}'.format(name)


def add_plan_constraints(constraints, domain, init, goal):
    import pddl
    [skeleton] = constraints.skeletons
    # TODO: can search over skeletons first and then fall back
    # TODO: unify this with the constraint ordering

    new_actions = []
    order_value_facts = [(ORDER_PREDICATE, 't{}'.format(i)) for i in range(len(skeleton) + 1)]
    init.append(order_value_facts[0])
    goal = And(goal, order_value_facts[-1])
    domain.predicate_dict[ORDER_PREDICATE] = make_predicate(ORDER_PREDICATE, ['?x'])

    order_facts = list(map(obj_from_value_expression, order_value_facts))
    bound_parameters = set()
    for i, (name, args) in enumerate(skeleton):
        action = find_unique(lambda a: a.name == name, domain.actions)
        new_action = deepcopy(action)
        assert len(args) == len(new_action.parameters)
        arg_from_parameter = {p.name: a for p, a in safe_zip(new_action.parameters, args)}


        free = [p for a, p in safe_zip(args, new_action.parameters) if is_parameter(a)]
        wildcards = [p.name for a, p in safe_zip(args, new_action.parameters) if a == ANY]
        constants = [p.name for a, p in safe_zip(args, new_action.parameters)
                     if not is_parameter(a) and a != ANY]
        print(free, wildcards, constants)

        skeleton_parameters = list(filter(is_parameter, args))
        existing_parameters = [p for p in skeleton_parameters if p in bound_parameters]
        local_from_global = {a: p.name for a, p in safe_zip(args, new_action.parameters) if is_parameter(a)}
        print(arg_from_parameter)
        print(local_from_global)

        # TODO: could also just remove the free parameter from the action

        print()

        # for param in new_action:
        #    print(param)

        print(new_action)

        new_preconditions = [(ASSIGNED_PREDICATE, to_constant(p), local_from_global[p])
                             for p in existing_parameters] + [order_facts[i]] + \
                            [Equal(p, Object.from_value(arg_from_parameter[p])) for p in constants]
        new_effects = [(ASSIGNED_PREDICATE, to_constant(p), local_from_global[p])
                       for p in skeleton_parameters] + [Not(order_facts[i]), order_facts[i + 1]]
        new_action.precondition = pddl.Conjunction(
            [new_action.precondition, make_preconditions(new_preconditions)]).simplified()
        new_action.effects.extend(make_effects(new_effects))
        new_action.dump()
        new_actions.append(new_action)
        bound_parameters.update(skeleton_parameters)
    if constraints.exact:
        domain.actions[:] = []
    domain.actions.extend(new_actions)
    return goal