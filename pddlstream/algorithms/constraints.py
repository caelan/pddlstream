from copy import deepcopy

from pddlstream.algorithms.downward import make_predicate, make_preconditions, make_effects, add_predicate
from pddlstream.language.constants import Or, And, is_parameter, Equal, Not
from pddlstream.language.conversion import obj_from_value_expression
from pddlstream.language.object import Object
from pddlstream.utils import find_unique, safe_zip, str_from_object, INF

ANY = '*'
ASSIGNED_PREDICATE = 'assigned'
ORDER_PREDICATE = 'order'

class PlanConstraints(object):
    def __init__(self, skeletons=None, exact=True, hint=False, max_cost=INF):
        self.skeletons = skeletons
        self.exact = exact
        self.hint = hint # Search over skeletons first and then fall back
        self.max_cost = max_cost
        #self.max_length = max_length
        if self.hint:
            raise NotImplementedError()
    def __repr__(self):
        return '{}{}'.format(self.__class__.__name__, str_from_object(self.__dict__))

# TODO: rename other costs to be terminate_cost (or decision cost)
# TODO: partially ordered plans, AND/OR tree

def to_constant(parameter):
    name = parameter[1:]
    return '@{}'.format(name)


def add_plan_constraints(constraints, domain, init, goal):
    if constraints.skeletons is None:
        return goal
    import pddl
    # TODO: can search over skeletons first and then fall back
    # TODO: unify this with the constraint ordering
    new_actions = []
    new_goals = []
    for num, skeleton in enumerate(constraints.skeletons):
        order_value_facts = [(ORDER_PREDICATE, 'n{}'.format(num), 't{}'.format(step))
                             for step in range(len(skeleton) + 1)]
        init.append(order_value_facts[0])
        new_goals.append(order_value_facts[-1])
        order_facts = list(map(obj_from_value_expression, order_value_facts))
        bound_parameters = set()
        for step, (name, args) in enumerate(skeleton):
            # TODO: could also just remove the free parameter from the action
            action = find_unique(lambda a: a.name == name, domain.actions)
            new_action = deepcopy(action)
            assert len(args) == len(new_action.parameters)
            arg_from_parameter = {p.name: a for p, a in safe_zip(new_action.parameters, args)}
            #free = [p.name for a, p in safe_zip(args, new_action.parameters) if is_parameter(a)]
            #wildcards = [p.name for a, p in safe_zip(args, new_action.parameters) if a == ANY]
            constants = [p.name for a, p in safe_zip(args, new_action.parameters)
                         if not is_parameter(a) and a != ANY]
            skeleton_parameters = list(filter(is_parameter, args))
            existing_parameters = [p for p in skeleton_parameters if p in bound_parameters]
            local_from_global = {a: p.name for a, p in safe_zip(args, new_action.parameters) if is_parameter(a)}

            new_preconditions = [(ASSIGNED_PREDICATE, Object.from_value(to_constant(p)), local_from_global[p])
                                 for p in existing_parameters] + [order_facts[step]] + \
                                [Equal(p, Object.from_value(arg_from_parameter[p])) for p in constants]
            new_action.precondition = pddl.Conjunction(
                [new_action.precondition, make_preconditions(new_preconditions)]).simplified()

            new_effects = [(ASSIGNED_PREDICATE, Object.from_value(to_constant(p)), local_from_global[p])
                           for p in skeleton_parameters] + [Not(order_facts[step]), order_facts[step + 1]]
            new_action.effects.extend(make_effects(new_effects))
            # TODO: should negate the effects of all other sequences here

            new_actions.append(new_action)
            bound_parameters.update(skeleton_parameters)
    add_predicate(domain, make_predicate(ORDER_PREDICATE, ['?num', '?step']))
    if constraints.exact:
        domain.actions[:] = []
    domain.actions.extend(new_actions)
    return And(goal, Or(*new_goals))
