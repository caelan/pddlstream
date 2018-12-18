from __future__ import print_function

from copy import deepcopy

from pddlstream.algorithms.downward import make_predicate, make_preconditions, make_effects, add_predicate
from pddlstream.language.constants import Or, And, is_parameter, Equal, Not, str_from_plan
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.utils import find_unique, safe_zip, str_from_object, INF

WILD = '*'
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
    def dump(self):
        print('{}(exact={}, max_cost={})'.format(self.__class__.__name__, self.exact, self.max_cost))
        if self.skeletons is None:
            return
        for i, skeleton in enumerate(self.skeletons):
            print(i, str_from_plan(skeleton))
    def __repr__(self):
        return '{}{}'.format(self.__class__.__name__, str_from_object(self.__dict__))

# TODO: rename other costs to be terminate_cost (or decision cost)
# TODO: partially ordered plans, AND/OR tree

def to_constant(parameter):
    name = parameter[1:]
    return '@{}'.format(name)

def to_obj(value):
    # Allows both raw values as well as objects to be specified
    if any(isinstance(value, Class) for Class in [Object, OptimisticObject]):
        return value
    return Object.from_value(value)

##################################################

def make_assignment_facts(local_from_global, parameters):
    return [(ASSIGNED_PREDICATE, to_obj(to_constant(p)), local_from_global[p])
            for p in parameters]

def add_plan_constraints(constraints, domain, evaluations, goal_exp):
    if (constraints is None) or (constraints.skeletons is None):
        return goal_exp
    import pddl
    # TODO: can search over skeletons first and then fall back
    # TODO: unify this with the constraint ordering
    # TODO: can constrain to use a plan prefix
    new_actions = []
    new_goals = []
    for num, skeleton in enumerate(constraints.skeletons):
        # TODO: change the prefix for these
        order_facts = [(ORDER_PREDICATE, to_obj('n{}'.format(num)), to_obj('t{}'.format(step)))
                        for step in range(len(skeleton) + 1)]
        evaluations[evaluation_from_fact(order_facts[0])] = None
        new_goals.append(order_facts[-1])
        bound_parameters = set()
        for step, (name, args) in enumerate(skeleton):
            # TODO: could also just remove the free parameter from the action
            new_action = deepcopy(find_unique(lambda a: a.name == name, domain.actions))
            arg_from_parameter = {p.name: a for p, a in safe_zip(new_action.parameters, args)}
            constants = [p.name for a, p in safe_zip(args, new_action.parameters)
                         if not is_parameter(a) and a != WILD]
            skeleton_parameters = list(filter(is_parameter, args))
            existing_parameters = [p for p in skeleton_parameters if p in bound_parameters]
            local_from_global = {a: p.name for a, p in safe_zip(args, new_action.parameters) if is_parameter(a)}

            new_preconditions = make_assignment_facts(local_from_global, existing_parameters) + [order_facts[step]] \
                                + [Equal(p, to_obj(arg_from_parameter[p])) for p in constants]
            new_action.precondition = pddl.Conjunction(
                [new_action.precondition, make_preconditions(new_preconditions)]).simplified()

            new_effects = make_assignment_facts(local_from_global, skeleton_parameters) \
                          + [Not(order_facts[step]), order_facts[step + 1]]
            new_action.effects.extend(make_effects(new_effects))
            # TODO: should also negate the effects of all other sequences here

            new_actions.append(new_action)
            bound_parameters.update(skeleton_parameters)
            #new_action.dump()
    add_predicate(domain, make_predicate(ORDER_PREDICATE, ['?num', '?step']))
    if constraints.exact:
        domain.actions[:] = []
    domain.actions.extend(new_actions)
    new_goal_exp = And(goal_exp, Or(*new_goals))
    return new_goal_exp
