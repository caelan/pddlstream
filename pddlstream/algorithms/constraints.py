from __future__ import print_function

from copy import deepcopy

from pddlstream.algorithms.downward import make_predicate, make_preconditions, make_effects, add_predicate
from pddlstream.language.constants import Or, And, is_parameter, Equal, Not, str_from_plan, EQ
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.utils import find_unique, safe_zip, str_from_object, INF, is_hashable

WILD = '*'
ASSIGNED_PREDICATE = 'assigned'
ORDER_PREDICATE = 'order'
GROUP_PREDICATE = 'group'
INTERNAL_EVALUATION = None

class PlanConstraints(object):
    def __init__(self, skeletons=None, groups={}, exact=True, hint=False, max_cost=INF):
        self.skeletons = skeletons
        self.groups = groups # Could make this a list of lists
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
    for group in constraints.groups:
        for value in constraints.groups[group]:
            # TODO: could make all constants groups (like an equality group)
            fact = (GROUP_PREDICATE, to_obj(group), to_obj(value))
            evaluations[evaluation_from_fact(fact)] = INTERNAL_EVALUATION
    new_actions = []
    new_goals = []
    for num, skeleton in enumerate(constraints.skeletons):
        # TODO: change the prefix for these
        order_facts = [(ORDER_PREDICATE, to_obj('n{}'.format(num)), to_obj('t{}'.format(step)))
                        for step in range(len(skeleton) + 1)]
        evaluations[evaluation_from_fact(order_facts[0])] = INTERNAL_EVALUATION
        new_goals.append(order_facts[-1])
        bound_parameters = set()
        for step, (name, args) in enumerate(skeleton):
            # TODO: could also just remove the free parameter from the action
            new_action = deepcopy(find_unique(lambda a: a.name == name, domain.actions))
            constant_pairs = [(a, p.name) for a, p in safe_zip(args, new_action.parameters)
                         if not is_parameter(a) and a != WILD]
            skeleton_parameters = list(filter(is_parameter, args))
            existing_parameters = [p for p in skeleton_parameters if p in bound_parameters]
            local_from_global = {a: p.name for a, p in safe_zip(args, new_action.parameters) if is_parameter(a)}

            group_preconditions = [(GROUP_PREDICATE if is_hashable(a) and (a in constraints.groups) else EQ,
                                    to_obj(a), p) for a, p in constant_pairs]
            new_preconditions = make_assignment_facts(local_from_global, existing_parameters) + \
                                group_preconditions + [order_facts[step]]
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
