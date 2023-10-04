from __future__ import print_function

try:
    from collections import Sequence
except ImportError:
    from collections.abc import Sequence
from itertools import product

from pddlstream.language.constants import EQ, AND, OR, NOT, CONNECTIVES, QUANTIFIERS, OPERATORS, OBJECTIVES, \
    Head, Evaluation, get_prefix, get_args, is_parameter, is_plan, Fact, Not, Equal, Action, StreamAction, \
    FunctionAction, DurativeAction, Solution, Assignment, OptPlan, Certificate
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.utils import str_from_object, apply_mapping

def replace_expression(parent, fn):
    prefix = get_prefix(parent)
    if prefix == EQ:
        assert(len(parent) == 3)
        value = parent[2]
        if isinstance(parent[2], Sequence):
            value = replace_expression(value, fn)
        return prefix, replace_expression(parent[1], fn), value
    elif prefix in (CONNECTIVES + OBJECTIVES):
        children = parent[1:]
        return (prefix,) + tuple(replace_expression(child, fn) for child in children)
    elif prefix in QUANTIFIERS:
        assert(len(parent) == 3)
        parameters = parent[1]
        child = parent[2]
        return prefix, parameters, replace_expression(child, fn)
    name = get_prefix(parent).lower()
    args = get_args(parent)
    return Fact(name, map(fn, args))

def obj_from_value_expression(parent):
    return replace_expression(parent, lambda o: o if is_parameter(o) else Object.from_value(o))

def value_from_obj_expression(parent):
    return replace_expression(parent, lambda o: o.value)

def value_from_evaluation(evaluation):
    return value_from_obj_expression(fact_from_evaluation(evaluation))

##################################################

def get_formula_operators(formula):
    if formula is None:
        return set()
    prefix = get_prefix(formula)
    if prefix not in OPERATORS:
        return set()
    operators = {prefix}
    for subformula in formula[1:]:
        operators.update(get_formula_operators(subformula))
    return operators

def dnf_from_positive_formula(parent):
    if parent is None:
        return []
    prefix = get_prefix(parent)
    assert(prefix not in (QUANTIFIERS + (NOT, EQ))) # also check if atom?
    children = []
    if prefix == AND:
        for combo in product(*(dnf_from_positive_formula(child) for child in parent[1:])):
            children.append([fact for clause in combo for fact in clause])
    elif prefix == OR:
        for child in parent[1:]:
            children.extend(dnf_from_positive_formula(child))
    else:
        # TODO: IMPLY
        children.append([tuple(parent)])
    return children

def list_from_conjunction(parent):
    if parent is None:
        return []
    clauses = dnf_from_positive_formula(parent)
    if not clauses:
        return clauses
    if len(clauses) >= 2:
        raise ValueError('Formula {} has more than one conjunctive clauses'.format(parent))
    return clauses[0]

def substitute_expression(parent, mapping):
    if any(isinstance(parent, Class) for Class in [str, Object, OptimisticObject]):
        return mapping.get(parent, parent)
    return tuple(substitute_expression(child, mapping) for child in parent)

def substitute_fact(fact, mapping):
    return Fact(get_prefix(fact), apply_mapping(get_args(fact), mapping))

##################################################

def pddl_from_object(obj):
    if isinstance(obj, str):
        return obj
    return obj.pddl

def pddl_list_from_expression(tree):
    if isinstance(tree, Object) or isinstance(tree, OptimisticObject):
        return pddl_from_object(tree)
    if isinstance(tree, str):
        return tree
    return tuple(map(pddl_list_from_expression, tree))

##################################################

def is_atom(evaluation):
    return evaluation.value is True

def is_negated_atom(evaluation):
    return evaluation.value is False

def objects_from_evaluations(evaluations):
    # TODO: assumes object predicates
    objects = set()
    for evaluation in evaluations:
        objects.update(evaluation.head.args)
    return objects

##################################################

def head_from_fact(fact):
    return Head(get_prefix(fact), get_args(fact))

def evaluation_from_fact(fact):
    prefix = get_prefix(fact)
    if prefix == EQ:
        head, value = fact[1:]
    elif prefix == NOT:
        head = fact[1]
        value = False
    else:
        head = fact
        value = True
    return Evaluation(head_from_fact(head), value)

def fact_from_evaluation(evaluation):
    fact = Fact(evaluation.head.function, evaluation.head.args)
    if is_atom(evaluation):
        return fact
    elif is_negated_atom(evaluation):
        return Not(fact)
    return Equal(fact, evaluation.value)

# def state_from_evaluations(evaluations):
#     # TODO: default value?
#     # TODO: could also implement within predicates
#     state = {}
#     for evaluation in evaluations:
#         if evaluation.head in state:
#             assert(evaluation.value == state[evaluation.head])
#         state[evaluation.head] = evaluation.value
#     return state

##################################################

def obj_from_pddl(pddl):
    if pddl in Object._obj_from_name:
        return Object.from_name(pddl)
    elif pddl in OptimisticObject._obj_from_name:
        return OptimisticObject.from_name(pddl)
    raise ValueError(pddl)

def values_from_objects(objects):
    return tuple(obj.value for obj in objects)
    #return tuple(map(value_from_object, objects))

def temporal_from_sequential(action):
    # TODO: clean this up
    assert isinstance(action, DurativeAction)
    name, args, start, duration = action
    if name[-2] != '-':
        return action
    new_name, index = name[:-2], int(name[-1])
    if index != 0: # Only keeps the start action
        return None
    return DurativeAction(new_name, args, start, duration)

def transform_action_args(action, fn):
    if isinstance(action, Action):
        name, args = action
        return Action(name, tuple(map(fn, args)))
    elif isinstance(action, DurativeAction):
        action = temporal_from_sequential(action)
        if action is None:
            return None
        name, args, start, duration = action
        return DurativeAction(name, tuple(map(fn, args)), start, duration)
    elif isinstance(action, StreamAction):
        name, inputs, outputs = action
        return StreamAction(name, tuple(map(fn, inputs)), tuple(map(fn, outputs)))
    elif isinstance(action, FunctionAction):
        name, inputs = action
        return FunctionAction(name, tuple(map(fn, inputs)))
    elif isinstance(action, Assignment):
        args, = action
        return Assignment(tuple(map(fn, args)))
    raise NotImplementedError(action)

def transform_plan_args(plan, fn):
    if not is_plan(plan):
        return plan
    return list(filter(lambda a: a is not None, [transform_action_args(action, fn) for action in plan]))

# TODO: would be better just to rename everything at the start. Still need to handle constants
def obj_from_pddl_plan(pddl_plan):
    return transform_plan_args(pddl_plan, obj_from_pddl)

def param_from_object(obj):
    if isinstance(obj, OptimisticObject):
        return repr(obj)
        #return obj.pddl
    if isinstance(obj, Object):
        return obj.value
    raise ValueError(obj)

def params_from_objects(objects):
    return tuple(map(param_from_object, objects))

def objects_from_values(values):
    return tuple(map(Object.from_value, values))

##################################################

#def expression_holds(expression, evaluations):
#    pass

def revert_solution(plan, cost, evaluations):
    all_facts = list(map(value_from_evaluation, evaluations))
    if isinstance(plan, OptPlan):
        action_plan = transform_plan_args(plan.action_plan, param_from_object)
        preimage_facts = list(map(value_from_obj_expression, plan.preimage_facts))
    else:
        action_plan = transform_plan_args(plan, param_from_object)
        preimage_facts = None
    certificate = Certificate(all_facts, preimage_facts)
    return Solution(action_plan, cost, certificate)

#def opt_obj_from_value(value):
#    if Object.has_value(value):
#        return Object.from_value(value)
#    return OptimisticObject.from_opt(value)
#    # TODO: better way of doing this?
#    #return OptimisticObject._obj_from_inputs.get(value, Object.from_value(value))

def str_from_head(head):
    return '{}{}'.format(get_prefix(head), str_from_object(get_args(head)))

def str_from_fact(fact):
    prefix = get_prefix(fact)
    if prefix == NOT:
        return '~{}'.format(str_from_fact(fact[1]))
    if prefix == EQ: # TODO: predicate = vs function =
        _, head, value = fact
        return '{}={}'.format(str_from_fact(head), value)
    return str_from_head(fact)
