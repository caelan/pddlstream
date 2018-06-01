from __future__ import print_function

import collections
from collections import namedtuple

from pddlstream.object import Object, OptimisticObject
from pddlstream.utils import str_from_tuple

EQ = '=' # xnor
AND = 'and'
OR = 'or'
NOT = 'not'
EXISTS = 'exists'
FORALL = 'forall'
WHEN = 'when'
#IMPLIES = 'implies' # TODO: FastDownward doesn't support this
MINIMIZE = 'minimize'
MAXIMIZE = 'maximize'

PARAMETER = '?'
TYPE = '-'

CONNECTIVES = (AND, OR, NOT)
QUANTIFIERS = (FORALL, EXISTS)
OPERATORS = CONNECTIVES + QUANTIFIERS + (WHEN,)

Problem = namedtuple('Problem', ['domain_pddl', 'constant_map', 'stream_pddl', 'stream_map', 'init', 'goal'])
Head = namedtuple('Head', ['function', 'args'])
Evaluation = namedtuple('Evaluation', ['head', 'value'])
Atom = lambda head: Evaluation(head, True)
NegatedAtom = lambda head: Evaluation(head, False)

##################################################

def And(*expressions):
    return (AND,) + tuple(expressions)

def Or(*expressions):
    return (OR,) + tuple(expressions)

def Not(expression):
    return (NOT, expression)

def Equal(expression1, expression2):
    return (EQ, expression1, expression2)

def Minimize(expression):
    return (MINIMIZE, expression)

def Type(param, ty):
    return (param, TYPE, ty)

def Exists(args, expression):
    return (EXISTS, args, expression)

def ForAll(args, expression):
    return (FORALL, args, expression)

##################################################

def get_prefix(expression):
    return expression[0]

def get_args(head):
    return head[1:]

def is_parameter(expression):
    return isinstance(expression, str) and expression.startswith(PARAMETER)

def is_head(expression):
    return get_prefix(expression) not in OPERATORS

def replace_expression(parent, fn):
    prefix = get_prefix(parent)
    if prefix == EQ:
        assert(len(parent) == 3)
        value = parent[2]
        if isinstance(parent[2], collections.Sequence):
            value = replace_expression(value, fn)
        return prefix, replace_expression(parent[1], fn), value
    elif prefix in CONNECTIVES:
        children = parent[1:]
        return (prefix,) + tuple(replace_expression(child, fn) for child in children)
    elif prefix in QUANTIFIERS:
        assert(len(parent) == 3)
        parameters = parent[1]
        child = parent[2]
        return prefix, parameters, replace_expression(child, fn)
    name = get_prefix(parent).lower()
    args = get_args(parent)
    return (name,) + tuple(map(fn, args))

def obj_from_value_expression(parent):
    return replace_expression(parent, lambda o: o if is_parameter(o) else Object.from_value(o))

def value_from_obj_expression(parent):
    return replace_expression(parent, lambda o: o.value)

##################################################

def list_from_conjunction(parent):
    if not parent:
        return []
    prefix = get_prefix(parent)
    assert(prefix not in (QUANTIFIERS + (NOT, OR, EQ)))
    if prefix == AND:
        children = []
        for child in parent[1:]:
            children += list_from_conjunction(child)
        return children
    return [tuple(parent)]

def substitute_expression(parent, mapping):
    if isinstance(parent, str) or isinstance(parent, Object) or isinstance(parent, OptimisticObject):
        return mapping.get(parent, parent)
    return tuple(substitute_expression(child, mapping) for child in parent)

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

def evaluations_from_init(init):
    return [evaluation_from_fact(obj_from_value_expression(fact)) for fact in init]

##################################################

# TODO: generic method for replacing args?

def fact_from_evaluation(evaluation):
    head = (evaluation.head.function,) + evaluation.head.args
    if is_atom(evaluation):
        return head
    elif is_negated_atom(evaluation):
        return (NOT, head)
    return (EQ, head, evaluation.value)

def init_from_evaluation(evaluation):
    head = (evaluation.head.function,) + values_from_objects(evaluation.head.args)
    if is_atom(evaluation):
        return head
    elif is_negated_atom(evaluation):
        return (NOT, head)
    return (EQ, head, evaluation.value)

def init_from_evaluations(evaluations):
    return list(map(init_from_evaluation, evaluations))

def state_from_evaluations(evaluations):
    # TODO: default value?
    # TODO: could also implement within predicates
    state = {}
    for evaluation in evaluations:
        if evaluation.head in state:
            assert(evaluation.value == state[evaluation.head])
        state[evaluation.head] = evaluation.value
    return state

##################################################

def obj_from_pddl(pddl):
    if pddl in Object._obj_from_name:
        return Object.from_name(pddl)
    elif pddl in OptimisticObject._obj_from_name:
        return OptimisticObject.from_name(pddl)
    else:
        raise ValueError(pddl)


def values_from_objects(objects):
    return tuple(obj.value for obj in objects)
    #return tuple(map(value_from_object, objects))

# TODO: would be better just to rename everything at the start. Still need to handle constants
def obj_from_pddl_plan(pddl_plan):
    if pddl_plan is None:
        return None
    return [(action, tuple(map(obj_from_pddl, args))) for action, args in pddl_plan]

def param_from_object(obj):
    if isinstance(obj, OptimisticObject):
        return obj.pddl
    return obj.value

def params_from_objects(objects):
    return tuple(map(param_from_object, objects))

def value_from_obj_plan(obj_plan):
    if obj_plan is None:
        return None
    #return [(action,) + tuple(values_from_objects(args)) for action, args in obj_plan]
    #return [(action, tuple(values_from_objects(args))) for action, args in obj_plan]
    value_plan = []
    for operator in obj_plan:
        if len(operator) == 2:
            name, args = operator
            new_operator = (name, params_from_objects(args)) # values_from_objects
        elif len(operator) == 3:
            name, inputs, outputs = operator
            new_inputs = params_from_objects(inputs) # values_from_objects
            new_outputs = outputs
            if isinstance(new_outputs, collections.Sequence):
                new_outputs = params_from_objects(new_outputs) # values_from_objects
            new_operator = (name, new_inputs, new_outputs)
        else:
            raise ValueError(operator)
        value_plan.append(new_operator)
    return value_plan

##################################################

#def expression_holds(expression, evaluations):
#    pass

def revert_solution(plan, cost, evaluations):
    return value_from_obj_plan(plan), cost, init_from_evaluations(evaluations)


#def opt_obj_from_value(value):
#    if Object.has_value(value):
#        return Object.from_value(value)
#    return OptimisticObject.from_opt(value)
#    # TODO: better way of doing this?
#    #return OptimisticObject._obj_from_inputs.get(value, Object.from_value(value))


def str_from_head(head):
    return '{}{}'.format(get_prefix(head), str_from_tuple(get_args(head)))