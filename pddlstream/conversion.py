from __future__ import print_function

from collections import namedtuple

from pddlstream.object import Object, OptimisticObject

EQ = '=' # xnor
AND = 'and'
OR = 'or'
NOT = 'not'
EXISTS = 'exists'
FORALL = 'forall'
WHEN = 'when'
IMPLIES = 'implies'
CONNECTIVES = (EQ, AND, OR, NOT)
QUANTIFIERS = (FORALL, EXISTS)
OPERATORS = CONNECTIVES + QUANTIFIERS

TOTAL_COST = 'total-cost' # TotalCost
DOMAIN_NAME = 'pddlstream'
PROBLEM_NAME = DOMAIN_NAME

Problem = namedtuple('Problem', ['init', 'goal', 'domain', 'streams', 'constants'])
Head = namedtuple('Head', ['function', 'args'])
Evaluation = namedtuple('Evaluation', ['head', 'value'])
Atom = lambda head: Evaluation(head, True)
NegatedAtom = lambda head: Evaluation(head, False)

#CONSTANTS = ':constants'
#OBJECTS = ':objects'

##################################################

#def partition(array, i):
#    return array[:i], array[i:]

def convert_head(atom):
    name, args = atom[0], atom[1:]
    return tuple([name.lower()] + map(Object.from_value, args))

convert_atom = convert_head

def get_prefix(expression):
    return expression[0]

def get_args(head):
    return head[1:]

def is_head(expression):
    return get_prefix(expression) not in OPERATORS


def substitute_expression(parent, mapping):
    if isinstance(parent, str) or isinstance(parent, Object):
        return mapping.get(parent, parent)
    return tuple(substitute_expression(child, mapping) for child in parent)

def convert_expression(expression):
    prefix = expression[0]
    if prefix in CONNECTIVES:
        children = expression[1:]
        return tuple([prefix] + map(convert_expression, children))
    elif prefix in QUANTIFIERS:
        assert(len(expression) == 3)
        parameters = expression[1]
        child = expression[2]
        return prefix, parameters, convert_expression(child)
    return convert_head(expression)

def list_from_conjunction(expression):
    if not expression:
        return []
    prefix = get_prefix(expression)
    assert(prefix not in (QUANTIFIERS + (NOT, OR, EQ)))
    if prefix == AND:
        children = []
        for child in expression[1:]:
            children += list_from_conjunction(child)
        return children
    return [tuple(expression)]

##################################################

def pddl_from_object(obj):
    return obj.pddl

def pddl_from_objects(objects):
    return ' '.join(sorted(map(pddl_from_object, objects)))

def pddl_list_from_expression(tree):
    if isinstance(tree, Object) or isinstance(tree, OptimisticObject):
        return pddl_from_object(tree)
    if isinstance(tree, str):
        return tree
    return tuple(map(pddl_list_from_expression, tree))

def pddl_from_expression(tree):
    if isinstance(tree, str):
        return tree
    return '({})'.format(' '.join(map(pddl_from_expression, tree)))


def pddl_from_evaluation(evaluation):
    head = (evaluation.head.function,) + tuple(evaluation.head.args)
    if is_atom(evaluation):
        return pddl_from_expression(pddl_list_from_expression(head))
    if is_negated_atom(evaluation):
        return None
    expression = (EQ, head, str(evaluation.value))
    return pddl_from_expression(pddl_list_from_expression(expression))


def pddl_from_evaluations(evaluations):
    return '\n\t\t'.join(sorted(filter(lambda s: s is not None,
                                       map(pddl_from_evaluation, evaluations))))


def get_pddl_problem(init_evaluations, goal_expression,
                     problem_name=DOMAIN_NAME, domain_name=PROBLEM_NAME,
                     objective=(TOTAL_COST,)):
    # TODO: mako or some more elegant way of creating this
    objects = objects_from_evaluations(init_evaluations)
    s = '(define (problem {})\n' \
           '\t(:domain {})\n' \
           '\t(:objects {})\n' \
           '\t(:init {})\n' \
           '\t(:goal {})'.format(problem_name, domain_name,
                                 pddl_from_objects(objects),
                                 pddl_from_evaluations(init_evaluations),
                                 pddl_from_expression(pddl_list_from_expression(goal_expression)))
    #objective = None # minimizes length
    if objective is not None:
        s += '\n\t(:metric minimize {})'.format(
            pddl_from_expression(pddl_list_from_expression(objective)))
    return s + ')\n'

##################################################

def values_from_objects(objects):
    return tuple(obj.value for obj in objects)

def objects_from_values(values):
    return tuple(map(Object.from_value, values))

def is_atom(evaluation):
    return evaluation.value is True

def is_negated_atom(evaluation):
    return evaluation.value is False

def atoms_from_evaluations(evaluations):
    return map(lambda e: e.head, filter(is_atom, evaluations))

def objects_from_evaluations(evaluations):
    # TODO: assumes object predicates
    objects = set()
    for evaluation in evaluations:
        objects.update(evaluation.head.args)
    return objects

##################################################

def evaluations_from_init(init):
    evaluations = []
    for fact in init:
        prefix = get_prefix(fact)
        if prefix == EQ:
            head, value = fact[1:]
        elif prefix == NOT:
            head = fact[1]
            value = False
        else:
            head = fact
            value = True
        func, args = get_prefix(head), get_args(head)
        head = Head(func.lower(), tuple(map(Object.from_value, args)))
        evaluations.append(Evaluation(head, value))
    return evaluations

def init_from_evaluations(evaluations):
    init = []
    for evaluation in evaluations:
        head = (evaluation.head.function, values_from_objects(evaluation.head.args))
        if is_atom(evaluation):
            init.append(head)
        elif is_negated_atom(evaluation):
            init.append((NOT, head))
        else:
            init.append((EQ, head, evaluation.value))
    return init

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

# TODO: would be better just to rename everything at the start. Still need to handle constants
def obj_from_pddl_plan(pddl_plan):
    if pddl_plan is None:
        return None
    return [(action, map(obj_from_pddl, args)) for action, args in pddl_plan]


def value_from_obj_plan(obj_plan):
    if obj_plan is None:
        return None
    return [(action, values_from_objects(args)) for action, args in obj_plan]
