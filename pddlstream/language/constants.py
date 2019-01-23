from __future__ import print_function

from collections import namedtuple

from pddlstream.utils import INF, str_from_object

EQ = '=' # xnor
AND = 'and'
OR = 'or'
NOT = 'not'
EXISTS = 'exists'
FORALL = 'forall'
WHEN = 'when'
IMPLY = 'imply'
MINIMIZE = 'minimize'
MAXIMIZE = 'maximize'
INCREASE = 'increase'
PARAMETER = '?'
TYPE = '-'
OBJECT = 'object'
TOTAL_COST = 'total-cost' # TotalCost

CONNECTIVES = (AND, OR, NOT, IMPLY)
QUANTIFIERS = (FORALL, EXISTS)
OBJECTIVES = (MINIMIZE, MAXIMIZE, INCREASE)
OPERATORS = CONNECTIVES + QUANTIFIERS + (WHEN,) # + OBJECTIVES

FAILED = None
INFEASIBLE = False

PDDLProblem = namedtuple('PDDLProblem', ['domain_pddl', 'constant_map',
                                         'stream_pddl', 'stream_map', 'init', 'goal'])
PDDLAction = namedtuple('PDDLAction', ['name', 'args'])
PDDLStream = namedtuple('PDDLStream', ['name', 'inputs', 'outputs'])
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


def concatenate(*args):
    output = []
    for arg in args:
        output.extend(arg)
    return tuple(output)


def Fact(predicate, args):
    return (predicate,) + tuple(args)


def is_parameter(expression):
    return isinstance(expression, str) and expression.startswith(PARAMETER)


def get_parameter_name(expression):
    if is_parameter(expression):
        return expression[len(PARAMETER):]
    return expression


def is_head(expression):
    return get_prefix(expression) not in OPERATORS

##################################################

def is_plan(plan):
    return not any(plan is status for status in [FAILED, INFEASIBLE])


def get_length(plan):
    return len(plan) if is_plan(plan) else INF


def str_from_action(action):
    name, args = action
    return '{}{}'.format(name, str_from_object(tuple(args)))


def str_from_plan(plan):
    if not is_plan(plan):
        return str(plan)
    return str_from_object(list(map(str_from_action, plan)))


def print_solution(solution):
    plan, cost, evaluations = solution
    solved = is_plan(plan)
    print()
    print('Solved: {}'.format(solved))
    print('Cost: {}'.format(cost))
    print('Length: {}'.format(get_length(plan)))
    print('Evaluations: {}'.format(len(evaluations)))
    if not solved:
        return
    for i, (name, args) in enumerate(plan):
        print('{}) {} {}'.format(i+1, name, ' '.join(map(str_from_object, args))))
    #    print('{}) {}{}'.format(i+1, name, str_from_object(tuple(args))))


def partition_facts(facts):
    functions = []
    negated = []
    positive = []
    for fact in facts:
        prefix = get_prefix(fact)
        if prefix in (EQ, MINIMIZE):
            functions.append(fact[1])
        elif prefix == NOT:
            negated.append(fact[1])
        else:
            positive.append(fact)
    return positive, negated, functions