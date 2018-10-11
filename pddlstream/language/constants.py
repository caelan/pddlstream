from collections import namedtuple

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
PARAMETER = '?'
TYPE = '-'
CONNECTIVES = (AND, OR, NOT, IMPLY)
QUANTIFIERS = (FORALL, EXISTS)
OPERATORS = CONNECTIVES + QUANTIFIERS + (WHEN,)
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