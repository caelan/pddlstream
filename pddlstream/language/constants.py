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
TOTAL_TIME = 'total-time'

CONNECTIVES = (AND, OR, NOT, IMPLY)
QUANTIFIERS = (FORALL, EXISTS)
OBJECTIVES = (MINIMIZE, MAXIMIZE, INCREASE)
OPERATORS = CONNECTIVES + QUANTIFIERS + (WHEN,) # + OBJECTIVES

FAILED = None
INFEASIBLE = False

# TODO: rename PDDLProblem
PDDLProblem = namedtuple('PDDLProblem', ['domain_pddl', 'constant_map',
                                         'stream_pddl', 'stream_map', 'init', 'goal'])
Solution = namedtuple('Solution', ['plan', 'cost', 'certificate'])
Certificate = namedtuple('Certificate', ['all_facts', 'preimage_facts'])

OptPlan = namedtuple('OptPlan', ['action_plan', 'preimage_facts'])
# TODO: stream and axiom plans
# TODO: annotate which step each fact is first used via layer

Assignment =  namedtuple('Assignment', ['args'])
Action = namedtuple('Action', ['name', 'args'])
DurativeAction = namedtuple('DurativeAction', ['name', 'args', 'start', 'duration'])
StreamAction = namedtuple('StreamAction', ['name', 'inputs', 'outputs'])

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


def Imply(expression1, expression2):
    return (IMPLY, expression1, expression2)


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
    name, args = action[:2]
    return '{}{}'.format(name, str_from_object(tuple(args)))


def str_from_plan(plan):
    if not is_plan(plan):
        return str(plan)
    return str_from_object(list(map(str_from_action, plan)))


def print_solution(solution):
    plan, cost, evaluations = solution
    solved = is_plan(plan)
    if plan is None:
        num_deferred = 0
    else:
        num_deferred = len([action for action in plan if isinstance(action, StreamAction)])
    print()
    print('Solved: {}'.format(solved))
    print('Cost: {}'.format(cost))
    print('Length: {}'.format(get_length(plan) - num_deferred))
    print('Deferred: {}'.format(num_deferred))
    print('Evaluations: {}'.format(len(evaluations)))
    if not solved:
        return
    step = 1
    for action in plan:
        if isinstance(action, DurativeAction):
            name, args, start, duration = action
            print('{:.2f} - {:.2f}) {} {}'.format(start, start+duration, name,
                                                  ' '.join(map(str_from_object, args))))
        elif isinstance(action, Action):
            name, args = action
            print('{:2}) {} {}'.format(step, name, ' '.join(map(str_from_object, args))))
            #print('{}) {}{}'.format(step, name, str_from_object(tuple(args))))
            step += 1
        elif isinstance(action, StreamAction):
            name, inputs, outputs = action
            print('    {}({})->({})'.format(name, ', '.join(map(str_from_object, inputs)),
                                          ', '.join(map(str_from_object, outputs))))
        else:
            raise NotImplementedError(action)


def get_function(term):
    if get_prefix(term) in (EQ, MINIMIZE, NOT):
        return term[1]
    return term


def partition_facts(facts):
    functions = []
    negated = []
    positive = []
    for fact in facts:
        prefix = get_prefix(fact)
        func = get_function(fact)
        if prefix in (EQ, MINIMIZE):
            functions.append(func)
        elif prefix == NOT:
            negated.append(func)
        else:
            positive.append(func)
    return positive, negated, functions


def is_cost(o):
    return get_prefix(o) == MINIMIZE


def get_costs(objectives):
    return [o for o in objectives if is_cost(o)]


def get_constraints(objectives):
    return [o for o in objectives if not is_cost(o)]
