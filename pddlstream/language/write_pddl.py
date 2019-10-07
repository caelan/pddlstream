import re
import math

from pddlstream.language.constants import AND, OR, OBJECT, TOTAL_COST, TOTAL_TIME, is_cost, get_prefix, \
    CONNECTIVES, QUANTIFIERS
from pddlstream.language.conversion import pddl_from_object, is_atom, is_negated_atom, objects_from_evaluations
from pddlstream.language.object import Object, OptimisticObject

DEFAULT_TYPE = OBJECT # number

def pddl_parameter(param):
    return '{} - {}'.format(param, DEFAULT_TYPE)
    #return param

def pddl_parameters(parameters):
    return ' '.join(map(pddl_parameter, parameters))

def pddl_head(name, args):
    return '({})'.format(' '.join([name] + list(map(pddl_from_object, args))))

def pddl_from_evaluation(evaluation):
    #if evaluation.head.function == TOTAL_COST:
    #    return None
    head = pddl_head(evaluation.head.function, evaluation.head.args)
    if is_atom(evaluation):
        return head
    elif is_negated_atom(evaluation):
        return '(not {})'.format(head)
    #value = int(evaluation.value)
    value = evaluation.value # floats are fine for temporal planners
    #value = int(math.ceil(evaluation.value))
    return '(= {} {})'.format(head, value)

def pddl_functions(predicates):
    return '\n\t\t'.join(sorted(p.pddl() for p in predicates))

def pddl_connective(literals, connective):
    if not literals:
        return '()'
    if len(literals) == 1:
        return literals[0].pddl()
    return '({} {})'.format(connective, ' '.join(l.pddl() for l in literals))

def pddl_conjunction(literals):
    return pddl_connective(literals, AND)

def pddl_disjunction(literals):
    return pddl_connective(literals, OR)

def pddl_from_expression(expression):
    if isinstance(expression, Object) or isinstance(expression, OptimisticObject):
        return pddl_from_object(expression)
    if isinstance(expression, str):
        return expression
    return '({})'.format(' '.join(map(pddl_from_expression, expression)))

##################################################

def pddl_problem(problem, domain, evaluations, goal_expression, objective=None):
    objects = objects_from_evaluations(evaluations)
    s = '(define (problem {})\n' \
           '\t(:domain {})\n' \
           '\t(:objects {})\n' \
           '\t(:init \n\t\t{})\n' \
           '\t(:goal {})'.format(
        problem, domain,
        ' '.join(sorted(map(pddl_from_object, objects))), # map(pddl_parameter,
        '\n\t\t'.join(sorted(filter(lambda p: p is not None,
                                    map(pddl_from_evaluation, evaluations)))),
        pddl_from_expression(goal_expression))
    if objective is not None:
        s += '\n\t(:metric minimize ({}))'.format(objective)
    return s + ')\n'


def get_problem_pddl(evaluations, goal_exp, domain_pddl, temporal=True):
    [domain_name] = re.findall(r'\(domain ([^ ]+)\)', domain_pddl)
    problem_name = domain_name
    objective = TOTAL_TIME if temporal else TOTAL_COST
    problem_pddl = pddl_problem(domain_name, problem_name, evaluations, goal_exp, objective=objective)
    #write_pddl(domain_pddl, problem_pddl, TEMP_DIR)
    return problem_pddl