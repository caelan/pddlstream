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

# Writing PDDL
# https://github.mit.edu/search?l=Python&p=2&q=user%3Acaelan+user%3Acaelan+%22%3Arequirements%22&type=Code
# https://github.mit.edu/caelan/ss/blob/4673395c011c2d2fa877234d10b3d1d5bf584548/ss/to_pddl.py
# https://github.mit.edu/caelan/compact-planning/blob/6ae60a01f69eec93108773955e0463ea05f1e458/drp_pddl_2/dynamic_problem.py
# https://github.mit.edu/caelan/stripstream/blob/c8c6cd1d6bd5e2e8e31cd5603e28a8e0d7bb2cdc/stripstream/algorithms/universe.py
# https://github.mit.edu/caelan/ss/blob/4673395c011c2d2fa877234d10b3d1d5bf584548/ss/to_pddl.py

# TODO: finish this
# def pddl_domain(domain, constants, predicates, functions, actions, axioms):
#     # Need types for tpshe
#     # Need to declare constants here that are used in actions
#     return '(define (domain {})\n' \
#            '\t(:requirements :typing)\n' \
#            '\t(:types {})\n' \
#            '\t(:constants {})\n' \
#            '\t(:predicates {})\n' \
#            '\t(:functions {})\n' \
#            '{})\n'.format(domain, DEFAULT_TYPE,
#                           pddl_parameter(' '.join(sorted(constants))),
#                           pddl_functions(predicates),
#                           pddl_functions(functions),
#                           '\n'.join(list(pddl_actions(actions)) +
#                                     list(pddl_axioms(axioms))))

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