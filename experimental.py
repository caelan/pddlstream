
#CONSTANTS = ':constants'
#OBJECTS = ':objects'

#def partition(array, i):
#    return array[:i], array[i:]

# def pddl_from_expression(tree):
#     if isinstance(tree, str):
#         return tree
#     return '({})'.format(' '.join(map(pddl_from_expression, tree)))
#
#
# def pddl_from_evaluation(evaluation):
#     head = (evaluation.head.function,) + tuple(evaluation.head.args)
#     if is_atom(evaluation):
#         return pddl_from_expression(pddl_list_from_expression(head))
#     if is_negated_atom(evaluation):
#         return None
#     expression = (EQ, head, str(evaluation.value))
#     return pddl_from_expression(pddl_list_from_expression(expression))
#
#
# def pddl_from_evaluations(evaluations):
#     return '\n\t\t'.join(sorted(filter(lambda s: s is not None,
#                                        map(pddl_from_evaluation, evaluations))))
#
#
# def get_pddl_problem(init_evaluations, goal_expression,
#                      problem_name=DOMAIN_NAME, domain_name=PROBLEM_NAME,
#                      objective=(TOTAL_COST,)):
#     # TODO: mako or some more elegant way of creating this
#     objects = objects_from_evaluations(init_evaluations)
#     s = '(define (problem {})\n' \
#            '\t(:domain {})\n' \
#            '\t(:objects {})\n' \
#            '\t(:init {})\n' \
#            '\t(:goal {})'.format(problem_name, domain_name,
#                                  pddl_from_objects(objects),
#                                  pddl_from_evaluations(init_evaluations),
#                                  pddl_from_expression(pddl_list_from_expression(goal_expression)))
#     #objective = None # minimizes length
#     if objective is not None:
#         s += '\n\t(:metric minimize {})'.format(
#             pddl_from_expression(pddl_list_from_expression(objective)))
#     return s + ')\n'
