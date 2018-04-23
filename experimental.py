def brainstorm():
    #print(DOMAIN_PDDL)
    #print(PROBLEM_PDDL)
    #print(parse_lisp(DOMAIN_PDDL.encode('latin-1')))
    #print(parse_lisp(DOMAIN_PDDL.encode('ISO-8859-1')))
    #print(parse_lisp(PROBLEM_PDDL))
    #print(parse_domain(DOMAIN_PDDL))
    #print(parse_domain(parse_lisp(DOMAIN_PDDL)))
    #return

    domain_path, problem_path = write_pddl(DOMAIN_PDDL, PROBLEM_PDDL)
    print(parse_domain(domain_path))

    task = translate_paths(domain_path, problem_path) # TODO: might need to make these wrt temp
    print(task.objects)
    task.dump()
    #print(task.__dict__)
    #return
    import sys
    # TODO: could even directly convert and mutate the task
    return


"""
# regex = r'(\(\w+(?:\s\w+)*\))'
# print(re.findall(regex, solution))
ground_from_name = defaultdict(list)
for action in ground_task.actions:
    ground_from_name[action.name].append(action)
action_plan = []
for name in solution.split('\n')[:-2]:
    assert (len(ground_from_name[name]) == 1)
    # TODO: later select a particular ground action that satisfies the conditions
    action_plan.append(ground_from_name[name][0])
"""

#def solve_finite(evaluations, goal_expression, domain, domain_pddl, **kwargs):
#    problem_pddl = get_pddl_problem(evaluations, goal_expression, domain_name=domain.name)
#    plan_pddl, cost = solve_from_pddl(domain_pddl, problem_pddl, **kwargs)
#    return obj_from_pddl_plan(plan_pddl), cost

#CONSTANTS = ':constants'
#OBJECTS = ':objects'

#def pddl_from_objects(objects):
#    return ' '.join(sorted(map(pddl_from_object, objects)))

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
