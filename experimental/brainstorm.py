# Two components here:
# - Axioms
# - Negative Axioms
# - Negative bounds on axioms
#
# Precondition kept if in fluent_facts
# Dropped if in init_facts (static facts)
# Error if not in init_facts (not possible from binding)

"""
 with Verbose(True):
     model = build_model.compute_model(pddl_to_prolog.translate(task)) # Need to instantiate as normal
     #fluent_facts = instantiate.get_fluent_facts(task, model)
     #init_facts = set(task.init)
     real_init = get_init(evaluations)
     opt_facts = set(task.init) - set(real_init)
     task.init = real_init
     fluent_facts = instantiate.get_fluent_facts(task, model) | opt_facts
     print(fluent_facts)

     init_facts = set(real_init) # Shouldn't matter

 axiom_model = filter(lambda a: isinstance(a.predicate, pddl.Axiom), model)
 print(len(axiom_model)) # 6 x 10 x 10
 instantiated_axioms = instantiate_axioms(axiom_model, fluent_facts, init_facts)
 print(len(instantiated_axioms), instantiated_axioms[:3])

 axioms, axiom_init, axiom_layer_dict = axiom_rules.handle_axioms(
     ground_task.actions, instantiated_axioms, ground_task.goal_list)

 # TODO: can even build once, instantiate in each state, and then invert

 print(len(axioms), axioms[:3])
 # TODO: can instantiate in each state without axioms as well (multiple small model builds)
 """

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
print(plan_cost(action_plan))
state = set(task.init)
axiom_plan = []
# TODO: remove conditional effects
for action in action_plan:
    axiom_plan.append([])
    assert(is_applicable(state, action))
    apply_action(state, action)
    print(state)
"""

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
