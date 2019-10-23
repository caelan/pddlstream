from __future__ import print_function

from pddlstream.utils import INF, elapsed_time, find, user_input, Verbose, str_from_object
from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.conversion import Certificate, Object, \
    obj_from_value_expression, fact_from_evaluation, evaluation_from_fact, value_from_obj_expression, \
    transform_plan_args, value_from_evaluation
from pddlstream.language.constants import PDDLProblem, Action, get_function, get_prefix, print_solution, AND, get_args, And
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, \
    get_action_instances, apply_action, fact_from_fd, evaluation_from_fd, \
    conditions_hold, has_costs, get_fluents
from pddlstream.algorithms.common import evaluations_from_init

import pddl

SEPARATOR = '\n' + 80*'-'  + '\n'

def serialize_goal(goal):
    if get_prefix(goal) == AND:
        return get_args(goal)
    return [goal]

def partition_facts(domain, facts):
    fluents = get_fluents(domain)
    static_facts = []
    fluent_facts = []
    for fact in facts:
        if get_prefix(get_function(fact)).lower() in fluents:
            fluent_facts.append(fact)
        else:
            static_facts.append(fact)
    return static_facts, fluent_facts

##################################################

def solve_serialized(initial_problem, stream_info={}, unit_costs=False, unit_efforts=False, verbose=True, **kwargs):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = initial_problem
    _, _, domain, streams = parse_problem(
        initial_problem, stream_info, constraints=None, unit_costs=unit_costs, unit_efforts=unit_efforts)
    static_init, _ = partition_facts(domain, init) # might not be able to reprove static_int
    #global_all, global_preimage = [], []
    global_plan = []
    global_cost = 0
    state = list(init)
    goals = serialize_goal(goal)
    for i in range(len(goals)):
        # TODO: option in algorithms to pass in existing facts
        for stream in streams:
            stream.reset()
        goal = And(*goals[:i+1])
        print('Goal:', str_from_object(goal))
        # No strict need to reuse streams because generator functions
        local_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, state, goal)
        #local_problem = PDDLProblem(domain_pddl, constant_map, streams, None, state, goal)
        with Verbose(verbose):
            solution = solve_focused(local_problem, stream_info=stream_info, unit_costs=unit_costs,
                                     unit_efforts=unit_efforts, verbose=True, **kwargs)
        print_solution(solution)
        local_plan, local_cost, local_certificate = solution
        if local_plan is None:
            # TODO: replan upon failure
            global_certificate = Certificate(all_facts={}, preimage_facts=None)
            return None, INF, global_certificate
        #if not local_plan:
        #    break
        _, fluent_facts = partition_facts(domain, state)
        #state = local_certificate.all_facts # TODO: include functions
        state = fluent_facts + local_certificate.preimage_facts + static_init
        #print('State:', state)

        # TODO: record failed facts
        #local_plan = local_plan[:1]
        global_plan.extend(local_plan)  # TODO: compute preimage of the executed plan
        global_cost += local_cost

        static_state, _ = partition_facts(domain, state)
        #global_all.extend(partition_facts(domain, local_certificate.all_facts)[0])
        #global_preimage.extend(static_state)
        print('Static:', static_state)

        evaluations = evaluations_from_init(state)
        goal_exp = obj_from_value_expression(goal)
        problem = get_problem(evaluations, goal_exp, domain, unit_costs)
        task = task_from_domain_problem(domain, problem)
        task.init = set(task.init)
        for instance in get_action_instances(task, transform_plan_args(local_plan, Object.from_value)):
            apply_action(task.init, instance)
        fluents = get_fluents(domain)
        fluent_state = [value_from_evaluation(evaluation_from_fd(atom))
                        for atom in task.init if isinstance(atom, pddl.Atom) and atom.predicate in fluents]
        print('Fluent:', fluent_state)
        state = static_state + fluent_state
        print(SEPARATOR)
        user_input('Continue?')
        # TODO: could also just test the goal here
        # TODO: constrain future plan skeletons

    global_certificate = Certificate(all_facts={}, preimage_facts=None)
    return global_plan, global_cost, global_certificate
