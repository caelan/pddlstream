from __future__ import print_function

from pddlstream.algorithms.meta import solve_restart, solve
from pddlstream.language.temporal import parse_domain
from pddlstream.utils import INF, Verbose, str_from_object, SEPARATOR
from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.conversion import Certificate, Object, \
    transform_plan_args, value_from_evaluation
from pddlstream.language.constants import PDDLProblem, get_function, get_prefix, print_solution, AND, get_args, And, \
    Solution, Or, is_plan
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, \
    get_action_instances, apply_action, evaluation_from_fd, get_fluents
from pddlstream.algorithms.common import evaluations_from_init


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

def apply_actions(domain, state, plan, unit_costs=False):
    import pddl
    # Goal serialization just assumes the tail of the plan includes an abstract action to achieve each condition
    static_state, _ = partition_facts(domain, state)
    print('Static:', static_state)
    # TODO: might need properties that involve an object that aren't useful yet
    evaluations = evaluations_from_init(state)
    #goal_exp = obj_from_value_expression(goal)
    goal_exp = None
    problem = get_problem(evaluations, goal_exp, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    task.init = set(task.init)
    for instance in get_action_instances(task, transform_plan_args(plan, Object.from_value)):
        apply_action(task.init, instance)
    fluents = get_fluents(domain)
    fluent_state = [value_from_evaluation(evaluation_from_fd(atom))
                    for atom in task.init if isinstance(atom, pddl.Atom) and (atom.predicate in fluents)]
    print('Fluent:', fluent_state)
    state = static_state + fluent_state
    return state

##################################################

def solve_serialized(initial_problem, stream_info={}, unit_costs=False, unit_efforts=False, verbose=True,
                     retain_facts=True, **kwargs):
    # TODO: be careful of CanMove deadends
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = initial_problem
    _, _, domain, streams = parse_problem(
        initial_problem, stream_info, constraints=None, unit_costs=unit_costs, unit_efforts=unit_efforts)
    static_init, _ = partition_facts(domain, init) # might not be able to reprove static_int
    #global_all, global_preimage = [], []
    global_plan = []
    global_cost = 0
    state = list(init)
    goals = serialize_goal(goal)
    # TODO: instead just track how the true init updates
    for i in range(len(goals)):
        # TODO: option in algorithms to pass in existing facts
        for stream in streams:
            stream.reset()
        goal = And(*goals[:i+1])
        print('Goal:', str_from_object(goal))
        # No strict need to reuse streams because generator functions
        #local_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, state, goal)
        local_problem = PDDLProblem(domain_pddl, constant_map, streams, None, state, goal)
        with Verbose(verbose):
            solution = solve_focused(local_problem, stream_info=stream_info, unit_costs=unit_costs,
                                     unit_efforts=unit_efforts, verbose=True, **kwargs)
        print_solution(solution)
        local_plan, local_cost, local_certificate = solution
        if local_plan is None:
            # TODO: replan upon failure
            global_certificate = Certificate(all_facts={}, preimage_facts=None)
            return Solution(None, INF, global_certificate)

        if retain_facts:
            state = local_certificate.all_facts
        else:
            _, fluent_facts = partition_facts(domain, state)
            state = static_init + fluent_facts + local_certificate.preimage_facts # TODO: include functions
        #print('State:', state)
        # TODO: indicate when each fact is used
        # TODO: record failed facts
        global_plan.extend(local_plan)  # TODO: compute preimage of the executed plan
        global_cost += local_cost

        static_state, _ = partition_facts(domain, state)
        #global_all.extend(partition_facts(domain, local_certificate.all_facts)[0])
        #global_preimage.extend(static_state)
        print('Static:', static_state)
        state = apply_actions(domain, state, local_plan, unit_costs=unit_costs)
        print(SEPARATOR)
        #user_input('Continue?')
        # TODO: could also just test the goal here
        # TODO: constrain future plan skeletons

    global_certificate = Certificate(all_facts={}, preimage_facts=None)
    return global_plan, global_cost, global_certificate

##################################################

def solve_deferred(initial_problem, stream_info={}, unit_costs=False, unit_efforts=False, verbose=True,
                   retain_facts=True, **kwargs):
    # TODO: serialize solving deferred problems
    # TODO: can impose plan skeleton constraints as well
    # TODO: investigate case where the first plan skeleton isn't feasible (e.g. due to blockage)
    raise NotImplementedError()

#######################################################

def create_simplified_problem(problem, use_actions=False, use_streams=False, new_goal=None):
    # TODO: check whether goal is a conjunction
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal_parts = problem
    if not use_streams:
        stream_pddl = None
    if new_goal is None:
        new_goal = goal_parts
    domain = parse_domain(domain_pddl) # TODO: Constant map value @base not mentioned in domain :constants
    if not use_actions:
        domain.actions[:] = [] # No actions
    return PDDLProblem(domain, constant_map, stream_pddl, stream_map, init, new_goal)


def test_init_goal(problem, **kwargs):
    problem = create_simplified_problem(problem, use_actions=False, use_streams=False, new_goal=None)
    plan, cost, certificate = solve(problem, **kwargs)
    assert not plan
    is_goal = is_plan(plan)
    return is_goal, certificate

#######################################################

def solve_all_goals(initial_problem, **kwargs):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal_parts = initial_problem
    # TODO(caelan): cleaner specification of goal ordering
    goal_formula = And(*goal_parts)
    print(solve_all_goals.__name__, goal_formula)
    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)
    return solve_restart(problem, **kwargs)


def solve_first_goal(initial_problem, **kwargs):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal_parts = initial_problem

    achieved_parts = []
    unachieved_parts = []
    for task_part in goal_parts:
        # TODO: store any stream evaluations (tests) and limit complexity
        problem = create_simplified_problem(initial_problem, new_goal=task_part)
        solution = solve_restart(problem, **kwargs)
        plan, _, _ = solution
        if plan is None:
            unachieved_parts.append(task_part)
        elif len(plan) == 0:
            achieved_parts.append(task_part)
        else:
            raise RuntimeError(task_part)
    # print(achieved_parts)
    # print(unachieved_parts)

    # TODO: reset to initial state if not achieved
    goal_formula = And(*achieved_parts)
    if unachieved_parts:
        goal_formula = And(Or(*unachieved_parts), goal_formula)
    print(solve_all_goals.__name__, goal_formula)
    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)
    return solve_restart(problem, **kwargs)


def solve_next_goal(initial_problem, serialize=True, **kwargs):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal_parts = initial_problem
    # TODO: store serialization state to ensure progress is made
    # goal_formula = And(Or(*task_parts), *reset_parts) # TODO: still possibly having the disjunctive goal issue
    indices = list(range(0, len(goal_parts), 1)) if serialize else [len(goal_parts)]
    for i in indices:
        goal_parts = goal_parts[:i+1]
        goal_formula = And(*goal_parts)
        problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)
        print('Goal {}: {}'.format(i, goal_formula))

        # TODO: reset streams?
        solution = solve_restart(problem, **kwargs)
        # TODO: detect which goals were achieved
        plan, _, _ = solution
        if plan is None:
            return solution
        if (i == len(indices) - 1) or (len(plan) >= 1):
            return solution
    return Solution(plan=[], cost=0, certificate=[])
