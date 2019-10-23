from pddlstream.utils import INF, elapsed_time, find
from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.common import SolutionStore
from pddlstream.language.fluent import compile_fluent_streams
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.conversion import Object, \
    obj_from_value_expression, fact_from_evaluation, evaluation_from_fact, value_from_obj_expression
from pddlstream.language.constants import PDDLProblem
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, \
    get_action_instances, apply_action, parse_domain, fact_from_fd, evaluation_from_fd, conditions_hold, has_costs


# TODO: hierarchical action specification
# TODO: make a generate that saves all the stream outputs (that way don't need to reload)
# TODO: blackbox function that changes the state

def solve_execution(initial_problem, stream_info={}, resuse_state=True,
                    replan_frequency=None,

                    action_info={}, synthesizers=[],
                  max_time=INF, max_cost=INF, unit_costs=False,
                  effort_weight=None, eager_layers=1,
                  visualize=False, verbose=True, postprocess=False, **search_kwargs):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = initial_problem
    #initial_evaluations, goal_expression, domain, externals = parse_problem(initial_problem, stream_info)
    domain = parse_domain(domain_pddl)
    #compile_fluent_streams(domain, externals)
    # TODO: convert generators

    state = list(init) # TODO: extract only fluents
    while True:
        local_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, state, goal)
        value_plan, _, new_init = solve_focused(local_problem, unit_costs=unit_costs, stream_info=stream_info)
        if value_plan is None:
            break
        if resuse_state:
            state = new_init
        if replan_frequency is not None:
            value_plan = value_plan[:replan_frequency]
        obj_plan = [(name, tuple(map(Object.from_value, args))) for name, args in value_plan]

        # TODO: compute and keep preimage of the executed plan


        # TODO: is there a way to operate on the hashable objects w/o doing things
        evaluations = list(map(evaluation_from_fact, map(obj_from_value_expression, state)))
        problem = get_problem(evaluations, obj_from_value_expression(goal), domain, unit_costs)
        task = task_from_domain_problem(domain, problem)
        fd_state = set(task.init)
        for instance in get_action_instances(task, obj_plan):
            apply_action(fd_state, instance)
        print(fd_state)

        print(list(map(fact_from_evaluation, map(evaluation_from_fd, fd_state))))

        #state = list(map(value_from_obj_expression, map(fact_from_fd, fd_state)))
        state = list(map(value_from_obj_expression,
                         map(fact_from_evaluation, map(evaluation_from_fd, fd_state))))

        # TODO: replan on failure to execute?
        print(problem.goal)


        # TODO: has negative and equality in it

        print(state)
        raw_input('Continue?')

        # TODO: option in algorithms to load all existing facts

# TODO: deterministic hierarchical planner
# TODO: start using the previous file on my old Mac
# Streams can represent refinement

# TODO: two approaches
# 1) Compute plan suffix preimage
# 2) Constrain plan suffix

# TODO: start from the original evaluations or use the stream plan preimage
# TODO: only use streams in the states between the two actions
# TODO: apply hierarchical planning to restrict the set of streams that considered on each subproblem
# TODO: plan up to first action that only has one