from pddlstream.conversion import evaluations_from_init, obj_from_value_expression, obj_from_pddl_plan, \
    fact_from_evaluation
from pddlstream.fast_downward import parse_domain, get_problem, task_from_domain_problem, \
    solve_from_task
from pddlstream.object import Object
from pddlstream.stream import parse_stream_pddl


def parse_constants(domain, constant_map):
    for constant in domain.constants:
        if constant.name.startswith(Object._prefix):
            # TODO: remap names
            raise NotImplementedError('Constants are not currently allowed to begin with {}'.format(Object._prefix))
        if constant.name not in constant_map:
            raise ValueError('Undefined constant {}'.format(constant.name))
        obj = Object(constant_map[constant.name], name=constant.name)
        # TODO: add object predicate
    del domain.constants[:] # So not set twice

def parse_problem(problem):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = problem
    domain = parse_domain(domain_pddl)
    if len(domain.types) != 1:
        raise NotImplementedError('Types are not currently supported')
    parse_constants(domain, constant_map)
    streams = parse_stream_pddl(stream_pddl, stream_map)
    evaluations = set(evaluations_from_init(init))
    goal_expression = obj_from_value_expression(goal)
    return evaluations, goal_expression, domain, streams


def solve_finite(evaluations, goal_expression, domain, unit_costs=True, **kwargs):
    problem = get_problem(evaluations, goal_expression, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    plan_pddl, cost = solve_from_task(task, **kwargs)
    return obj_from_pddl_plan(plan_pddl), cost


def get_optimistic_constraints(evaluations, stream_plan):
    # TODO: approximates needed facts using produced ones
    constraints = set()
    for stream in stream_plan:
        constraints.update(stream.get_certified())
    return constraints - set(map(fact_from_evaluation, evaluations))