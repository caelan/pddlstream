from pddlstream.conversion import evaluations_from_init, obj_from_value_expression, obj_from_pddl_plan, \
    values_from_objects, evaluation_from_fact, fact_from_evaluation
from pddlstream.fast_downward import parse_domain, get_problem, task_from_domain_problem, \
    solve_from_task, instantiate_task
from pddlstream.stream import parse_stream_pddl, StreamResult
from pddlstream.utils import str_from_tuple


def parse_problem(problem):
    init, goal, domain_pddl, stream_pddl, stream_map, constant_map = problem
    evaluations = set(evaluations_from_init(init))
    goal_expression = obj_from_value_expression(goal)
    domain = parse_domain(domain_pddl)
    if len(domain.types) != 1:
        raise NotImplementedError('Types are not currently supported')
    if domain.constants:
        raise NotImplementedError('Constants are not currently supported')
    if constant_map:
        raise NotImplementedError('A constant map is not currently supported')
    # TODO: instantiate to rename here
    streams = parse_stream_pddl(stream_pddl, stream_map)
    return evaluations, goal_expression, domain, streams


def solve_finite(evaluations, goal_expression, domain, unit_costs=True, **kwargs):
    problem = get_problem(evaluations, goal_expression, domain, unit_costs=unit_costs)
    task = task_from_domain_problem(domain, problem)
    plan_pddl, cost = solve_from_task(task, **kwargs)
    return obj_from_pddl_plan(plan_pddl), cost

def process_stream_queue(instantiator, evaluations, optimistic=False, verbose=True):
    stream_instance = instantiator.stream_queue.popleft()
    output_values_list = list(stream_instance.next_outputs() if not optimistic else
                              stream_instance.next_optimistic())
    if verbose:
        # TODO: move verbose into next_values_fn?
        stream_instance.dump_output_list(output_values_list)
    stream_results = []
    for output_values in output_values_list:
        stream_results.append(StreamResult(stream_instance, output_values))
        for fact in stream_results[-1].get_certified():
            evaluation = evaluation_from_fact(fact)
            instantiator.add_atom(evaluation)
            if evaluations is not None:
                evaluations.add(evaluation)
    if not optimistic and not stream_instance.enumerated:
        instantiator.stream_queue.append(stream_instance)
    return stream_results


def get_partial_orders(stream_plan):
    # TODO: only show the first atom achieved?
    partial_orders = set()
    for i, stream1 in enumerate(stream_plan):
        for stream2 in stream_plan[i+1:]: # Prevents circular
            if set(stream1.get_certified()) & set(stream2.get_certified()):
                partial_orders.add((stream1, stream2))
    return partial_orders


def get_optimistic_constraints(evaluations, stream_plan):
    # TODO: approximates needed facts using produced ones
    constraints = set()
    for stream in stream_plan:
        constraints.update(stream.get_certified())
    return constraints - set(map(fact_from_evaluation, evaluations))