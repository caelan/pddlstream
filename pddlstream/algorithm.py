from pddlstream.conversion import evaluations_from_init, obj_from_value_expression, obj_from_pddl_plan, \
    values_from_objects, evaluation_from_fact
from pddlstream.fast_downward import parse_domain, get_problem, task_from_domain_problem, \
    solve_from_task, instantiate_task
from pddlstream.stream import parse_stream, StreamResult
from pddlstream.utils import str_from_tuple


def parse_problem(problem):
    init, goal, domain_pddl, stream_pddl, stream_map, constant_map = problem
    evaluations = set(evaluations_from_init(init))
    goal_expression = obj_from_value_expression(goal)
    domain = parse_domain(domain_pddl)
    assert(len(domain.types) == 1)
    assert(not domain.constants)
    streams = parse_stream(stream_pddl, stream_map)
    return evaluations, goal_expression, domain, streams


def solve_finite(evaluations, goal_expression, domain, **kwargs):
    problem = get_problem(evaluations, goal_expression, domain)
    task = task_from_domain_problem(domain, problem)
    instantiate_task(task)
    plan_pddl, cost = solve_from_task(task, **kwargs)
    return obj_from_pddl_plan(plan_pddl), cost


def print_output_values_list(stream_instance, output_values_list):
    print('{}:{}->[{}]'.format(stream_instance.stream.name,
                               str_from_tuple(values_from_objects(stream_instance.input_values)),
                               ', '.join(map(str_from_tuple, map(values_from_objects, output_values_list)))))


def process_stream_queue(instantiator, evaluations, next_values_fn, revisit=True, verbose=True):
    stream_instance = instantiator.stream_queue.popleft()
    output_values_list = next_values_fn(stream_instance)
    if verbose:
        print_output_values_list(stream_instance, output_values_list)
    stream_results = []
    for output_values in output_values_list:
        stream_results.append(StreamResult(stream_instance, output_values))
        for fact in stream_results[-1].get_certified():
            evaluation = evaluation_from_fact(fact)
            instantiator.add_atom(evaluation)
            if evaluations is not None:
                evaluations.add(evaluation)
    if revisit and not stream_instance.enumerated:
        instantiator.stream_queue.append(stream_instance)
    return stream_results