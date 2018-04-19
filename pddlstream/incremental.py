import time

from pddlstream.conversion import evaluations_from_init, convert_expression, get_pddl_problem, value_from_obj_plan, \
    obj_from_pddl_plan, substitute_expression, Head, get_prefix, get_args, Evaluation, \
    init_from_evaluations
from pddlstream.fast_downward import parse_domain, run_fast_downward
from pddlstream.instantiation import Instantiator
from pddlstream.problem import Object
from pddlstream.stream import parse_stream
from pddlstream.utils import INF, elapsed_time


def parse_problem(problem):
    init, goal, domain_pddl, stream_pddl, stream_map, constant_map = problem
    evaluations = evaluations_from_init(init)
    goal_expression = convert_expression(goal)
    domain = parse_domain(domain_pddl)
    assert(len(domain.types) == 1)
    assert(not domain.constants)
    streams = parse_stream(stream_pddl, stream_map)

    return evaluations, goal_expression, domain, streams

def solve_finite(evaluations, goal_expression, domain, domain_pddl, **kwargs):
    problem_pddl = get_pddl_problem(evaluations, goal_expression, domain_name=domain.name)
    plan_pddl, cost = run_fast_downward(domain_pddl, problem_pddl, **kwargs)
    return obj_from_pddl_plan(plan_pddl), cost

def revert_solution(plan, evaluations):
    return value_from_obj_plan(plan), init_from_evaluations(evaluations)

def solve_no_streams(problem, **kwargs):
    domain_pddl = problem[2]
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    plan, cost = solve_finite(evaluations, goal_expression, domain, domain_pddl, **kwargs)
    return revert_solution(plan, evaluations)


def process_stream_queue(instantiator, evaluations, verbose=True):
    stream_instance = instantiator.stream_queue.popleft()
    output_values_list = stream_instance.next_outputs()
    if verbose:
        print('{}:{}->{}'.format(stream_instance.stream.name,
                                 stream_instance.input_values, output_values_list))
    for output_values in output_values_list:
        converted_values = map(Object.from_value, output_values)
        certified_atoms = substitute_expression(stream_instance.stream.certified,
                                                stream_instance.mapping(converted_values))
        for atom in certified_atoms:
            head = Head(get_prefix(atom), get_args(atom))
            # TODO: use an existing method here?
            evaluation = Evaluation(head, True)
            instantiator.add_atom(head)
            evaluations.append(evaluation)
    if not stream_instance.enumerated:
        instantiator.stream_queue.append(stream_instance)

def solve_exhaustive(problem, max_time=INF, verbose=True, **kwargs):
    start_time = time.time()
    domain_pddl = problem[2]
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    instantiator = Instantiator(evaluations, streams)
    while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
        process_stream_queue(instantiator, evaluations, verbose=verbose)
    plan, cost = solve_finite(evaluations, goal_expression, domain, domain_pddl, **kwargs)
    return revert_solution(plan, evaluations)

def solve_incremental(problem, max_time=INF, max_cost=INF, verbose=True, **kwargs):
    start_time = time.time()
    num_iterations = 0
    domain_pddl = problem[2]
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    instantiator = Instantiator(evaluations, streams)
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Elapsed time: {:.3f}'.format(
            num_iterations, len(evaluations), elapsed_time(start_time)))
        plan, cost = solve_finite(evaluations, goal_expression, domain, domain_pddl, **kwargs)
        if cost < max_cost:
            return revert_solution(plan, evaluations)
        if not instantiator.stream_queue:
            break
        for _ in range(len(instantiator.stream_queue)):
            if max_time <= elapsed_time(start_time):
                break
            process_stream_queue(instantiator, evaluations, verbose=verbose)
    return revert_solution(None, evaluations)
