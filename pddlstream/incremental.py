from pddlstream.fast_downward import parse_domain, run_fast_downward
from pddlstream.problem import Object

from pddlstream.conversion import evaluations_from_init, convert_expression, get_pddl_problem, value_from_obj_plan, \
    obj_from_pddl_plan, substitute_expression, Head, get_prefix, get_args, Evaluation, \
    init_from_evaluations
from pddlstream.instantiation import Instantiator
from pddlstream.stream import parse_stream


def solve_finite(problem, **kwargs):
    init, goal, domain_pddl, stream_pddl, stream_map, constant_map = problem
    evaluations = evaluations_from_init(init)
    goal_expression = convert_expression(goal)
    domain = parse_domain(domain_pddl)
    assert(len(domain.types) == 1)
    assert(not domain.constants)

    problem_pddl = get_pddl_problem(evaluations, goal_expression,
                                    domain_name=domain.name)
    plan_pddl, cost = run_fast_downward(domain_pddl, problem_pddl)
    plan = value_from_obj_plan(obj_from_pddl_plan(plan_pddl))
    return plan, init


def solve_exhaustive(problem, **kwargs):
    init, goal, domain_pddl, stream_pddl, stream_map, constant_map = problem
    domain = parse_domain(domain_pddl)
    assert (len(domain.types) == 1)
    assert (not domain.constants)

    evaluations = evaluations_from_init(init)
    streams = parse_stream(stream_pddl, stream_map)
    print(streams)

    instantiator = Instantiator(evaluations, streams)
    for stream_instance in instantiator:
        output_values_list = stream_instance.next_outputs()
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

    goal_expression = convert_expression(goal)
    problem_pddl = get_pddl_problem(evaluations, goal_expression,
                                    domain_name=domain.name)
    plan, cost = run_fast_downward(domain_pddl, problem_pddl, **kwargs)
    return value_from_obj_plan(obj_from_pddl_plan(plan)), \
           init_from_evaluations(evaluations)