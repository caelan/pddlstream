from pddlstream.conversion import get_pddl_problem, value_from_obj_plan, \
    obj_from_pddl_plan, substitute_expression, Head, get_prefix, get_args, Evaluation, \
    init_from_evaluations, evaluations_from_init, convert_expression, values_from_objects, \
    objects_from_values, TOTAL_COST, pddl_from_object, Atom
from pddlstream.fast_downward import solve_from_pddl, parse_domain, Domain, OBJECT
from pddlstream.instantiation import Instantiator
from pddlstream.object import Object
from pddlstream.stream import parse_stream, StreamInstance
from pddlstream.utils import INF, elapsed_time
import time

from pddlstream.incremental import parse_problem, solve_finite2, revert_solution, process_stream_queue

# ('move', [<TypedObject ?q1: object>, <TypedObject ?q2: object>],
# <pddl.conditions.Conjunction object at 0x10c3369d0>,
# [<pddl.effects.Effect object at 0x10c336b50>, <pddl.effects.Effect object at 0x10c336bd0>],
# None)

def stream_action(stream_result, name, effect_scale=1):
    #from pddl_parser.parsing_functions import parse_action
    import pddl

    effort = effect_scale

    parameters = []
    preconditions = []
    for fact in stream_result.stream_instance.get_domain():
        predicate = get_prefix(fact)
        args = map(pddl_from_object, get_args(fact))
        preconditions.append(pddl.Atom(predicate, args))
    precondition = pddl.Conjunction(preconditions)

    effects = []
    for fact in stream_result.get_certified():
        predicate = get_prefix(fact)
        args = map(pddl_from_object, get_args(fact))
        effects.append(pddl.Effect(parameters=[], condition=pddl.Truth(),
                                   literal=pddl.Atom(predicate, args)))

    fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
    expression = pddl.NumericConstant(effort) # Integer
    increase = pddl.Increase(fluent=fluent, expression=expression) # Can also be None

    return pddl.Action(name=name, parameters=parameters, num_external_parameters=len(parameters),
                    precondition=precondition, effects=effects, cost=increase)
    # TODO: previous problem seemed to be new predicates


def add_stream_actions(domain, stream_results):
    import pddl

    stream_result_from_name = {}
    new_actions = domain.actions[:]
    output_objects = []
    for i, stream_result in enumerate(stream_results):
        name = '{}{}'.format(stream_result.stream_instance.stream.name, i)
        stream_result_from_name[name] = stream_result
        new_actions.append(stream_action(stream_result, name))
        output_objects += stream_result.output_values

    new_constants = domain.constants[:]
    for obj in set(output_objects):
        new_constants.append(pddl.TypedObject(pddl_from_object(obj), OBJECT))
    # to_untyped_strips
    # free_variables
    new_domain = Domain(domain.name, domain.requirements, domain.types, domain.type_dict, new_constants,
                        domain.predicates, domain.predicate_dict, domain.functions, new_actions, domain.axioms)
    return new_domain, stream_result_from_name


def stream_plan_1(evaluations, goal_expression, domain, stream_results, **kwargs):
    new_domain, stream_result_from_name = add_stream_actions(domain, stream_results)
    combined_plan, combined_cost = solve_finite2(evaluations, goal_expression, new_domain, **kwargs)
    if combined_plan is None:
        return None, None
    stream_plan = []
    action_plan = []
    for action, args in combined_plan:
        if action in stream_result_from_name:
            stream_plan.append(stream_result_from_name[action])
        else:
            action_plan.append((action, args))
    return stream_plan, action_plan

def stream_plan_2(evaluations, goal_expression, domain, stream_results, **kwargs):
    opt_evaluations = set(evaluations)
    for stream_result in stream_results:
        for fact in stream_result.get_certified():
            opt_evaluations.add(Atom(Head(get_prefix(fact), get_args(fact))))
    action_plan, action_cost = solve_finite2(opt_evaluations, goal_expression, domain, **kwargs)
    stream_plan = []
    return stream_plan, action_plan


def solve_focused(problem, max_time=INF, **kwargs):
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)

    while elapsed_time(start_time) < max_time:
        # TODO: version that just calls one of the incremental algorithms
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        instantiator = Instantiator(evaluations, streams)
        stream_results = []
        while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
            stream_results += process_stream_queue(instantiator, None,
                                                   StreamInstance.next_optimistic,
                                                   revisit=False, verbose=False)

        stream_plan, action_plan = stream_plan_1(evaluations, goal_expression, domain, stream_results, **kwargs)
        stream_plan, action_plan = stream_plan_2(evaluations, goal_expression, domain, stream_results, **kwargs)

        new_domain, stream_result_from_name = add_stream_actions(domain,stream_results)
        print(new_domain)

        opt_plan, opt_cost = solve_finite2(evaluations, goal_expression, new_domain, verbose=True, **kwargs)
        #opt_plan, opt_cost = solve_finite2(opt_evaluations, goal_expression, domain, **kwargs)
        print(opt_plan)
        break

    print(evaluations)
    plan, cost = solve_finite2(evaluations, goal_expression, domain, **kwargs)
    print(plan)
    return revert_solution(plan, cost, evaluations)