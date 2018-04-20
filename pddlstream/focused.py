from pddlstream.conversion import get_pddl_problem, value_from_obj_plan, \
    obj_from_pddl_plan, substitute_expression, Head, get_prefix, get_args, Evaluation, \
    init_from_evaluations, evaluations_from_init, convert_expression, values_from_objects, \
    objects_from_values, TOTAL_COST, pddl_from_object, Atom, evaluation_from_fact
from pddlstream.fast_downward import solve_from_pddl, parse_domain, Domain, OBJECT
from pddlstream.instantiation import Instantiator
from pddlstream.object import Object
from pddlstream.stream import parse_stream, StreamInstance, StreamResult
from pddlstream.utils import INF, elapsed_time
import time

from pddlstream.incremental import parse_problem, solve_finite, revert_solution, \
    process_stream_queue, print_output_values_list

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

def exhaustive_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    if stream_results:
        return stream_results, []
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    if plan is None:
        return None, plan
    return [], plan

def incremental_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    if plan is not None:
        return [], plan
    if stream_results:
        return stream_results, plan
    return None, plan

def simultaneous_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    new_domain, stream_result_from_name = add_stream_actions(domain, stream_results)
    combined_plan, combined_cost = solve_finite(evaluations, goal_expression, new_domain, **kwargs)
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

def sequential_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    opt_evaluations = set(evaluations)
    for stream_result in stream_results:
        for fact in stream_result.get_certified():
            opt_evaluations.add(Atom(Head(get_prefix(fact), get_args(fact))))
    action_plan, action_cost = solve_finite(opt_evaluations, goal_expression, domain, **kwargs)
    stream_plan = []




    return stream_plan, action_plan

def process_stream_plan(evaluations, stream_plan, disabled, verbose, quick_fail=True):
    new_evaluations = []
    for opt_stream_result in stream_plan:
        stream_instance = opt_stream_result.stream_instance
        domain = set(map(evaluation_from_fact, stream_instance.get_domain()))
        if not (domain <= evaluations):
            continue
        disabled.append(stream_instance)
        stream_instance.disabled = True
        # TODO: assert something about the arguments
        output_values_list = stream_instance.next_outputs()
        if verbose:
            print_output_values_list(stream_instance, output_values_list)
        if quick_fail and (not output_values_list):
            break
        for output_values in output_values_list:
            stream_result = StreamResult(stream_instance, output_values)
            for fact in stream_result.get_certified():
                evaluation = evaluation_from_fact(fact)
                evaluations.add(evaluation) # To be used on next iteration
                new_evaluations.append(evaluation)
    return new_evaluations

def reset_disabled(disabled):
    for stream_instance in disabled:
        stream_instance.disabled = False
    disabled[:] = []

def solve_focused(problem, max_time=INF, verbose=False, **kwargs):
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    disabled = []
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
        # exhaustive_stream_plan | incremental_stream_plan | incremental | simultaneous_stream_plan | sequential_stream_plan
        solve_stream_plan = sequential_stream_plan
        stream_plan, action_plan = solve_stream_plan(evaluations, goal_expression,
                                                     domain, stream_results, **kwargs)
        print('Stream plan: {}\n'
              'Action plan: {}'.format(stream_plan, action_plan))
        if stream_plan is None:
            if not disabled:
                break
            reset_disabled(disabled)
        elif len(stream_plan) == 0:
            best_plan = action_plan
            break
        else:
            process_stream_plan(evaluations, stream_plan, disabled, verbose)

    return revert_solution(best_plan, best_cost, evaluations)