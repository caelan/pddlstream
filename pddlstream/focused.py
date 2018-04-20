import time

from pddlstream.conversion import Head, get_prefix, get_args, TOTAL_COST, pddl_from_object, Atom, evaluation_from_fact
from pddlstream.fast_downward import Domain, OBJECT
from pddlstream.incremental import parse_problem, solve_finite, revert_solution, \
    process_stream_queue, print_output_values_list
from pddlstream.instantiation import Instantiator
from pddlstream.stream import StreamInstance, StreamResult
from pddlstream.utils import INF, elapsed_time, find


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

def generate_stream_actions(stream_results):
    stream_result_from_name = {}
    stream_actions = []
    for i, stream_result in enumerate(stream_results):
        name = '{}{}'.format(stream_result.stream_instance.stream.name, i)
        stream_result_from_name[name] = stream_result
        stream_actions.append(stream_action(stream_result, name))
    return stream_actions, stream_result_from_name


def add_stream_actions(domain, stream_results):
    import pddl
    stream_actions, stream_result_from_name = generate_stream_actions(stream_results)
    output_objects = []
    for stream_result in stream_result_from_name.values():
        output_objects += stream_result.output_values
    new_constants = [pddl.TypedObject(pddl_from_object(obj), OBJECT) for obj in set(output_objects)]
    # to_untyped_strips
    # free_variables
    new_domain = Domain(domain.name, domain.requirements, domain.types, domain.type_dict,
                        domain.constants[:] + new_constants,
                        domain.predicates, domain.predicate_dict, domain.functions,
                        domain.actions[:] + stream_actions, domain.axioms)
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


from pddlstream.incremental import get_problem, task_from_domain_problem
from pddlstream.fast_downward import solve_from_task


def sequential_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    import pddl
    opt_evaluations = set(evaluations)
    for stream_result in stream_results:
        for fact in stream_result.get_certified():
            opt_evaluations.add(Atom(Head(get_prefix(fact), get_args(fact))))
    #action_plan, action_cost = solve_finite(opt_evaluations, goal_expression, domain, **kwargs)
    problem = get_problem(opt_evaluations, goal_expression, domain)
    task = task_from_domain_problem(domain, problem)
    action_plan, _ = solve_from_task(task, **kwargs)
    print(action_plan)
    if action_plan is None:
        return None, None

    import pddl_to_prolog
    import build_model
    import instantiate
    init_facts = set(task.init)
    model = build_model.compute_model(pddl_to_prolog.translate(task))
    fluent_facts = instantiate.get_fluent_facts(task, model)
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    print(init_facts)
    print(fluent_facts)
    print(type_to_objects)

    # TODO: add ordering constraints to simplify the optimization
    task.actions = []
    for i, (name, args) in enumerate(action_plan):
        action = find(lambda a: a.name == name, domain.actions)
        #parameters = action.parameters[:action.num_external_parameters]
        var_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        new_name = '{}-{}'.format(name, i)
        new_parameters = action.parameters[len(args):]
        new_preconditions = []
        print(var_mapping)
        action.precondition.dump()
        action.precondition.instantiate(var_mapping, init_facts, fluent_facts, new_preconditions)
        new_effects = []
        for eff in action.effects:
            eff.instantiate(var_mapping, init_facts, fluent_facts, type_to_objects, new_effects)
        print(new_effects)
        new_effects = [pddl.Effect([], pddl.Conjunction(conditions), effect)
                       for conditions, effect in new_effects]

        print(new_preconditions)
        print(new_effects)
        action.precondition.dump()
        task.actions.append(pddl.Action(new_name, new_parameters, 0,
                                   pddl.Conjunction(new_preconditions), new_effects, 1))



    combined_plan, _ = solve_from_task(task, **kwargs)
    print(action_plan)

    print(task.actions)




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
        # exhaustive_stream_plan | incremental_stream_plan | simultaneous_stream_plan | sequential_stream_plan
        solve_stream_plan = incremental_stream_plan
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