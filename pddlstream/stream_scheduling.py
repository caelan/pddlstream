from pddlstream.conversion import get_prefix, pddl_from_object, get_args, TOTAL_COST, Atom, Head, obj_from_pddl, \
    obj_from_pddl_plan
from pddlstream.fast_downward import OBJECT, Domain, get_problem, task_from_domain_problem, solve_from_task, get_init
from pddlstream.incremental import solve_finite
from pddlstream.utils import Verbose, find


def get_stream_action(stream_result, name, effect_scale=1):
    #from pddl_parser.parsing_functions import parse_action
    import pddl

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

    effort = effect_scale
    fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
    expression = pddl.NumericConstant(effort) # Integer
    increase = pddl.Increase(fluent=fluent, expression=expression) # Can also be None

    return pddl.Action(name=name, parameters=parameters, num_external_parameters=len(parameters),
                    precondition=precondition, effects=effects, cost=increase)
    # TODO: previous problem seemed to be new predicates


def get_stream_actions(stream_results):
    stream_result_from_name = {}
    stream_actions = []
    for i, stream_result in enumerate(stream_results):
        name = '{}-{}'.format(stream_result.stream_instance.stream.name, i)
        stream_result_from_name[name] = stream_result
        stream_actions.append(get_stream_action(stream_result, name))
    return stream_actions, stream_result_from_name


def add_stream_actions(domain, stream_results):
    import pddl
    stream_actions, stream_result_from_name = get_stream_actions(stream_results)
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


def simultaneous_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    new_domain, stream_result_from_name = add_stream_actions(domain, stream_results)
    combined_plan, combined_cost = solve_finite(evaluations, goal_expression, new_domain, **kwargs)
    if combined_plan is None:
        return None, None
    stream_plan = []
    action_plan = []
    for name, args in combined_plan:
        if name in stream_result_from_name:
            stream_plan.append(stream_result_from_name[name])
        else:
            action_plan.append((name, args))
    return stream_plan, action_plan


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
    if action_plan is None:
        return None, None
    real_init = get_init(evaluations)

    import pddl_to_prolog
    import build_model
    import instantiate
    with Verbose(False):
        model = build_model.compute_model(pddl_to_prolog.translate(task))
    opt_facts = set(task.init) - set(real_init)
    fluent_facts = instantiate.get_fluent_facts(task, model) | opt_facts
    task.init = real_init
    task.actions, stream_result_from_name = get_stream_actions(stream_results)
    init_facts = set(task.init)
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)

    # TODO: add ordering constraints to simplify the optimization
    action_from_name = {}
    for i, (name, args) in enumerate(action_plan):
        action = find(lambda a: a.name == name, domain.actions)
        #parameters = action.parameters[:action.num_external_parameters]
        var_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        new_name = '{}-{}'.format(name, i)
        new_parameters = action.parameters[len(args):]
        new_preconditions = []
        action.precondition.instantiate(var_mapping, init_facts, fluent_facts, new_preconditions)
        new_effects = []
        for eff in action.effects:
            eff.instantiate(var_mapping, init_facts, fluent_facts, type_to_objects, new_effects)
        new_effects = [pddl.Effect([], pddl.Conjunction(conditions), effect)
                      for conditions, effect in new_effects]
        task.actions.append(pddl.Action(new_name, new_parameters, 0,
                                   pddl.Conjunction(new_preconditions), new_effects, 1))
        action_from_name[new_name] = (name, map(obj_from_pddl, args))

    combined_plan, _ = solve_from_task(task, **kwargs)
    if combined_plan is None:
        return None, obj_from_pddl_plan(action_plan)
    stream_plan = []
    action_plan = []
    for name, args in combined_plan:
        if name in stream_result_from_name:
            stream_plan.append(stream_result_from_name[name])
        else:
            action_plan.append(action_from_name[name])
    return stream_plan, action_plan