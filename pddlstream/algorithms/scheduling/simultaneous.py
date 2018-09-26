from pddlstream.algorithms.algorithm import solve_finite
from pddlstream.algorithms.downward import OBJECT, Domain, \
    make_preconditions, make_effects, make_cost, COST_SCALE
from pddlstream.algorithms.scheduling.utils import get_results_from_head, apply_streams, partition_results
from pddlstream.language.constants import Head, And, Not
from pddlstream.language.conversion import pddl_from_object, obj_from_pddl, substitute_expression
from pddlstream.language.function import FunctionResult
from pddlstream.language.optimizer import UNSATISFIABLE, is_optimizer_result
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.utils import INF, find_unique

BOUND_PREDICATE = '_bound'

# ORDER_OUTPUT = False
# CALLED_PREDICATE = '_called_{}'
# CALL_PREFIX = '_{}' # '_call_{}'
#
# def enforce_output_order(result, preconditions, effects):
#     instance_name = '{}_{}'.format(result.external.name,  # No spaces & parens
#                                    ','.join(map(pddl_from_object, result.instance.input_objects)))
#     predicate_name = CALLED_PREDICATE.format(instance_name)
#     if 1 <= result.call_index:
#         preconditions.extend([
#             (predicate_name, CALL_PREFIX.format(result.call_index - 1)),
#             # Not((predicate_name, CALL_PREFIX.format(result.call_index))), # Not needed
#         ])
#     if 1 < len(result.instance.opt_results):
#         effects.extend([
#             (predicate_name, CALL_PREFIX.format(result.call_index)),
#             Not((predicate_name, CALL_PREFIX.format(result.call_index - 1))),
#         ])

def enforce_single_binding(result, preconditions, effects):
    binding_facts = [(BOUND_PREDICATE, pddl_from_object(out)) for out in result.output_objects]
    preconditions.extend(Not(fact) for fact in binding_facts)
    effects.extend(fact for fact in binding_facts)

def get_stream_actions(results, unique_binding=False, unit=True, effort_scale=1):
    #from pddl_parser.parsing_functions import parse_action
    import pddl
    stream_result_from_name = {}
    stream_actions = []
    for i, result in enumerate(results):
        #if not isinstance(stream_result, StreamResult):
        if type(result) == FunctionResult:
            continue
        effort = 1 if unit else result.instance.get_effort()
        if effort == INF:
            continue
        # TODO: state constraints
        # TODO: selectively negate axioms
        result_name = '{}-{}'.format(result.external.name, i)
        #result_name = '{}_{}_{}'.format(result.external.name, # No spaces & parens
        #                        ','.join(map(pddl_from_object, result.instance.input_objects)),
        #                        ','.join(map(pddl_from_object, result.output_objects)))
        assert result_name not in stream_result_from_name
        stream_result_from_name[result_name] = result

        preconditions = list(result.instance.get_domain())
        effects = list(result.get_certified())
        #if ORDER_OUTPUT:
        #    enforce_output_order(result, preconditions, effects)
        if unique_binding:
            enforce_single_binding(result, preconditions, effects)
        if is_optimizer_result(result):
            effects.append(substitute_expression(result.external.stream_fact, result.get_mapping()))
        parameters = []  # Usually all parameters are external
        stream_actions.append(pddl.Action(name=result_name, parameters=parameters,
                                          num_external_parameters=len(parameters),
                                          precondition=make_preconditions(preconditions),
                                          effects=make_effects(effects),
                                          cost=make_cost(effort_scale * effort))) # Can also be None
    return stream_actions, stream_result_from_name


def add_stream_actions(domain, stream_results, **kwargs):
    import pddl
    stream_actions, stream_result_from_name = get_stream_actions(stream_results, **kwargs)
    output_objects = []
    for stream_result in stream_result_from_name.values():
        if isinstance(stream_result, StreamResult):
            output_objects.extend(map(pddl_from_object, stream_result.output_objects))
    new_constants = list({pddl.TypedObject(obj, OBJECT) for obj in output_objects} | set(domain.constants))
    # to_untyped_strips
    # free_variables
    new_domain = Domain(domain.name, domain.requirements, domain.types, domain.type_dict, new_constants,
                        domain.predicates, domain.predicate_dict, domain.functions,
                        domain.actions[:] + stream_actions, domain.axioms)
    return new_domain, stream_result_from_name

##################################################

def get_action_cost(domain, results_from_head, name, args):
    import pddl
    action = find_unique(lambda a: a.name == name, domain.actions)
    if action.cost is None:
        return 0
    if isinstance(action.cost.expression, pddl.NumericConstant):
        return action.cost.expression.value
    var_mapping = {p.name: a for p, a in zip(action.parameters, args)}
    args = tuple(var_mapping[p] for p in action.cost.expression.args)
    head = Head(action.cost.expression.symbol, args)
    [(value, _)] = results_from_head[head]
    return value

def get_plan_cost(function_evaluations, action_plan, domain, unit_costs):
    if action_plan is None:
        return INF
    if unit_costs:
        return len(action_plan)
    action_cost = 0.
    results_from_head = get_results_from_head(function_evaluations)
    for name, args in action_plan:
        action_cost += get_action_cost(domain, results_from_head, name, args)
    return action_cost / COST_SCALE

##################################################

def extract_function_results(results_from_head, action, pddl_args):
    import pddl
    if (action.cost is None) or not isinstance(action.cost.expression, pddl.PrimitiveNumericExpression):
        return None
    var_mapping = {p.name: a for p, a in zip(action.parameters, pddl_args)}
    pddl_args = tuple(obj_from_pddl(var_mapping[p]) for p in action.cost.expression.args)
    head = Head(action.cost.expression.symbol, pddl_args)
    [(_, result)] = results_from_head[head]
    if result is None:
        return None
    return result

def extract_function_plan(function_evaluations, action_plan, domain, unit_costs):
    if unit_costs:
        return []
    function_plan = set()
    results_from_head = get_results_from_head(function_evaluations)
    for name, args in action_plan:
        action = find_unique(lambda a: a.name == name, domain.actions)
        pddl_args = tuple(map(pddl_from_object, args))
        result = extract_function_results(results_from_head, action, pddl_args)
        if result is not None:
            function_plan.add(result)
    return list(function_plan)

##################################################

# TODO: add effort costs additively to actions
# TODO: can augment state with the set of action constraints that are used (but not axiom)

def augment_goal(domain, goal_expression):
    # TODO: only do this if optimizers are present
    #return goal_expression
    import pddl
    predicate = pddl.predicates.Predicate(UNSATISFIABLE, tuple())
    if predicate.name not in domain.predicate_dict:
        domain.predicates.append(predicate)
        domain.predicate_dict[predicate.name] = predicate
    negated_atom = pddl.NegatedAtom(UNSATISFIABLE, tuple())
    for action in domain.actions:
        if negated_atom not in action.precondition.parts:
            action.precondition = pddl.Conjunction([action.precondition, negated_atom]).simplified()
    return And(goal_expression, Not((UNSATISFIABLE,)))

def partition_plan(combined_plan, stream_result_from_name):
    stream_plan, action_plan = [], []
    for name, args in combined_plan:
        if name in stream_result_from_name:
            stream_plan.append(stream_result_from_name[name])
        else:
            action_plan.append((name, args))
    return stream_plan, action_plan

def simultaneous_stream_plan(evaluations, goal_expression, domain, stream_results,
                             negated, unit_costs=False, **kwargs):
    if negated:
        raise NotImplementedError(negated)
    for result in stream_results:
        if isinstance(result.external, Stream) and result.external.is_fluent():
            raise NotImplementedError('Fluents are not supported')

    applied_streams, deferred_streams = partition_results(evaluations, stream_results, lambda r: False)
    opt_evaluations = apply_streams(evaluations, applied_streams)
    stream_domain, stream_result_from_name = add_stream_actions(domain, deferred_streams)
    if any(map(is_optimizer_result, stream_results)):
        goal_expression = augment_goal(stream_domain, goal_expression)
    combined_plan, _ = solve_finite(opt_evaluations, goal_expression,
                                    stream_domain, unit_costs=unit_costs, **kwargs)
    if combined_plan is None:
        return None, INF

    stream_plan, action_plan = partition_plan(combined_plan, stream_result_from_name)
    function_plan = extract_function_plan(opt_evaluations, action_plan, domain, unit_costs)
    cost = get_plan_cost(opt_evaluations, action_plan, domain, unit_costs)
    combined_plan = stream_plan + function_plan + action_plan
    return combined_plan, cost
