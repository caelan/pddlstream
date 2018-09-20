from collections import defaultdict

from pddlstream.algorithms.algorithm import solve_finite
from pddlstream.algorithms.downward import TOTAL_COST, OBJECT, Domain, fd_from_fact
from pddlstream.language.constants import Head, And, Not
from pddlstream.language.conversion import pddl_from_object, obj_from_pddl, evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult
from pddlstream.language.optimizer import UNSATISFIABLE
from pddlstream.utils import INF, find_unique, str_from_object


def get_results_from_head(evaluations):
    results_from_head = defaultdict(list)
    for evaluation, stream_result in evaluations.items():
        results_from_head[evaluation.head].append((evaluation.value, stream_result))
        #results_from_head[evaluation.head].append(stream_result)
    return results_from_head

def combine_function_evaluations(evaluations, stream_results):
    function_evaluations = {e: None for e in evaluations}
    for result in stream_results:
        if isinstance(result, FunctionResult):
            for fact in result.get_certified():
                function_evaluations[evaluation_from_fact(fact)] = result
    return function_evaluations

##################################################

CALLED_PREFIX = '_called_{}'
CALL_PREFIX = '_call_{}'

def make_preconditions(preconditions):
    import pddl
    return pddl.Conjunction(list(map(fd_from_fact, preconditions)))

def make_effects(effects):
    import pddl
    return [pddl.Effect(parameters=[], condition=pddl.Truth(), literal=fd_from_fact(fact)) for fact in effects]

def make_cost(cost):
    import pddl
    fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
    expression = pddl.NumericConstant(cost)
    return pddl.Increase(fluent=fluent, expression=expression)

def get_stream_actions(results, unit=True, effort_scale=1):
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

        instance_name = '{}_{}'.format(result.external.name, # No spaces & parens
            ','.join(map(pddl_from_object, result.instance.input_objects)))
        predicate_name = CALLED_PREFIX.format(instance_name)
        preconditions = list(result.instance.get_domain())
        if 1 <= result.call_index:
            preconditions.extend([
                (predicate_name, CALL_PREFIX.format(result.call_index-1)),
                #Not((predicate_name, CALL_PREFIX.format(result.call_index))), # Not needed
            ])
        effects = list(result.get_certified())
        if 1 < len(result.instance.opt_results):
            effects.extend([
                (predicate_name, CALL_PREFIX.format(result.call_index)),
                Not((predicate_name, CALL_PREFIX.format(result.call_index-1))),
            ])
        parameters = []  # Usually all parameters are external
        stream_actions.append(pddl.Action(name=result_name, parameters=parameters,
                                          num_external_parameters=len(parameters),
                                          precondition=make_preconditions(preconditions),
                                          effects=make_effects(effects),
                                          cost=make_cost(effort_scale * effort))) # Can also be None
    return stream_actions, stream_result_from_name


def add_stream_actions(domain, stream_results):
    import pddl
    stream_actions, stream_result_from_name = get_stream_actions(stream_results)
    output_objects = []
    for stream_result in stream_result_from_name.values():
        if isinstance(stream_result, StreamResult):
            output_objects.extend(map(pddl_from_object, stream_result.output_objects))
    new_constants = list({pddl.TypedObject(obj, OBJECT) for obj in output_objects} | set(domain.constants))
    # to_untyped_strips
    # free_variables
    new_domain = Domain(domain.name, domain.requirements, domain.types, domain.type_dict,
                        new_constants,
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

def get_plan_cost(function_evaluations, action_plan, domain):
    action_cost = 0
    results_from_head = get_results_from_head(function_evaluations)
    for name, args in action_plan:
        action_cost += get_action_cost(domain, results_from_head, name, args)
    return action_cost

##################################################

def extract_function_results(results_from_head, action, pddl_args):
    import pddl
    if (action.cost is None) or not isinstance(action.cost.expression, pddl.PrimitiveNumericExpression):
        return None
    var_mapping = {p.name: a for p, a in zip(action.parameters, pddl_args)}
    pddl_args = tuple(obj_from_pddl(var_mapping[p]) for p in action.cost.expression.args)
    head = Head(action.cost.expression.symbol, pddl_args)
    [(value, stream_result)] = results_from_head[head]
    if stream_result is None:
        return None
    return stream_result

def extract_function_plan(function_evaluations, action_plan, domain):
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
    return And(goal_expression, Not((UNSATISFIABLE,)))

def simultaneous_stream_plan(evaluations, goal_expression, domain, stream_results,
                             negated, unit_costs=True, **kwargs):
    if negated:
        raise NotImplementedError(negated)

    function_evaluations = combine_function_evaluations(evaluations, stream_results)
    stream_domain, stream_result_from_name = add_stream_actions(domain, stream_results)
    combined_plan, _ = solve_finite(function_evaluations, augment_goal(stream_domain, goal_expression),
                                    stream_domain, unit_costs=unit_costs, **kwargs)
    if combined_plan is None:
        return None, INF

    stream_plan, action_plan = [], []
    for name, args in combined_plan:
        if name in stream_result_from_name:
            stream_plan.append(stream_result_from_name[name])
        else:
            action_plan.append((name, args))

    if unit_costs:
        function_plan = []
        action_cost = len(action_plan)
    else:
        function_plan = extract_function_plan(function_evaluations, action_plan, domain)
        action_cost = get_plan_cost(function_evaluations, action_plan, domain)
    combined_plan = stream_plan + function_plan + action_plan
    return combined_plan, action_cost
