from pddlstream.algorithms.algorithm import solve_finite
from pddlstream.algorithms.downward import Domain, make_action, make_predicate, make_parameters, make_domain, \
    get_cost_scale, add_predicate
from pddlstream.algorithms.scheduling.utils import get_results_from_head, apply_streams, partition_results
from pddlstream.language.constants import Head, And, Not
from pddlstream.language.conversion import pddl_from_object, obj_from_pddl, substitute_expression, is_parameter
from pddlstream.language.function import FunctionResult
from pddlstream.language.optimizer import UNSATISFIABLE, is_optimizer_result
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.language.external import compute_result_effort
from pddlstream.utils import INF, find_unique

BOUND_PREDICATE = '_bound'


def enforce_single_binding(result, preconditions, effects):
    binding_facts = [(BOUND_PREDICATE, pddl_from_object(out)) for out in result.output_objects]
    preconditions.extend(Not(fact) for fact in binding_facts)
    effects.extend(fact for fact in binding_facts)

def get_stream_actions(results, unique_binding=False, effort_scale=1, **kwargs):
    result_from_name = {}
    stream_actions = []
    for i, result in enumerate(results):
        #if not isinstance(stream_result, StreamResult):
        if type(result) == FunctionResult:
            continue
        effort = compute_result_effort(result, **kwargs)
        if effort == INF:
            continue
        # TODO: state constraints
        # TODO: selectively negate axioms
        result_name = '{}-{}'.format(result.external.name, i)
        #result_name = '{}_{}_{}'.format(result.external.name, # No spaces & parens
        #                        ','.join(map(pddl_from_object, result.instance.input_objects)),
        #                        ','.join(map(pddl_from_object, result.output_objects)))
        assert result_name not in result_from_name
        result_from_name[result_name] = result

        preconditions = list(result.instance.get_domain())
        effects = list(result.get_certified())
        if unique_binding:
            enforce_single_binding(result, preconditions, effects)
        if is_optimizer_result(result): # These effects don't seem to be pruned
            effects.append(substitute_expression(result.external.stream_fact, result.get_mapping()))
        stream_actions.append(make_action(result_name, [], preconditions, effects, effort_scale * effort))
    return stream_actions, result_from_name


def add_stream_actions(domain, results, **kwargs):
    stream_actions, result_from_name = get_stream_actions(results, **kwargs)
    output_objects = []
    for result in result_from_name.values():
        if isinstance(result, StreamResult):
            output_objects.extend(map(pddl_from_object, result.output_objects))
    new_constants = list(make_parameters(set(output_objects) | set(domain.constants)))
    # to_untyped_strips, free_variables
    new_domain = make_domain(constants=new_constants, predicates=domain.predicates,
                             actions=domain.actions[:] + stream_actions, axioms=domain.axioms)
    #new_domain = copy.copy(domain)
    """
    optimizer_results = list(filter(is_optimizer_result, stream_results))
    optimizer_facts = {substitute_expression(result.external.stream_fact, result.get_mapping())
                       for result in optimizer_results}
    optimizers = {result.external.optimizer for result in optimizer_results}
    print(optimizers)
    for optimizer in optimizers:
        for stream in optimizer.streams:
            print(stream.instance.get_constraints())
            print(stream.instance)
    #print(optimizer_results)
    #print(optimizer_facts)
    """
    return new_domain, result_from_name

##################################################

def get_action_cost(domain, results_from_head, name, args):
    import pddl
    action = find_unique(lambda a: a.name == name, domain.actions)
    if action.cost is None:
        return 0.
    if isinstance(action.cost.expression, pddl.NumericConstant):
        return action.cost.expression.value / get_cost_scale()
    var_mapping = {p.name: a for p, a in zip(action.parameters, args)}
    args = tuple(var_mapping[p] for p in action.cost.expression.args)
    head = Head(action.cost.expression.symbol, args)
    [(value, _)] = results_from_head[head]
    return value

def get_plan_cost(function_evaluations, action_plan, domain, unit_costs):
    # TODO: deprecate in favor of using the raw action instances
    if action_plan is None:
        return INF
    if unit_costs:
        return len(action_plan)
    results_from_head = get_results_from_head(function_evaluations)
    return sum([0.] + [get_action_cost(domain, results_from_head, name, args)
                       for name, args in action_plan])

##################################################

def extract_function_results(results_from_head, action, pddl_args):
    import pddl
    if action.cost is None:
        return None
    expression = action.cost.expression
    if not isinstance(expression, pddl.PrimitiveNumericExpression):
        return None
    var_mapping = {p.name: a for p, a in zip(action.parameters, pddl_args)}
    obj_args = tuple(obj_from_pddl(var_mapping[p] if is_parameter(p) else p)
                     for p in expression.args)
    head = Head(expression.symbol, obj_args)
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

def add_unsatisfiable_to_goal(domain, goal_expression, negate_actions=False):
    import pddl
    add_predicate(domain, make_predicate(UNSATISFIABLE, []))
    if negate_actions:
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

##################################################

def simultaneous_stream_plan(evaluations, goal_expression, domain, stream_results,
                             negated, unit_costs=False, **kwargs):
    # TODO: remove this method in favor of the more general relaxed plan
    if negated:
        raise NotImplementedError(negated)
    for result in stream_results:
        if isinstance(result.external, Stream) and result.external.is_fluent():
            raise NotImplementedError('Fluents are not supported')
    # TODO: warning check if using simultaneous_stream_plan with non-eager functions

    applied_streams, deferred_streams = partition_results(evaluations, stream_results, lambda r: False)
    opt_evaluations = apply_streams(evaluations, applied_streams)
    stream_domain, stream_result_from_name = add_stream_actions(domain, deferred_streams)
    if any(map(is_optimizer_result, stream_results)):
        goal_expression = add_unsatisfiable_to_goal(stream_domain, goal_expression)
    combined_plan, _ = solve_finite(opt_evaluations, goal_expression,
                                    stream_domain, unit_costs=unit_costs, **kwargs)
    if combined_plan is None:
        return None, INF

    stream_plan, action_plan = partition_plan(combined_plan, stream_result_from_name)
    function_plan = extract_function_plan(opt_evaluations, action_plan, domain, unit_costs)
    cost = get_plan_cost(opt_evaluations, action_plan, domain, unit_costs)
    combined_plan = stream_plan + function_plan + action_plan
    return combined_plan, cost
