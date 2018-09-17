from collections import defaultdict

from pddlstream.algorithms.algorithm import solve_finite
from pddlstream.algorithms.downward import TOTAL_COST, OBJECT, Domain, fd_from_fact
from pddlstream.language.conversion import pddl_from_object, obj_from_pddl, evaluation_from_fact
from pddlstream.language.constants import Head
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult
from pddlstream.utils import INF, find


def evaluations_from_stream_plan(evaluations, stream_plan):
    # TODO: move this to another file
    result_from_evaluation = {e: None for e in evaluations}
    opt_evaluations = set(evaluations)
    for result in stream_plan:
        if isinstance(result, StreamResult):
            effort = result.instance.get_effort()
            if effort == INF:
                continue
        assert(not result.instance.disabled)
        assert(not result.instance.enumerated)
        domain = set(map(evaluation_from_fact, result.instance.get_domain()))
        if not (domain <= opt_evaluations):
            continue
        for fact in result.get_certified():
            evaluation = evaluation_from_fact(fact)
            if evaluation not in result_from_evaluation:
                result_from_evaluation[evaluation] = result
                opt_evaluations.add(evaluation)
    return result_from_evaluation

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

def get_stream_action(result, name, unit=True, effort_scale=1):
    #from pddl_parser.parsing_functions import parse_action
    import pddl

    parameters = []
    preconditions = [fd_from_fact(fact) for fact in result.instance.get_domain()]
    effects = [pddl.Effect(parameters=[], condition=pddl.Truth(), literal=fd_from_fact(fact))
               for fact in result.get_certified()]

    effort = 1 if unit else result.instance.get_effort()
    if effort == INF:
        return None
    fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
    expression = pddl.NumericConstant(effort_scale * effort)
    cost = pddl.Increase(fluent=fluent, expression=expression) # Can also be None

    # TODO: is num_external_parameters correct?
    # Usually all parameters are external
    return pddl.Action(name=name, parameters=parameters, num_external_parameters=len(parameters),
                       precondition=pddl.Conjunction(preconditions), effects=effects, cost=cost)
    # TODO: previous problem seemed to be new predicates


def get_stream_actions(results):
    stream_result_from_name = {}
    stream_actions = []
    for i, stream_result in enumerate(results):
        #if not isinstance(stream_result, StreamResult):
        if type(stream_result) == FunctionResult:
            continue
        name = '{}-{}'.format(stream_result.instance.external.name, i)
        stream_action = get_stream_action(stream_result, name)
        if stream_action is None:
            continue
        stream_result_from_name[name] = stream_result
        stream_actions.append(stream_action)
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
    action = find(lambda a: a.name == name, domain.actions)
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
        action = find(lambda a: a.name == name, domain.actions)
        pddl_args = tuple(map(pddl_from_object, args))
        result = extract_function_results(results_from_head, action, pddl_args)
        if result is not None:
            function_plan.add(result)
    return list(function_plan)

##################################################

def simultaneous_stream_plan(evaluations, goal_expression, domain, stream_results,
                             negated, unit_costs=True, **kwargs):
    if negated:
        raise NotImplementedError(negated)

    function_evaluations = combine_function_evaluations(evaluations, stream_results)
    stream_domain, stream_result_from_name = add_stream_actions(domain, stream_results)
    combined_plan, _ = solve_finite(function_evaluations, goal_expression, stream_domain,
                                    unit_costs=unit_costs, **kwargs)
    if combined_plan is None:
        return None, INF # TODO: return plan cost

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
