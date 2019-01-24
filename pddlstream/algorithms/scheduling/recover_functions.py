from collections import defaultdict

from pddlstream.language.constants import is_parameter, Head
from pddlstream.language.conversion import obj_from_pddl


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


def compute_function_plan(opt_evaluations, action_plan):
    results_from_head = defaultdict(list)
    for evaluation, stream_result in opt_evaluations.items():
        results_from_head[evaluation.head].append((evaluation.value, stream_result))
        #results_from_head[evaluation.head].append(stream_result)
    function_plan = set()
    for action_instance in action_plan:
        action = action_instance.action
        if action is None:
            continue
        args = [action_instance.var_mapping[p.name] for p in action.parameters]
        function_result = extract_function_results(results_from_head, action, args)
        if function_result is not None:
            function_plan.add(function_result)
    return function_plan