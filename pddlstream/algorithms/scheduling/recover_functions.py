from collections import defaultdict

from pddlstream.utils import INF
from pddlstream.language.constants import is_parameter, Head
from pddlstream.language.conversion import obj_from_pddl


def extract_function_result(results_from_head, action, pddl_args):
    import pddl
    if action.cost is None:
        return None
    # TODO: retrieve constant action costs
    # TODO: associate costs with the steps they are applied
    expression = action.cost.expression
    if not isinstance(expression, pddl.PrimitiveNumericExpression):
        return None
    var_mapping = {p.name: a for p, a in zip(action.parameters, pddl_args)}
    obj_args = tuple(obj_from_pddl(var_mapping[p] if is_parameter(p) else p)
                     for p in expression.args)
    head = Head(expression.symbol, obj_args)
    [result] = results_from_head[head]
    if result is None:
        return None
    return result


def compute_function_plan(opt_evaluations, action_plan):
    results_from_head = defaultdict(list)
    for evaluation, result in opt_evaluations.items():
        results_from_head[evaluation.head].append(result)
    step_from_function = {}
    for step, action_instance in enumerate(action_plan):
        action = action_instance.action
        if action is None:
            continue
        args = [action_instance.var_mapping[p.name] for p in action.parameters]
        result = extract_function_result(results_from_head, action, args)
        if result is not None:
            step_from_function[result] = min(step, step_from_function.get(result, INF))
            if not result.is_deferrable():
                step_from_function[result] = 0
            #function_from_instance[action_instance] = result
    return step_from_function
