from pddlstream.algorithm import solve_finite
from pddlstream.conversion import evaluation_from_fact
from pddlstream.focused import query_stream
from pddlstream.stream import Function
from pddlstream.utils import INF


def exhaustive_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    if stream_results:
        return stream_results, None, INF
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    if plan is None:
        return None, plan, cost
    return [], plan, cost


def incremental_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    stream_plan = []
    for opt_stream_result in stream_results:
        instance = opt_stream_result.stream_instance
        domain_evals = set(map(evaluation_from_fact, instance.get_domain()))
        if isinstance(instance.stream, Function) and (domain_evals <= evaluations):
            for stream_result in query_stream(instance, False):
                evaluations.update(map(evaluation_from_fact, stream_result.get_certified()))
        else:
            stream_plan.append(opt_stream_result)
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    if plan is not None:
        return [], plan, cost
    if stream_plan:
        return stream_plan, plan, cost
    return None, plan, cost