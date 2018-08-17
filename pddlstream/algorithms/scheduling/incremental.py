from pddlstream.algorithms.algorithm import solve_finite, add_certified
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.function import Function
from pddlstream.utils import INF


def exhaustive_stream_plan(evaluations, goal_expression, domain, stream_results,
                           negated, **kwargs):
    if stream_results:
        return stream_results, INF
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    if plan is None:
        return None, cost
    return plan, cost

def evaluate_functions(evaluations, stream_results):
    stream_plan = []
    for opt_result in stream_results:
        instance = opt_result.instance
        domain_evals = set(map(evaluation_from_fact, instance.get_domain()))
        if isinstance(instance.external, Function) and (domain_evals <= evaluations):
            new_results, new_facts = instance.next_results()
            assert not new_facts
            for result in new_results:
                add_certified(evaluations, result)
        else:
            stream_plan.append(opt_result)
    return stream_plan

def incremental_stream_plan(evaluations, goal_expression, domain, stream_results,
                             negated, **kwargs):
    stream_plan = evaluate_functions(evaluations, stream_results)
    plan, cost = solve_finite(evaluations, goal_expression, domain, **kwargs)
    if plan is not None:
        return plan, cost
    if stream_plan:
        return stream_plan, cost
    return None, cost