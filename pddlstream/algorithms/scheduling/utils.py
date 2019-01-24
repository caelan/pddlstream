from pddlstream.algorithms.downward import add_predicate, make_predicate
from pddlstream.language.constants import And, Not
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams
from pddlstream.utils import INF

def partition_results(evaluations, results, apply_now):
    applied_results = []
    deferred_results = []
    opt_evaluations = set(evaluations)
    for result in results:
        assert(not result.instance.disabled)
        assert(not result.instance.enumerated)
        domain = set(map(evaluation_from_fact, result.instance.get_domain()))
        if isinstance(result, FunctionResult) or (apply_now(result) and (domain <= opt_evaluations)):
            applied_results.append(result)
            opt_evaluations.update(map(evaluation_from_fact, result.get_certified()))
        else:
            deferred_results.append(result)
    return applied_results, deferred_results

def evaluations_from_stream_plan(evaluations, stream_results, max_effort=INF):
    opt_evaluations = set(evaluations)
    for result in stream_results:
        assert(not result.instance.disabled)
        assert(not result.instance.enumerated)
        domain = set(map(evaluation_from_fact, result.instance.get_domain()))
        assert(domain <= opt_evaluations)
        opt_evaluations.update(map(evaluation_from_fact, result.get_certified()))
    node_from_atom = get_achieving_streams(evaluations, stream_results)
    result_from_evaluation = {evaluation_from_fact(f): n.result
                              for f, n in node_from_atom.items() if n.effort < max_effort}
    return result_from_evaluation

def partition_external_plan(external_plan):
    function_plan = list(filter(lambda r: isinstance(r, FunctionResult), external_plan))
    stream_plan = list(filter(lambda r: r not in function_plan, external_plan))
    return stream_plan, function_plan

def partition_combined_plan(combined_plan, stream_result_from_name):
    stream_plan, action_plan = [], []
    for name, args in combined_plan:
        if name in stream_result_from_name:
            stream_plan.append(stream_result_from_name[name])
        else:
            action_plan.append((name, args))
    return stream_plan, action_plan

def add_unsatisfiable_to_goal(domain, goal_expression):
    import pddl
    from pddlstream.language.optimizer import UNSATISFIABLE, BLOCK_ADDITIONS
    add_predicate(domain, make_predicate(UNSATISFIABLE, []))
    if not BLOCK_ADDITIONS:
        negated_atom = pddl.NegatedAtom(UNSATISFIABLE, tuple())
        for action in domain.actions:
            if negated_atom not in action.precondition.parts:
                action.precondition = pddl.Conjunction([action.precondition, negated_atom]).simplified()
    return And(goal_expression, Not((UNSATISFIABLE,)))
