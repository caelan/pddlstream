from collections import defaultdict

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
    result_from_evaluation = {evaluation_from_fact(f): n.stream_result
                              for f, n in node_from_atom.items() if n.effort < max_effort}
    return result_from_evaluation

##################################################

def get_results_from_head(evaluations):
    results_from_head = defaultdict(list)
    for evaluation, stream_result in evaluations.items():
        results_from_head[evaluation.head].append((evaluation.value, stream_result))
        #results_from_head[evaluation.head].append(stream_result)
    return results_from_head


def apply_streams(evaluations, stream_results):
    function_evaluations = {e: None for e in evaluations}
    for result in stream_results:
        for fact in result.get_certified():
            function_evaluations[evaluation_from_fact(fact)] = result
    return function_evaluations


def partition_external_plan(external_plan):
    function_plan = list(filter(lambda r: isinstance(r, FunctionResult), external_plan))
    stream_plan = list(filter(lambda r: r not in function_plan, external_plan))
    return stream_plan, function_plan