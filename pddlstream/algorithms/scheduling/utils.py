from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.stream import StreamResult
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams
from pddlstream.utils import INF

"""
def evaluations_from_stream_plan(evaluations, stream_plan):
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
"""

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
