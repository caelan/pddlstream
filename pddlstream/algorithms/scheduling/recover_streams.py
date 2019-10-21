from collections import namedtuple, defaultdict
from heapq import heappop, heappush

from pddlstream.language.conversion import is_negated_atom, fact_from_evaluation, evaluation_from_fact
from pddlstream.language.statistics import check_effort
from pddlstream.utils import HeapElement, INF

Node = namedtuple('Node', ['effort', 'result']) # TODO: include level
EFFORT_OP = sum # max | sum
NULL_COND = (None,)

def get_achieving_streams(evaluations, stream_results, max_effort=INF, **effort_args):
    unprocessed_from_atom = defaultdict(list)
    node_from_atom = {NULL_COND: Node(0, None)}
    conditions_from_stream = {}
    remaining_from_stream = {}
    for result in stream_results:
        conditions_from_stream[result] = result.instance.get_domain() + (NULL_COND,)
        remaining_from_stream[result] = len(conditions_from_stream[result])
        for atom in conditions_from_stream[result]:
            unprocessed_from_atom[atom].append(result)
    for atom in evaluations:
        if not is_negated_atom(atom):
            node_from_atom[fact_from_evaluation(atom)] = Node(0, None)

    queue = [HeapElement(node.effort, atom) for atom, node in node_from_atom.items()]
    while queue:
        atom = heappop(queue).value
        if atom not in unprocessed_from_atom:
            continue
        for result in unprocessed_from_atom[atom]:
            remaining_from_stream[result] -= 1
            if remaining_from_stream[result]:
                continue
            effort = result.get_effort(**effort_args)
            total_effort = effort + EFFORT_OP(
                node_from_atom[cond].effort for cond in conditions_from_stream[result])
            if max_effort <= total_effort:
                continue
            for new_atom in result.get_certified():
                if (new_atom not in node_from_atom) or (total_effort < node_from_atom[new_atom].effort):
                    node_from_atom[new_atom] = Node(total_effort, result)
                    heappush(queue, HeapElement(total_effort, new_atom))
        del unprocessed_from_atom[atom]
    del node_from_atom[NULL_COND]
    return node_from_atom

def evaluations_from_stream_plan(evaluations, stream_results, max_effort=INF):
    opt_evaluations = set(evaluations)
    for result in stream_results:
        assert(not result.instance.disabled)
        assert(not result.instance.enumerated)
        domain = set(map(evaluation_from_fact, result.instance.get_domain()))
        assert(domain <= opt_evaluations)
        opt_evaluations.update(map(evaluation_from_fact, result.get_certified()))
    node_from_atom = get_achieving_streams(evaluations, stream_results)
    result_from_evaluation = {evaluation_from_fact(f): n.result for f, n in node_from_atom.items()
                              if check_effort(n.effort, max_effort)}
    return result_from_evaluation

def extract_stream_plan(node_from_atom, target_facts, stream_plan,
                        step_from_fact=None, step_from_stream=None):
    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: can optimize for all streams & axioms all at once
    for fact in target_facts:
        if fact not in node_from_atom:
            raise RuntimeError('Preimage fact {} is not achievable!'.format(fact))
            #RuntimeError: Preimage fact ('new-axiom@0',) is not achievable!
        result = node_from_atom[fact].result
        if result is None:
            continue
        if step_from_fact is not None:
            assert step_from_stream is not None
            step = step_from_fact[fact] if result.external.info.defer else 0
            step_from_stream[result] = min(step, step_from_stream.get(result, INF))
            for domain_fact in result.instance.get_domain():
                step_from_fact[domain_fact] = min(step_from_stream[result], step_from_stream.get(result, INF))
        extract_stream_plan(node_from_atom, result.instance.get_domain(), stream_plan,
                            step_from_fact, step_from_stream)
        # TODO: dynamic programming version of this
        if result not in stream_plan:
            stream_plan.append(result) # TODO: don't add if satisfied
