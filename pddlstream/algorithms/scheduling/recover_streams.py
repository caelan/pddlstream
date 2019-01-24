from collections import namedtuple, defaultdict
from heapq import heappop, heappush

from pddlstream.language.conversion import is_atom, is_negated_atom, fact_from_evaluation
from pddlstream.language.effort import compute_result_effort, EFFORT_OP
from pddlstream.utils import HeapElement, INF

Node = namedtuple('Node', ['effort', 'result']) # TODO: include level

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
            effort = compute_result_effort(result, **effort_args)
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


def extract_stream_plan(node_from_atom, target_facts, stream_plan):
    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: can optimize for all streams & axioms all at once
    for fact in target_facts:
        if fact not in node_from_atom:
            raise RuntimeError('Preimage fact {} is not achievable!'.format(fact))
        stream_result = node_from_atom[fact].result
        if (stream_result is None) or (stream_result in stream_plan):
            continue
        extract_stream_plan(node_from_atom, stream_result.instance.get_domain(), stream_plan)
        stream_plan.append(stream_result) # TODO: don't add if satisfied
