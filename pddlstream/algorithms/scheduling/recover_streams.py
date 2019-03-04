from collections import namedtuple, defaultdict
from heapq import heappop, heappush

from pddlstream.algorithms.downward import task_from_domain_problem, get_problem, plan_preimage, fact_from_fd
from pddlstream.algorithms.scheduling.apply_fluents import convert_fluent_streams
from pddlstream.algorithms.scheduling.negative import get_negative_predicates, recover_negative_axioms, convert_negative
from pddlstream.algorithms.scheduling.recover_functions import compute_function_plan
from pddlstream.language.constants import get_prefix, EQ

from pddlstream.language.conversion import is_atom, is_negated_atom, fact_from_evaluation, evaluation_from_fact
from pddlstream.language.function import Function
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

##################################################

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

def recover_stream_plan(evaluations, current_plan, opt_evaluations, goal_expression, domain, node_from_atom,
                        action_plan, axiom_plans, negative):
    from pddlstream.algorithms.scheduling.postprocess import postprocess_stream_plan
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects
    # TODO: node_from_atom is a subset of opt_evaluations (only missing functions)
    real_task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain))
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain))
    negative_from_name = get_negative_predicates(negative)

    real_states, combined_plan = recover_negative_axioms(
        real_task, opt_task, axiom_plans, action_plan, negative_from_name)
    function_plan = compute_function_plan(opt_evaluations, action_plan)

    full_preimage = plan_preimage(combined_plan, [])
    stream_preimage = set(full_preimage) - real_states[0]
    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, stream_preimage))
    function_plan.update(convert_negative(negative_preimage, negative_from_name, full_preimage, real_states))
    positive_preimage = stream_preimage - negative_preimage

    step_from_fact = {fact_from_fd(l): full_preimage[l] for l in positive_preimage if not l.negated}
    target_facts = {fact for fact in step_from_fact.keys() if get_prefix(fact) != EQ}
    #stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_results)
    # visualize_constraints(map(fact_from_fd, target_facts))

    stream_plan = []
    for result in current_plan:
        if isinstance(result.external, Function) or (result.external in negative):
            function_plan.add(result) # Prevents these results from being pruned
        else:
            stream_plan.append(result)
    curr_evaluations = evaluations_from_stream_plan(evaluations, stream_plan, max_effort=None)
    extraction_facts = target_facts - set(map(fact_from_evaluation, curr_evaluations))
    extract_stream_plan(node_from_atom, extraction_facts, stream_plan)
    stream_plan = postprocess_stream_plan(evaluations, domain, stream_plan, target_facts)
    stream_plan = convert_fluent_streams(stream_plan, real_states, action_plan, step_from_fact, node_from_atom)

    return stream_plan + list(function_plan)
