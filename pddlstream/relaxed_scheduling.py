import os
from collections import defaultdict
from collections import namedtuple
from heapq import heappush, heappop

from pddlstream.conversion import obj_from_pddl_plan, is_atom, evaluation_from_fact, fact_from_evaluation
from pddlstream.fast_downward import get_problem, task_from_domain_problem, instantiate_task, run_search, safe_rm_dir, parse_solution, \
    pddl_to_sas, clear_dir, TEMP_DIR, TRANSLATE_OUTPUT
from pddlstream.sequential_scheduling import real_from_optimistic, evaluations_from_stream_plan
from pddlstream.simultaneous_scheduling import fact_from_fd
from pddlstream.utils import Verbose


# TODO: reuse the ground problem when solving for sequential subgoals

def action_preimage(action, preimage):
    for conditions, effect in action.add_effects + action.del_effects:
        assert(not conditions)
        if effect in preimage:
            preimage.remove(effect)
    preimage.update(action.precondition)


Node = namedtuple('Node', ['effort', 'stream_result']) # TODO: level

def get_stream_effort(evaluations, stream_results, op=sum):
    # TODO: could do this with bound_stream_instances instead
    unprocessed_from_atom = defaultdict(list)
    node_from_atom = {None: Node(0, None)}
    conditions_from_stream = {}
    remaining_from_stream = {}
    for stream_result in stream_results:
        conditions_from_stream[stream_result] = stream_result.stream_instance.get_domain() + (None,)
        remaining_from_stream[stream_result] = len(conditions_from_stream[stream_result])
        for atom in conditions_from_stream[stream_result]:
            unprocessed_from_atom[atom].append(stream_result)
    for atom in evaluations:
        if is_atom(atom):
            node_from_atom[fact_from_evaluation(atom)] = Node(0, None)

    queue = [(node.effort, atom) for atom, node in node_from_atom.items()]
    while queue:
        _, atom = heappop(queue)
        if atom not in unprocessed_from_atom:
            continue
        for stream_result in unprocessed_from_atom[atom]:
            remaining_from_stream[stream_result] -= 1
            if remaining_from_stream[stream_result]:
                continue
            effort = 1
            total_effort = op(node_from_atom[cond].effort for cond in conditions_from_stream[stream_result]) + effort
            for new_atom in stream_result.get_certified():
                if (new_atom not in node_from_atom) or (total_effort < node_from_atom[new_atom].effort):
                    node_from_atom[new_atom] = Node(total_effort, stream_result)
                    heappush(queue, (total_effort, new_atom))
        del unprocessed_from_atom[atom]
    del node_from_atom[None]
    return node_from_atom

def extract_stream_plan(node_from_atom, facts, stream_plan):
    for fact in facts:
        stream_result = node_from_atom[fact].stream_result
        if (stream_result is None) or (stream_result in stream_plan):
            continue
        extract_stream_plan(node_from_atom, stream_result.stream_instance.get_domain(), stream_plan)
        stream_plan.append(stream_result)


def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    # TODO: alternatively could translate with stream actions on real state and just discard them
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain))

    with Verbose(False):
        ground_task = instantiate_task(task)
        sas_task = pddl_to_sas(ground_task)
        clear_dir(TEMP_DIR)
        with open(os.path.join(TEMP_DIR, TRANSLATE_OUTPUT), "w") as output_file:
            sas_task.output(output_file)
    solution = run_search(TEMP_DIR, verbose=False, **kwargs)
    safe_rm_dir(TEMP_DIR)
    action_plan, _ = parse_solution(solution)
    if action_plan is None:
        return None, action_plan

    print(task.axioms, ground_task.original_axioms, ground_task.axioms)
    for axiom in ground_task.original_axioms:
        print(axiom)

    for axiom in ground_task.axioms:
        print(axiom.condition)
        print(axiom.effect)
    assert(not ground_task.axioms)

    action_from_parameters = defaultdict(list)
    for action in ground_task.reachable_action_params:
        for full_parameters in ground_task.reachable_action_params[action]:
            external_parameters = tuple(full_parameters[:action.num_external_parameters])
            action_from_parameters[(action.name, external_parameters)].append((action, full_parameters))


    fluent_facts, init_facts, function_assignments, type_to_objects = real_from_optimistic(evaluations, task)

    import pddl_to_prolog
    import build_model
    import pddl
    import axiom_rules
    instantiated_axioms = []
    with Verbose(False):
        model = build_model.compute_model(pddl_to_prolog.translate(task))
        for atom in model:
            if isinstance(atom.predicate, pddl.Axiom):
                axiom = atom.predicate
                variable_mapping = dict([(par.name, arg)
                                         for par, arg in zip(axiom.parameters, atom.args)])
                inst_axiom = axiom.instantiate(variable_mapping, init_facts, fluent_facts)
                if inst_axiom:
                    instantiated_axioms.append(inst_axiom)

    axioms, axiom_init, axiom_layer_dict = axiom_rules.handle_axioms(
        ground_task.actions, instantiated_axioms, ground_task.goal_list)

    print(len(instantiated_axioms))
    print(len(axioms))
    # TODO: can instantiate in each state without axioms as well (multiple small model builds)


    """    
    print(plan_cost(action_plan))
    state = set(task.init)
    axiom_plan = []
    # TODO: remove conditional effects
    for action in action_plan:
        axiom_plan.append([])
        assert(is_applicable(state, action))
        apply_action(state, action)
        print(state)
    """


    fd_plan = []
    for pair in action_plan:
        candidates = action_from_parameters[pair]
        assert(len(candidates) == 1)
        action, parameters = candidates[0]
        variable_mapping = {p.name: a for p, a in zip(action.parameters, parameters)}
        fd_plan.append(action.instantiate(variable_mapping, init_facts,
                                         fluent_facts, type_to_objects,
                                         task.use_min_cost_metric, function_assignments))

    preimage = set(ground_task.goal_list)
    for action in reversed(fd_plan):
        action_preimage(action, preimage)
    preimage -= init_facts
    preimage = filter(lambda l: not l.negated, preimage)
    # TODO: need to include all conditions
    # TODO: need to invert axioms back

    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: backtrace streams and axioms
    # TODO: can optimize for streams & axioms all at once
    #visualize_constraints(map(fact_from_fd, preimage))

    node_from_atom = get_stream_effort(evaluations, stream_results)
    stream_plan = []
    extract_stream_plan(node_from_atom, map(fact_from_fd, preimage), stream_plan)

    return stream_plan, obj_from_pddl_plan(action_plan)
