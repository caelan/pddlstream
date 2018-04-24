import os
from collections import defaultdict
from collections import namedtuple
from heapq import heappush, heappop

from pddlstream.conversion import obj_from_pddl_plan, is_atom, evaluation_from_fact, fact_from_evaluation
from pddlstream.fast_downward import get_problem, task_from_domain_problem, instantiate_task, run_search, safe_rm_dir, parse_solution, \
    pddl_to_sas, clear_dir, TEMP_DIR, TRANSLATE_OUTPUT, apply_action, get_init
from pddlstream.sequential_scheduling import real_from_optimistic, evaluations_from_stream_plan
from pddlstream.simultaneous_scheduling import fact_from_fd
from pddlstream.utils import Verbose


# TODO: reuse the ground problem when solving for sequential subgoals

def action_preimage(action, preimage):
    for conditions, effect in action.add_effects + action.del_effects:
        assert(not conditions)
        # TODO: can later select which conditional effects are used
        # TODO: might need to truely decide whether one should hold or not for a preimage. Maybe I should do that here
        if effect in preimage:
            preimage.remove(effect)
    preimage.update(action.precondition)

# def clone_task(task, init=None, actions=None):
#     import pddl
#     if init is None:
#         init = task.init
#     if actions is None:
#         actions = task.init
#     return pddl.Task(
#         task.domain_name, task.task_name, task.requirements, task.types, task.objects,
#         task.predicates, task.functions, init, task.goal, actions, task.axioms, task.use_metric)

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

def instantiate_axioms(model, init_facts, fluent_facts):
    import pddl
    instantiated_axioms = []
    for atom in model:
        if isinstance(atom.predicate, pddl.Axiom):
            axiom = atom.predicate
            variable_mapping = dict([(par.name, arg)
                                     for par, arg in zip(axiom.parameters, atom.args)])
            inst_axiom = axiom.instantiate(variable_mapping, init_facts, fluent_facts)
            # TODO: can do a custom instantiation to preserve conditions
            # TODO: try catch for if precondition not included
            if inst_axiom:
                instantiated_axioms.append(inst_axiom)
    return instantiated_axioms

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
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

    action_from_parameters = defaultdict(list)
    for action in ground_task.reachable_action_params:
        for full_parameters in ground_task.reachable_action_params[action]:
            external_parameters = tuple(full_parameters[:action.num_external_parameters])
            action_from_parameters[(action.name, external_parameters)].append((action, full_parameters))
    # TODO: handle the negative stream fact case which is different

    import pddl_to_prolog
    import build_model
    import pddl
    import axiom_rules
    import instantiate

    real_init = get_init(evaluations)
    with Verbose(False):
        model = build_model.compute_model(pddl_to_prolog.translate(task))
        fluent_facts = instantiate.get_fluent_facts(task, model) | (set(task.init) - set(real_init))
    init_facts = set(real_init)
    function_assignments = {fact.fluent: fact.expression for fact in init_facts
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    #fluent_facts, init_facts, function_assignments, type_to_objects = real_from_optimistic(evaluations, task)

    fd_plan = []
    for pair in action_plan:
        candidates = action_from_parameters[pair]
        assert(len(candidates) == 1)
        action, parameters = candidates[0]
        variable_mapping = {p.name: a for p, a in zip(action.parameters, parameters)}
        fd_plan.append(action.instantiate(variable_mapping, init_facts,
                                         fluent_facts, type_to_objects,
                                         task.use_min_cost_metric, function_assignments))

    task.actions = []
    opt_state = set(task.init)
    real_state = set(real_init)
    for i in range(len(fd_plan)+1):
        task.init = opt_state
        #model = build_model.compute_model(pddl_to_prolog.translate(task)) # Changes based on init
        axiom_model = filter(lambda a: isinstance(a.predicate, pddl.Axiom), model)
        fluent_facts = instantiate.get_fluent_facts(task, model) | (opt_state - real_state)
        instantiated_axioms = instantiate_axioms(axiom_model, real_state, fluent_facts)
        actions = [fd_plan[i]] if i != len(fd_plan) else []
        goal_list = ground_task.goal_list if i == len(fd_plan) else []
        axioms, axiom_init, _ = axiom_rules.handle_axioms(
            actions, instantiated_axioms, goal_list) # TODO: axiom_init?
        print(len(model), len(axiom_model), len(instantiated_axioms), len(axioms))
        print(instantiated_axioms[:3])
        print(axioms[:3])
        # TODO: could always add all conditions

        if i != len(fd_plan):
            apply_action(opt_state, fd_plan[i])
            apply_action(real_state, fd_plan[i])

    preimage = set(ground_task.goal_list)
    for action in reversed(fd_plan):
        action_preimage(action, preimage)
    preimage -= set(real_init)
    preimage = filter(lambda l: not l.negated, preimage)
    #visualize_constraints(map(fact_from_fd, preimage))
    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: can optimize for all streams & axioms all at once

    node_from_atom = get_stream_effort(evaluations, stream_results)
    stream_plan = []
    extract_stream_plan(node_from_atom, map(fact_from_fd, preimage), stream_plan)

    return stream_plan, obj_from_pddl_plan(action_plan)
