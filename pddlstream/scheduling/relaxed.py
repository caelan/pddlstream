import os
from collections import defaultdict, deque, namedtuple
from heapq import heappush, heappop

from pddlstream.conversion import obj_from_pddl_plan, is_atom, fact_from_evaluation
from pddlstream.fast_downward import get_problem, task_from_domain_problem, instantiate_task, run_search, safe_rm_dir, \
    parse_solution, \
    pddl_to_sas, clear_dir, TEMP_DIR, TRANSLATE_OUTPUT, apply_action, get_init
from pddlstream.scheduling.simultaneous import fact_from_fd, evaluations_from_stream_plan, extract_function_results, \
    get_results_from_head
from pddlstream.utils import Verbose, INF, MockSet


# TODO: reuse the ground problem when solving for sequential subgoals

def action_preimage(action, preimage):
    for conditions, effect in action.add_effects + action.del_effects:
        assert(not conditions)
        # TODO: can later select which conditional effects are used
        # TODO: might need to truely decide whether one should hold or not for a preimage. Maybe I should do that here
        if effect in preimage:
            preimage.remove(effect)
    preimage.update(action.precondition)

def axiom_preimage(axiom, preimage):
    if axiom.effect in preimage:
        preimage.remove(axiom.effect)
    preimage.update(axiom.condition)

Node = namedtuple('Node', ['effort', 'stream_result']) # TODO: level

def get_achieving_streams(evaluations, stream_results, op=sum):
    # TODO: could do this with bound_stream_instances instead
    unprocessed_from_atom = defaultdict(list)
    node_from_atom = {None: Node(0, None)}
    conditions_from_stream = {}
    remaining_from_stream = {}
    for stream_result in stream_results:
        conditions_from_stream[stream_result] = stream_result.instance.get_domain() + (None,)
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
        extract_stream_plan(node_from_atom, stream_result.instance.get_domain(), stream_plan)
        stream_plan.append(stream_result)

def get_achieving_axioms(state, axioms):
    unprocessed_from_atom = defaultdict(list)
    axiom_from_atom = {}
    remaining_from_stream = {}

    def process_axiom(axiom):
        if (not remaining_from_stream[id(axiom)]) and (axiom.effect not in axiom_from_atom):
            axiom_from_atom[axiom.effect] = axiom
            queue.append(axiom.effect)

    for atom in state:
        axiom_from_atom[atom] = None
    queue = deque(axiom_from_atom.keys())
    for axiom in axioms:
        for atom in axiom.condition:
            unprocessed_from_atom[atom].append(axiom)
        remaining_from_stream[id(axiom)] = len(axiom.condition)
        process_axiom(axiom)

    while queue:
        atom = queue.popleft()
        for axiom in unprocessed_from_atom[atom]:
            remaining_from_stream[id(axiom)] -= 1
            process_axiom(axiom)
    return axiom_from_atom

def extract_axioms(axiom_from_atom, facts, axiom_plan):
    for fact in facts:
        if fact not in axiom_from_atom:
            continue
        axiom = axiom_from_atom[fact]
        if (axiom is None) or (axiom in axiom_plan):
            continue
        extract_axioms(axiom_from_atom, axiom.condition, axiom_plan)
        axiom_plan.append(axiom)

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

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, unit_costs=True, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    problem = get_problem(opt_evaluations, goal_expression, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)

    with Verbose(False):
        ground_task = instantiate_task(task, simplify_axioms=False)
        if ground_task is None:
            return None, None, INF
        sas_task = pddl_to_sas(ground_task)
        clear_dir(TEMP_DIR)
        with open(os.path.join(TEMP_DIR, TRANSLATE_OUTPUT), "w") as output_file:
            sas_task.output(output_file)
    solution = run_search(TEMP_DIR, **kwargs)
    safe_rm_dir(TEMP_DIR)
    action_plan, action_cost = parse_solution(solution)
    if action_plan is None:
        return None, action_plan, action_cost

    action_from_parameters = defaultdict(list) # Could also obtain from below
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
    function_assignments = {fact.fluent: fact.expression for fact in task.init # init_facts
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    results_from_head = get_results_from_head(opt_evaluations)
    fluent_facts = MockSet()
    init_facts = set()

    fd_plan = []
    function_plan = set()
    for pair in action_plan:
        candidates = action_from_parameters[pair]
        assert(len(candidates) == 1) # TODO: extend to consider additional candidates
        action, args = candidates[0]
        variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        instance = action.instantiate(variable_mapping, init_facts,
                           fluent_facts, type_to_objects,
                           task.use_min_cost_metric, function_assignments)
        assert(instance is not None)
        fd_plan.append(instance)
        if not unit_costs:
            function_plan.update(extract_function_results(results_from_head, action, args))

    task.actions = []
    opt_state = set(task.init)
    real_state = set(real_init)
    preimage_plan = []
    for i in range(len(fd_plan)+1):
        task.init = opt_state
        with Verbose(False):
            model = build_model.compute_model(pddl_to_prolog.translate(task)) # Changes based on init
        axiom_model = filter(lambda a: isinstance(a.predicate, pddl.Axiom), model)
        fluent_facts = instantiate.get_fluent_facts(task, model) | (opt_state - real_state)
        instantiated_axioms = instantiate_axioms(axiom_model, real_state, fluent_facts)

        action = fd_plan[i] if i != len(fd_plan) else None
        goal_list = ground_task.goal_list if i == len(fd_plan) else []
        preconditions = goal_list if action is None else action.precondition
        with Verbose(False):
            axioms, axiom_init, _ = axiom_rules.handle_axioms(
                [] if action is None else [action], instantiated_axioms, goal_list)

        axiom_from_atom = get_achieving_axioms(opt_state, axioms) # TODO: add axiom_init here
        axiom_plan = []  # Could always add all conditions
        extract_axioms(axiom_from_atom, preconditions, axiom_plan)
        # TODO: add axiom init to reset state?
        preimage_plan += axiom_plan

        if action is not None:
            apply_action(opt_state, action)
            apply_action(real_state, action)
            preimage_plan.append(action)

    preimage = set(ground_task.goal_list)
    for action in reversed(preimage_plan):
        if isinstance(action, pddl.PropositionalAction):
            action_preimage(action, preimage)
        elif isinstance(action, pddl.PropositionalAxiom):
            axiom_preimage(action, preimage)
        else:
            raise ValueError(action)
    preimage -= set(real_init)
    preimage = filter(lambda l: not l.negated, preimage)
    #visualize_constraints(map(fact_from_fd, preimage))
    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: can optimize for all streams & axioms all at once

    node_from_atom = get_achieving_streams(evaluations, stream_results)
    stream_plan = []
    extract_stream_plan(node_from_atom, map(fact_from_fd, preimage), stream_plan)

    return (stream_plan + list(function_plan)), obj_from_pddl_plan(action_plan), action_cost
