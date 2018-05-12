import os
from collections import defaultdict, deque, namedtuple, OrderedDict
from heapq import heappush, heappop

from pddlstream.conversion import obj_from_pddl_plan, is_atom, fact_from_evaluation, obj_from_pddl, And
from pddlstream.fast_downward import get_problem, task_from_domain_problem, instantiate_task, run_search, safe_rm_dir, \
    parse_solution, pddl_to_sas, clear_dir, TEMP_DIR, TRANSLATE_OUTPUT, apply_action, get_init, fact_from_fd, solve_from_task
from pddlstream.scheduling.simultaneous import evaluations_from_stream_plan, extract_function_results, \
    get_results_from_head
from pddlstream.utils import Verbose, INF, MockSet, find_unique, implies, argmax
from pddlstream.function import PredicateResult
from pddlstream.algorithm import neighbors_from_orders
from pddlstream.scheduling.simultaneous import get_stream_actions


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

def plan_preimage(plan, goal):
    import pddl
    preimage = set(goal)
    for action in reversed(plan):
        if isinstance(action, pddl.PropositionalAction):
            action_preimage(action, preimage)
        elif isinstance(action, pddl.PropositionalAxiom):
            axiom_preimage(action, preimage)
        else:
            raise ValueError(action)
    return preimage

##################################################

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
        stream_plan.append(stream_result) # TODO: don't add if satisfied

##################################################

def instantiate_axioms(task, model, init_facts, fluent_facts):
    import pddl
    instantiated_axioms = []
    for atom in model:
        if isinstance(atom.predicate, pddl.Axiom):
            #axiom = atom.predicate
            axiom = find_unique(lambda ax: ax.name == atom.predicate.name, task.axioms)
            variable_mapping = dict([(par.name, arg)
                                     for par, arg in zip(axiom.parameters, atom.args)])
            inst_axiom = axiom.instantiate(variable_mapping, init_facts, fluent_facts)
            # TODO: can do a custom instantiation to preserve conditions
            # TODO: try catch for if precondition not included
            if inst_axiom:
                instantiated_axioms.append(inst_axiom)
    return instantiated_axioms

##################################################

def get_achieving_axioms(state, axioms, axiom_init, negated_from_name={}):
    unprocessed_from_atom = defaultdict(list)
    axiom_from_atom = {}
    remaining_from_stream = {}

    def process_axiom(axiom):
        if (not remaining_from_stream[id(axiom)]) and (axiom.effect not in axiom_from_atom):
            axiom_from_atom[axiom.effect] = axiom
            queue.append(axiom.effect)

    for atom in list(state) + axiom_init:
        axiom_from_atom[atom] = None
    queue = deque(axiom_from_atom.keys())
    for axiom in axioms:
        conditions = filter(lambda a: a.predicate not in negated_from_name, axiom.condition)
        for atom in conditions:
            #if atom.negate() not in axiom_init:
            unprocessed_from_atom[atom].append(axiom)
        remaining_from_stream[id(axiom)] = len(conditions)
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

##################################################

def get_goal_instance(goal):
    import pddl
    #name = '@goal-reachable'
    name = '@goal'
    precondition =  goal.parts if isinstance(goal, pddl.Conjunction) else [goal]
    return pddl.PropositionalAction(name, precondition, [], None)

def recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_plan, negative,
                        unit_costs, optimize=True):
    import pddl_to_prolog
    import build_model
    import pddl
    import axiom_rules
    import instantiate

    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs))
    real_init = get_init(evaluations)
    function_assignments = {fact.fluent: fact.expression for fact in task.init  # init_facts
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    results_from_head = get_results_from_head(opt_evaluations)
    fluent_facts = MockSet()
    init_facts = set()

    action_instances = []
    function_plan = set()
    for name, args in action_plan:
        action = find_unique(lambda a: a.name == name, task.actions)
        assert (len(action.parameters) == len(args))
        variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        instance = action.instantiate(variable_mapping, init_facts,
                                      fluent_facts, type_to_objects,
                                      task.use_min_cost_metric, function_assignments)
        assert (instance is not None)
        action_instances.append(instance)
        if not unit_costs:
            function_plan.update(extract_function_results(results_from_head, action, args))
    action_instances.append(get_goal_instance(task.goal))
    # TODO: negative atoms in actions

    negative_from_name = {n.name: n for n in negative}
    task.actions = []
    opt_state = set(task.init)
    real_state = set(real_init)
    preimage_plan = []
    for instance in action_instances:
        task.init = opt_state
        original_axioms = task.axioms
        if negative_from_name:
            task.axioms = []
            for axiom in original_axioms:
                assert (isinstance(axiom.condition, pddl.Conjunction))
                parts = filter(lambda a: a.predicate not in negative_from_name, axiom.condition.parts)
                task.axioms.append(pddl.Axiom(axiom.name, axiom.parameters,
                                              axiom.num_external_parameters, pddl.Conjunction(parts)))
        with Verbose(False):
            model = build_model.compute_model(pddl_to_prolog.translate(task))  # Changes based on init
        task.axioms = original_axioms

        fluent_facts = instantiate.get_fluent_facts(task, model) | (opt_state - real_state)
        mock_fluent = MockSet(lambda item: (item.predicate in negative_from_name) or
                                           (item in fluent_facts))
        instantiated_axioms = instantiate_axioms(task, model, real_state, mock_fluent)

        with Verbose(False):
            helpful_axioms, axiom_init, _ = axiom_rules.handle_axioms([instance], instantiated_axioms, [])
        axiom_from_atom = get_achieving_axioms(opt_state, helpful_axioms, axiom_init, negative_from_name)
        axiom_plan = []  # Could always add all conditions
        extract_axioms(axiom_from_atom, instance.precondition, axiom_plan)
        # TODO: add axiom init to reset state?
        preimage_plan += axiom_plan
        apply_action(opt_state, instance)
        apply_action(real_state, instance)
        preimage_plan.append(instance)

    preimage = plan_preimage(preimage_plan, set())
    preimage -= set(real_init)
    negative_preimage = filter(lambda a: a.predicate in negative_from_name, preimage)
    preimage -= set(negative_preimage)
    preimage = filter(lambda l: not l.negated, preimage)
    # visualize_constraints(map(fact_from_fd, preimage))
    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: can optimize for all streams & axioms all at once

    for literal in negative_preimage:
        negative = negative_from_name[literal.predicate]
        instance = negative.get_instance(map(obj_from_pddl, literal.args))
        value = not literal.negated
        if instance.enumerated:
            assert (instance.value == value)
        else:
            function_plan.add(PredicateResult(instance, value))

    node_from_atom = get_achieving_streams(evaluations, stream_results)
    preimage_facts = map(fact_from_fd, preimage)
    stream_plan = []
    extract_stream_plan(node_from_atom, preimage_facts, stream_plan)
    if not optimize: # TODO: detect this based on unique or not
        return stream_plan + list(function_plan)

    # TODO: search in space of partially ordered plans
    # TODO: local optimization - remove one and see if feasible

    task = task_from_domain_problem(domain, get_problem(evaluations, And(*preimage_facts), domain, unit_costs=True))
    task.actions, stream_result_from_name = get_stream_actions(stream_results)
    new_plan, _ = solve_from_task(task, planner='max-astar', debug=False)
    # TODO: investigate admissible heuristics
    if new_plan is None:
        return stream_plan + list(function_plan)

    new_stream_plan = [stream_result_from_name[name] for name, _ in new_plan]
    return new_stream_plan + list(function_plan)

##################################################

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, negative, unit_costs, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs))
    action_plan, action_cost = solve_from_task(task, **kwargs)
    if action_plan is None:
        return None, action_cost

    stream_plan = recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_plan,
                                      negative, unit_costs)
    action_plan = obj_from_pddl_plan(action_plan)
    combined_plan = stream_plan + action_plan

    return combined_plan, action_cost
