from collections import defaultdict, deque, namedtuple
from heapq import heappush, heappop

from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, apply_action, fact_from_fd, \
    solve_from_task, get_literals, conditions_hold, OBJECT
from pddlstream.algorithms.scheduling.simultaneous import evaluations_from_stream_plan, extract_function_results, \
    get_results_from_head, get_stream_actions
from pddlstream.language.conversion import obj_from_pddl_plan, is_atom, fact_from_evaluation, obj_from_pddl, \
    And, get_prefix, get_args
from pddlstream.language.function import PredicateResult, Predicate
from pddlstream.language.state_stream import StateStream
from pddlstream.utils import Verbose, MockSet, HeapElement


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
    none = (None,) # None
    node_from_atom = {none: Node(0, None)}
    conditions_from_stream = {}
    remaining_from_stream = {}
    for stream_result in stream_results:
        conditions_from_stream[stream_result] = stream_result.instance.get_domain() + (none,)
        remaining_from_stream[stream_result] = len(conditions_from_stream[stream_result])
        for atom in conditions_from_stream[stream_result]:
            unprocessed_from_atom[atom].append(stream_result)
    for atom in evaluations:
        if is_atom(atom):
            node_from_atom[fact_from_evaluation(atom)] = Node(0, None)

    queue = [HeapElement(node.effort, atom) for atom, node in node_from_atom.items()]
    while queue:
        atom = heappop(queue).value
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
                    heappush(queue, HeapElement(total_effort, new_atom))
        del unprocessed_from_atom[atom]
    del node_from_atom[none]
    return node_from_atom

def extract_stream_plan(node_from_atom, facts, stream_plan):
    for fact in facts:
        if fact not in node_from_atom:
            raise RuntimeError('Preimage fact {} is not achievable!'.format(fact))
        stream_result = node_from_atom[fact].stream_result
        if (stream_result is None) or (stream_result in stream_plan):
            continue
        extract_stream_plan(node_from_atom, stream_result.instance.get_domain(), stream_plan)
        stream_plan.append(stream_result) # TODO: don't add if satisfied

##################################################

def instantiate_axioms(model, init_facts, fluent_facts, axiom_remap={}):
    import pddl
    instantiated_axioms = []
    for atom in model:
        if isinstance(atom.predicate, pddl.Axiom):
            axiom = axiom_remap.get(atom.predicate, atom.predicate)
            variable_mapping = dict([(par.name, arg)
                                     for par, arg in zip(axiom.parameters, atom.args)])
            inst_axiom = axiom.instantiate(variable_mapping, init_facts, fluent_facts)
            if inst_axiom:
                instantiated_axioms.append(inst_axiom)
    return instantiated_axioms

##################################################

def get_derived_predicates(axioms):
    axioms_from_name = defaultdict(list)
    for axiom in axioms:
        axioms_from_name[axiom.name].append(axiom)
    return axioms_from_name

def get_necessary_axioms(instance, axioms, negative_from_name):
    import pddl

    axioms_from_name = get_derived_predicates(axioms)
    atom_queue = []
    processed_atoms = set()
    def add_literals(literals):
        for literal in literals:
            atom = literal.positive()
            if atom not in processed_atoms:
                atom_queue.append(literal.positive())
                processed_atoms.add(atom)

    add_literals(instance.precondition)
    for (cond, _) in (instance.add_effects + instance.del_effects):
        add_literals(cond)

    axiom_from_action = {}
    partial_instantiations = set()
    while atom_queue:
        literal = atom_queue.pop()
        for axiom in axioms_from_name[literal.predicate]:
            derived_parameters = axiom.parameters[:axiom.num_external_parameters]
            var_mapping = {p.name: a for p, a in zip(derived_parameters, literal.args) if a[0] != '?'}
            key = (axiom, frozenset(var_mapping.items()))
            if key in partial_instantiations:
                continue
            partial_instantiations.add(key)
            parts = []
            for literal in get_literals(axiom.condition):
                if literal.predicate in negative_from_name:
                    continue
                parts.append(literal.rename_variables(var_mapping))
            # new_condition = axiom.condition.uniquify_variables(None, var_mapping)
            effect_args = [var_mapping.get(a.name, a.name) for a in derived_parameters]
            effect = pddl.Effect([], pddl.Truth(), pddl.conditions.Atom(axiom.name, effect_args))
            free_parameters = [p for p in axiom.parameters if p.name not in var_mapping]
            new_action = pddl.Action(axiom.name, free_parameters, 0, pddl.Conjunction(parts), [effect], None)
            # Creating actions so I can partially instantiate (impossible with axioms)
            axiom_from_action[new_action] = (axiom, var_mapping)
            add_literals(parts)
    return axiom_from_action

def instantiate_necessary_axioms(model, init_facts, fluent_facts, axiom_remap={}):
    import pddl
    instantiated_axioms = []
    for atom in model:
        if isinstance(atom.predicate, pddl.Action):
            action = atom.predicate
            var_mapping = {p.name: a for p, a in zip(action.parameters, atom.args)}
            axiom, existing_var_mapping = axiom_remap[action]
            var_mapping.update(existing_var_mapping)
            inst_axiom = axiom.instantiate(var_mapping, init_facts, fluent_facts)
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
        conditions = list(filter(lambda a: a.predicate not in negated_from_name, axiom.condition))
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
    #precondition = get_literals(goal)
    return pddl.PropositionalAction(name, precondition, [], None)

def get_state_stream_result(state_stream, literal, real_state):
    import pddl
    assert literal.negated
    [cert] = state_stream.certified
    # TODO: only consider facts produced by initial/actions
    object_from_input = dict(zip(get_args(cert), map(obj_from_pddl, literal.args)))
    input_objects = tuple(object_from_input[inp] for inp in state_stream.inputs)
    fluent_facts = list(filter(lambda f: isinstance(f, pddl.Atom) and
                                         (f.predicate in state_stream.fluents), real_state))
    state_instance = state_stream.get_instance(input_objects, map(fact_from_fd, fluent_facts))
    assert not state_stream.outputs
    output_objects = tuple()
    return state_stream._Result(state_instance, output_objects)

def recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_plan, negative,
                        unit_costs, optimize=True):
    import pddl_to_prolog
    import build_model
    import pddl
    import axiom_rules
    import instantiate
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects

    real_task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs))
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs))
    function_assignments = {fact.fluent: fact.expression for fact in opt_task.init  # init_facts
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(opt_task.objects, opt_task.types)
    results_from_head = get_results_from_head(opt_evaluations)

    negative_from_name = {n.name: n for n in filter(lambda n: isinstance(n, Predicate), negative)}
    for n in filter(lambda n: isinstance(n, StateStream), negative):
        for p, np in n.negated_predicates.items():
            negative_from_name[np] = n
            #negative_from_name[np] = p

    action_instances = []
    for name, args in action_plan: # TODO: negative atoms in actions
        candidates = []
        for action in opt_task.actions:
            if action.name != name:
                continue
            if len(action.parameters) != len(args):
                raise NotImplementedError('Existential quantifiers are not currently '
                                          'supported in preconditions: {}'.format(name))
            variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
            action_instance = action.instantiate(variable_mapping, set(),
                                          MockSet(), type_to_objects,
                                          opt_task.use_min_cost_metric, function_assignments)
            assert (action_instance is not None)
            candidates.append(((action, args), action_instance))
        if not candidates:
            raise RuntimeError('Could not find an applicable action {}'.format(name))
        action_instances.append(candidates)
    action_instances.append([(None, get_goal_instance(opt_task.goal))])

    axioms_from_name = get_derived_predicates(opt_task.axioms)
    opt_task.actions = []
    opt_state = set(opt_task.init)
    real_state = set(real_task.init)
    real_states = [real_state]
    preimage_plan = []
    indices_from_atoms = defaultdict(set)
    function_plan = set()
    state_plan = set()
    for layer in action_instances:
        for pair, action_instance in layer:
            nonderived_preconditions = [l for l in action_instance.precondition if l.predicate not in axioms_from_name]
            #nonderived_preconditions = action_instance.precondition
            if not conditions_hold(opt_state, nonderived_preconditions):
                continue
            opt_task.init = opt_state
            original_axioms = opt_task.axioms
            axiom_from_action = get_necessary_axioms(action_instance, original_axioms, negative_from_name)
            opt_task.axioms = []
            opt_task.actions = axiom_from_action.keys()
            # TODO: maybe it would just be better to drop the negative throughout this process until this end
            with Verbose(False):
                model = build_model.compute_model(pddl_to_prolog.translate(opt_task))  # Changes based on init
            opt_task.axioms = original_axioms

            delta_state = opt_state - real_state
            opt_facts = instantiate.get_fluent_facts(opt_task, model) | delta_state
            mock_fluent = MockSet(lambda item: (item.predicate in negative_from_name) or
                                               (item in opt_facts))
            instantiated_axioms = instantiate_necessary_axioms(model, real_state, mock_fluent, axiom_from_action)
            with Verbose(False):
                helpful_axioms, axiom_init, _ = axiom_rules.handle_axioms([action_instance], instantiated_axioms, [])
            axiom_from_atom = get_achieving_axioms(opt_state, helpful_axioms, axiom_init, negative_from_name)
            axiom_plan = []  # Could always add all conditions
            extract_axioms(axiom_from_atom, action_instance.precondition, axiom_plan)
            # TODO: test if no derived solution

            # TODO: compute required stream facts in a forward way and allow opt facts that are already known required
            for effects in [action_instance.add_effects, action_instance.del_effects]:
                for i, (conditions, effect) in enumerate(effects[::-1]):
                    if any(c.predicate in axioms_from_name for c in conditions):
                        raise NotImplementedError('Conditional effects cannot currently involve derived predicates')
                    if conditions_hold(real_state, conditions):
                        # Holds in real state
                        effects[i] = ([], effect)
                    elif not conditions_hold(opt_state, conditions):
                        # Does not hold in optimistic state
                        effects.pop(i)
                    else:
                        # TODO: handle more general case where can choose to achieve particular conditional effects
                        raise NotImplementedError('Conditional effects cannot currently involve certified predicates')

            for literal in action_instance.precondition:
                if literal.predicate in negative_from_name:
                    negated = negative_from_name[literal.predicate]
                    state_plan.add(get_state_stream_result(negated, literal, real_state))

            # TODO: add axiom init to reset state?
            layer_plan = axiom_plan + [action_instance]
            for atom in (plan_preimage(layer_plan, []) - real_state):
                indices_from_atoms[atom].add(len(real_states))
            preimage_plan.extend(layer_plan)
            apply_action(opt_state, action_instance)
            apply_action(real_state, action_instance)
            real_states.append(real_state) # TODO: only add initial fluents propagated?
            if not unit_costs and (pair is not None):
                function_plan.update(extract_function_results(results_from_head, *pair))
            break
        else:
            raise RuntimeError('No action instances are applicable')

    # TODO: could instead just accumulate difference between real and opt
    preimage = plan_preimage(preimage_plan, []) - set(real_task.init)
    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, preimage))
    preimage -= negative_preimage
    # visualize_constraints(map(fact_from_fd, preimage))
    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: can optimize for all streams & axioms all at once

    for literal in negative_preimage:
        #print(literal, indices_from_atoms[literal])
        #for i in indices_from_atoms[literal]:
        #    print(i, real_states[i])
        negative = negative_from_name[literal.predicate]
        if isinstance(negative, StateStream):
            continue
        action_instance = negative.get_instance(map(obj_from_pddl, literal.args))
        value = not literal.negated
        if action_instance.enumerated:
            assert (action_instance.value == value)
        else:
            function_plan.add(PredicateResult(action_instance, value, opt_index=action_instance.opt_index))
    #print(indices_from_atoms)
    #raw_input('awefwaf')

    node_from_atom = get_achieving_streams(evaluations, stream_results)
    preimage_facts = list(map(fact_from_fd, filter(lambda l: not l.negated, preimage)))
    stream_plan = []
    extract_stream_plan(node_from_atom, preimage_facts, stream_plan)
    initial_plan = stream_plan + list(function_plan) + list(state_plan)
    if not optimize: # TODO: detect this based on unique or not
        return initial_plan

    # TODO: search in space of partially ordered plans
    # TODO: local optimization - remove one and see if feasible

    reschedule_problem = get_problem(evaluations, And(*preimage_facts), domain, unit_costs=True)
    reschedule_task = task_from_domain_problem(domain, reschedule_problem)
    reschedule_task.actions, stream_result_from_name = get_stream_actions(stream_results)
    new_plan, _ = solve_from_task(reschedule_task, planner='max-astar', debug=False)
    # TODO: investigate admissible heuristics
    if new_plan is None:
        return initial_plan

    new_stream_plan = [stream_result_from_name[name] for name, _ in new_plan]
    return new_stream_plan + list(function_plan) + list(state_plan)

##################################################

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, negative, unit_costs, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    problem = get_problem(opt_evaluations, goal_expression, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    action_plan, action_cost = solve_from_task(task, **kwargs)
    if action_plan is None:
        return None, action_cost
    # TODO: just use solve finite?

    stream_plan = recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_plan,
                                      negative, unit_costs)
    action_plan = obj_from_pddl_plan(action_plan)
    combined_plan = stream_plan + action_plan

    return combined_plan, action_cost
