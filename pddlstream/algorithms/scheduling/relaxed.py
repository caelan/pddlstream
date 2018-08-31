from collections import defaultdict, deque, namedtuple
from heapq import heappush, heappop

from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, apply_action, fact_from_fd, \
    get_literals, conditions_hold, fd_from_fact, get_goal_instance
from pddlstream.algorithms.scheduling.simultaneous import evaluations_from_stream_plan, extract_function_results, \
    get_results_from_head, get_stream_actions
from pddlstream.algorithms.search import abstrips_solve_from_task, solve_from_task
from pddlstream.language.conversion import obj_from_pddl_plan, is_atom, fact_from_evaluation, obj_from_pddl, And
from pddlstream.language.function import PredicateResult, Predicate
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.utils import Verbose, MockSet, HeapElement

# TODO: reuse the ground problem when solving for sequential subgoals

def add_preimage_condition(condition, preimage, i):
    for literal in condition:
        #preimage[literal] = preimage.get(literal, set()) | {i}
        preimage.setdefault(literal, set()).add(i)
    #preimage.update(condition)

def add_preimage_effect(effect, preimage):
    preimage.pop(effect, None)
    #if effect in preimage:
    #    # Fluent effects kept, static dropped
    #    preimage.remove(effect)

def action_preimage(action, preimage, i):
    for conditions, effect in (action.add_effects + action.del_effects):
        assert(not conditions)
        # TODO: can later select which conditional effects are used
        # TODO: might need to truely decide whether one should hold or not for a preimage. Maybe I should do that here
        add_preimage_effect(effect, preimage)
    add_preimage_condition(action.precondition, preimage, i)

def axiom_preimage(axiom, preimage, i):
    add_preimage_effect(axiom.effect, preimage)
    add_preimage_condition(axiom.condition, preimage, i)

def plan_preimage(plan, goal):
    import pddl
    # TODO: store layers in which each of these where used
    #preimage = set(goal)
    preimage = {condition: {len(plan)} for condition in goal}
    for i, operator in reversed(list(enumerate(plan))):
        if isinstance(operator, pddl.PropositionalAction):
            action_preimage(operator, preimage, i)
        elif isinstance(operator, pddl.PropositionalAxiom):
            axiom_preimage(operator, preimage, i)
        else:
            raise ValueError(operator)
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
            new_action = pddl.Action(axiom.name, free_parameters, len(free_parameters),
                                     pddl.Conjunction(parts), [effect], None)
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

#def get_predicate_result():
#    pass # TODO: refactor

def instantiate_actions(opt_task, type_to_objects, function_assignments, action_plan):
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
    return action_instances

def simplify_conditional_effects(real_state, opt_state, action_instance, axioms_from_name):
    # TODO: compute required stream facts in a forward way and allow opt facts that are already known required
    for effects in [action_instance.add_effects, action_instance.del_effects]:
        for i, (conditions, effect) in reversed(list(enumerate(effects))):
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

def reschedule(evaluations, preimage_facts, domain, stream_results):
    # TODO: search in space of partially ordered plans
    # TODO: local optimization - remove one and see if feasible
    reschedule_problem = get_problem(evaluations, And(*preimage_facts), domain, unit_costs=True)
    reschedule_task = task_from_domain_problem(domain, reschedule_problem)
    reschedule_task.actions, stream_result_from_name = get_stream_actions(stream_results)
    new_plan, _ = solve_from_task(reschedule_task, planner='ff-astar', debug=True)
    # TODO: investigate other admissible heuristics
    return [stream_result_from_name[name] for name, _ in new_plan]

def convert_fluent_streams(stream_plan, real_states, preimage, node_from_atom):
    import pddl
    new_stream_plan = []
    for result in stream_plan:
        external = result.instance.external
        if (result.opt_index != 0) or (not external.is_fluent()):
            new_stream_plan.append(result)
            continue
        state_indices = set()
        for atom in result.get_certified():
            if node_from_atom[atom].stream_result == result:
                state_indices.update(preimage[fd_from_fact(atom)])
        if len(state_indices) != 1:
            raise NotImplementedError() # Pass all fluents and make two axioms
        # TODO: can handle case where no outputs easily
        [state_index] = state_indices
        fluent_facts = list(map(fact_from_fd, filter(lambda f: isinstance(f, pddl.Atom) and (f.predicate in external.fluents), real_states[state_index])))
        new_instance = external.get_instance(result.instance.input_objects, fluent_facts=fluent_facts)
        new_stream_plan.append(external._Result(new_instance, result.output_objects, opt_index=result.opt_index))
    return new_stream_plan

def recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_plan, negative,
                        unit_costs, optimize=False):
    # TODO: toggle optimize more
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
    action_instances = instantiate_actions(opt_task, type_to_objects, function_assignments, action_plan)
    negative_from_name = {external.name: external for external in negative if isinstance(external, Predicate)}
    negative_from_name.update({external.blocked_predicate: external for external in negative
                               if isinstance(external, Stream) and external.is_negated()})

    axioms_from_name = get_derived_predicates(opt_task.axioms)
    opt_task.actions = []
    opt_state = set(opt_task.init)
    real_state = set(real_task.init)
    real_states = [real_state.copy()] # TODO: had old way of doing this (~July 2018)
    preimage_plan = []
    function_plan = set()
    for layer in action_instances:
        for pair, action_instance in layer:
            nonderived_preconditions = [l for l in action_instance.precondition
                                        if l.predicate not in axioms_from_name]
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

            delta_state = (opt_state - real_state) # Optimistic facts
            opt_facts = instantiate.get_fluent_facts(opt_task, model) | delta_state
            mock_fluent = MockSet(lambda item: (item.predicate in negative_from_name) or (item in opt_facts))
            instantiated_axioms = instantiate_necessary_axioms(model, real_state, mock_fluent, axiom_from_action)
            with Verbose(False):
                helpful_axioms, axiom_init, _ = axiom_rules.handle_axioms([action_instance], instantiated_axioms, [])
            axiom_from_atom = get_achieving_axioms(opt_state, helpful_axioms, axiom_init, negative_from_name)
            axiom_plan = []  # Could always add all conditions
            extract_axioms(axiom_from_atom, action_instance.precondition, axiom_plan)
            simplify_conditional_effects(real_state, opt_state, action_instance, axioms_from_name)
            # TODO: test if no derived solution

            # TODO: add axiom init to reset state?
            preimage_plan.extend(axiom_plan + [action_instance])
            apply_action(opt_state, action_instance)
            apply_action(real_state, action_instance)
            real_states.append(real_state.copy())
            if not unit_costs and (pair is not None):
                function_plan.update(extract_function_results(results_from_head, *pair))
            break
        else:
            raise RuntimeError('No action instances are applicable')

    # TODO: could instead just accumulate difference between real and opt
    preimage = plan_preimage(preimage_plan, [])
    stream_preimage = set(preimage) - set(real_task.init)
    # visualize_constraints(map(fact_from_fd, preimage))
    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: can optimize for all streams & axioms all at once

    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, stream_preimage))
    for literal in negative_preimage:
        negative = negative_from_name[literal.predicate]
        if isinstance(negative, Predicate):
            predicate_instance = negative.get_instance(map(obj_from_pddl, literal.args))
            value = not literal.negated
            if predicate_instance.enumerated:
                assert (predicate_instance.value == value)
            else:
                function_plan.add(PredicateResult(predicate_instance, value,
                                                  opt_index=predicate_instance.opt_index))
        elif isinstance(negative, Stream):
            #assert not negative.is_fluent()
            object_from_input = dict(zip(negative.inputs, map(obj_from_pddl, literal.args)))
            input_objects = tuple(object_from_input[inp] for inp in negative.inputs)

            fluent_facts_list = []
            if negative.is_fluent():
                for state_index in preimage[literal]:
                    fluent_facts_list.append(list(map(fact_from_fd,
                        filter(lambda f: isinstance(f, pddl.Atom) and (f.predicate in negative.fluents),
                               real_states[state_index]))))
            else:
                fluent_facts_list.append(frozenset())

            for fluent_facts in fluent_facts_list:
                negative_instance = negative.get_instance(input_objects, fluent_facts=fluent_facts)
                if not negative_instance.successes:
                    function_plan.add(StreamResult(negative_instance, tuple(),
                                                   opt_index=negative_instance.opt_index))
        else:
            raise ValueError(negative)

    node_from_atom = get_achieving_streams(evaluations, stream_results)
    preimage_facts = list(map(fact_from_fd, filter(lambda l: not l.negated, stream_preimage - negative_preimage)))
    stream_plan = []
    extract_stream_plan(node_from_atom, preimage_facts, stream_plan)
    stream_plan = convert_fluent_streams(stream_plan, real_states, preimage, node_from_atom)

    if optimize: # TODO: detect this based on unique or not
        new_stream_plan = reschedule(evaluations, preimage_facts, domain, stream_results)
        if new_stream_plan is not None:
            stream_plan = new_stream_plan
    return stream_plan + list(function_plan)

##################################################

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, negative, unit_costs, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    problem = get_problem(opt_evaluations, goal_expression, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    #action_plan, action_cost = solve_from_task(task, **kwargs)
    #action_plan, action_cost = serialized_solve_from_task(task, **kwargs)
    action_plan, action_cost = abstrips_solve_from_task(task, **kwargs)
    #action_plan, action_cost = abstrips_solve_from_task_sequential(task, **kwargs)
    if action_plan is None:
        return None, action_cost
    # TODO: just use solve finite?

    stream_plan = recover_stream_plan(evaluations, goal_expression, domain,
                                      stream_results, action_plan, negative, unit_costs)
    combined_plan = stream_plan + obj_from_pddl_plan(action_plan)
    return combined_plan, action_cost
