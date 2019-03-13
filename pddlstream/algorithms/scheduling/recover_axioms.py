from collections import defaultdict

from pddlstream.algorithms.downward import get_literals, apply_action, \
    get_derived_predicates, literal_holds
from pddlstream.algorithms.instantiate_task import get_goal_instance, filter_negated, get_achieving_axioms
from pddlstream.language.constants import is_parameter
from pddlstream.utils import Verbose, MockSet, safe_zip


def get_necessary_axioms(conditions, axioms, negative_from_name):
    if not conditions or not axioms:
        return {}
    axioms_from_name = get_derived_predicates(axioms)
    atom_queue = []
    processed_atoms = set()
    def add_literals(literals):
        for lit in literals:
            atom = lit.positive()
            if atom not in processed_atoms:
                atom_queue.append(atom) # Previously was lit.positive() for some reason?
                processed_atoms.add(atom)

    import pddl
    add_literals(conditions)
    axiom_from_action = {}
    partial_instantiations = set()
    while atom_queue:
        literal = atom_queue.pop()
        for axiom in axioms_from_name[literal.predicate]:
            derived_parameters = axiom.parameters[:axiom.num_external_parameters]
            var_mapping = {p.name: a for p, a in zip(derived_parameters, literal.args) if not is_parameter(a)}
            key = (axiom, frozenset(var_mapping.items()))
            if key in partial_instantiations:
                continue
            partial_instantiations.add(key)
            parts = [l.rename_variables(var_mapping) for l in get_literals(axiom.condition)
                     if l.predicate not in negative_from_name] # Assumes a conjunction?
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

##################################################

def instantiate_necessary_axioms(model, static_facts, fluent_facts, axiom_remap={}):
    import pddl
    instantiated_axioms = []
    for atom in model:
        if isinstance(atom.predicate, pddl.Action):
            action = atom.predicate
            var_mapping = {p.name: a for p, a in zip(action.parameters, atom.args)}
            axiom, existing_var_mapping = axiom_remap[action]
            var_mapping.update(existing_var_mapping)
            inst_axiom = axiom.instantiate(var_mapping, static_facts, fluent_facts)
            if inst_axiom:
                instantiated_axioms.append(inst_axiom)
    return instantiated_axioms

##################################################

def extract_axioms(axiom_from_atom, conditions, axiom_plan, negated_from_name={}):
    success = True
    for fact in filter_negated(conditions, negated_from_name):
        if fact not in axiom_from_atom:
            success = False
            continue
        axiom = axiom_from_atom[fact]
        if (axiom is None) or (axiom in axiom_plan):
            continue
        extract_axioms(axiom_from_atom, axiom.condition, axiom_plan, negated_from_name=negated_from_name)
        axiom_plan.append(axiom)
    return success

##################################################

def is_useful_atom(atom, conditions_from_predicate):
    # TODO: this is currently a bottleneck. Instantiate for all actions along the plan first? (apply before checking)
    import pddl
    if not isinstance(atom, pddl.Atom):
        return False
    for atom2 in conditions_from_predicate[atom.predicate]:
        if all(is_parameter(a2) or (a1 == a2) for a1, a2 in safe_zip(atom.args, atom2.args)):
            return True
    return False

def extraction_helper(init, instantiated_axioms, goals, negative_from_name={}):
    import axiom_rules
    # TODO: filter instantiated_axioms that aren't applicable?
    with Verbose(False):
        helpful_axioms, axiom_init, _ = axiom_rules.handle_axioms(
            [], instantiated_axioms, goals)
    axiom_init = set(axiom_init)
    axiom_effects = {axiom.effect for axiom in helpful_axioms}
    #assert len(axiom_effects) == len(axiom_init)
    for pre in list(goals) + list(axiom_effects):
        if pre.positive() not in axiom_init:
            axiom_init.add(pre.positive().negate())
    axiom_from_atom, _ = get_achieving_axioms(init | axiom_init, helpful_axioms, negative_from_name)
    axiom_plan = []  # Could always add all conditions
    success = extract_axioms(axiom_from_atom, goals, axiom_plan, negative_from_name)
    if not success:
        print('Warning! Could not extract an axiom plan')
        #return None
    return axiom_plan

def extract_axiom_plan(task, goals, negative_from_name, static_state=set()):
    import pddl_to_prolog
    import build_model
    import instantiate
    # TODO: only reinstantiate the negative axioms
    if not negative_from_name:
        return []
    axioms_from_name = get_derived_predicates(task.axioms)
    derived_goals = {l for l in goals if l.predicate in axioms_from_name}
    assert all(literal_holds(task.init, l) # or (l.predicate in negative_from_name)
               for l in set(goals) - derived_goals)
    axiom_from_action = get_necessary_axioms(derived_goals, task.axioms, negative_from_name)
    if not axiom_from_action:
        return []
    conditions_from_predicate = defaultdict(set)
    for axiom, mapping in axiom_from_action.values():
        for literal in get_literals(axiom.condition):
            conditions_from_predicate[literal.predicate].add(literal.rename_variables(mapping))

    original_init = task.init
    original_actions = task.actions
    original_axioms = task.axioms
    # TODO: retrieve initial state based on if helpful
    task.init = {atom for atom in task.init if is_useful_atom(atom, conditions_from_predicate)}
    # TODO: store map from predicate to atom
    task.actions = axiom_from_action.keys()
    task.axioms = []
    # TODO: maybe it would just be better to drop the negative throughout this process until this end
    with Verbose(False):
        model = build_model.compute_model(pddl_to_prolog.translate(task))  # Changes based on init
    opt_facts = instantiate.get_fluent_facts(task, model) | (task.init - static_state)
    mock_fluent = MockSet(lambda item: (item.predicate in negative_from_name) or (item in opt_facts))
    instantiated_axioms = instantiate_necessary_axioms(model, static_state, mock_fluent, axiom_from_action)
    axiom_plan = extraction_helper(task.init, instantiated_axioms, derived_goals, negative_from_name)
    task.init = original_init
    task.actions = original_actions
    task.axioms = original_axioms
    return axiom_plan

##################################################

def backtrack_axioms(conditions, axioms_from_effect, visited_atoms):
    visited_axioms = []
    for atom in conditions:
        if atom in visited_atoms:
            continue
        visited_atoms.add(atom)
        for axiom in axioms_from_effect[atom]:
            visited_axioms.append(axiom)
            visited_axioms.extend(backtrack_axioms(axiom.condition, axioms_from_effect, visited_atoms))
    return visited_axioms

def recover_axioms_plans(instantiated, action_instances):
    # TODO: normalize axioms first
    axioms_from_effect = defaultdict(list)
    for axiom in instantiated.axioms:
        axioms_from_effect[axiom.effect].append(axiom)
    #derived_predicates = get_derived_predicates(instantiated.task.axioms)

    state = set(instantiated.task.init)
    axiom_plans = []
    for action in action_instances + [get_goal_instance(instantiated.task.goal)]:
        axioms = backtrack_axioms(action.precondition, axioms_from_effect, set())
        #axiom_instances = list(filter(lambda ax: all(l.predicate in derived_predicates or literal_holds(state, l)
        #                                             for l in ax.condition), axioms))
        derived_goals = {literal for literal in action.precondition if not literal_holds(state, literal)}
        axiom_from_atom, _ = get_achieving_axioms(state, axioms)
        axiom_plans.append([])
        assert extract_axioms(axiom_from_atom, derived_goals, axiom_plans[-1])
        apply_action(state, action)
    return axiom_plans