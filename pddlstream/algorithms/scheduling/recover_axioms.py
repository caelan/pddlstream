from collections import defaultdict, deque

from pddlstream.algorithms.downward import get_literals
from pddlstream.language.constants import is_parameter

def get_derived_predicates(axioms):
    axioms_from_name = defaultdict(list)
    for axiom in axioms:
        axioms_from_name[axiom.name].append(axiom)
    return axioms_from_name


def get_necessary_axioms(instance, axioms, negative_from_name):
    import pddl

    if not axioms:
        return {}
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
            var_mapping = {p.name: a for p, a in zip(derived_parameters, literal.args) if not is_parameter(a)}
            key = (axiom, frozenset(var_mapping.items()))
            if key in partial_instantiations:
                continue
            partial_instantiations.add(key)
            parts = []
            for literal in get_literals(axiom.condition): # TODO: assumes a conjunction?
                if literal.predicate not in negative_from_name:
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
