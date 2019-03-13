from collections import defaultdict, deque

from pddlstream.algorithms.downward import literal_holds, get_derived_predicates, apply_action
from pddlstream.algorithms.instantiate_task import get_goal_instance
from pddlstream.algorithms.scheduling.recover_axioms import extract_axioms


class SuccessorNode(object):
    def __init__(self, depth=0):
        self.depth = depth
        self.children = {}
        self.instances = []
    def get_child(self, value):
        if value not in self.children:
            self.children[value] = SuccessorNode(depth=self.depth + 1)
        return self.children[value]
    def get_successors(self, atom_order, state):
        if len(atom_order) <= self.depth:
            return self.instances
        atom = atom_order[self.depth]
        instances = []
        for value, node in self.children.items():
            if (value is None) or (literal_holds(state, atom) is value):
                instances.extend(node.get_successors(atom_order, state))
        return instances

def get_fluents(init, action_instances):
    fluents = set()
    for action in action_instances:  # TODO: just actions if no action_instances
        for cond, eff in action.add_effects:
            assert not cond
            if not literal_holds(init, eff):
                fluents.add(eff)
        for cond, eff in action.del_effects:
            assert not cond
            if not literal_holds(init, eff.negate()):
                fluents.add(eff)
    return fluents

class SuccessorGenerator(object):
    def __init__(self, instantiated, action_instances=[]):
        derived_predicates = get_derived_predicates(instantiated.task.axioms)
        conditions = {literal.positive() for axiom in instantiated.axioms for literal in axiom.condition}
        state = set(instantiated.task.init)
        fluents = get_fluents(state, action_instances) & conditions
        self.fluent_order = list(fluents)

        applicable_axioms = []
        axiom_from_literal = defaultdict(list)
        # TODO: could also just use get_achieving_axioms
        self.root = SuccessorNode()
        for axiom in instantiated.axioms:
            if all((l.predicate in derived_predicates) or (l.positive() in fluents) or
                           literal_holds(state, l) for l in axiom.condition):
                applicable_axioms.append(axiom)
                for literal in axiom.condition:
                    if literal in fluents:
                        axiom_from_literal[literal].append(axiom)

                fluent_conds = {l.positive(): not l.negated for l in axiom.condition}
                node = self.root
                for atom in self.fluent_order:
                    value = fluent_conds.get(atom, None)
                    node = node.get_child(value)
                node.instances.append(axiom)
    def get_successors(self, state):
        return self.root.get_successors(self.fluent_order, state)

##################################################

def mark_axiom(queue, remaining_from_axiom, axiom, axiom_from_atom):
    if not remaining_from_axiom[id(axiom)]:
        axiom_from_atom[axiom.effect].append(axiom)
        queue.append(axiom.effect)


def mark_iteration(state, axioms_from_literal, fluents_from_axiom, remaining_from_axiom, static_axioms):
    axioms_from_atom = defaultdict(list)
    for literal in axioms_from_literal:
        if literal_holds(state, literal):
            axioms_from_atom[literal].append(None)
    queue = deque(axioms_from_atom.keys())
    for axiom in static_axioms:
        mark_axiom(queue, remaining_from_axiom, axiom, axioms_from_atom)
    while queue:
        literal = queue.popleft()
        for axiom in axioms_from_literal[literal]:
            remaining_from_axiom[id(axiom)] -= 1
            mark_axiom(queue, remaining_from_axiom, axiom, axioms_from_atom)
    for literal, axioms in axioms_from_atom.items():
        for axiom in axioms:
            if axiom is not None:
                remaining_from_axiom[id(axiom)] = fluents_from_axiom[id(axiom)]
    # TODO: still some overhead here
    # TODO: could process these layer by layer instead
    return {atom: axioms[0] for atom, axioms in axioms_from_atom.items()}


def recover_axioms_plans2(instantiated, action_instances):
    #import axiom_rules
    #with Verbose(False):
    #    normalized_axioms, axiom_init, axiom_layer_dict = axiom_rules.handle_axioms(
    #        [], instantiated.axioms, instantiated.goal_list)
    #state = set(instantiated.task.init + axiom_init)
    normalized_axioms = instantiated.axioms # TODO: ignoring negated because cannot reinstantiate correctly
    state = set(instantiated.task.init)
    fluents = get_fluents(state, action_instances)

    unprocessed_from_atom = defaultdict(list)
    fluents_from_axiom = {}
    remaining_from_axiom = {}
    for axiom in normalized_axioms:
        fluent_conditions = []
        for literal in axiom.condition:
            if literal.positive() in fluents:
                fluent_conditions.append(literal)
            elif not literal_holds(state, literal):
                fluent_conditions = None
                break
        if fluent_conditions is None:
            continue
        for literal in fluent_conditions:
            unprocessed_from_atom[literal].append(axiom)
        fluents_from_axiom[id(axiom)] = len(fluent_conditions)
        remaining_from_axiom[id(axiom)] = fluents_from_axiom[id(axiom)]
    static_axioms = [axiom for axiom, num in fluents_from_axiom.items() if num == 0]

    axiom_plans = []
    for action in action_instances + [get_goal_instance(instantiated.task.goal)]:
        axiom_from_atom = mark_iteration(state, unprocessed_from_atom,
                                         fluents_from_axiom, remaining_from_axiom, static_axioms)
        preimage = []
        for literal in action.precondition:
            if not literal_holds(state, literal):
                preimage.append(literal)
                assert literal in axiom_from_atom
        for cond, eff in (action.add_effects + action.del_effects):
            # TODO: add conditional effects that must hold here
            assert not cond
        axiom_plans.append([])
        assert extract_axioms(axiom_from_atom, preimage, axiom_plans[-1])
        apply_action(state, action)
    return axiom_plans