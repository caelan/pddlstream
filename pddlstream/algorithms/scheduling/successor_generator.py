from collections import defaultdict

from pddlstream.algorithms.downward import literal_holds
from pddlstream.algorithms.scheduling.recover_axioms import get_derived_predicates


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