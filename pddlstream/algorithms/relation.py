from collections import defaultdict

from pddlstream.language.constants import is_parameter
from pddlstream.utils import INF, get_mapping


def compute_order(domain, atoms):
    # Most constrained variable/atom to least constrained
    # TODO: dynamically select the atom with the fewest options (minimize new additions)
    # Operating on dual (select constraints rather than vars) because lower arity
    order = []
    parameters = set() # Include constants
    for _ in range(len(domain)):
        min_new = INF
        min_index = None
        for index in set(range(len(domain))) - set(order):
            if set(filter(is_parameter, domain[index].args)) <= parameters:
                min_new = 0
                min_index = index
            if len(atoms[index]) < min_new:
                min_new = len(atoms[index])
                min_index = index
        order.append(min_index)
        parameters.update(filter(is_parameter, domain[min_index].args))
    return order

##################################################

# TODO: all solutions constraint satisfaction point of view: constraint propagation
# https://en.wikipedia.org/wiki/Local_consistency
# Cluster into components and then order?

class Relation(object):
    def __init__(self, heading, body):
        self.heading = tuple(heading)
        self.body = list(body)
    def get_mapping(self, element):
        return get_mapping(self.heading, element)
    def project_element(self, attributes, element):
        value_from_attribute = self.get_mapping(element)
        assert all(attr in value_from_attribute for attr in attributes)
        return tuple(value_from_attribute[attr] for attr in attributes)
    def get_conditional(self, inputs):
        outputs = [attribute for attribute in self.heading if attribute not in inputs]
        two_from_overlap = defaultdict(set)
        for element in self.body:
            key = self.project_element(inputs, element)
            value = self.project_element(outputs, element)
            two_from_overlap[key].add(value)  # TODO: preserve ordering
        # TODO: return a relation object?
        return two_from_overlap
    def subtract_attributes(self, attributes):
        return tuple(attribute for attribute in self.heading if attribute not in attributes)
    def dump(self):
        print(self.heading)
        for element in self.body:
            print(element)
    def __repr__(self):
        return '|{}| x {}'.format(', '.join(map(str, self.heading)), len(self.body))


def overlapping_attributes(relation1, relation2):
    return tuple(attribute for attribute in relation2.heading if attribute in relation1.heading)


def join(relation1, relation2):
    # Alternatively, Cartesian product then filter
    overlap = overlapping_attributes(relation1, relation2)
    new_heading = relation1.heading + relation2.subtract_attributes(overlap)
    new_body = []
    two_from_overlap = relation2.get_conditional(overlap)
    for element in relation1.body:
        key = relation1.project_element(overlap, element)
        for value in two_from_overlap[key]:
            new_body.append(element + value)
    return Relation(new_heading, new_body)


def solve_satisfaction(relations):
    solution = Relation([], [tuple()])
    for relation in relations:
        solution = join(solution, relation)
    return solution