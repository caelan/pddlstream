from pddlstream.conversion import value_from_obj_expression, obj_from_value_expression


class ConstraintSolver(object):
    def __init__(self, stream_map):
        self.stream_map = stream_map
        self.fn = stream_map.get('constraint-solver', None)
        self.problems = {}
    def solve(self, constraints, verbose=False):
        if self.fn is None:
            return []
        # TODO: prune any fully constrained things
        key = frozenset(constraints)
        if key in self.problems:
            return []
        new_facts = self.fn(list(map(value_from_obj_expression, constraints)))
        if verbose:
            print('{}: {}'.format(self.__class__.__name__, new_facts))
        self.problems[key] = map(obj_from_value_expression, new_facts)
        return self.problems[key]
        # TODO: evaluate functions as well
        # TODO: certify if optimal
    #def __repr__(self):
    #    return self.__class__.__name__