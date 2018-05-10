from pddlstream.conversion import value_from_obj_expression, obj_from_value_expression, values_from_objects
from pddlstream.object import Object


# TODO: other streams?
# TODO: could also put in plan skeleton, current fluent state, etc
# TODO: could just be external functions
# TODO: allow other parameters here
# TODO: force the use of context to be a unique value
# TODO: revisit a stream with and without context automatically
# TODO: could hash to avoid revisiting period

def create_immediate_context(stream_result, stream_plan):
    # TODO: consider previous bindings and choose one value
    streams = []
    for result in stream_plan:
        if set(stream_result.output_objects) & set(result.instance.input_objects):
            if all(isinstance(o, Object) for o in set(result.instance.input_objects) - set(stream_result.output_objects)):
                streams.append((result.instance.external.name,) + values_from_objects(result.instance.input_objects))
    return values_from_objects(stream_result.output_objects), streams

# TODO: local optimization for this
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


class DynamicStream(object):
    # TODO: load from a file as well
    def __init__(self, name, streams, gen_fn):
        self.name = name
        self.streams = streams # Names of streams. Include counts in the future
        self.gen_fn = gen_fn