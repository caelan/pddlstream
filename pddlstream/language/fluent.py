from pddlstream.language.constants import get_prefix, get_args
from pddlstream.language.exogenous import replace_literals
from pddlstream.language.stream import Stream
from pddlstream.utils import find_unique, get_mapping

def ensure_no_fluent_streams(streams):
    for stream in streams:
        if isinstance(stream, Stream) and stream.is_fluent():
            raise NotImplementedError('Algorithm does not support fluent stream: {}'.format(stream.name))

def get_predicate_map(state_streams):
    predicate_map = {}
    for state_stream in state_streams:
        for fact in state_stream.certified:
            predicate = get_prefix(fact)
            if predicate in predicate_map:
                # TODO: could make a conjunction condition instead
                raise NotImplementedError()
            predicate_map[predicate] = state_stream
    return predicate_map

def compile_fluent_streams(domain, externals):
    state_streams = list(filter(lambda e: isinstance(e, Stream) and
                                          (e.is_negated() or e.is_fluent()), externals))
    predicate_map = get_predicate_map(state_streams)
    if not predicate_map:
        return state_streams

    # TODO: could make free parameters free
    # TODO: allow functions on top the produced values?
    # TODO: check that generated values are not used in the effects of any actions
    # TODO: could treat like a normal stream that generates values (but with no inputs required/needed)
    def fn(literal):
        if literal.predicate not in predicate_map:
            return literal
        # TODO: other checks on only inputs
        stream = predicate_map[literal.predicate]
        certified = find_unique(lambda f: get_prefix(f) == literal.predicate, stream.certified)
        mapping = get_mapping(get_args(certified), literal.args)
        #assert all(arg in mapping for arg in stream.inputs) # Certified must contain all inputs
        if not all(arg in mapping for arg in stream.inputs):
            # TODO: this excludes typing. This is not entirely safe
            return literal
        blocked_args = tuple(mapping[arg] for arg in stream.inputs)
        blocked_literal = literal.__class__(stream.blocked_predicate, blocked_args).negate()
        if stream.is_negated():
            # TODO: add stream conditions here
            return blocked_literal
        return pddl.Conjunction([literal, blocked_literal])

    import pddl
    for action in domain.actions:
        action.precondition = replace_literals(fn, action.precondition).simplified()
        # TODO: throw an error if the effect would be altered
        for effect in action.effects:
            effect.condition = replace_literals(fn, effect.condition).simplified()
    for axiom in domain.axioms:
        axiom.condition = replace_literals(fn, axiom.condition).simplified()
    return state_streams
