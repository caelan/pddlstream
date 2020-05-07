from pddlstream.language.constants import get_prefix, get_args
from pddlstream.language.exogenous import replace_literals
from pddlstream.language.external import get_domain_predicates
from pddlstream.language.stream import Stream
from pddlstream.utils import find_unique, get_mapping


def get_predicate_map(state_streams):
    predicate_map = {}
    for state_stream in state_streams:
        for fact in state_stream.certified:
            predicate = get_prefix(fact)
            if predicate in predicate_map:
                # TODO: could make a disjunctive condition instead
                raise NotImplementedError('Only one fluent stream can certify a predicate: {}'.format(predicate))
            predicate_map[predicate] = state_stream
    return predicate_map

def remap_certified(literal, stream):
    certified = find_unique(lambda f: get_prefix(f) == literal.predicate, stream.certified)
    mapping = get_mapping(get_args(certified), literal.args)
    if not all(arg in mapping for arg in stream.inputs): # Certified must contain all inputs
        return None
    return mapping

def compile_fluent_streams(domain, externals):
    state_streams = set(filter(lambda e: isinstance(e, Stream) and e.is_special(), externals))
    predicate_map = get_predicate_map(state_streams)
    if not predicate_map:
        return state_streams
    # TODO: allow usage as long as in the same action (e.g. for costs functions)
    # TODO: could create a separate action per control parameter
    if get_domain_predicates(externals) & set(predicate_map):
        raise RuntimeError('Fluent streams certified facts cannot be domain facts')

    # TODO: could make free parameters free
    # TODO: could treat like a normal stream that generates values (but with no inputs required/needed)
    import pddl
    def fn(literal, action):
        if literal.predicate not in predicate_map:
            return literal
        # TODO: other checks on only inputs
        stream = predicate_map[literal.predicate]
        mapping = remap_certified(literal, stream)
        if mapping is None:
            # TODO: this excludes typing. This is not entirely safe
            return literal
        output_args = set(mapping[arg] for arg in stream.outputs)
        for effect in action.effects:
            if isinstance(effect, pddl.Effect) and (output_args & set(effect.literal.args)):
                raise RuntimeError('Fluent stream outputs cannot be in action effects: {}'.format(
                    effect.literal.predicate))
        blocked_args = tuple(mapping[arg] for arg in stream.inputs)
        blocked_literal = literal.__class__(stream.blocked_predicate, blocked_args).negate()
        if stream.is_negated():
            # TODO: add stream conditions here
            return blocked_literal
        return pddl.Conjunction([literal, blocked_literal])

    for action in domain.actions:
        action.precondition = replace_literals(fn, action.precondition, action).simplified()
        for effect in action.effects:
            effect.condition = replace_literals(fn, effect.condition, action).simplified()
    for axiom in domain.axioms:
        axiom.condition = replace_literals(fn, axiom.condition, action).simplified()
    return state_streams
