from collections import defaultdict

from pddlstream.algorithms.downward import fd_from_fact, get_conjunctive_parts, get_disjunctive_parts
from pddlstream.language.constants import get_prefix, get_args
from pddlstream.language.conversion import substitute_expression
from pddlstream.language.fluent import get_predicate_map
from pddlstream.language.function import Function
from pddlstream.language.optimizer import UNSATISFIABLE, ConstraintStream
from pddlstream.language.stream import Stream
from pddlstream.utils import find_unique, get_mapping

UNIVERSAL_TO_CONDITIONAL = False
AUTOMATICALLY_NEGATE = True # TODO: fix Yang's bug
# TODO: AUTOMATICALLY_NEGATE = False can omit collisions


def get_predicates(expression):
    import pddl.conditions
    if isinstance(expression, pddl.conditions.ConstantCondition):
        return set()
    if isinstance(expression, pddl.conditions.JunctorCondition) or \
            isinstance(expression, pddl.conditions.QuantifiedCondition):
        predicates = set()
        for part in expression.parts:
            predicates.update(get_predicates(part))
        return predicates
    if isinstance(expression, pddl.conditions.Literal):
        return {expression.predicate}
    raise ValueError(expression)


def universal_to_conditional(action):
    import pddl
    new_parts = []
    unsatisfiable = fd_from_fact((UNSATISFIABLE,))
    for quant in get_conjunctive_parts(action.precondition):
        if isinstance(quant, pddl.UniversalCondition):
            condition = quant.parts[0]
            # TODO: normalize first?
            if isinstance(condition, pddl.Disjunction) or isinstance(condition, pddl.Literal):
                action.effects.append(pddl.Effect(quant.parameters, condition.negate(), unsatisfiable))
                continue
        new_parts.append(quant)
    action.precondition = pddl.Conjunction(new_parts)


def process_conditional_effect(effect, negative_from_predicate):
    import pddl
    new_parts = []
    stream_facts = []
    for disjunctive in get_conjunctive_parts(effect.condition):
        for literal in get_disjunctive_parts(disjunctive):
            # TODO: assert only one disjunctive part
            if isinstance(literal, pddl.Literal) and (literal.predicate in negative_from_predicate):
                stream = negative_from_predicate[literal.predicate]
                if not isinstance(stream, ConstraintStream):
                    new_parts.append(literal)
                    continue
                certified = find_unique(lambda f: get_prefix(f) == literal.predicate, stream.certified)
                mapping = get_mapping(get_args(certified), literal.args)
                stream_facts.append(fd_from_fact(substitute_expression(stream.stream_fact, mapping)))
                # TODO: add the negated literal as precondition here?
            else:
                new_parts.append(literal)
    return new_parts, stream_facts


def optimizer_conditional_effects(domain, externals):
    import pddl
    #from pddlstream.algorithms.scheduling.negative import get_negative_predicates
    # TODO: extend this to predicates
    if UNIVERSAL_TO_CONDITIONAL:
        negative_streams = list(filter(lambda e: e.is_negated, externals))
    else:
        negative_streams = list(filter(lambda e: isinstance(e, ConstraintStream) and e.is_negated, externals))
    negative_from_predicate = get_predicate_map(negative_streams)
    if not negative_from_predicate:
        return
    for action in domain.actions:
        universal_to_conditional(action)
        new_effects = []
        for effect in action.effects:
            if effect.literal.predicate != UNSATISFIABLE:
                new_effects.append(effect)
                continue
            new_parts, stream_facts = process_conditional_effect(effect, negative_from_predicate)
            if not stream_facts:
                new_effects.append(effect)
            for stream_fact in stream_facts:
                new_effects.append(pddl.Effect(effect.parameters, pddl.Conjunction(new_parts), stream_fact))
        action.effects = new_effects


def enforce_simultaneous(domain, externals):
    optimizer_conditional_effects(domain, externals)
    axiom_predicates = set()
    for axiom in domain.axioms:
        axiom_predicates.update(get_predicates(axiom.condition))
    for external in externals:
        if isinstance(external, ConstraintStream) and not external.info.simultaneous:
            #isinstance(external, ComponentStream) and not external.outputs
            # Only need for ConstraintStream because VariableStream used in action args
            # TODO: apply recursively to domain conditions?
            predicates = {get_prefix(fact) for fact in external.certified}
            if predicates & axiom_predicates:
                external.info.simultaneous = True

##################################################

def get_domain_predicates(external):
    return set(map(get_prefix, external.domain))


def get_certified_predicates(external):
    if isinstance(external, Stream):
        return set(map(get_prefix, external.certified))
    if isinstance(external, Function):
        return {get_prefix(external.head)}
    raise ValueError(external)


def get_interacting_externals(externals):
    external_pairs = set()
    for external1 in externals:
        for external2 in externals:
            # TODO: handle case where no domain conditions
            if get_certified_predicates(external1) & get_domain_predicates(external2):
                # TODO: count intersection when arity of zero
                external_pairs.add((external1, external2))
                if external1.is_negated:
                    raise ValueError('Stream [{}] can certify [{}] and thus cannot be negated'.format(
                        external1.name, external2.name))
    return external_pairs


def get_certifiers(externals):
    certifiers = defaultdict(set)
    for external in externals:
        for predicate in get_certified_predicates(external):
            certifiers[predicate].add(external)
    return certifiers


def get_negated_predicates(domain):
    # TODO: generalize to more complicated formulas and recursive axioms
    import pddl
    negated_action_preconditions = set()
    for action in domain.actions:
        for part in get_conjunctive_parts(action.precondition):
            # TODO: at least check more complicated parts for usage
            if isinstance(part, pddl.NegatedAtom):
                negated_action_preconditions.add(part.predicate)
    negated_predicates = set()
    for axiom in domain.axioms:
        if axiom.name not in negated_action_preconditions:
            continue
        for part in get_conjunctive_parts(axiom.condition):
            if isinstance(part, pddl.NegatedAtom):
                negated_predicates.add(part.predicate)
    return negated_predicates


def automatically_negate_externals(domain, externals):
    negated_predicates = get_negated_predicates(domain)
    certifiers = get_certifiers(externals)
    producers = {e1 for e1, _ in get_interacting_externals(externals)}
    non_producers = set(externals) - producers
    for external in non_producers:
        #if external.is_fluent:
        #external.num_opt_fns = 0 # Streams that can be evaluated at the end as tests
        if isinstance(external, Stream) and not external.is_negated \
                and external.is_test and not external.is_fluent and external.could_succeed() \
                and all((predicate in negated_predicates) and (len(certifiers[predicate]) == 1)
                        for predicate in get_certified_predicates(external)):
            # TODO: could instead only negate if in a negative axiom
            external.info.negate = True
            print('Setting negate={} for stream [{}]'.format(external.is_negated, external.name))
