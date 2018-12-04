from pddlstream.algorithms.downward import fact_from_fd
from pddlstream.language.conversion import obj_from_pddl
from pddlstream.language.function import Predicate, PredicateResult
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.utils import get_mapping


def get_negative_predicates(negative):
    negative_from_name = {external.name: external for external in negative if isinstance(external, Predicate)}
    negative_from_name.update({external.blocked_predicate: external for external in negative
                               if isinstance(external, Stream) and external.is_negated()})
    return negative_from_name


def convert_negative_predicate(negative, literal, negative_plan):
    predicate_instance = negative.get_instance(map(obj_from_pddl, literal.args))
    value = not literal.negated
    if predicate_instance.enumerated:
        assert (predicate_instance.value == value)
    else:
        negative_plan.add(PredicateResult(predicate_instance, value,
                                          opt_index=predicate_instance.opt_index))


def convert_negative_stream(negative, literal, step_from_atom, real_states, negative_plan):
    import pddl
    # assert not negative.is_fluent()
    fluent_facts_list = []
    if negative.is_fluent():
        # TODO: ensure that only used once?
        for state_index in step_from_atom[literal]:
            fluent_facts_list.append(list(map(fact_from_fd, filter(
                lambda f: isinstance(f, pddl.Atom) and (f.predicate in negative.fluents), real_states[state_index]))))
    else:
        fluent_facts_list.append(frozenset())

    object_from_input = get_mapping(negative.inputs, map(obj_from_pddl, literal.args))
    input_objects = tuple(object_from_input[inp] for inp in negative.inputs)
    for fluent_facts in fluent_facts_list:
        negative_instance = negative.get_instance(input_objects, fluent_facts=fluent_facts)
        if not negative_instance.successes:
            negative_plan.add(StreamResult(negative_instance, tuple(),
                                           opt_index=negative_instance.opt_index))


def convert_negative(negative_preimage, negative_from_name, step_from_atom, real_states):
    negative_plan = set()
    for literal in negative_preimage:
        negative = negative_from_name[literal.predicate]
        if isinstance(negative, Predicate):
            convert_negative_predicate(negative, literal, negative_plan)
        elif isinstance(negative, Stream):
            convert_negative_stream(negative, literal, step_from_atom, real_states, negative_plan)
        else:
            raise ValueError(negative)
    return negative_plan