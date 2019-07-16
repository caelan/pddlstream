from pddlstream.algorithms.downward import fd_from_fact, fact_from_fd
from pddlstream.algorithms.scheduling.negative import get_negative_result
from pddlstream.algorithms.scheduling.recover_streams import extract_stream_plan
from pddlstream.algorithms.scheduling.utils import get_instance_facts
from pddlstream.language.optimizer import ComponentStream
from pddlstream.language.constants import get_args, get_prefix
from pddlstream.language.stream import Stream


def using_optimizers(results):
    return any(isinstance(result.external, ComponentStream) for result in results)

def add_optimizer_effects(instantiated, node_from_atom):
    # TODO: instantiate axioms with negative on effects for blocking
    # TODO: fluent streams using conditional effects. Special fluent predicate for inputs to constraint
    # TODO: bug! The FD instantiator prunes the result.external.stream_fact
    for instance in instantiated.actions:
        # TODO: need to handle case where a negative preconditions is used in an optimizer
        for condition, effect in (instance.add_effects + instance.del_effects):
            for literal in condition:
                fact = fact_from_fd(literal)
                if (fact in node_from_atom) and (node_from_atom[fact].result is not None):
                    raise NotImplementedError(literal)
        facts = get_instance_facts(instance, node_from_atom)
        stream_plan = []
        extract_stream_plan(node_from_atom, facts, stream_plan)
        # TODO: can detect if some of these are simultaneous and add them as preconditions
        for result in stream_plan:
            if isinstance(result.external, ComponentStream):
                # TODO: need to make multiple versions if several ways of achieving the action
                atom = fd_from_fact(result.stream_fact)
                instantiated.atoms.add(atom)
                effect = (tuple(), atom)
                instance.add_effects.append(effect)
                instance.effect_mappings.append(effect + (None, None))
                # domain = {fact for result in stream_plan if result.external.info.simultaneous
                #          for fact in result.instance.get_domain()}
                # TODO: can streams depending on these be used if dependent preconditions are added to the action

def recover_simultaneous(results, negative_streams, deferred_from_name, instances):
    result_from_stream_fact = {}
    for result in results:
        if isinstance(result.external, Stream):
            assert result.stream_fact not in result_from_stream_fact
            result_from_stream_fact[result.stream_fact] = result

    negative_from_stream_predicate = {}
    for state_stream in negative_streams:
        if not isinstance(state_stream, Stream):
            continue
        predicate = get_prefix(state_stream.stream_fact)
        if predicate in negative_from_stream_predicate:
            # TODO: could make a conjunction condition instead
            raise NotImplementedError()
        negative_from_stream_predicate[predicate] = state_stream

    stream_plan = []
    action_plan = []
    for instance in instances:
        if instance.name in deferred_from_name:
            result = deferred_from_name[instance.name]
            if result not in stream_plan:
                stream_plan.append(result)
        else:
            action_plan.append(instance)
        for conditions, effect in instance.add_effects:
            # Assumes effects are in order
            assert not conditions
            fact = fact_from_fd(effect)
            if fact in result_from_stream_fact:
                result = result_from_stream_fact[fact]
            elif effect.predicate in negative_from_stream_predicate:
                negative = negative_from_stream_predicate[effect.predicate]
                result = get_negative_result(negative, get_args(fact))
            else:
                continue
            if result not in stream_plan:
                stream_plan.append(result)
    return stream_plan, action_plan