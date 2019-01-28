from collections import defaultdict
from itertools import product

from pddlstream.algorithms.downward import fd_from_fact
from pddlstream.algorithms.scheduling.recover_streams import extract_stream_plan
from pddlstream.algorithms.scheduling.utils import get_instance_facts
from pddlstream.language.constants import get_args, Not
from pddlstream.language.conversion import substitute_expression
from pddlstream.language.object import UniqueOptValue
from pddlstream.language.optimizer import BLOCK_ADDITIONS, UNSATISFIABLE
from pddlstream.algorithms.recover_optimizers import is_optimizer_result
from pddlstream.utils import get_mapping


def using_optimizers(stream_results):
    return any(map(is_optimizer_result, stream_results))


def get_stream_fact(result):
    assert is_optimizer_result(result)
    return substitute_expression(result.external.stream_fact, result.get_mapping())


def get_stream_facts(results):
    return [get_stream_fact(result) for result in results if is_optimizer_result(result)]


def add_optimizer_effects(instantiated, node_from_atom):
    # TODO: instantiate axioms with negative on effects for blocking
    # TODO: fluent streams using conditional effects. Special fluent predicate for inputs to constraint
    # This strategy will only work for relaxed to ensure that the current state is applied
    # TODO: bug! The FD instantiator prunes the result.external.stream_fact
    for instance in instantiated.actions:
        facts = get_instance_facts(instance, node_from_atom)
        # TODO: need to handle case where a negative preconditions is used in an optimizer
        if any(condition for condition, effect in instance.add_effects + instance.del_effects):
            raise NotImplementedError()
        stream_plan = []
        extract_stream_plan(node_from_atom, facts, stream_plan)
        for result in stream_plan:
            if not is_optimizer_result(result):
                continue
            # TODO: need to make multiple versions if several ways of achieving the action
            atom = fd_from_fact(get_stream_fact(result))
            instantiated.atoms.add(atom)
            effect = (tuple(), atom)
            instance.add_effects.append(effect)
            # domain = {fact for result in stream_plan if result.external.info.simultaneous
            #          for fact in result.instance.get_domain()}
            # TODO: can streams depending on these be used if dependent preconditions are added to the action


def add_optimizer_axioms(results, instantiated):
    # Ends up being a little slower than version in optimizer.py when not blocking shared

    # TODO: the motivation for BLOCK_ADDITIONS was that an optimizer called on a disconnected subset of constraints
    # might produce a solution but not all possible solutions. Blocking might prevent future things
    # Instead, just jointly optimize for the full cluster
    import pddl
    results_from_instance = defaultdict(list)
    for result in results:
        results_from_instance[result.instance].append(result)
    optimizer_results = list(filter(is_optimizer_result, results))
    optimizers = {result.external.optimizer for result in optimizer_results}
    for optimizer in optimizers:
        facts_from_arg = defaultdict(list)
        if BLOCK_ADDITIONS:
            optimizer_facts = {get_stream_fact(result) for result in optimizer_results
                               if result.external.optimizer is optimizer}
            for fact in optimizer_facts:
                for arg in get_args(fact):
                    facts_from_arg[arg].append(fact)

        for stream in optimizer.streams:
            if not stream.instance.disabled:
                continue
            if stream.instance._disabled_axiom is not None:
                # Avoids double blocking
                continue
            output_variables = []
            for out in stream.output_objects:
                assert isinstance(out.param, UniqueOptValue)
                output_variables.append([r.output_objects[out.param.output_index]
                                         for r in results_from_instance[out.param.instance]])
            constraints = stream.instance.get_constraints()
            for combo in product(*output_variables):
                # TODO: add any static predicates?
                # Instantiates axioms
                mapping = get_mapping(stream.output_objects, combo)
                name = '({})'.join(UNSATISFIABLE)
                blocked = set(substitute_expression(constraints, mapping))
                # TODO: partial disable, if something has no outputs, then adding things isn't going to help
                if BLOCK_ADDITIONS and (not stream.instance.enumerated or stream.instance.successes):
                    # In the event the optimizer gives different results with different inputs
                    additional = {fact for arg in combo for fact in facts_from_arg[arg]} - blocked
                else:
                    # Assumes the optimizer is submodular
                    additional = {}
                condition = list(map(fd_from_fact, blocked | set(map(Not, additional))))
                effect = fd_from_fact((UNSATISFIABLE,))
                instantiated.axioms.append(pddl.PropositionalAxiom(name, condition, effect))
                instantiated.atoms.add(effect)
