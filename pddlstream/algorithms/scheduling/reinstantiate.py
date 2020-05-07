from pddlstream.algorithms.downward import apply_action, get_conjunctive_parts
from pddlstream.algorithms.instantiate_task import get_goal_instance
from pddlstream.utils import MockSet
from pddlstream.language.optimizer import UNSATISFIABLE

import pddl
import instantiate


def reinstantiate_action(state, instance, negative_from_name={}):
    # Recomputes the instances with without any pruned preconditions
    # TODO: making the assumption that no negative derived predicates
    action = instance.action
    var_mapping = instance.var_mapping
    init_facts = set()
    fluent_facts = MockSet()
    precondition = []
    try:
        action.precondition.instantiate(var_mapping, init_facts, fluent_facts, precondition)
    except pddl.conditions.Impossible:
        return None
    precondition = list(set(precondition))
    effects = []
    effect_from_literal = {literal: (cond, effect, effect_mapping)
                           for cond, literal, effect, effect_mapping in instance.effect_mappings}
    for effect in action.effects:
        if effect.literal.predicate == UNSATISFIABLE:
            # Condition must be false for plan to succeed
            conditions = set(get_conjunctive_parts(effect.condition))
            negative = {literal for literal in conditions if literal.predicate in negative_from_name}
            if not negative:
                continue
            assert len(negative) == 1
            # TODO: handle the case where negative is not used (not (CFree ..))
            normal_conjunction = pddl.Conjunction(conditions - negative)
            # TODO: assumes that can instantiate with just predicate_to_atoms
            normal_effect = pddl.Effect(effect.parameters, normal_conjunction, effect.literal)
            # TODO: avoid recomputing these
            objects_by_type = instantiate.get_objects_by_type([], [])
            predicate_to_atoms = instantiate.get_atoms_by_predicate(state)
            result = []
            normal_effect.instantiate(var_mapping, state, {effect.literal},
                                      objects_by_type, predicate_to_atoms, result)
            for _, _, _, mapping in result:
                for literal in negative:
                    new_literal = literal.rename_variables(mapping).negate()
                    assert(not new_literal.free_variables())
                    precondition.append(new_literal)

    for literal in instance.applied_effects:
        cond, effect, effect_mapping = effect_from_literal[literal]
        if effect is None: # Stream effect
            #effects.append((cond, literal, cond, effect))
            continue
        else:
            effect._instantiate(effect_mapping, init_facts, fluent_facts, effects)

    new_effects = []
    for cond, effect, e, m in effects:
        precondition.extend(cond)
        new_effects.append(([], effect, e, m))
    return pddl.PropositionalAction(instance.name, precondition, new_effects, instance.cost, action, var_mapping)


def reinstantiate_action_instances(task, old_instances, **kwargs):
    # Recomputes the instances with without any pruned preconditions
    state = set(task.init)
    new_instances = []
    for old_instance in old_instances:
        # TODO: better way of instantiating conditional effects (when not fluent)
        new_instance = reinstantiate_action(state, old_instance, **kwargs)
        assert (new_instance is not None)
        new_instances.append(new_instance)
        apply_action(state, new_instance)
    new_instances.append(get_goal_instance(task.goal)) # TODO: move this?
    return new_instances


def reinstantiate_axiom_instances(old_instances):
    init_facts = set()
    fluent_facts = MockSet()
    new_instances = []
    for old_instance in old_instances:
        axiom = old_instance.axiom
        var_mapping = old_instance.var_mapping
        new_instance = axiom.instantiate(var_mapping, init_facts, fluent_facts)
        assert (new_instance is not None)
        new_instances.append(new_instance)
    return new_instances
