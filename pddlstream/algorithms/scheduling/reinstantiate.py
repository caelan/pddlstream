from pddlstream.algorithms.downward import get_fluents, apply_action, has_conditional_effects
from pddlstream.algorithms.instantiate_task import get_goal_instance
from pddlstream.utils import MockSet

from collections import defaultdict
import pddl


def reinstantiate_action(instance):
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
    effects = []
    effect_from_literal = {literal: (cond, effect, effect_mapping)
                           for cond, literal, effect, effect_mapping in instance.effect_mappings}
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


def reinstantiate_action_instances(task, old_instances):
    # Recomputes the instances with without any pruned preconditions
    state = set(task.init)
    new_instances = []
    for old_instance in old_instances:
        # TODO: better way of instantiating conditional effects (when not fluent)
        new_instance = reinstantiate_action(old_instance)
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