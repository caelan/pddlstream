from pddlstream.algorithms.downward import get_fluents, apply_action, get_goal_instance, has_conditional_effects
from pddlstream.utils import MockSet

from collections import defaultdict


# def reinstantiate_action(instance):
#     # Recomputes the instances with without any pruned preconditions
#     import pddl
#     action = instance.action
#     var_mapping = instance.var_mapping
#     init_facts = set()
#     fluent_facts = MockSet()
#     precondition = []
#     try:
#         action.precondition.instantiate(var_mapping, init_facts, fluent_facts, precondition)
#     except pddl.conditions.Impossible:
#         return None
#     effects = []
#     for eff, effect_mapping in instance.effect_mappings:
#         eff._instantiate(effect_mapping, init_facts, fluent_facts, effects)
#     return pddl.PropositionalAction(instance.name, precondition, effects, instance.cost, action, var_mapping)


def reinstantiate_action_instances(task, old_instances):
    import pddl
    import instantiate
    # Recomputes the instances with without any pruned preconditions
    state = set(task.init)
    function_assignments = {fact.fluent: fact.expression for fact in task.init
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    predicate_to_atoms = defaultdict(set)
    for atom in state:
        if isinstance(atom, pddl.Atom):
            predicate_to_atoms[atom.predicate].add(atom)
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    init_facts = set()
    fluent_facts = MockSet()
    new_instances = []
    for old_instance in old_instances:
        # TODO: better way of instantiating conditional effects (when not fluent)
        #new_instance = reinstantiate_action(old_instance)
        action = old_instance.action
        var_mapping = old_instance.var_mapping
        new_instance = action.instantiate(var_mapping, init_facts, fluent_facts, type_to_objects,
                                          task.use_min_cost_metric, function_assignments, predicate_to_atoms)
        assert (new_instance is not None)
        new_instances.append(new_instance)
        apply_action(state, new_instance)
        for cond, eff in new_instance.del_effects:
            predicate_to_atoms[eff.predicate].discard(eff)
        for cond, eff in new_instance.add_effects:
            predicate_to_atoms[eff.predicate].add(eff)
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