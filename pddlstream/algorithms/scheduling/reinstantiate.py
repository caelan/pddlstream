from pddlstream.algorithms.downward import get_fluents, apply_action, get_goal_instance
from pddlstream.utils import MockSet


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
    fluents = get_fluents(task)
    function_assignments = {fact.fluent: fact.expression for fact in task.init
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    init_facts = set()
    fluent_facts = MockSet()
    new_instances = []
    state = set(task.init)
    for old_instance in old_instances:
        # TODO: better way of instantiating conditional effects (when not fluent)
        #new_instance = reinstantiate_action(old_instance)
        predicate_to_atoms = instantiate.get_atoms_by_predicate(
            {a for a in state if isinstance(a, pddl.Atom) and (a.predicate in fluents)})
        action = old_instance.action
        var_mapping = old_instance.var_mapping
        new_instance = action.instantiate(var_mapping, init_facts, fluent_facts, type_to_objects,
                                          task.use_min_cost_metric, function_assignments, predicate_to_atoms)
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