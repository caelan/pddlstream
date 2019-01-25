from pddlstream.algorithms.downward import fact_from_fd, plan_preimage, conditions_hold, apply_action, \
    get_goal_instance, GOAL_NAME
from pddlstream.algorithms.scheduling.recover_axioms import get_derived_predicates, extract_axiom_plan
from pddlstream.algorithms.scheduling.utils import simplify_conditional_effects
from pddlstream.language.conversion import obj_from_pddl
from pddlstream.language.function import Predicate, PredicateResult
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.utils import get_mapping, safe_zip, MockSet


def get_negative_predicates(negative):
    negative_from_name = {external.name: external for external in negative if isinstance(external, Predicate)}
    negative_from_name.update({external.blocked_predicate: external for external in negative
                               if isinstance(external, Stream) and external.is_negated()})
    return negative_from_name

##################################################

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
        for step in step_from_atom[literal]:
            fluent_facts_list.append(list(map(fact_from_fd, filter(
                lambda f: isinstance(f, pddl.Atom) and (f.predicate in negative.fluents), real_states[step]))))
    else:
        fluent_facts_list.append(frozenset())

    object_from_input = get_mapping(negative.inputs, map(obj_from_pddl, literal.args))
    input_objects = tuple(object_from_input[inp] for inp in negative.inputs)
    for fluent_facts in fluent_facts_list:
        negative_instance = negative.get_instance(input_objects, fluent_facts=fluent_facts)
        if not negative_instance.successes:
            negative_plan.add(StreamResult(negative_instance, output_objects=tuple(),
                                           opt_index=negative_instance.opt_index,
                                           call_index=negative_instance.num_calls, optimistic=True))


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

##################################################

def reinstantiate_action(instance):
    # Recomputes the instances with without any pruned preconditions
    import pddl
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
    for eff, effect_mapping in instance.effect_mappings:
        eff._instantiate(effect_mapping, init_facts, fluent_facts, effects)
    return pddl.PropositionalAction(instance.name, precondition, effects, instance.cost, action, var_mapping)

def reinstantiate_action_instances(task, old_instances):
    new_instances = []
    for old_instance in old_instances:
        new_instance = reinstantiate_action(old_instance)
        assert (new_instance is not None)
        new_instances.append(new_instance)
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

##################################################

def recover_negative_axioms(real_task, opt_task, axiom_plans, action_plan, negative_from_name):
    action_plan = reinstantiate_action_instances(opt_task, action_plan)
    simplify_conditional_effects(opt_task, action_plan)
    axiom_plans = list(map(reinstantiate_axiom_instances, axiom_plans))
    axioms_from_name = get_derived_predicates(opt_task.axioms)

    # TODO: could instead just accumulate difference between real and opt
    opt_task.init = set(opt_task.init)
    real_states = [set(real_task.init)]
    preimage_plan = []
    for axiom_plan, action_instance in safe_zip(axiom_plans, action_plan):
        for literal in action_instance.precondition:
            # TODO: check conditional effects
            if literal.predicate in negative_from_name:
                raise NotImplementedError('Negated predicates not currently supported within actions: {}'
                                          .format(literal.predicate))
        preimage = list(plan_preimage(axiom_plan + [action_instance], []))
        assert conditions_hold(opt_task.init, (l for l in preimage if l.predicate not in axioms_from_name))
        new_axiom_plan = extract_axiom_plan(opt_task, preimage, negative_from_name, static_state=real_states[-1])
        assert new_axiom_plan is not None
        preimage_plan.extend(new_axiom_plan + axiom_plan + [action_instance])
        if action_instance.name != GOAL_NAME:
            apply_action(opt_task.init, action_instance)
            real_states.append(set(real_states[-1]))
            apply_action(real_states[-1], action_instance)
    return real_states, preimage_plan
