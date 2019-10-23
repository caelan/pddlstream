from pddlstream.algorithms.downward import fact_from_fd, plan_preimage, apply_action, \
    GOAL_NAME, get_derived_predicates, literal_holds
from pddlstream.algorithms.scheduling.recover_axioms import extract_axiom_plan
from pddlstream.algorithms.scheduling.reinstantiate import reinstantiate_action_instances, reinstantiate_axiom_instances
from pddlstream.language.conversion import obj_from_pddl
from pddlstream.language.function import Predicate, PredicateResult
from pddlstream.language.stream import Stream
from pddlstream.utils import safe_zip


def convert_negative_predicate(negative, literal, negative_plan):
    input_objects = tuple(map(obj_from_pddl, literal.args)) # Might be negative
    predicate_instance = negative.get_instance(input_objects)
    value = not literal.negated
    if predicate_instance.enumerated:
        assert (predicate_instance.value == value)
    else:
        negative_plan.add(PredicateResult(predicate_instance, value,
                                          opt_index=predicate_instance.opt_index))

def get_negative_result(negative, input_objects, fluent_facts=frozenset()):
    instance = negative.get_instance(input_objects, fluent_facts=fluent_facts)
    optimistic = not instance.successful # TODO: clean this up
    return instance._Result(instance, output_objects=tuple(), opt_index=instance.opt_index,
                            call_index=instance.num_calls, optimistic=optimistic)

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

    input_objects = tuple(map(obj_from_pddl, literal.args)) # Might be negative
    for fluent_facts in fluent_facts_list:
        result = get_negative_result(negative, input_objects, fluent_facts)
        #if not result.instance.successful: # Doesn't work with reachieve=True
        negative_plan.add(result)

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

def recover_negative_axioms(real_task, opt_task, axiom_plans, action_plan, negative_from_name):
    action_plan = reinstantiate_action_instances(opt_task, action_plan, negative_from_name=negative_from_name)
    # https://github.com/caelan/pddlstream/commit/18b303e19bbab9f8e0016fbb2656f461067e1e94#diff-55454a85485551f9139e20a446b56a83L53
    #simplify_conditional_effects(opt_task, action_plan, negative_from_name)
    axiom_plans = list(map(reinstantiate_axiom_instances, axiom_plans))
    axioms_from_name = get_derived_predicates(opt_task.axioms)

    # TODO: could instead just accumulate difference between real and opt
    opt_task.init = set(opt_task.init)
    real_states = [set(real_task.init)]
    preimage_plan = []
    for axiom_plan, action_instance in safe_zip(axiom_plans, action_plan):
        preimage = [l for l in plan_preimage(axiom_plan + [action_instance])
                    if (l.predicate in axioms_from_name)]
        #assert conditions_hold(opt_task.init, conditions)
        # TODO: only add derived facts and negative facts to fluent state to make normalizing easier
        negative_axiom_plan = extract_axiom_plan(opt_task, preimage, negative_from_name,
                                                 static_state=opt_task.init)
                                                 #static_state=real_states[-1])
        assert negative_axiom_plan is not None
        preimage_plan.extend(negative_axiom_plan + axiom_plan + [action_instance])
        if action_instance.name != GOAL_NAME:
            apply_action(opt_task.init, action_instance)
            real_states.append(set(real_states[-1]))
            apply_action(real_states[-1], action_instance)
    return real_states, preimage_plan
