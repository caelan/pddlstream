from pddlstream.algorithms.downward import fact_from_fd, plan_preimage, conditions_hold, apply_action, get_goal_instance
from pddlstream.algorithms.scheduling.recover_axioms import get_derived_predicates, extract_axiom_plan
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

##################################################

def reinstantiate_action_instances(task, old_instances):
    import pddl
    import instantiate
    # Recomputes the instances with without any pruned preconditions
    function_assignments = {fact.fluent: fact.expression for fact in task.init
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    init_facts = set()
    fluent_facts = MockSet()
    new_instances = []
    for old_instance in old_instances:
        action = old_instance.action
        #if action is None:
        #    new_instances.append(old_instance) # goal_instance
        var_mapping = old_instance.var_mapping
        new_instance = action.instantiate(var_mapping, init_facts, fluent_facts, type_to_objects,
                                          task.use_min_cost_metric, function_assignments)
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

def simplify_conditional_effects(real_state, opt_state, action_instance, axioms_from_name):
    # TODO: move this back to relaxed.py and compute real_states separately
    # TODO: compute required stream facts in a forward way and allow opt facts that are already known required
    for effects in [action_instance.add_effects, action_instance.del_effects]:
        for i, (conditions, effect) in reversed(list(enumerate(effects))):
            if any(c.predicate in axioms_from_name for c in conditions):
                raise NotImplementedError('Conditional effects cannot currently involve derived predicates')
            if conditions_hold(real_state, conditions):
                # Holds in real state
                effects[i] = ([], effect)
            elif not conditions_hold(opt_state, conditions):
                # Does not hold in optimistic state
                effects.pop(i)
            else:
                # TODO: handle more general case where can choose to achieve particular conditional effects
                raise NotImplementedError('Conditional effects cannot currently involve certified predicates')

##################################################

def recover_negative_axioms(real_task, opt_task, axiom_plans, action_plan, negative_from_name):
    action_plan = reinstantiate_action_instances(opt_task, action_plan)
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
        simplify_conditional_effects(real_states[-1], opt_task.init, action_instance, axioms_from_name)
        preimage = list(plan_preimage(axiom_plan + [action_instance], []))
        assert conditions_hold(opt_task.init, (l for l in preimage if l.predicate not in axioms_from_name))
        new_axiom_plan = extract_axiom_plan(opt_task, preimage, negative_from_name, static_state=real_states[-1])
        assert new_axiom_plan is not None
        preimage_plan.extend(new_axiom_plan + axiom_plan + [action_instance])
        apply_action(opt_task.init, action_instance)
        real_states.append(set(real_states[-1]))
        apply_action(real_states[-1], action_instance)
    return real_states, preimage_plan
