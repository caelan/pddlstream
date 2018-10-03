from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, apply_action, fact_from_fd, \
    conditions_hold, get_goal_instance, plan_preimage, get_literals, instantiate_task, \
    sas_from_instantiated, scale_cost, fd_from_fact
from pddlstream.algorithms.scheduling.postprocess import reschedule_stream_plan, prune_stream_plan
from pddlstream.algorithms.scheduling.recover_axioms import get_derived_predicates, extract_axiom_plan
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan
from pddlstream.algorithms.scheduling.simultaneous import extract_function_results, \
    add_stream_actions, partition_plan, get_plan_cost, augment_goal
from pddlstream.algorithms.scheduling.utils import partition_results, \
    get_results_from_head, apply_streams
from pddlstream.algorithms.search import abstrips_solve_from_task
from pddlstream.language.conversion import obj_from_pddl_plan, obj_from_pddl, substitute_expression
from pddlstream.language.function import PredicateResult, Predicate
from pddlstream.language.optimizer import partition_external_plan, is_optimizer_result
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.utils import Verbose, MockSet, INF

DO_RESCHEDULE = False

def instantiate_actions(opt_task, type_to_objects, function_assignments, action_plan):
    action_instances = []
    for name, args in action_plan: # TODO: negative atoms in actions
        candidates = []
        for action in opt_task.actions:
            if action.name != name:
                continue
            if len(action.parameters) != len(args):
                raise NotImplementedError('Existential quantifiers are not currently '
                                          'supported in preconditions: {}'.format(name))
            variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
            action_instance = action.instantiate(variable_mapping, set(), MockSet(), type_to_objects,
                                          opt_task.use_min_cost_metric, function_assignments)
            assert (action_instance is not None)
            candidates.append(((action, args), action_instance))
        if not candidates:
            raise RuntimeError('Could not find an applicable action {}'.format(name))
        action_instances.append(candidates)
    action_instances.append([(None, get_goal_instance(opt_task.goal))])
    return action_instances

def simplify_conditional_effects(real_state, opt_state, action_instance, axioms_from_name):
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

def convert_negative(negative_preimage, negative_from_name, step_from_atom, real_states):
    negative_plan = set()
    for literal in negative_preimage:
        negative = negative_from_name[literal.predicate]
        if isinstance(negative, Predicate):
            predicate_instance = negative.get_instance(map(obj_from_pddl, literal.args))
            value = not literal.negated
            if predicate_instance.enumerated:
                assert (predicate_instance.value == value)
            else:
                negative_plan.add(PredicateResult(predicate_instance, value,
                                                  opt_index=predicate_instance.opt_index))
        elif isinstance(negative, Stream):
            #assert not negative.is_fluent()
            object_from_input = dict(zip(negative.inputs, map(obj_from_pddl, literal.args)))
            input_objects = tuple(object_from_input[inp] for inp in negative.inputs)

            fluent_facts_list = []
            if negative.is_fluent():
                for state_index in step_from_atom[literal]:
                    fluent_facts_list.append(list(map(fact_from_fd,
                        filter(lambda f: isinstance(f, pddl.Atom) and (f.predicate in negative.fluents),
                               real_states[state_index]))))
            else:
                fluent_facts_list.append(frozenset())

            for fluent_facts in fluent_facts_list:
                negative_instance = negative.get_instance(input_objects, fluent_facts=fluent_facts)
                if not negative_instance.successes:
                    negative_plan.add(StreamResult(negative_instance, tuple(),
                                                   opt_index=negative_instance.opt_index))
        else:
            raise ValueError(negative)
    return negative_plan

def convert_fluent_streams(stream_plan, real_states, step_from_fact, node_from_atom):
    import pddl
    steps_from_stream = {}
    for result in reversed(stream_plan):
        steps_from_stream[result] = set()
        for fact in result.get_certified():
            if (fact in step_from_fact) and (node_from_atom[fact].stream_result == result):
                steps_from_stream[result].update(step_from_fact[fact])
        for fact in result.instance.get_domain():
            step_from_fact[fact] = step_from_fact.get(fact, set()) | steps_from_stream[result]

    new_stream_plan = []
    for result in stream_plan:
        external = result.instance.external
        if (result.opt_index != 0) or (not external.is_fluent()):
            new_stream_plan.append(result)
            continue
        if len(steps_from_stream[result]) != 1:
            raise NotImplementedError() # Pass all fluents and make two axioms
        # TODO: can handle case where no outputs easily
        [state_index] = steps_from_stream[result]
        fluent_facts = list(map(fact_from_fd, filter(lambda f: isinstance(f, pddl.Atom) and (f.predicate in external.fluents), real_states[state_index])))
        new_instance = external.get_instance(result.instance.input_objects, fluent_facts=fluent_facts)
        result = new_instance.get_result(result.output_objects, opt_index=result.opt_index)
        new_stream_plan.append(result)
    return new_stream_plan

##################################################

def get_negative_predicates(negative):
    negative_from_name = {external.name: external for external in negative if isinstance(external, Predicate)}
    negative_from_name.update({external.blocked_predicate: external for external in negative
                               if isinstance(external, Stream) and external.is_negated()})
    return negative_from_name

def recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_plan, negative, unit_costs):
    import pddl
    import instantiate
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects

    real_task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs))
    node_from_atom = get_achieving_streams(evaluations, stream_results)
    opt_evaluations = apply_streams(evaluations, stream_results)
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs))
    function_assignments = {fact.fluent: fact.expression for fact in opt_task.init  # init_facts
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(opt_task.objects, opt_task.types)
    results_from_head = get_results_from_head(opt_evaluations)
    action_instances = instantiate_actions(opt_task, type_to_objects, function_assignments, action_plan)
    negative_from_name = get_negative_predicates(negative)
    axioms_from_name = get_derived_predicates(opt_task.axioms)

    opt_task.init = set(opt_task.init)
    real_states = [set(real_task.init)] # TODO: had old way of doing this (~July 2018)
    preimage_plan = []
    function_plan = set()
    for layer in action_instances:
        for pair, action_instance in layer:
            nonderived_preconditions = [l for l in action_instance.precondition
                                        if l.predicate not in axioms_from_name]
            if not conditions_hold(opt_task.init, nonderived_preconditions):
                continue
            axiom_plan = extract_axiom_plan(opt_task, action_instance, negative_from_name,
                                            static_state=real_states[-1])
            simplify_conditional_effects(real_states[-1], opt_task.init, action_instance, axioms_from_name)
            preimage_plan.extend(axiom_plan + [action_instance])
            apply_action(opt_task.init, action_instance)
            real_states.append(set(real_states[-1]))
            apply_action(real_states[-1], action_instance)
            if not unit_costs and (pair is not None):
                function_result = extract_function_results(results_from_head, *pair)
                if function_result is not None:
                    function_plan.add(function_result)
            break
        else:
            raise RuntimeError('No action instances are applicable')

    # TODO: could instead just accumulate difference between real and opt
    full_preimage = plan_preimage(preimage_plan, [])
    stream_preimage = set(full_preimage) - real_states[0]
    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, stream_preimage))
    positive_preimage = stream_preimage - negative_preimage
    function_plan.update(convert_negative(negative_preimage, negative_from_name, full_preimage, real_states))

    step_from_fact = {fact_from_fd(l): full_preimage[l] for l in positive_preimage if not l.negated}
    target_facts = list(step_from_fact.keys())
    #stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_results)
    stream_plan = []
    extract_stream_plan(node_from_atom, target_facts, stream_plan)
    stream_plan = prune_stream_plan(evaluations, stream_plan, target_facts)
    stream_plan = convert_fluent_streams(stream_plan, real_states, step_from_fact, node_from_atom)
    # visualize_constraints(map(fact_from_fd, stream_preimage))

    if DO_RESCHEDULE: # TODO: detect this based on unique or not
        # TODO: maybe test if partial order between two ways of achieving facts, if not prune
        new_stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_plan)
        if new_stream_plan is not None:
            stream_plan = new_stream_plan
    return stream_plan + list(function_plan)

##################################################

def add_stream_costs(node_from_atom, instantiated, effort_weight):
    for instance in instantiated.actions:
        # Ignores conditional effect costs
        facts = []
        for precondition in get_literals(instance.action.precondition):
            if precondition.negated:
                continue
            args = [instance.var_mapping.get(arg, arg) for arg in precondition.args]
            literal = precondition.__class__(precondition.predicate, args)
            fact = fact_from_fd(literal)
            if fact in node_from_atom:
                facts.append(fact)
        #effort = COMBINE_OP([0] + [node_from_atom[fact].effort for fact in facts])
        stream_plan = []
        extract_stream_plan(node_from_atom, facts, stream_plan)
        effort = sum([0] + [r.instance.get_effort() for r in stream_plan])
        if effort_weight is not None:
            instance.cost += scale_cost(effort_weight*effort)
        for result in stream_plan:
            # TODO: need to make multiple versions if several ways of achieving the action
            if is_optimizer_result(result):
                fact = substitute_expression(result.external.stream_fact, result.get_mapping())
                atom = fd_from_fact(fact)
                instantiated.atoms.add(atom)
                effect = (tuple(), atom)
                instance.add_effects.append(effect)

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, negative, unit_costs, effort_weight,
                        debug=False, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    applied_results, deferred_results = partition_results(evaluations, stream_results, lambda r: r.external.info.simultaneous)
    stream_domain, result_from_name = add_stream_actions(domain, deferred_results)
    node_from_atom = get_achieving_streams(evaluations, applied_results)
    opt_evaluations = apply_streams(evaluations, applied_results) # if n.effort < INF
    if any(map(is_optimizer_result, stream_results)):
        goal_expression = augment_goal(stream_domain, goal_expression)
    problem = get_problem(opt_evaluations, goal_expression, stream_domain, unit_costs)

    with Verbose(debug):
        instantiated = instantiate_task(task_from_domain_problem(stream_domain, problem))
    if instantiated is None:
        return None, INF
    if (effort_weight is not None) or any(map(is_optimizer_result, applied_results)):
        add_stream_costs(node_from_atom, instantiated, effort_weight)
    with Verbose(debug):
        sas_task = sas_from_instantiated(instantiated)
    #sas_task = sas_from_pddl(task)
    #action_plan, _ = serialized_solve_from_task(sas_task, debug=debug, **kwargs)
    action_plan, _ = abstrips_solve_from_task(sas_task, debug=debug, **kwargs)
    #action_plan, _ = abstrips_solve_from_task_sequential(sas_task, debug=debug, **kwargs)
    if action_plan is None:
        return None, INF

    applied_plan, function_plan = partition_external_plan(recover_stream_plan(
        evaluations, goal_expression, stream_domain, applied_results, action_plan, negative, unit_costs))
    deferred_plan, action_plan = partition_plan(action_plan, result_from_name)
    stream_plan = applied_plan + deferred_plan + function_plan
    action_plan = obj_from_pddl_plan(action_plan)
    cost = get_plan_cost(opt_evaluations, action_plan, domain, unit_costs)
    combined_plan = stream_plan + action_plan
    return combined_plan, cost
