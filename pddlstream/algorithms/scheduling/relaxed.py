from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, apply_action, fact_from_fd, \
    conditions_hold, get_goal_instance, plan_preimage, get_literals, instantiate_task, \
    sas_from_instantiated, scale_cost, fd_from_fact, parse_action, literal_holds
from pddlstream.algorithms.scheduling.postprocess import reschedule_stream_plan, prune_stream_plan
from pddlstream.algorithms.scheduling.recover_axioms import get_derived_predicates, extract_axiom_plan, \
    extraction_helper
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan, \
    get_instance_effort
from pddlstream.algorithms.scheduling.simultaneous import extract_function_results, \
    add_stream_actions, partition_plan, get_plan_cost, augment_goal
from pddlstream.algorithms.scheduling.utils import partition_results, \
    get_results_from_head, apply_streams
from pddlstream.algorithms.search import abstrips_solve_from_task, solve_from_task
from pddlstream.language.conversion import obj_from_pddl_plan, obj_from_pddl, substitute_expression
from pddlstream.language.function import PredicateResult, Predicate
from pddlstream.language.optimizer import partition_external_plan, is_optimizer_result, UNSATISFIABLE
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.language.object import UniqueOptValue
from pddlstream.language.constants import get_args, Not
from pddlstream.utils import Verbose, MockSet, INF, get_mapping

from collections import defaultdict
from itertools import product
import copy

DO_RESCHEDULE = False

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

def get_negative_predicates(negative):
    negative_from_name = {external.name: external for external in negative if isinstance(external, Predicate)}
    negative_from_name.update({external.blocked_predicate: external for external in negative
                               if isinstance(external, Stream) and external.is_negated()})
    return negative_from_name

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

    # TODO: move the fluent streams to the end
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
        fluent_facts = list(map(fact_from_fd, filter(
            lambda f: isinstance(f, pddl.Atom) and (f.predicate in external.fluents), real_states[state_index])))
        new_instance = external.get_instance(result.instance.input_objects, fluent_facts=fluent_facts)
        result = new_instance.get_result(result.output_objects, opt_index=result.opt_index)
        new_stream_plan.append(result)
    return new_stream_plan

##################################################

def recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_instances, axiom_plans,
                        negative, unit_costs):
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects

    real_task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs))
    node_from_atom = get_achieving_streams(evaluations, stream_results)
    opt_evaluations = apply_streams(evaluations, stream_results)
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs))
    action_instances = reinstantiate_action_instances(opt_task, action_instances)
    axiom_plans = list(map(reinstantiate_axiom_instances, axiom_plans))
    results_from_head = get_results_from_head(opt_evaluations)
    negative_from_name = get_negative_predicates(negative)
    axioms_from_name = get_derived_predicates(opt_task.axioms)

    opt_task.init = set(opt_task.init)
    real_states = [set(real_task.init)]
    preimage_plan = []
    function_plan = set()
    for action_instance in action_instances:
        for literal in action_instance.precondition:
            # TODO: check conditional effects
            if literal.predicate in negative_from_name:
                raise NotImplementedError('Negated predicates not currently supported within actions: {}'
                                          .format(literal.predicate))
        simplify_conditional_effects(real_states[-1], opt_task.init, action_instance, axioms_from_name)
        axiom_plan = extract_axiom_plan(opt_task, action_instance, negative_from_name,
                                        static_state=real_states[-1])
        assert axiom_plan is not None
        preimage_plan.extend(axiom_plan + [action_instance])
        apply_action(opt_task.init, action_instance)
        real_states.append(set(real_states[-1]))
        apply_action(real_states[-1], action_instance)
        if not unit_costs and (action_instance.action is not None):
            action = action_instance.action
            args = [action_instance.var_mapping[p.name] for p in action.parameters]
            function_result = extract_function_results(results_from_head, action, args)
            if function_result is not None:
                function_plan.add(function_result)

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

def add_stream_costs(node_from_atom, instantiated, unit_efforts, effort_weight):
    # TODO: instantiate axioms with negative on effects for blocking
    # TODO: fluent streams using conditional effects. Special fluent predicate for inputs to constraint
    # This strategy will only work for relaxed to ensure that the current state is applied
    for instance in instantiated.actions:
        # TODO: prune stream actions here?
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
        if unit_efforts:
            effort = len(stream_plan)
        else:
            effort = scale_cost(sum([0] + [r.instance.get_effort() for r in stream_plan]))
        if effort_weight is not None:
            instance.cost += effort_weight*effort
        # TODO: bug! The FD instantiator prunes the result.external.stream_fact
        for result in stream_plan:
            # TODO: need to make multiple versions if several ways of achieving the action
            if is_optimizer_result(result):
                fact = substitute_expression(result.external.stream_fact, result.get_mapping())
                atom = fd_from_fact(fact)
                instantiated.atoms.add(atom)
                effect = (tuple(), atom)
                instance.add_effects.append(effect)
        #domain = {fact for result in stream_plan if result.external.info.simultaneous
        #          for fact in result.instance.get_domain()}
        # TODO: can streams depending on these to be used if the dependent preconditions are added to the action

##################################################

def add_optimizer_axioms(results, instantiated):
    # Ends up being a little slower than version in optimizer.py when not blocking shared
    # TODO: add this to simultaneous
    import pddl
    results_from_instance = defaultdict(list)
    for result in results:
        results_from_instance[result.instance].append(result)
    optimizer_results = list(filter(is_optimizer_result, results))
    optimizers = {result.external.optimizer for result in optimizer_results}
    for optimizer in optimizers:
        optimizer_facts = {substitute_expression(result.external.stream_fact, result.get_mapping())
                           for result in optimizer_results if result.external.optimizer is optimizer}
        facts_from_arg = defaultdict(list)
        for fact in optimizer_facts:
            for arg in get_args(fact):
                facts_from_arg[arg].append(fact)

        for stream in optimizer.streams:
            if not stream.instance.disabled:
                continue
            constraints = stream.instance.get_constraints()
            output_variables = []
            for out in stream.output_objects:
                assert isinstance(out.param, UniqueOptValue)
                output_variables.append([r.output_objects[out.param.output_index]
                                         for r in results_from_instance[out.param.instance]])
            for combo in product(*output_variables):
                mapping = get_mapping(stream.output_objects, combo)
                name = '({})'.join(UNSATISFIABLE)
                blocked = set(substitute_expression(constraints, mapping))
                additional = {fact for arg in combo for fact in facts_from_arg[arg]} - blocked
                # TODO: like a partial disable, if something has no outputs, then adding things isn't going to help
                if stream.instance.enumerated and not stream.instance.successes:
                    # Assumes the optimizer is submodular
                    condition = list(map(fd_from_fact, blocked))
                else:
                    condition = list(map(fd_from_fact, blocked | set(map(Not, additional))))
                effect = fd_from_fact((UNSATISFIABLE,))
                instantiated.axioms.append(pddl.PropositionalAxiom(name, condition, effect))
                instantiated.atoms.add(effect)

##################################################

# TODO: change name of actions to directly recover which instance was used by the planner

def rename_instantiated_actions(instantiated):
    actions = instantiated.actions[:]
    renamed_actions = []
    action_from_name = {}
    for i, action in enumerate(actions):
        renamed_actions.append(copy.copy(action))
        renamed_name = 'a{}'.format(i)
        renamed_actions[-1].name = '({})'.format(renamed_name)
        action_from_name[renamed_name] = action # Change reachable_action_params?
    instantiated.actions[:] = renamed_actions
    return action_from_name

def recover_axioms_plans(instantiated, action_instances):
    task = instantiated.task
    derived_predicates = get_derived_predicates(task.axioms)
    state = set(task.init)
    axiom_plans = []
    for action_instance in action_instances + [get_goal_instance(task.goal)]:
        axiom_instances = filter(lambda ax: all(l.predicate in derived_predicates or literal_holds(state, l)
                                                for l in ax.condition), instantiated.axioms)
        axiom_plans.append(extraction_helper(state, axiom_instances, action_instance))
        apply_action(state, action_instance)
    return axiom_plans

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, negative, unit_costs,
                        unit_efforts, effort_weight, debug=False, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    # TODO: only consider axioms that have stream conditions?
    applied_results, deferred_results = partition_results(evaluations, stream_results,
                                                          apply_now=lambda r: not r.external.info.simultaneous)
    stream_domain, result_from_name = add_stream_actions(domain, deferred_results)
    node_from_atom = get_achieving_streams(evaluations, applied_results)
    opt_evaluations = apply_streams(evaluations, applied_results) # if n.effort < INF
    if any(map(is_optimizer_result, stream_results)):
        goal_expression = augment_goal(stream_domain, goal_expression)
    problem = get_problem(opt_evaluations, goal_expression, stream_domain, unit_costs) # begin_metric

    with Verbose(debug):
        instantiated = instantiate_task(task_from_domain_problem(stream_domain, problem))
    if instantiated is None:
        return None, INF
    if (effort_weight is not None) or any(map(is_optimizer_result, applied_results)):
        add_stream_costs(node_from_atom, instantiated, unit_efforts, effort_weight)
    add_optimizer_axioms(stream_results, instantiated)
    action_from_name = rename_instantiated_actions(instantiated)

    with Verbose(debug):
        sas_task = sas_from_instantiated(instantiated)
        sas_task.metric = True

    # TODO: apply remapping to hierarchy as well
    # solve_from_task | serialized_solve_from_task | abstrips_solve_from_task | abstrips_solve_from_task_sequential
    action_plan, _ = solve_from_task(sas_task, debug=debug, **kwargs)
    if action_plan is None:
        return None, INF
    action_instances = [action_from_name[name] for name, _ in action_plan]
    axiom_plans = recover_axioms_plans(instantiated, action_instances)

    applied_plan, function_plan = partition_external_plan(recover_stream_plan(
        evaluations, goal_expression, stream_domain, applied_results, action_instances, axiom_plans, negative, unit_costs))
    action_plan = obj_from_pddl_plan(parse_action(instance.name) for instance in action_instances)
    deferred_plan, action_plan = partition_plan(action_plan, result_from_name)
    stream_plan = applied_plan + deferred_plan + function_plan
    cost = get_plan_cost(opt_evaluations, action_plan, domain, unit_costs)
    combined_plan = stream_plan + action_plan
    return combined_plan, cost
