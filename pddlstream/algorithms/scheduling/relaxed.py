import copy
from collections import defaultdict
from itertools import product

from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, apply_action, fact_from_fd, \
    get_goal_instance, plan_preimage, get_literals, instantiate_task, \
    sas_from_instantiated, scale_cost, fd_from_fact, parse_action, literal_holds
from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.algorithms.scheduling.negative import get_negative_predicates, convert_negative, recover_negative_axioms
from pddlstream.algorithms.scheduling.postprocess import postprocess_stream_plan
from pddlstream.algorithms.scheduling.recover_axioms import get_derived_predicates, extraction_helper
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan
from pddlstream.algorithms.scheduling.simultaneous import extract_function_results, \
    add_stream_actions, partition_plan, get_plan_cost, augment_goal
from pddlstream.algorithms.scheduling.utils import partition_results, \
    get_results_from_head, apply_streams
from pddlstream.algorithms.search import solve_from_task
from pddlstream.language.constants import get_args, Not
from pddlstream.language.conversion import obj_from_pddl_plan, substitute_expression, pddl_from_object
from pddlstream.language.object import UniqueOptValue, OptimisticObject
from pddlstream.language.optimizer import partition_external_plan, is_optimizer_result, UNSATISFIABLE
from pddlstream.utils import Verbose, INF, get_mapping, neighbors_from_orders

def compute_function_plan(opt_evaluations, action_plan, unit_costs):
    function_plan = set()
    if unit_costs:
        return function_plan
    results_from_head = get_results_from_head(opt_evaluations)
    for action_instance in action_plan:
        action = action_instance.action
        if action is None:
            continue
        args = [action_instance.var_mapping[p.name] for p in action.parameters]
        function_result = extract_function_results(results_from_head, action, args)
        if function_result is not None:
            function_plan.add(function_result)
    return function_plan

def convert_fluent_streams(stream_plan, real_states, action_plan, step_from_fact, node_from_atom):
    import pddl
    assert len(real_states) == len(action_plan) + 1
    steps_from_stream = {}
    for result in reversed(stream_plan):
        steps_from_stream[result] = set()
        for fact in result.get_certified():
            if (fact in step_from_fact) and (node_from_atom[fact].stream_result == result):
                steps_from_stream[result].update(step_from_fact[fact])
        for fact in result.instance.get_domain():
            step_from_fact[fact] = step_from_fact.get(fact, set()) | steps_from_stream[result]
            # TODO: apply this recursively

    # TODO: ensure that derived facts aren't in fluents?
    # TODO: handle case where costs depend on the outputs
    _, outgoing_edges = neighbors_from_orders(get_partial_orders(
        stream_plan, init_facts=map(fact_from_fd, filter(lambda f: isinstance(f, pddl.Atom), real_states[0]))))
    static_plan = []
    fluent_plan = []
    for result in stream_plan:
        external = result.external
        if (result.opt_index != 0) or (not external.is_fluent()):
            static_plan.append(result)
            continue
        if outgoing_edges[result]:
            # No way of taking into account the binding of fluent inputs when preventing cycles
            raise NotImplementedError('Fluent stream is required for another stream: {}'.format(result))
        #if (len(steps_from_stream[result]) != 1) and result.output_objects:
        #    raise NotImplementedError('Fluent stream required in multiple states: {}'.format(result))
        for state_index in steps_from_stream[result]:
            new_output_objects = [  # OptimisticObject.from_opt(out.value, object())
                OptimisticObject.from_opt(out.value, UniqueOptValue(result.instance, object(), i))
                for i, out in enumerate(result.output_objects)]
            if new_output_objects and (state_index < len(action_plan)):
                # TODO: check that the objects aren't used in any effects
                instance = copy.copy(action_plan[state_index])
                action_plan[state_index] = instance
                output_mapping = get_mapping(map(pddl_from_object, result.output_objects),
                                             map(pddl_from_object, new_output_objects))
                instance.var_mapping = {p: output_mapping.get(v, v)
                                        for p, v in instance.var_mapping.items()}
            fluent_facts = list(map(fact_from_fd, filter(
                lambda f: isinstance(f, pddl.Atom) and (f.predicate in external.fluents), real_states[state_index])))
            new_instance = external.get_instance(result.instance.input_objects, fluent_facts=fluent_facts)
            new_result = new_instance.get_result(new_output_objects, opt_index=result.opt_index)
            fluent_plan.append(new_result)
    return static_plan + fluent_plan

##################################################

def recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_plan, axiom_plans,
                        negative, unit_costs):
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects

    real_task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs))
    node_from_atom = get_achieving_streams(evaluations, stream_results)
    opt_evaluations = apply_streams(evaluations, stream_results)
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs))
    negative_from_name = get_negative_predicates(negative)

    real_states, combined_plan = recover_negative_axioms(real_task, opt_task, axiom_plans, action_plan, negative_from_name)
    function_plan = compute_function_plan(opt_evaluations, action_plan, unit_costs)

    full_preimage = plan_preimage(combined_plan, [])
    stream_preimage = set(full_preimage) - real_states[0]
    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, stream_preimage))
    positive_preimage = stream_preimage - negative_preimage
    function_plan.update(convert_negative(negative_preimage, negative_from_name, full_preimage, real_states))

    step_from_fact = {fact_from_fd(l): full_preimage[l] for l in positive_preimage if not l.negated}
    target_facts = list(step_from_fact.keys())
    #stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_results)
    # visualize_constraints(map(fact_from_fd, target_facts))
    stream_plan = []
    extract_stream_plan(node_from_atom, target_facts, stream_plan)
    stream_plan = postprocess_stream_plan(evaluations, domain, stream_plan, target_facts)
    stream_plan = convert_fluent_streams(stream_plan, real_states, action_plan, step_from_fact, node_from_atom)

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
        # TODO: apply all axiom_instances unaffected by negative conditions
        preimage = list(plan_preimage([action_instance], []))
        axiom_instances = filter(lambda ax: all(l.predicate in derived_predicates or literal_holds(state, l)
                                                for l in ax.condition), instantiated.axioms)
        # Only instantiate if preimage has goal
        axiom_plan = extraction_helper(state, axiom_instances, preimage)
        assert axiom_plan is not None
        axiom_plans.append(axiom_plan)
        apply_action(state, action_instance)
    return axiom_plans

def pddl_from_instance(instance):
    action = instance.action
    args = [instance.var_mapping[p.name]
            for p in action.parameters[:action.num_external_parameters]]
    return action.name, args

##################################################

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

    # TODO: apply renaming to hierarchy as well
    # solve_from_task | serialized_solve_from_task | abstrips_solve_from_task | abstrips_solve_from_task_sequential
    action_plan, _ = solve_from_task(sas_task, debug=debug, **kwargs)
    if action_plan is None:
        return None, INF
    action_instances = [action_from_name[name] for name, _ in action_plan]
    axiom_plans = recover_axioms_plans(instantiated, action_instances)

    applied_plan, function_plan = partition_external_plan(recover_stream_plan(
        evaluations, goal_expression, stream_domain, applied_results, action_instances, axiom_plans, negative, unit_costs))
    #action_plan = obj_from_pddl_plan(parse_action(instance.name) for instance in action_instances)
    action_plan = obj_from_pddl_plan(map(pddl_from_instance, action_instances))

    deferred_plan, action_plan = partition_plan(action_plan, result_from_name)
    stream_plan = applied_plan + deferred_plan + function_plan
    cost = get_plan_cost(opt_evaluations, action_plan, domain, unit_costs)
    combined_plan = stream_plan + action_plan
    return combined_plan, cost
