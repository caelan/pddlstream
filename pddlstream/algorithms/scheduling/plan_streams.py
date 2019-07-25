from __future__ import print_function

import copy

from collections import defaultdict, namedtuple

from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, get_cost_scale, \
    scale_cost, fd_from_fact, make_domain, make_predicate, evaluation_from_fd, plan_preimage, fact_from_fd, conditions_hold
from pddlstream.algorithms.instantiate_task import instantiate_task, sas_from_instantiated
from pddlstream.algorithms.scheduling.add_optimizers import add_optimizer_effects, \
    using_optimizers, recover_simultaneous
from pddlstream.algorithms.scheduling.apply_fluents import convert_fluent_streams
from pddlstream.algorithms.scheduling.negative import recover_negative_axioms, convert_negative
from pddlstream.algorithms.scheduling.postprocess import postprocess_stream_plan
from pddlstream.algorithms.scheduling.recover_axioms import recover_axioms_plans
from pddlstream.algorithms.scheduling.recover_functions import compute_function_plan
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan, \
    evaluations_from_stream_plan
from pddlstream.algorithms.scheduling.stream_action import add_stream_actions
from pddlstream.algorithms.scheduling.utils import partition_results, \
    add_unsatisfiable_to_goal, get_instance_facts
from pddlstream.algorithms.search import solve_from_task
from pddlstream.algorithms.algorithm import UNIVERSAL_TO_CONDITIONAL
from pddlstream.language.constants import And, Not, get_prefix, EQ, Action
from pddlstream.language.conversion import obj_from_pddl_plan, evaluation_from_fact, \
    fact_from_evaluation, transform_plan_args, transform_action_args, pddl_from_object, obj_from_pddl
from pddlstream.language.external import Result
from pddlstream.language.exogenous import get_fluent_domain
from pddlstream.language.function import Function
from pddlstream.language.stream import StreamResult
from pddlstream.language.optimizer import UNSATISFIABLE
from pddlstream.language.statistics import compute_plan_effort
from pddlstream.language.temporal import SimplifiedDomain, solve_tfd
from pddlstream.language.write_pddl import get_problem_pddl
from pddlstream.utils import Verbose, INF, flatten

OptSolution = namedtuple('OptSolution', ['stream_plan', 'action_plan', 'cost',
                                         'supporting_facts', 'axiom_plan'])

##################################################

def add_stream_efforts(node_from_atom, instantiated, effort_weight, **kwargs):
    if effort_weight is None:
        return
    # TODO: make effort just a multiplier (or relative) to avoid worrying about the scale
    # TODO: regularize & normalize across the problem?
    #efforts = []
    for instance in instantiated.actions:
        # TODO: prune stream actions here?
        # TODO: round each effort individually to penalize multiple streams
        facts = get_instance_facts(instance, node_from_atom)
        #effort = COMBINE_OP([0] + [node_from_atom[fact].effort for fact in facts])
        stream_plan = []
        extract_stream_plan(node_from_atom, facts, stream_plan)
        effort = compute_plan_effort(stream_plan, **kwargs)
        instance.cost += scale_cost(effort_weight*effort)
        #efforts.append(effort)
    #print(min(efforts), efforts)

##################################################

def rename_instantiated_actions(instantiated):
    # TODO: rename SAS instead?
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

def pddl_from_instance(instance):
    action = instance.action
    args = [instance.var_mapping[p.name]
            for p in action.parameters[:action.num_external_parameters]]
    return Action(action.name, args)

##################################################

def get_plan_cost(action_plan, cost_from_action):
    if action_plan is None:
        return INF
    # TODO: return cost per action instance
    #return sum([0.] + [instance.cost for instance in action_plan])
    scaled_cost = sum([0.] + [cost_from_action[instance] for instance in action_plan])
    return scaled_cost / get_cost_scale()

def instantiate_optimizer_axioms(instantiated, domain, results):
    # Needed for instantiating axioms before adding stream action effects
    # Otherwise, FastDownward will prune these unreachable axioms
    # TODO: compute this first and then apply the eager actions
    stream_init = {fd_from_fact(result.stream_fact)
                   for result in results if isinstance(result, StreamResult)}
    evaluations = list(map(evaluation_from_fd, stream_init | instantiated.atoms))
    temp_domain = make_domain(predicates=[make_predicate(UNSATISFIABLE, [])],
                              axioms=[ax for ax in domain.axioms if ax.name == UNSATISFIABLE])
    temp_problem = get_problem(evaluations, Not((UNSATISFIABLE,)), temp_domain)
    # TODO: UNSATISFIABLE might be in atoms making the goal always infeasible
    with Verbose():
        # TODO: the FastDownward instantiation will prune static preconditions
        new_instantiated = instantiate_task(task_from_domain_problem(temp_domain, temp_problem),
                                            check_infeasible=False, prune_static=False)
        assert new_instantiated is not None
    instantiated.axioms.extend(new_instantiated.axioms)
    instantiated.atoms.update(new_instantiated.atoms)

##################################################

def recover_stream_plan(evaluations, current_plan, opt_evaluations, goal_expression, domain, node_from_atom,
                        action_plan, axiom_plans, negative, replan_step):
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects
    # TODO: node_from_atom is a subset of opt_evaluations (only missing functions)
    real_task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain))
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain))
    negative_from_name = {external.blocked_predicate: external for external in negative if external.is_negated()}
    real_states, combined_plan = recover_negative_axioms(
        real_task, opt_task, axiom_plans, action_plan, negative_from_name)
    function_from_instance = compute_function_plan(opt_evaluations, action_plan)
    function_plan = set(function_from_instance.values())

    full_preimage = plan_preimage(combined_plan, [])
    stream_preimage = set(full_preimage) - real_states[0]
    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, stream_preimage))
    function_plan.update(convert_negative(negative_preimage, negative_from_name, full_preimage, real_states))
    positive_preimage = stream_preimage - negative_preimage
    #supporting_facts = set(map(fact_from_fd, full_preimage))

    steps_from_fact = {fact_from_fd(l): full_preimage[l] for l in positive_preimage if not l.negated}
    target_facts = {fact for fact in steps_from_fact.keys() if get_prefix(fact) != EQ}
    #stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_results)
    # visualize_constraints(map(fact_from_fd, target_facts))

    # TODO: get_steps_from_stream
    stream_plan = []
    step_from_stream = {}
    for result in current_plan:
        # TODO: actually compute when these are needed + dependencies
        step_from_stream[result] = 0
        if isinstance(result.external, Function) or (result.external in negative):
            function_plan.add(result) # Prevents these results from being pruned
        else:
            stream_plan.append(result)

    curr_evaluations = evaluations_from_stream_plan(evaluations, stream_plan, max_effort=None)
    extraction_facts = target_facts - set(map(fact_from_evaluation, curr_evaluations))
    step_from_fact = {fact: min(steps_from_fact[fact]) for fact in extraction_facts}
    extract_stream_plan(node_from_atom, extraction_facts, stream_plan,
                        step_from_fact, step_from_stream)
    stream_plan = postprocess_stream_plan(evaluations, domain, stream_plan, target_facts)

    eager_plan = []
    actions_from_step = {}
    for result in (stream_plan + list(function_plan)):
        if (result.opt_index != 0) or (step_from_stream.get(result, 0) < replan_step):
            eager_plan.append(result)
        else:
            actions_from_step.setdefault(step_from_stream[result], []).append(result.get_action())
    eager_plan = convert_fluent_streams(eager_plan, real_states, action_plan, steps_from_fact, node_from_atom)

    # print(action_plan)
    # # TODO: propagate this forward in the future
    # start_from_stream = {}
    # for result in eager_plan:
    #     stuff = list(map(fd_from_fact, get_fluent_domain(result)))
    #     index = len(real_states)
    #     for i, state in enumerate(real_states):
    #         if conditions_hold(state, stuff):
    #             start_from_stream[result] = i
    #             index = i
    #             break
    #     #else:
    #     #start_from_stream[result] = len(real_states)
    #     print(index, result)

    # TODO: some sort of obj side-effect bug that requires obj_from_pddl to be applied last (likely due to fluent streams)
    #action_plan = transform_plan_args(map(pddl_from_instance, action_instances), obj_from_pddl)
    for step, action in enumerate(action_plan):
        actions_from_step.setdefault(step, []).append(transform_action_args(
            pddl_from_instance(action), obj_from_pddl))
    action_plan = list(flatten(actions_from_step[step] for step in sorted(actions_from_step)))

    return eager_plan, action_plan

##################################################

def solve_optimistic_temporal(domain, stream_domain, applied_results, all_results,
                              opt_evaluations, node_from_atom, goal_expression,
                              effort_weight, debug=False, **kwargs):
    # TODO: assert that the unused parameters are off
    assert domain is stream_domain
    #assert len(applied_results) == len(all_results)
    problem = get_problem(opt_evaluations, goal_expression, domain)
    with Verbose():
        instantiated = instantiate_task(task_from_domain_problem(domain, problem))
    if instantiated is None:
        return instantiated, None, None, INF
    problem = get_problem_pddl(opt_evaluations, goal_expression, domain.pddl)
    pddl_plan, makespan = solve_tfd(domain.pddl, problem, debug=debug)
    if pddl_plan is None:
        return instantiated, None, pddl_plan, makespan
    instance_from_action_args = defaultdict(list)
    for instance in instantiated.actions:
        tokens = instance.name.strip('()').split(' ')
        name, args = tokens[0], tuple(tokens[1:])
        instance_from_action_args[name, args].append(instance)
        #instance.action, instance.var_mapping
    action_instances = []
    for action in pddl_plan:
        instances = instance_from_action_args[action.name, action.args]
        assert len(instances) == 1 # TODO: support 2 <= case
        action_instances.append(instances[0])
    plan = obj_from_pddl_plan(pddl_plan)
    return instantiated, action_instances, plan, makespan

def solve_optimistic_sequential(domain, stream_domain, applied_results, all_results,
                                opt_evaluations, node_from_atom, goal_expression,
                                effort_weight, debug=False, **kwargs):
    problem = get_problem(opt_evaluations, goal_expression, stream_domain)  # begin_metric
    with Verbose():
        instantiated = instantiate_task(task_from_domain_problem(stream_domain, problem))
    if instantiated is None:
        return instantiated, None, INF

    cost_from_action = {action: action.cost for action in instantiated.actions}
    add_stream_efforts(node_from_atom, instantiated, effort_weight)
    if using_optimizers(applied_results):
        add_optimizer_effects(instantiated, node_from_atom)
        # TODO: reachieve=False when using optimizers or should add applied facts
        instantiate_optimizer_axioms(instantiated, domain, all_results)
    action_from_name = rename_instantiated_actions(instantiated)
    with Verbose(debug):
        sas_task = sas_from_instantiated(instantiated)
        sas_task.metric = True

    # TODO: apply renaming to hierarchy as well
    # solve_from_task | serialized_solve_from_task | abstrips_solve_from_task | abstrips_solve_from_task_sequential
    renamed_plan, _ = solve_from_task(sas_task, debug=debug, **kwargs)
    if renamed_plan is None:
        return instantiated, None, INF
    action_instances = [action_from_name[name] for name, _ in renamed_plan]
    cost = get_plan_cost(action_instances, cost_from_action)
    return instantiated, action_instances, cost

##################################################

def plan_streams(evaluations, goal_expression, domain, all_results, negative, effort_weight, max_effort,
                 simultaneous=False, reachieve=True, replan_actions=set(), **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    # TODO: only consider axioms that have stream conditions?
    #reachieve = reachieve and not using_optimizers(all_results)
    applied_results, deferred_results = partition_results(
        evaluations, all_results, apply_now=lambda r: not (simultaneous or r.external.info.simultaneous))
    stream_domain, deferred_from_name = add_stream_actions(domain, deferred_results)

    if reachieve and not using_optimizers(all_results):
        achieved_results = {n.result for n in evaluations.values() if isinstance(n.result, Result)}
        init_evaluations = {e for e, n in evaluations.items() if n.result not in achieved_results}
        applied_results = achieved_results | set(applied_results)
        evaluations = init_evaluations # For clarity
    # TODO: could iteratively increase max_effort
    node_from_atom = get_achieving_streams(evaluations, applied_results, # TODO: apply to all_results?
                                           max_effort=max_effort)
    opt_evaluations = {evaluation_from_fact(f): n.result for f, n in node_from_atom.items()}
    if UNIVERSAL_TO_CONDITIONAL or using_optimizers(all_results):
        goal_expression = add_unsatisfiable_to_goal(stream_domain, goal_expression)

    optimistic_fn = solve_optimistic_temporal if isinstance(stream_domain, SimplifiedDomain) \
        else solve_optimistic_sequential
    instantiated, action_instances, cost = optimistic_fn(
        domain, stream_domain, applied_results, all_results, opt_evaluations,
        node_from_atom, goal_expression, effort_weight, **kwargs)
    if action_instances is None:
        return None, None, cost

    axiom_plans = recover_axioms_plans(instantiated, action_instances)
    # TODO: extract out the minimum set of conditional effects that are actually required
    #simplify_conditional_effects(instantiated.task, action_instances)
    stream_plan, action_instances = recover_simultaneous(
        applied_results, negative, deferred_from_name, action_instances)

    action_plan = transform_plan_args(map(pddl_from_instance, action_instances), obj_from_pddl)
    replan_step = min([step+1 for step, action in enumerate(action_plan)
                       if action.name in replan_actions] or [len(action_plan)])
    stream_plan, action_plan = recover_stream_plan(evaluations, stream_plan, opt_evaluations, goal_expression, stream_domain,
                                                   node_from_atom, action_instances, axiom_plans, negative, replan_step)
    return stream_plan, action_plan, cost
