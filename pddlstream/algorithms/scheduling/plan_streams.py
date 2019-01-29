import copy

from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, apply_action, fact_from_fd, \
    get_goal_instance, plan_preimage, instantiate_task, get_cost_scale, \
    sas_from_instantiated, scale_cost, literal_holds
from pddlstream.algorithms.scheduling.add_optimizers import add_optimizer_effects, add_optimizer_axioms, \
    using_optimizers, recover_simultaneous
from pddlstream.algorithms.scheduling.apply_fluents import convert_fluent_streams
from pddlstream.algorithms.scheduling.negative import get_negative_predicates, convert_negative, \
    recover_negative_axioms
from pddlstream.algorithms.scheduling.postprocess import postprocess_stream_plan
from pddlstream.algorithms.scheduling.recover_axioms import get_derived_predicates, extraction_helper
from pddlstream.algorithms.scheduling.recover_functions import compute_function_plan
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan
from pddlstream.algorithms.scheduling.stream_action import add_stream_actions
from pddlstream.algorithms.scheduling.utils import partition_results, \
    add_unsatisfiable_to_goal, get_instance_facts, simplify_conditional_effects, evaluations_from_stream_plan
from pddlstream.algorithms.search import solve_from_task
from pddlstream.language.constants import EQ, get_prefix
from pddlstream.language.conversion import obj_from_pddl_plan, evaluation_from_fact
from pddlstream.language.effort import compute_plan_effort
from pddlstream.language.external import Result
from pddlstream.language.function import Function
from pddlstream.utils import Verbose, INF


def recover_stream_plan(evaluations, current_plan, opt_evaluations, goal_expression, domain, node_from_atom,
                        action_plan, axiom_plans, negative):
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects
    # TODO: node_from_atom is a subset of opt_evaluations (only missing functions)
    curr_evaluations = evaluations_from_stream_plan(evaluations, current_plan, max_effort=None)
    real_task = task_from_domain_problem(domain, get_problem(curr_evaluations, goal_expression, domain))
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain))
    negative_from_name = get_negative_predicates(negative)

    real_states, combined_plan = recover_negative_axioms(
        real_task, opt_task, axiom_plans, action_plan, negative_from_name)
    function_plan = compute_function_plan(opt_evaluations, action_plan)

    full_preimage = plan_preimage(combined_plan, [])
    stream_preimage = set(full_preimage) - real_states[0]
    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, stream_preimage))
    positive_preimage = stream_preimage - negative_preimage
    function_plan.update(convert_negative(negative_preimage, negative_from_name, full_preimage, real_states))

    step_from_fact = {fact_from_fd(l): full_preimage[l] for l in positive_preimage if not l.negated}
    target_facts = [fact for fact in step_from_fact.keys() if get_prefix(fact) != EQ]
    #stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_results)
    # visualize_constraints(map(fact_from_fd, target_facts))

    stream_plan = []
    for result in current_plan:
        if isinstance(result.external, Function) or (result.external in negative):
            function_plan.add(result) # Prevents these results from being pruned
        else:
            stream_plan.append(result)
    extract_stream_plan(node_from_atom, target_facts, stream_plan)
    stream_plan = postprocess_stream_plan(evaluations, domain, stream_plan, target_facts)
    stream_plan = convert_fluent_streams(stream_plan, real_states, action_plan, step_from_fact, node_from_atom)

    return stream_plan + list(function_plan)

##################################################

def add_stream_efforts(node_from_atom, instantiated, effort_weight, **kwargs):
    if effort_weight is None:
        return
    # TODO: make effort just a multiplier (or relative) to avoid worrying about the scale
    #efforts = [] # TODO: regularize & normalize across the problem?
    for instance in instantiated.actions:
        # TODO: prune stream actions here?
        # TODO: round each effort individually to penalize multiple streams
        facts = get_instance_facts(instance, node_from_atom)
        #effort = COMBINE_OP([0] + [node_from_atom[fact].effort for fact in facts])
        stream_plan = []
        extract_stream_plan(node_from_atom, facts, stream_plan)
        if effort_weight is not None:
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

def get_plan_cost(action_plan, cost_from_action):
    if action_plan is None:
        return INF
    #return sum([0.] + [instance.cost for instance in action_plan])
    scaled_cost = sum([0.] + [cost_from_action[instance] for instance in action_plan])
    return scaled_cost / get_cost_scale()

def instantiate_optimizer_axioms(instantiated, evaluations, goal_expression, domain, results):
    # Needed for instantiating axioms before adding stream action effects
    # Otherwise, FastDownward will prune these unreachable axioms
    # TODO: compute this first and then apply the eager actions
    #stream_evaluations = set(map(evaluation_from_fact, get_stream_facts(applied_results)))
    stream_domain, result_from_name = add_stream_actions(domain, results)
    problem = get_problem(evaluations, goal_expression, stream_domain)
    with Verbose():
        new_instantiated = instantiate_task(task_from_domain_problem(stream_domain, problem))
    instantiated.axioms[:] = new_instantiated.axioms
    instantiated.atoms.update(new_instantiated.atoms)

##################################################

def plan_streams(evaluations, goal_expression, domain, all_results, negative,
                 unit_efforts, effort_weight, max_effort,
                 simultaneous=False, reachieve=True, debug=False, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    # TODO: only consider axioms that have stream conditions?
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
                                           unit_efforts=unit_efforts, max_effort=max_effort)
    opt_evaluations = {evaluation_from_fact(f): n.result for f, n in node_from_atom.items()}
    if using_optimizers(all_results):
        goal_expression = add_unsatisfiable_to_goal(stream_domain, goal_expression)
    problem = get_problem(opt_evaluations, goal_expression, stream_domain) # begin_metric
    with Verbose(debug):
        instantiated = instantiate_task(task_from_domain_problem(stream_domain, problem))
    if instantiated is None:
        return None, INF

    if using_optimizers(all_results):
        # TODO: reachieve=False when using optimizers or should add applied facts
        instantiate_optimizer_axioms(instantiated, evaluations, goal_expression, domain, all_results)
    cost_from_action = {action: action.cost for action in instantiated.actions}
    add_stream_efforts(node_from_atom, instantiated, effort_weight, unit_efforts=unit_efforts)
    if using_optimizers(applied_results):
        add_optimizer_effects(instantiated, node_from_atom)
    add_optimizer_axioms(all_results, instantiated)
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
    simplify_conditional_effects(instantiated.task, action_instances)
    stream_plan, action_instances = recover_simultaneous(
        applied_results, negative, deferred_from_name, action_instances)
    cost = get_plan_cost(action_instances, cost_from_action)
    axiom_plans = recover_axioms_plans(instantiated, action_instances)

    stream_plan = recover_stream_plan(evaluations, stream_plan, opt_evaluations, goal_expression,
                                      stream_domain, node_from_atom, action_instances, axiom_plans, negative)
    #action_plan = obj_from_pddl_plan(parse_action(instance.name) for instance in action_instances)
    action_plan = obj_from_pddl_plan(map(pddl_from_instance, action_instances))

    combined_plan = stream_plan + action_plan
    return combined_plan, cost
