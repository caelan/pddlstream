import copy

from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, instantiate_task, get_cost_scale, \
    sas_from_instantiated, scale_cost
from pddlstream.algorithms.scheduling.add_optimizers import add_optimizer_effects, \
    using_optimizers, recover_simultaneous
from pddlstream.algorithms.scheduling.recover_axioms import recover_axioms_plans
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan, \
    recover_stream_plan
from pddlstream.algorithms.scheduling.stream_action import add_stream_actions
from pddlstream.algorithms.scheduling.utils import partition_results, \
    add_unsatisfiable_to_goal, get_instance_facts, simplify_conditional_effects
from pddlstream.algorithms.search import solve_from_task
from pddlstream.language.conversion import obj_from_pddl_plan, evaluation_from_fact
from pddlstream.language.external import Result
from pddlstream.language.statistics import compute_plan_effort
from pddlstream.utils import Verbose, INF

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
    # Need unit_costs=True otherwise obtain an instantiation error
    problem = get_problem(evaluations, goal_expression, stream_domain, unit_costs=True)
    with Verbose():
        new_instantiated = instantiate_task(task_from_domain_problem(stream_domain, problem))
    instantiated.axioms[:] = new_instantiated.axioms
    instantiated.atoms.update(new_instantiated.atoms)

##################################################

def plan_streams(evaluations, goal_expression, domain, all_results, negative, effort_weight, max_effort,
                 simultaneous=False, reachieve=True, debug=False, **kwargs):
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
    add_stream_efforts(node_from_atom, instantiated, effort_weight)
    if using_optimizers(applied_results):
        add_optimizer_effects(instantiated, node_from_atom)
    action_from_name = rename_instantiated_actions(instantiated)
    with Verbose(debug):
        sas_task = sas_from_instantiated(instantiated)
        sas_task.metric = True

    # TODO: apply renaming to hierarchy as well
    # solve_from_task | serialized_solve_from_task | abstrips_solve_from_task | abstrips_solve_from_task_sequential
    action_plan, raw_cost = solve_from_task(sas_task, debug=debug, **kwargs)
    #print(raw_cost)
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
