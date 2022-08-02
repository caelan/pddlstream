from __future__ import print_function

import math

from copy import deepcopy
from time import time
from collections import defaultdict

from pddlstream.algorithms.downward import run_search, TEMP_DIR, write_pddl, scale_cost, \
    MAX_FD_COST, DEFAULT_PLANNER, DEFAULT_MAX_TIME
from pddlstream.algorithms.instantiate_task import write_sas_task, translate_and_write_pddl, \
    parse_sequential_domain, parse_problem, task_from_domain_problem, sas_from_pddl, get_conjunctive_parts
from pddlstream.utils import INF, Verbose, safe_rm_dir, elapsed_time, int_ceil, implies
from pddlstream.algorithms.search import add_var, solve_from_task, parse_sas_plan

DEFAULT_MAX_PLANS = 1
DIVERSE_PLANNERS = []

SCALE = 1e3

def get_all_preconditions(sas_action):
    # TODO: use reinstantiate instead
    conditions = get_conjunctive_parts(sas_action.propositional_action.action.precondition)
    return [literal.rename_variables(sas_action.propositional_action.var_mapping) for literal in conditions]

def negative_log(likelihood):
    return int_ceil(-SCALE*math.log(likelihood))

def diverse_from_task(sas_task, use_probabilities=False, base_cost=0, # 0 | 1
                      prohibit_actions=[], prohibit_predicates=[],
                      planner=DEFAULT_PLANNER, max_planner_time=DEFAULT_MAX_TIME, max_plans=DEFAULT_MAX_PLANS,
                      hierarchy=[], temp_dir=TEMP_DIR, clean=False, debug=False, **search_args):
    # TODO: make a free version of the sas_action after it's applied
    if planner in DIVERSE_PLANNERS:
        return solve_from_task(sas_task, planner=planner, max_planner_time=max_planner_time, max_plans=max_plans,
                               temp_dir=temp_dir, clean=clean, debug=debug, hierarchy=hierarchy, **search_args)
    # TODO: non-probabilistic version of this for action costs
    assert prohibit_actions or prohibit_predicates
    assert not isinstance(prohibit_actions, dict)
    assert implies(use_probabilities, isinstance(prohibit_predicates, dict) and prohibit_predicates)
    import sas_tasks
    if isinstance(prohibit_predicates, dict):
        prohibit_predicates = {predicate.lower(): prob for predicate, prob in prohibit_predicates.items()}
    else:
        prohibit_predicates = list(map(str.lower, prohibit_predicates))
    start_time = time()
    plans = []
    var_from_action = {}
    # TODO: weighted average
    prob_from_precondition = defaultdict(lambda: 1.) # TODO: cost/effort
    actions_from_precondition = defaultdict(set)
    cost_from_action = {action: action.cost for action in sas_task.operators}
    with Verbose(debug):
        deadend_var = add_var(sas_task, layer=1)
        for sas_action in sas_task.operators:
            sas_action.prevail.append((deadend_var, 0))
            for precondition in get_all_preconditions(sas_action):
                actions_from_precondition[precondition].add(sas_action)
                if isinstance(prohibit_predicates, dict) and (precondition.predicate in prohibit_predicates):
                    # TODO: function of arguments
                    prob_from_precondition[precondition] = prohibit_predicates[precondition.predicate]
        sas_task.goal.pairs.append((deadend_var, 0))

        while (elapsed_time(start_time) < max_planner_time) and (len(plans) < max_plans):
            if use_probabilities:
                for sas_action in sas_task.operators:
                    #scale_cost(prob_from_precondition[action])
                    sas_action.cost = base_cost
                    likelihood = 1.
                    for precondition in get_all_preconditions(sas_action):
                        likelihood *= prob_from_precondition[precondition]
                    if likelihood == 0:
                        sas_action.cost += INF # TODO: remove the action
                    else:
                        sas_action.cost += negative_log(likelihood)
                    sas_action.cost = min(sas_action.cost, MAX_FD_COST)

            write_sas_task(sas_task, temp_dir)
            remaining_time = max_planner_time - elapsed_time(start_time)
            solutions = run_search(temp_dir, debug=debug, planner=planner,
                                   max_planner_time=remaining_time, max_plans=1, **search_args)
            if not solutions:
                break
            for plan, neg_log_prob in solutions:
                sas_plan = parse_sas_plan(sas_task, plan)
                cost = sum(cost_from_action[action] for action in sas_plan)
                plans.append((plan, cost))
                uncertain = set()
                condition = []
                for action, sas_action in zip(plan, sas_plan):
                    for precondition in get_all_preconditions(sas_action):
                        if precondition.predicate in prohibit_predicates:
                            if precondition not in var_from_action:
                                var = add_var(sas_task)
                                var_from_action[precondition] = var
                                for sas_action2 in actions_from_precondition[precondition]:
                                    sas_action2.pre_post.append((var, -1, 1, []))  # var, pre, post, cond
                            condition.append((var_from_action[precondition], 1))
                            uncertain.add(precondition)

                    if (prohibit_actions is True) or (action.name in prohibit_actions):
                        if sas_action not in var_from_action:
                            var = add_var(sas_task)
                            sas_action.pre_post.append((var, -1, 1, []))  # var, pre, post, cond
                            var_from_action[sas_action] = var
                        condition.append((var_from_action[sas_action], 1))
                        #uncertain.append(precondition)
                        #success_prob *= prob

                if use_probabilities:
                    # success_prob = compute_pre_prob(uncertain, prohibit_predicates)
                    success_prob = 1.
                    for precondition in uncertain:
                        success_prob *= prob_from_precondition[precondition]
                    print('Plan: {} | Cost: {} | Length: {} | -log(prob): {:.3f} '
                          '| Success prob: {:.3f} | Runtime: {:.3f}'.format(
                        len(plans), cost, len(plan), neg_log_prob, #negative_log(success_prob),
                        success_prob, elapsed_time(start_time)))
                    if success_prob == 1.:
                        return plans
                    for precondition in uncertain:
                        # TODO: implement new cost transform
                        # Bayes update
                        p_this = prob_from_precondition[precondition] # Pr(E_l)
                        #p_rest = success_prob / p_this # Pr(\bigcap_{l' \neq l} E_{l'})
                        #p_rest_fail = p_this*(1 - p_rest) # Pr(E_l \bigcap_{l' \neq l} E_{l'})
                        p_rest_fail = p_this - success_prob # equivalent to p_rest_fail above
                        p_fail = 1 - success_prob # 1 - Pr(\bigcap_{l'} E_{l'})
                        prob_from_precondition[precondition] = p_rest_fail / p_fail
                else:
                    print('Plan: {} | Cost: {} | Length: {} | Runtime: {:.3f}'.format(
                        len(plans), cost, len(plan), elapsed_time(start_time)))

                if condition:
                    axiom = sas_tasks.SASAxiom(condition=condition, effect=(deadend_var, 1))
                    sas_task.axioms.append(axiom)
        if clean:
            safe_rm_dir(temp_dir)
        print('Plans: {} | Total runtime: {:.3f}'.format(len(plans), elapsed_time(start_time)))
    return