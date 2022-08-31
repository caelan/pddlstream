from __future__ import print_function

from collections import defaultdict
from time import time

from pddlstream.algorithms.downward import run_search, TEMP_DIR, DEFAULT_PLANNER, DEFAULT_MAX_TIME
from pddlstream.algorithms.instantiate_task import write_sas_task, get_conjunctive_parts
from pddlstream.algorithms.search import add_var, solve_from_task, parse_sas_plan
from pddlstream.utils import INF, Verbose, safe_rm_dir, elapsed_time, int_ceil

def str_from_action(action):
    name, args = action
    return '{}({})'.format(name, ', '.join(args))

def str_from_plan(plan):
    if plan is None:
        return plan
    return '[{}]'.format(', '.join(map(str_from_action, plan)))

def get_all_preconditions(sas_action):
    # TODO: use reinstantiate instead
    conditions = get_conjunctive_parts(sas_action.propositional_action.action.precondition)
    return [literal.rename_variables(sas_action.propositional_action.var_mapping) for literal in conditions]

PRIOR_PLANS = [] # TODO: clear between runs

def diverse_from_task(sas_task, prohibit_actions=True, prohibit_predicates=[],
                      planner=DEFAULT_PLANNER, max_planner_time=DEFAULT_MAX_TIME, max_plans=1,
                      hierarchy=[], temp_dir=TEMP_DIR, clean=False, debug=False, **search_args):
    # TODO: make a free version of the sas_action after it's applied
    # if max_plans == 1:
    #     return solve_from_task(sas_task, planner=planner, max_planner_time=max_planner_time, max_plans=max_plans,
    #                            temp_dir=temp_dir, clean=clean, debug=debug, hierarchy=hierarchy, **search_args)
    assert prohibit_actions or prohibit_predicates
    assert not isinstance(prohibit_actions, dict)
    import sas_tasks
    # TODO: SAS translation might not be the same across iterations (but action names will)
    prohibit_predicates = list(map(str.lower, prohibit_predicates))
    start_time = time()
    plans = [] # TODO: persist over time
    var_from_action = {}
    actions_from_precondition = defaultdict(set)
    cost_from_action = {action: action.cost for action in sas_task.operators}
    with Verbose(True): # TODO: remove
        deadend_var = add_var(sas_task, layer=1)
        for sas_action in sas_task.operators:
            sas_action.prevail.append((deadend_var, 0))
            for precondition in get_all_preconditions(sas_action):
                actions_from_precondition[precondition].add(sas_action)
        sas_task.goal.pairs.append((deadend_var, 0))

        def forbid_plan(plan):
            sas_plan = parse_sas_plan(sas_task, plan)
            if sas_plan is None:
                return None
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

                if (prohibit_actions is True) or (action.name in prohibit_actions):
                    if sas_action not in var_from_action:
                        var = add_var(sas_task)
                        sas_action.pre_post.append((var, -1, 1, []))  # var, pre, post, cond
                        var_from_action[sas_action] = var
                    condition.append((var_from_action[sas_action], 1))
            if not condition:
                return None
            axiom = sas_tasks.SASAxiom(condition=condition, effect=(deadend_var, 1))
            sas_task.axioms.append(axiom)
            return axiom

        # TODO: option to not use PRIOR_PLANS
        for plan, _ in PRIOR_PLANS:
            forbid_plan(plan)

        while (elapsed_time(start_time) < max_planner_time) and (len(plans) < max_plans):
            write_sas_task(sas_task, temp_dir)
            remaining_time = max_planner_time - elapsed_time(start_time)
            with Verbose(debug):
                plan, cost = run_search(temp_dir, debug=debug, planner=planner,
                                        max_planner_time=remaining_time, **search_args) # max_plans=1
            solutions = [] if plan is None else [(plan, cost)]

            if not solutions:
                break
            for plan, _ in solutions:
                sas_plan = parse_sas_plan(sas_task, plan)
                assert sas_plan is not None
                cost = sum(cost_from_action[action] for action in sas_plan)
                plans.append((plan, cost))
                forbid_plan(plan)
                print('Plan: {} | Cost: {} | Length: {} | Runtime: {:.3f} | Remaining: {:.3f}\nActions: {}'.format(
                        len(plans), cost, len(plan), elapsed_time(start_time), max_planner_time - elapsed_time(start_time),
                    str_from_plan(plan)))
            if len(plan) == 0:
                break

        if clean:
            safe_rm_dir(temp_dir)
        print('Plans: {} | Total runtime: {:.3f}'.format(len(plans), elapsed_time(start_time)))
        # for i, (plan, cost) in enumerate(plans):
        #     print('Plan: {} | Cost: {} | Length: {} | Actions: {}'.format(
        #         i, cost, len(plan), str_from_plan(plan)))

    PRIOR_PLANS.extend(plans)
    return plans
