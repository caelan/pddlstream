from __future__ import print_function

import math

from copy import deepcopy
from time import time
from collections import defaultdict

from pddlstream.algorithms.downward import run_search, TEMP_DIR, write_pddl, DIVERSE_PLANNERS, scale_cost, \
    MAX_FD_COST, DEFAULT_PLANNER, DEFAULT_MAX_TIME, DEFAULT_MAX_PLANS
from pddlstream.algorithms.instantiate_task import write_sas_task, translate_and_write_pddl, \
    parse_sequential_domain, parse_problem, task_from_domain_problem, sas_from_pddl, get_conjunctive_parts
from pddlstream.utils import INF, Verbose, safe_rm_dir, elapsed_time, int_ceil, implies


# TODO: manual_patterns
# Specify on the discrete variables that are updated via conditional effects
# http://www.fast-downward.org/Doc/PatternCollectionGenerator
# TODO: receding horizon planning
# TODO: allow switch to higher-level in heuristic
# TODO: recursive application of these
# TODO: write the domain and problem PDDL files that are used for debugging purposes

def solve_from_task(sas_task, temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=[], **search_args):
    # TODO: can solve using another planner and then still translate using FastDownward
    # Can apply plan constraints (skeleton constraints) here as well
    start_time = time()
    with Verbose(debug):
        print('\n' + 50*'-' + '\n')
        write_sas_task(sas_task, temp_dir)
        solution = run_search(temp_dir, debug=True, **search_args)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    #for axiom in sas_task.axioms:
    #    # TODO: return the set of axioms here as well
    #    var, value = axiom.effect
    #    print(sas_task.variables.value_names[var])
    #    axiom.dump()
    return solution

def solve_from_pddl(domain_pddl, problem_pddl, temp_dir=TEMP_DIR, clean=False, debug=False, **search_args):
    # TODO: combine with solve_from_task
    #return solve_tfd(domain_pddl, problem_pddl)
    start_time = time()
    with Verbose(debug):
        write_pddl(domain_pddl, problem_pddl, temp_dir)
        #run_translate(temp_dir, verbose)
        translate_and_write_pddl(domain_pddl, problem_pddl, temp_dir, debug)
        solution = run_search(temp_dir, debug=debug, **search_args)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    return solution

##################################################

SCALE = 1e3

def get_all_preconditions(sas_action):
    # TODO: use reinstantiate instead
    conditions = get_conjunctive_parts(sas_action.propositional_action.action.precondition)
    return [literal.rename_variables(sas_action.propositional_action.var_mapping) for literal in conditions]

def diverse_from_task(sas_task, use_probabilities=True, prohibit_actions=[], prohibit_predicates=[],
                      planner=DEFAULT_PLANNER, max_planner_time=DEFAULT_MAX_TIME, max_plans=DEFAULT_MAX_PLANS,
                      hierarchy=[], temp_dir=TEMP_DIR,
                      clean=False, debug=False, **search_args):
    # TODO: make a free version of the sas_action after it's applied
    if planner in DIVERSE_PLANNERS:
        return solve_from_task(sas_task, planner=planner, max_planner_time=max_planner_time, max_plans=max_plans,
                               temp_dir=temp_dir, clean=clean, debug=debug, hierarchy=hierarchy, **search_args)
    # TODO: non-probabilistic version of this for action costs
    assert prohibit_actions or prohibit_predicates
    assert not isinstance(prohibit_actions, dict)
    assert implies(use_probabilities, prohibit_predicates)
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
                    sas_action.cost = 1 # 0 | 1
                    likelihood = 1.
                    for precondition in get_all_preconditions(sas_action):
                        likelihood *= prob_from_precondition[precondition]
                    if likelihood == 0:
                        sas_action.cost += INF
                    else:
                        sas_action.cost += int_ceil(-SCALE*math.log(likelihood))
                    sas_action.cost = min(sas_action.cost, MAX_FD_COST)

            write_sas_task(sas_task, temp_dir)
            remaining_time = max_planner_time - elapsed_time(start_time)
            solution = run_search(temp_dir, debug=debug, planner=planner,
                                  max_planner_time=remaining_time, max_plans=1, **search_args)
            if not solution:
                break
            plans.extend(solution)

            for plan, _ in solution:
                sas_plan = parse_sas_plan(sas_task, plan)
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
                    for precondition in uncertain:
                        p_this = prob_from_precondition[precondition]
                        p_rest = success_prob / p_this
                        p_this_fails = (1 - p_this)*p_rest
                        p_rest_fail = p_this*(1 - p_rest)
                        prob_from_precondition[precondition] = p_rest_fail / (p_this_fails + p_rest_fail)

                if condition:
                    axiom = sas_tasks.SASAxiom(condition=condition, effect=(deadend_var, 1))
                    sas_task.axioms.append(axiom)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', elapsed_time(start_time))
    return plans

def diverse_from_pddl(domain_pddl, problem_pddl, temp_dir=TEMP_DIR, debug=False, **kwargs):
    # TODO: methods for diverse manipulation solutions (like Marc)
    # TODO: assert not in DIVERSE_PLANNERS
    with Verbose(debug):
        write_pddl(domain_pddl, problem_pddl, temp_dir)
        domain = parse_sequential_domain(domain_pddl)
        problem = parse_problem(domain, problem_pddl)
        task = task_from_domain_problem(domain, problem)
        sas_task = sas_from_pddl(task)
        return diverse_from_task(sas_task, temp_dir=temp_dir, debug=debug, **kwargs)

##################################################

def apply_sas_operator(init, op):
    for var, pre, post, cond in op.pre_post:
        assert (pre == -1) or (init.values[var] == pre)
        assert not cond
        init.values[var] = post


def name_from_action(action, args):
    return '({})'.format(' '.join((action,) + args))

def parse_sas_plan(sas_task, plan):
    op_from_name = {op.name: op for op in sas_task.operators} # No need to keep repeats
    sas_plan = []
    for action, args in plan:
        name = name_from_action(action, args)
        sas_plan.append(op_from_name[name])
    return sas_plan

##################################################

SERIALIZE = 'serialize'

def plan_subgoals(sas_task, subgoal_plan, temp_dir, **kwargs):
    full_plan = []
    full_cost = 0
    for subgoal in subgoal_plan:
        sas_task.goal.pairs = subgoal
        write_sas_task(sas_task, temp_dir)
        plan, cost = run_search(temp_dir, debug=True, **kwargs)
        if plan is None:
            return None, INF
        full_plan.extend(plan)
        full_cost += cost
        for sas_action in parse_sas_plan(sas_task, plan):
            apply_sas_operator(sas_task.init, sas_action)
    return full_plan, full_cost


def serialized_solve_from_task(sas_task, temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=[], **kwargs):
    # TODO: specify goal grouping / group by predicate & objects
    # TODO: version that solves for all disjuctive subgoals at once
    start_time = time()
    with Verbose(debug):
        print('\n' + 50*'-' + '\n')
        subgoal_plan = [sas_task.goal.pairs[:i+1] for i in range(len(sas_task.goal.pairs))]
        plan, cost = plan_subgoals(sas_task, subgoal_plan, temp_dir, **kwargs)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    return plan, cost

##################################################

class ABSTRIPSLayer(object):
    def __init__(self, pos_pre=[], neg_pre=[], pos_eff=[], neg_eff=[], horizon=INF):
        self.pos_pre = pos_pre
        self.neg_pre = neg_pre
        self.pos_eff = pos_eff
        self.neg_eff = neg_eff
        self.horizon = horizon # TODO: cost units instead?
        assert 1 <= self.horizon
        if self.pos_eff:
            raise NotImplementedError()
        if self.neg_eff:
            raise NotImplementedError()

##################################################

def prune_hierarchy_pre_eff(sas_task, layers):
    positive_template = 'Atom {}('
    negated_template = 'NegatedAtom {}('
    pruned_pre = set()  # TODO: effects
    for layer in layers:
        pruned_pre.update(positive_template.format(p.lower()) for p in layer.pos_pre)
        pruned_pre.update(negated_template.format(p.lower()) for p in layer.neg_pre)
    pruned = set()
    for var, names in enumerate(sas_task.variables.value_names):
        for val, name in enumerate(names):
            if any(name.startswith(p) for p in pruned_pre):
                pruned.add((var, val))
    for op in sas_task.operators:
        for k, pair in reversed(list(enumerate(op.prevail))):
            if pair in pruned:
                op.prevail.pop(0)
    for k, pair in reversed(list(enumerate(sas_task.goal.pairs))):
        if pair in pruned:
            sas_task.goal.pairs.pop(0)
    return pruned


def add_var(sas_task, domain=2, layer=-1, init_value=0, prefix='value'):
    # http://www.fast-downward.org/TranslatorOutputFormat
    subgoal_var = len(sas_task.variables.ranges)
    sas_task.variables.ranges.append(domain)
    sas_task.variables.axiom_layers.append(layer)
    sas_task.variables.value_names.append(
        ['{}{}'.format(prefix, i) for i in range(domain)])
    sas_task.init.values.append(init_value)
    return subgoal_var


def add_subgoals(sas_task, subgoal_plan):
    if not subgoal_plan:
        return None

    subgoal_range = len(subgoal_plan) + 1
    subgoal_var = add_var(sas_task, domain=subgoal_range, init_value=0, prefix='subgoal')
    sas_task.goal.pairs.append((subgoal_var, subgoal_range - 1))

    # TODO: make this a subroutine that depends on the length
    for i, op in enumerate(sas_task.operators):
        if op.name not in subgoal_plan:
            continue
        subgoal = subgoal_plan.index(op.name) + 1
        pre_post = (subgoal_var, subgoal - 1, subgoal, [])
        op.pre_post.append(pre_post)
        # TODO: maybe this should be the resultant state instead?
        # TODO: prevail should just be the last prevail
        # name = '(subgoal{}_{})'.format(subgoal, i)
        # subgoal_cost = 1  # Can strengthen for stronger heuristics
        # local_sas_task.operators.append(sas_tasks.SASOperator(
        #    name, op.prevail, [pre_post], subgoal_cost))
    return subgoal_var


def abstrips_solve_from_task(sas_task, temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=None, **kwargs):
    # Like partial order planning in terms of precondition order
    # TODO: add achieve subgoal actions
    # TODO: most generic would be a heuristic on each state
    if hierarchy is None:
        return solve_from_task(sas_task, temp_dir=temp_dir, clean=clean, debug=debug, **kwargs)
    if hierarchy == SERIALIZE:
        return serialized_solve_from_task(sas_task, temp_dir=temp_dir, clean=clean, debug=debug, **kwargs)
    start_time = time()
    plan, cost = None, INF
    with Verbose(debug):
        print('\n' + 50*'-' + '\n')
        last_plan = []
        for level in range(len(hierarchy)+1):
            local_sas_task = deepcopy(sas_task)
            prune_hierarchy_pre_eff(local_sas_task, hierarchy[level:]) # TODO: break if no pruned
            add_subgoals(local_sas_task, last_plan)
            write_sas_task(local_sas_task, temp_dir)
            plan, cost = run_search(temp_dir, debug=True, **kwargs)
            if (level == len(hierarchy)) or (plan is None):
                # TODO: fall back on standard search
                break
            last_plan = [name_from_action(action, args) for action, args in plan]
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    return plan, cost

##################################################

# TODO: can structure these subproblems prioritizing depth rather than width
# TODO: reconcile shared objects on each level
# Each operator in the hierarchy is a legal "operator" that may need to be refined

def abstrips_solve_from_task_sequential(sas_task, temp_dir=TEMP_DIR, clean=False, debug=False,
                                        hierarchy=[], subgoal_horizon=1, **kwargs):
    # TODO: version that plans for each goal individually
    # TODO: can reduce to goal serialization if binary flag for each subgoal
    if not hierarchy:
        return solve_from_task(sas_task, temp_dir=temp_dir, clean=clean, debug=debug, **kwargs)
    start_time = time()
    plan, cost = None, INF
    with Verbose(debug):
        last_plan = None
        for level in range(len(hierarchy) + 1):
            local_sas_task = deepcopy(sas_task)
            prune_hierarchy_pre_eff(local_sas_task, hierarchy[level:])  # TODO: break if no pruned
            # The goal itself is effectively a subgoal
            # Handle this subgoal horizon
            subgoal_plan = [local_sas_task.goal.pairs[:]]
            # TODO: do I want to consider the "subgoal action" as a real action?
            if last_plan is not None:
                subgoal_var = add_subgoals(local_sas_task, last_plan)
                subgoal_plan = [[(subgoal_var, val)] for val in range(1,
                    local_sas_task.variables.ranges[subgoal_var], subgoal_horizon)] + subgoal_plan
                hierarchy_horizon = min(hierarchy[level-1].horizon, len(subgoal_plan))
                subgoal_plan = subgoal_plan[:hierarchy_horizon]
            plan, cost = plan_subgoals(local_sas_task, subgoal_plan, temp_dir, **kwargs)
            if (level == len(hierarchy)) or (plan is None):
                # TODO: fall back on normal
                # TODO: search in space of subgoals
                break
            last_plan = [name_from_action(action, args) for action, args in plan]
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    # TODO: record which level of abstraction each operator is at when returning
    # TODO: return instantiated actions here rather than names (including pruned pre/eff)
    return plan, cost
