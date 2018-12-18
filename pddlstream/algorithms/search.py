from __future__ import print_function

from copy import deepcopy
from time import time

from pddlstream.algorithms.downward import write_sas_task, parse_solution, run_search, TEMP_DIR, sas_from_pddl, write_pddl, \
    translate_and_write_pddl
from pddlstream.utils import INF, Verbose, safe_rm_dir

# TODO: manual_patterns
# Specify on the discrete variables that are updated via conditional effects
# http://www.fast-downward.org/Doc/PatternCollectionGenerator
# TODO: receding horizon planning
# TODO: allow switch to higher-level in heuristic
# TODO: recursive application of these

def solve_from_task(sas_task, temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=[], **kwargs):
    # TODO: can solve using another planner and then still translate using FastDownward
    # Can apply plan constraints (skeleton constraints) here as well
    start_time = time()
    with Verbose(debug):
        print('\n' + 50*'-' + '\n')
        write_sas_task(sas_task, temp_dir)
        solution = run_search(temp_dir, debug=True, **kwargs)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    #for axiom in sas_task.axioms:
    #    # TODO: return the set of axioms here as well
    #    var, value = axiom.effect
    #    print(sas_task.variables.value_names[var])
    #    axiom.dump()
    return parse_solution(solution)

def solve_from_pddl(domain_pddl, problem_pddl, temp_dir=TEMP_DIR, clean=False, debug=False, **kwargs):
    # TODO: combine with solve_from_task
    start_time = time()
    with Verbose(debug):
        write_pddl(domain_pddl, problem_pddl, temp_dir)
        #run_translate(temp_dir, verbose)
        translate_and_write_pddl(domain_pddl, problem_pddl, temp_dir, debug)
        solution = run_search(temp_dir, debug=debug, **kwargs)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    return parse_solution(solution)

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

def plan_subgoals(sas_task, subgoal_plan, temp_dir, **kwargs):
    full_plan = []
    full_cost = 0
    for subgoal in subgoal_plan:
        sas_task.goal.pairs = subgoal
        write_sas_task(sas_task, temp_dir)
        plan, cost = parse_solution(run_search(temp_dir, debug=True, **kwargs))
        if plan is None:
            return None, INF
        full_plan.extend(plan)
        full_cost += cost
        for sas_action in parse_sas_plan(sas_task, plan):
            apply_sas_operator(sas_task.init, sas_action)
    return full_plan, full_cost


def serialized_solve_from_task(sas_task, temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=[], **kwargs):
    # TODO: specify goal grouping / group by predicate & objects
    # TODO: version that solves for all subgoals at once
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


def add_subgoals(sas_task, subgoal_plan):
    if not subgoal_plan:
        return None
    subgoal_var = len(sas_task.variables.ranges)
    subgoal_range = len(subgoal_plan) + 1
    sas_task.variables.ranges.append(subgoal_range)
    sas_task.variables.axiom_layers.append(-1)
    sas_task.variables.value_names.append(
        ['subgoal{}'.format(i) for i in range(subgoal_range)])
    sas_task.init.values.append(0)
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


def abstrips_solve_from_task(sas_task, temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=[], **kwargs):
    # Like partial order planning in terms of precondition order
    # TODO: add achieve subgoal actions
    # TODO: most generic would be a heuristic on each state
    if not hierarchy:
        return solve_from_task(sas_task, temp_dir=temp_dir, clean=clean, debug=debug, **kwargs)
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
            plan, cost = parse_solution(run_search(temp_dir, debug=True, **kwargs))
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
