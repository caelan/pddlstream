from copy import deepcopy
from time import time

from pddlstream.algorithms.downward import write_task, parse_solution, run_search, TEMP_DIR, translate_task
from pddlstream.utils import INF, Verbose, safe_rm_dir

def solve_from_task(task, temp_dir=TEMP_DIR, clean=False, debug=False, **kwargs):
    start_time = time()
    with Verbose(debug):
        write_task(translate_task(task), temp_dir)
        solution = run_search(temp_dir, debug=True, **kwargs)
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


def plan_subgoals(sas_task, subgoal_plan, temp_dir, **kwargs):
    op_from_name = {op.name: op for op in sas_task.operators}  # No need to keep repeats
    full_plan = []
    full_cost = 0
    for subgoal in subgoal_plan:
        sas_task.goal.pairs = subgoal
        write_task(sas_task, temp_dir)
        plan, cost = parse_solution(run_search(temp_dir, debug=True, **kwargs))
        if plan is None:
            return None, INF
        full_plan.extend(plan)
        full_cost += cost
        for action, args in plan:
            apply_sas_operator(sas_task.init, op_from_name[name_from_action(action, args)])
    return full_plan, full_cost


def serialized_solve_from_task(task, temp_dir=TEMP_DIR, clean=False, debug=False, **kwargs):
    # TODO: specify goal grouping / group by predicate & objects
    # TODO: version that solves for all subgoals at once
    start_time = time()
    with Verbose(debug):
        sas_task = translate_task(task)
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


def abstrips_solve_from_task(task, temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=[], **kwargs):
    # Like partial order planning in terms of precondition order
    # TODO: add achieve subgoal actions
    # TODO: most generic would be a heuristic on each state
    start_time = time()
    plan, cost = None, INF
    with Verbose(debug):
        sas_task = translate_task(task)
        last_plan = []
        for level in range(len(hierarchy)+1):
            local_sas_task = deepcopy(sas_task)
            prune_hierarchy_pre_eff(local_sas_task, hierarchy[level:]) # TODO: break if no pruned
            add_subgoals(local_sas_task, last_plan)
            write_task(local_sas_task, temp_dir)
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

def abstrips_solve_from_task_sequential(task, temp_dir=TEMP_DIR, clean=False, debug=False,
                             hierarchy=[], subgoal_horizon=1, **kwargs):
    # TODO: version that plans for each goal individually
    # TODO: can reduce to goal serialization if binary flag for each subgoal
    start_time = time()
    plan, cost = None, INF
    with Verbose(debug):
        sas_task = translate_task(task)
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
