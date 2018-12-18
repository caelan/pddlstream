from __future__ import print_function

from itertools import product
from copy import deepcopy, copy

from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.reorder import separate_plan
from pddlstream.algorithms.scheduling.relaxed import relaxed_stream_plan
from pddlstream.algorithms.scheduling.utils import evaluations_from_stream_plan
from pddlstream.algorithms.constraints import add_plan_constraints, PlanConstraints, WILD
from pddlstream.language.constants import FAILED, INFEASIBLE, is_plan
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.function import FunctionResult, Function
from pddlstream.language.stream import StreamResult, Result
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.utils import INF, safe_zip, get_mapping

# TODO: lazily expand the shared objects in some cases to prevent increase in size
# TODO: constrain to using the previous plan skeleton
# TODO: only use samples in the preimage and plan as well as initial state

RECURSIVE = True
CONSTRAIN_STREAMS = False
CONSTRAIN_PLANS = True

def is_refined(stream_plan):
    if stream_plan is None:
        return True
    return max([0] + [r.opt_index for r in stream_plan]) == 0

##################################################

def optimistic_process_instance(instantiator, instance, effort):
    # TODO: convert streams to test streams with extremely high effort
    for result in instance.next_optimistic():
        new_facts = False
        for fact in result.get_certified():
            new_facts |= instantiator.add_atom(evaluation_from_fact(fact), effort)
        if isinstance(result, FunctionResult) or new_facts:
            yield result

def optimistic_process_function_queue(instantiator):
    while instantiator.function_queue:
        instance, effort = instantiator.pop_function()
        for result in optimistic_process_instance(instantiator, instance, effort):
            yield result

##################################################

def extract_level(evaluations, target_atom, combine_fn=max):
    result = evaluations[target_atom]
    if not isinstance(result, Result):
        return 0
    domain_atoms = map(evaluation_from_fact, result.get_domain())
    domain_level = combine_fn([extract_level(evaluations, atom, combine_fn=combine_fn)
                               for atom in domain_atoms] + [0])
    return domain_level + result.call_index + 1

# def process_and_solve_streams(evaluations, streams, solve_fn, initial_effort=0, **kwargs):
#     combine_fn = max
#     results = []
#     instantiator = Instantiator([], streams, combine_fn=combine_fn, **kwargs)
#     for evaluation in evaluations:
#         level = extract_level(evaluations, evaluation, combine_fn=combine_fn)
#         #instantiator.add_atom(evaluation, 0)
#         instantiator.add_atom(evaluation, level)
#     while instantiator.stream_queue:
#         instance, effort = instantiator.pop_stream()
#         if initial_effort < effort: # TODO: different increment
#             results.extend(optimistic_process_function_queue(instantiator))
#             plan, cost = solve_fn(results)
#             print(get_length(plan), cost, len(results), initial_effort, instance)
#             if plan is not None:
#                 return plan, cost, initial_effort
#             initial_effort = effort
#         results.extend(optimistic_process_instance(instantiator, instance, effort))
#     results.extend(optimistic_process_function_queue(instantiator))
#     plan, cost = solve_fn(results)
#     print(get_length(plan), cost, len(results), initial_effort)
#     return plan, cost, initial_effort

def optimistic_process_streams(evaluations, streams, effort_limit=INF, **effort_args):
    combine_fn = max
    instantiator = Instantiator([], streams, combine_fn=combine_fn, **effort_args)
    for evaluation in evaluations:
        #level = 0
        level = extract_level(evaluations, evaluation, combine_fn=combine_fn)
        instantiator.add_atom(evaluation, level)
    results = []
    while instantiator.stream_queue and (instantiator.min_effort() <= effort_limit):
        instance, effort = instantiator.pop_stream()
        results.extend(optimistic_process_instance(instantiator, instance, effort))
    results.extend(optimistic_process_function_queue(instantiator))
    full = not instantiator
    return results, full

##################################################

def optimistic_stream_grounding(stream_instance, bindings, evaluations, opt_evaluations,
                                bind=True, immediate=False):
    # TODO: combination for domain predicates
    opt_instances = []
    if not bind:
        bindings = {}
    input_objects = [bindings.get(i, [i]) for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = get_mapping(stream_instance.input_objects, combo)
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping))) # TODO: could just instantiate first
        if domain <= opt_evaluations:
            instance = stream_instance.external.get_instance(combo)
            if (instance.opt_index != 0) and (not immediate or (domain <= evaluations)):
                instance.opt_index -= 1
            opt_instances.append(instance)
    return opt_instances


def optimistic_process_stream_plan(evaluations, stream_plan):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    evaluations = set(evaluations)
    opt_evaluations = set(evaluations)
    opt_bindings = {}
    opt_results = []
    for opt_result in stream_plan:
        # TODO: just do the first step of the plan somehow
        for instance in optimistic_stream_grounding(
                opt_result.instance, opt_bindings, evaluations, opt_evaluations):
            results = instance.next_optimistic()
            opt_evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            opt_results.extend(results)
            for result in results:
                if isinstance(result, StreamResult): # Could not add if same value
                    for opt, obj in safe_zip(opt_result.output_objects, result.output_objects):
                        opt_bindings.setdefault(opt, []).append(obj)
    return opt_results, opt_bindings

##################################################

def compute_stream_results(evaluations, opt_results, externals, **effort_args):
    # TODO: start from the original evaluations or use the stream plan preimage
    # TODO: only use streams in the states between the two actions
    # TODO: apply hierarchical planning to restrict the set of streams that considered on each subproblem
    # TODO: plan up to first action that only has one
    # TODO: revisit considering double bound streams
    functions = list(filter(lambda s: type(s) is Function, externals))
    opt_evaluations = evaluations_from_stream_plan(evaluations, opt_results)
    return opt_results + optimistic_process_streams(opt_evaluations, functions, **effort_args)[0]

def compute_skeleton_constraints(action_plan, bindings):
    skeleton = []
    for name, args in action_plan:
        new_args = []
        for arg in args:
            if isinstance(arg, Object):
                new_args.append(arg)
            elif isinstance(arg, OptimisticObject):
                assert bindings.get(arg, [])
                if len(bindings[arg]) == 1:
                    new_args.append(bindings[arg][0])
                else:
                    # TODO: pass in a set of possible values
                    # Handle by making predicates for each binding
                    new_args.append(WILD)
            else:
                raise ValueError(arg)
        skeleton.append((name, new_args))
    # exact=False because we might need new actions
    return PlanConstraints(skeletons=[skeleton], exact=False, max_cost=INF)

##################################################

def recursive_solve_stream_plan(evaluations, externals, results, optimistic_solve_fn,
                                depth, constraints, **effort_args):
    if not RECURSIVE and (depth != 0):
        return None, INF, depth
    combined_plan, cost = optimistic_solve_fn(evaluations, results, constraints)
    if not is_plan(combined_plan):
        return combined_plan, cost, depth
    stream_plan, action_plan = separate_plan(combined_plan, stream_only=False)
    #dump_plans(stream_plan, action_plan, cost)
    #create_visualizations(evaluations, stream_plan, depth)
    #print(depth, get_length(stream_plan))
    if is_refined(stream_plan):
        return combined_plan, cost, depth
    opt_results, opt_bindings = optimistic_process_stream_plan(evaluations, stream_plan)
    if CONSTRAIN_STREAMS:
        next_results = compute_stream_results(evaluations, opt_results, externals, **effort_args)
    else:
        next_results, _ = optimistic_process_streams(evaluations, externals, **effort_args)
    next_constraints = None
    if CONSTRAIN_PLANS:
        next_constraints = compute_skeleton_constraints(action_plan, opt_bindings)
    return recursive_solve_stream_plan(evaluations, externals, next_results, optimistic_solve_fn,
                                       depth + 1, next_constraints, **effort_args)


def iterative_solve_stream_plan(evaluations, externals, optimistic_solve_fn, **effort_args):
    # TODO: enforce a max_depth or max plan cost?
    num_iterations = 0
    while True:
        num_iterations += 1
        results, full = optimistic_process_streams(evaluations, externals, **effort_args)
        combined_plan, cost, depth = recursive_solve_stream_plan(
            evaluations, externals, results, optimistic_solve_fn, depth=0, constraints=None, **effort_args)
        print('Attempt: {} | Results: {} | Depth: {} | Success: {}'.format(
            num_iterations, len(results), depth, combined_plan is not None))
        if is_plan(combined_plan):
            return combined_plan, cost
        if depth == 0:
            status = INFEASIBLE if full else FAILED
            return status, cost

##################################################

def iterative_solve_stream_plan_old(evaluations, externals, optimistic_solve_fn, **effort_args):
    # Previously didn't have unique optimistic objects that could be constructed at arbitrary depths
    constraints = None
    while True:
        #combined_plan, cost, initial_effort = process_and_solve_streams(
        #    evaluations, externals, solve_stream_plan_fn,
        #    initial_effort=initial_effort, unit_efforts=unit_efforts, max_effort=max_effort)
        results, full = optimistic_process_streams(evaluations, externals, **effort_args)
        combined_plan, cost = optimistic_solve_fn(evaluations, results, constraints)
        if combined_plan is None:
            status = INFEASIBLE if full else FAILED
            return status, cost
        stream_plan, action_plan = separate_plan(combined_plan, stream_only=False)
        if is_refined(stream_plan):
            return combined_plan, cost
        optimistic_process_stream_plan(evaluations, stream_plan)

##################################################

def get_optimistic_solve_fn(goal_exp, domain, negative, max_cost=INF, **kwargs):
    def fn(evaluations, results, constraints):
        if constraints is None:
            return relaxed_stream_plan(evaluations, goal_exp, domain, results, negative,
                                       max_cost=max_cost, **kwargs)
        #print(*relaxed_stream_plan(evaluations, goal_exp, domain, results, negative,
        #                               max_cost=max_cost, **kwargs))
        # TODO: be careful with the original plan constraints
        #constraints.dump()
        domain2 = deepcopy(domain)
        evaluations2 = copy(evaluations)
        goal_exp2 = add_plan_constraints(constraints, domain2, evaluations2, goal_exp)
        max_cost2 = max_cost if constraints is None else min(max_cost, constraints.max_cost)
        combined_plan, cost = relaxed_stream_plan(evaluations2, goal_exp2, domain2, results, negative,
                                                  max_cost=max_cost2, **kwargs)
        #print(combined_plan, cost)
        #raw_input('Continue?')
        return combined_plan, cost
    return fn