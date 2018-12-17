from __future__ import print_function

from itertools import product

from pddlstream.algorithms.algorithm import partition_externals
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.reorder import separate_plan
from pddlstream.algorithms.scheduling.utils import evaluations_from_stream_plan
from pddlstream.language.constants import FAILED, INFEASIBLE
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult, Result
from pddlstream.utils import INF, safe_zip, get_mapping

# TODO: lazily expand the shared objects in some cases to prevent increase in size
# TODO: constrain to using the previous plan skeleton
# TODO: only use samples in the preimage and plan as well as initial state

RECURSIVE = True

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

def optimistic_process_function_queue(instantiator, **kwargs):
    while instantiator.function_queue:
        for result in optimistic_process_instance(instantiator, *instantiator.pop_function(), **kwargs):
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
    evaluation_set = set(evaluations)
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
            if (instance.opt_index != 0) and (not immediate or (domain <= evaluation_set)):
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

# TODO: can instantiate all but subtract stream_results
# TODO: can even pass a subset of the fluent state
# TODO: can just compute the stream plan preimage
# TODO: replan constraining the initial state and plan skeleton
# TODO: reuse subproblems
# TODO: always start from the initial state (i.e. don't update)
# TODO: apply hierarchical planning to restrict the set of streams that considered on each subproblem

def recursive_solve_stream_plan(evaluations, externals, stream_results, solve_stream_plan_fn, depth):
    if not RECURSIVE and (depth != 0):
        return None, INF, depth
    combined_plan, cost = solve_stream_plan_fn(stream_results)
    stream_plan, action_plan = separate_plan(combined_plan, stream_only=False)
    #dump_plans(stream_plan, action_plan, cost)
    #create_visualizations(evaluations, stream_plan, depth)
    #print(depth, get_length(stream_plan))
    if stream_plan is None:
        return stream_plan, cost, depth
    if is_refined(stream_plan):
        return combined_plan, cost, depth
    stream_results, bindings = optimistic_process_stream_plan(evaluations, stream_plan)
    # TODO: should I just plan using all original plus expanded
    # TODO: might need new actions here (such as a move)
    # TODO: plan up to first action that only has one
    # TODO: only use streams in the states between the two actions
    streams, functions, _ = partition_externals(externals)
    stream_results.extend(optimistic_process_streams(
        evaluations_from_stream_plan(evaluations, stream_results), functions))
    return recursive_solve_stream_plan(evaluations, externals, stream_results,
                                       solve_stream_plan_fn, depth + 1)


def iterative_solve_stream_plan_old(evaluations, externals, optimistic_solve_fn, max_effort=INF):
    # TODO: option to toggle commit using max_depth?
    # TODO: constrain to use previous plan to some degree
    num_iterations = 0
    unit_efforts = True
    while True:
        num_iterations += 1
        stream_results = optimistic_process_streams(evaluations, externals, optimistic_solve_fn,
                                                    unit_efforts=unit_efforts, max_effort=max_effort)
        combined_plan, cost, depth = recursive_solve_stream_plan(
            evaluations, externals, stream_results, optimistic_solve_fn, depth=0)
        print('Attempt: {} | Results: {} | Depth: {} | Success: {}'.format(
            num_iterations, len(stream_results), depth, combined_plan is not None))
        #raw_input('Continue?') # TODO: inspect failures here
        if (combined_plan is not None) or (depth == 0):
            return combined_plan, cost

##################################################

def iterative_solve_stream_plan(evaluations, externals, optimistic_solve_fn, **effort_args):
    # Previously didn't have unique optimistic objects that could be constructed at arbitrary depths
    while True:
        #combined_plan, cost, initial_effort = process_and_solve_streams(
        #    evaluations, externals, solve_stream_plan_fn,
        #    initial_effort=initial_effort, unit_efforts=unit_efforts, max_effort=max_effort)
        results, full = optimistic_process_streams(evaluations, externals, **effort_args)
        combined_plan, cost = optimistic_solve_fn(results)
        if combined_plan is None:
            status = INFEASIBLE if full else FAILED
            return status, cost
        stream_plan, action_plan = separate_plan(combined_plan, stream_only=False)
        if is_refined(stream_plan):
            return combined_plan, cost
        optimistic_process_stream_plan(evaluations, stream_plan)
