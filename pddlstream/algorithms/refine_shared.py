from __future__ import print_function

from itertools import product

from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.reorder import separate_plan
from pddlstream.algorithms.scheduling.utils import evaluations_from_stream_plan
from pddlstream.algorithms.algorithm import dump_plans, partition_externals
from pddlstream.algorithms.visualization import create_visualizations
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.stream import StreamResult
from pddlstream.language.function import FunctionResult
from pddlstream.language.external import compute_instance_effort
from pddlstream.language.object import OptimisticObject
from pddlstream.utils import INF, get_length, safe_zip, get_mapping

# TODO: lazily expand the shared objects in some cases to prevent increase in size
# TODO: constrain to using the previous plan skeleton
# TODO: only use samples in the preimage and plan as well as initial state
# TODO: control the complexity of both real and optimistic samples based on effort/depth (or holdout)
# TODO: can iteratively increase bound when instantiating until finite heuristic

RECURSIVE = True
DOUBLE_BOUND = False

def get_stream_plan_index(stream_plan):
    return max([0] + [r.opt_index for r in stream_plan])

# def get_ancestors(obj):
#     if not isinstance(obj, OptimisticObject):
#         return {obj}
#     return {obj}

def is_double_bound(stream_instance, double_bindings):
    if double_bindings is None:
        return True
    # What if the same object input? Might be okay because not useful anyways?
    # Apply recursively, ensure that not counting history within a single object
    # Could just do one level of expansion as well (I think this is what's causing it
    # used_bindings = {}
    # for obj in stream_instance.input_objects:
    #     for ancestor in get_ancestors(obj):
    #         if ancestor in double_bindings:
    #             shared = double_bindings[ancestor]
    #             if ancestor in used_bindings.setdefault(shared, set()):
    #                 return True
    #             used_bindings[shared].add(ancestor)
    # return False
    bindings = [double_bindings[o] for o in stream_instance.input_objects
                if o in double_bindings]
    return len(set(bindings)) != len(bindings)

##################################################

def optimistic_process_instance(instantiator, instance, effort):
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

def optimistic_process_streams(evaluations, streams, double_bindings=None,
                               initial_effort=0, **kwargs):
    # TODO: enforce that the search uses one optimistic object before claiming the next (like in my first version)
    # Can even fall back on converting streams to test streams
    results = []
    instantiator = Instantiator(evaluations, streams, **kwargs)
    while instantiator.stream_queue:
        instance, effort = instantiator.pop_stream()
        if not is_double_bound(instance, double_bindings):
            continue
        if initial_effort < effort: # TODO: different increment
            # TODO: periodically solve here
            initial_effort = effort
        results.extend(optimistic_process_instance(instantiator, instance, effort))
    results.extend(optimistic_process_function_queue(instantiator))
    return results

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

# def stream_plan_preimage(stream_plan):
#     preimage = set()
#     for stream_result in reversed(stream_plan):
#         preimage -= set(stream_result.get_certified())
#         preimage |= set(stream_result.instance.get_domain())
#     return preimage

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
    plan_index = get_stream_plan_index(stream_plan)
    if plan_index == 0:
        return combined_plan, cost, depth
    stream_results, bindings = optimistic_process_stream_plan(evaluations, stream_plan)
    # TODO: should I just plan using all original plus expanded
    # TODO: might need new actions here (such as a move)
    # TODO: plan up to first action that only has one
    # Only use actions in the states between the two
    # planned_instances = []
    # for name, args in action_plan:
    #     input_objects = [bindings.get(i, [i]) for i in args]
    #     instances = []
    #     for combo in product(*input_objects):
    #         # TODO: prune actions that aren't feasible
    #         instances.append((name, combo))
    #     planned_instances.append(instances)
    # print(action_plan)
    # print(planned_instances)

    streams, functions, _ = partition_externals(externals)
    if DOUBLE_BOUND:
        # I don't think the double bound thing really makes entire sense here
        double_bindings = {v: k for k, values in bindings.items() if 2 <= len(values) for v in values}
        stream_results.extend(optimistic_process_streams(evaluations_from_stream_plan(
            evaluations, stream_results), streams, double_bindings=double_bindings))
    stream_results.extend(optimistic_process_streams(
        evaluations_from_stream_plan(evaluations, stream_results), functions))
    return recursive_solve_stream_plan(evaluations, externals, stream_results,
                                       solve_stream_plan_fn, depth + 1)


def iterative_solve_stream_plan_old(evaluations, externals, solve_stream_plan_fn, max_effort=INF):
    # TODO: option to toggle commit using max_depth?
    # TODO: constrain to use previous plan to some degree
    num_iterations = 0
    effort_limit = INF # 0 | INF
    unit_efforts = True
    while True:
        num_iterations += 1
        stream_results = optimistic_process_streams(
            evaluations, externals, unit_efforts=unit_efforts, max_effort=max_effort)
        combined_plan, cost, depth = recursive_solve_stream_plan(
            evaluations, externals, stream_results, solve_stream_plan_fn, depth=0)
        print('Attempt: {} | Results: {} | Depth: {} | Success: {}'.format(
            num_iterations, len(stream_results), depth, combined_plan is not None))
        #raw_input('Continue?') # TODO: inspect failures here
        if (combined_plan is not None) or (depth == 0):
            return combined_plan, cost

##################################################

def iterative_solve_stream_plan(evaluations, externals, solve_stream_plan_fn, max_effort=INF):
    #effort_limit = INF # 0 | INF
    unit_efforts = True
    num_iterations = 0
    while True:
        num_iterations += 1
        stream_results = optimistic_process_streams(
            evaluations, externals, unit_efforts=unit_efforts, max_effort=max_effort)
        combined_plan, cost = solve_stream_plan_fn(stream_results)
        if combined_plan is None:
            return combined_plan, cost
        stream_plan, action_plan = separate_plan(combined_plan, stream_only=False)
        if get_stream_plan_index(stream_plan) == 0:
            print('Attempt: {} | Results: {} | Success: {}'.format(
                num_iterations, len(stream_results), combined_plan is not None))
            return combined_plan, cost
        optimistic_process_stream_plan(evaluations, stream_plan)
