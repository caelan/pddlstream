from collections import defaultdict
from itertools import product

from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.reorder import separate_plan
from pddlstream.algorithms.scheduling.simultaneous import evaluations_from_stream_plan
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.stream import StreamResult


def get_stream_plan_index(stream_plan):
    if not stream_plan:
        return 0
    return max(r.opt_index for r in stream_plan)


def is_double_bound(stream_instance, double_bindings):
    if double_bindings is None:
        return True
    bindings = [double_bindings[o] for o in stream_instance.input_objects if o in double_bindings]
    return len(set(bindings)) != len(bindings)


def optimistic_process_streams(evaluations, streams, double_bindings=None):
    instantiator = Instantiator(evaluations, streams)
    stream_results = []
    while instantiator.stream_queue:
        stream_instance = instantiator.stream_queue.popleft()
        if not is_double_bound(stream_instance, double_bindings):
            continue
        for stream_result in stream_instance.next_optimistic():
            for fact in stream_result.get_certified():
                instantiator.add_atom(evaluation_from_fact(fact))
            stream_results.append(stream_result) # TODO: don't readd if all repeated facts?
    return stream_results

##################################################

def optimistic_stream_grounding(stream_instance, bindings, evaluations, opt_evaluations, immediate=False):
    # TODO: combination for domain predicates
    evaluation_set = set(evaluations)
    opt_instances = []
    input_objects = [bindings.get(i, [i]) for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = dict(zip(stream_instance.input_objects, combo))
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
    opt_bindings = defaultdict(list)
    opt_results = []
    for opt_result in stream_plan:
        # TODO: could just do first step
        for instance in optimistic_stream_grounding(opt_result.instance, opt_bindings,
                                                    evaluations, opt_evaluations):
            results = instance.next_optimistic()
            opt_evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            opt_results += results
            for result in results:
                if isinstance(result, StreamResult): # Could not add if same value
                    for opt, obj in zip(opt_result.output_objects, result.output_objects):
                        opt_bindings[opt].append(obj)
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

def recursive_solve_stream_plan(evaluations, streams, functions, stream_results, solve_stream_plan, depth):
    # TODO: check empty plan first?
    combined_plan, cost = solve_stream_plan(stream_results)
    stream_plan, action_plan = separate_plan(combined_plan, action_info=None, terminate=False, stream_only=False)
    if stream_plan is None:
        return stream_plan, cost, depth
    plan_index = get_stream_plan_index(stream_plan)
    if plan_index == 0:
        return combined_plan, cost, depth
    # TODO: should I just plan using all original plus expanded
    # TODO: might need new actions here (such as a move)
    stream_results, bindings = optimistic_process_stream_plan(evaluations, stream_plan)
    double_bindings = {v: k for k, values in bindings.items() if 2 <= len(values) for v in values}
    stream_results += optimistic_process_streams(evaluations_from_stream_plan(evaluations, stream_results),
                                                 streams, double_bindings=double_bindings)
    stream_results += optimistic_process_streams(evaluations_from_stream_plan(evaluations, stream_results),
                                                 functions)
    return recursive_solve_stream_plan(evaluations, streams, functions, stream_results,
                                       solve_stream_plan, depth + 1)


def iterative_solve_stream_plan(evaluations, streams, functions, solve_stream_plan):
    # TODO: option to toggle commit using max_depth?
    # TODO: constrain to use previous plan to some degree
    num_iterations = 0
    while True:
        num_iterations += 1
        stream_results = optimistic_process_streams(evaluations, streams + functions)
        combined_plan, cost, depth = recursive_solve_stream_plan(evaluations, streams, functions,
                                                                 stream_results, solve_stream_plan, 0)
        print('Attempt: {} | Results: {} | Depth: {} | Success: {}'.format(num_iterations, len(stream_results),
                                                                           depth, combined_plan is not None))
        #raw_input('Continue?') # TODO: inspect failures here
        if (combined_plan is not None) or (depth == 0):
            return combined_plan, cost
