from itertools import product

from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.reorder import separate_plan
from pddlstream.algorithms.scheduling.utils import evaluations_from_stream_plan
from pddlstream.algorithms.scheduling.recover_streams import get_instance_effort
from pddlstream.algorithms.algorithm import dump_plans
from pddlstream.algorithms.visualization import create_visualizations
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.stream import StreamResult
from pddlstream.language.object import OptimisticObject
from pddlstream.utils import INF

# TODO: lazily expand the shared objects in some cases to prevent increase in size

RECURSIVE = True
DOUBLE_BOUND = False

def get_stream_plan_index(stream_plan):
    if not stream_plan:
        return 0
    return max(r.opt_index for r in stream_plan)

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
    bindings = [double_bindings[o] for o in stream_instance.input_objects if o in double_bindings]
    return len(set(bindings)) != len(bindings)


def optimistic_process_streams(evaluations, streams, double_bindings=None, unit_efforts=False, max_effort=INF):
    # TODO: iteratively increase max_effort to bias towards easier streams to start
    # TODO: cut off instantiation using max_effort
    # TODO: make each repeated optimistic object have ordinal more effort
    # TODO: enforce that the search uses one optimistic object before claiming the next (like in my first version)
    # Can even fall back on converting streams to test streams
    # Additive max effort in case something requires a long sequence to achieve
    results = []
    #effort_from_fact = {}
    instantiator = Instantiator(evaluations, streams)
    while instantiator.stream_queue:
        instance = instantiator.stream_queue.popleft()
        if not is_double_bound(instance, double_bindings):
            continue
        effort = get_instance_effort(instance, unit_efforts)
        #op = sum # max | sum
        #total_effort = effort + op(effort_from_fact[fact] for fact in instance.get_domain())
        if max_effort <= effort:
            continue
        for stream_result in instance.next_optimistic():
            for fact in stream_result.get_certified():
                #effort_from_fact[fact] = min(effort_from_fact.get(fact, INF), effort)
                instantiator.add_atom(evaluation_from_fact(fact))
            results.append(stream_result) # TODO: don't readd if all repeated facts?
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
    opt_bindings = {}
    opt_results = []
    for opt_result in stream_plan:
        # TODO: just do the first step of the plan somehow
        for instance in optimistic_stream_grounding(opt_result.instance, opt_bindings,
                                                    evaluations, opt_evaluations):
            results = instance.next_optimistic()
            opt_evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            opt_results += results
            for result in results:
                if isinstance(result, StreamResult): # Could not add if same value
                    for opt, obj in zip(opt_result.output_objects, result.output_objects):
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

def recursive_solve_stream_plan(evaluations, streams, functions, stream_results, solve_stream_plan, depth):
    # TODO: check empty stream plan first?
    if not RECURSIVE and (depth != 0):
        return None, INF, depth
    combined_plan, cost = solve_stream_plan(stream_results)
    stream_plan, action_plan = separate_plan(combined_plan, action_info=None, terminate=False, stream_only=False)
    #dump_plans(stream_plan, action_plan, cost)
    #create_visualizations(evaluations, stream_plan, depth)
    if stream_plan is None:
        return stream_plan, cost, depth
    plan_index = get_stream_plan_index(stream_plan)
    if plan_index == 0:
        return combined_plan, cost, depth

    # TODO: should I just plan using all original plus expanded
    # TODO: might need new actions here (such as a move)
    stream_results, bindings = optimistic_process_stream_plan(evaluations, stream_plan)
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

    if DOUBLE_BOUND:
        # I don't think the double bound thing really makes entire sense here
        double_bindings = {v: k for k, values in bindings.items() if 2 <= len(values) for v in values}
        stream_results.extend(optimistic_process_streams(evaluations_from_stream_plan(evaluations, stream_results),
                                                         streams, double_bindings=double_bindings))
    stream_results.extend(optimistic_process_streams(evaluations_from_stream_plan(evaluations, stream_results), functions))
    return recursive_solve_stream_plan(evaluations, streams, functions, stream_results, solve_stream_plan, depth + 1)


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
