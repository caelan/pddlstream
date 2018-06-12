from collections import deque
from heapq import heappush
import time

from pddlstream.conversion import evaluation_from_fact, pddl_from_object
from pddlstream.downward import task_from_domain_problem, get_problem, fact_from_fd
from pddlstream.synthesizer import SynthStreamResult, get_synthetic_stream_plan
from pddlstream.skeleton import optimistic_process_streams, SkeletonKey, Skeleton, SkeletonQueue, instantiate_first
from pddlstream.reorder import get_action_instances, replace_derived, topological_sort, reorder_stream_plan
from pddlstream.scheduling.relaxed import get_goal_instance, plan_preimage, recover_stream_plan
from pddlstream.scheduling.simultaneous import evaluations_from_stream_plan

# TODO: convert back into plan for any sampled value

def extract_order(evaluations, preimage):
    stream_results = set()
    processed = set(map(fact_from_fd, preimage))
    queue = deque(processed)
    while queue:
        fact = queue.popleft()
        result = evaluations[evaluation_from_fact(fact)]
        if result is None:
            continue
        stream_results.add(result)
        for fact2 in result.instance.get_domain():
            if fact2 not in processed:
                queue.append(fact2)

    orders = set()
    for result in stream_results:
        for fact in result.instance.get_domain():
            result2 = evaluations[evaluation_from_fact(fact)]
            if result2 is not None:
                orders.add((result2, result))

    ordered_results = []
    for result in topological_sort(stream_results, orders):
        if isinstance(result, SynthStreamResult):
            ordered_results.extend(result.decompose())
        else:
            ordered_results.append(result)
    return ordered_results

def locally_optimize(evaluations, store, goal_expression, domain, functions, negative, dynamic_streams):
    action_plan = store.best_plan
    if action_plan is None:
        return None
    print('\nPostprocessing') # TODO: postprocess current skeleton as well

    task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs=False))
    plan_instances = get_action_instances(task, action_plan) + [get_goal_instance(task.goal)]
    replace_derived(task, set(), plan_instances)
    preimage = filter(lambda a: not a.negated, plan_preimage(plan_instances, []))

    opt_stream_plan = []
    opt_from_obj = {}
    for stream_result in extract_order(evaluations, preimage):
        input_objects = tuple(opt_from_obj.get(o, o) for o in stream_result.instance.input_objects)
        instance = stream_result.instance.external.get_instance(input_objects)
        #assert(not instance.disabled)
        instance.disabled = False
        opt_results = instance.next_optimistic()
        if not opt_results:
            continue
        opt_result = opt_results[0]
        opt_stream_plan.append(opt_result)
        for obj, opt in zip(stream_result.output_objects, opt_result.output_objects):
            opt_from_obj[obj] = opt

    opt_stream_plan += optimistic_process_streams(evaluations_from_stream_plan(evaluations, opt_stream_plan), functions)
    opt_action_plan = [(name, tuple(opt_from_obj.get(o, o) for o in args)) for name, args in action_plan]
    pddl_plan = [(name, map(pddl_from_object, args)) for name, args in opt_action_plan]
    stream_plan = recover_stream_plan(evaluations, goal_expression, domain, opt_stream_plan, pddl_plan, negative, unit_costs=False)

    stream_plan = reorder_stream_plan(stream_plan)
    stream_plan = get_synthetic_stream_plan(stream_plan, dynamic_streams)
    print('Stream plan: {}\n'
          'Action plan: {}'.format(stream_plan, opt_action_plan))
    opt_cost = 0 # TODO: compute this cost for real

    store.start_time = time.time()
    store.max_cost = store.best_cost
    #sampling_time = 10
    sampling_time = 0
    queue = SkeletonQueue(evaluations, store)
    queue.new_skeleton(stream_plan, opt_action_plan, opt_cost)
    queue.greedily_process(sampling_time)
    # TODO: compare solution cost and validity?
    # TODO: select which binding
    # TODO: repeatedly do this
