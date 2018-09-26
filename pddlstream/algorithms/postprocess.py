import time
from collections import deque

from pddlstream.algorithms.downward import task_from_domain_problem, get_problem, fact_from_fd, get_action_instances, \
    get_goal_instance, plan_preimage
from pddlstream.algorithms.reorder import replace_derived, reorder_stream_plan
from pddlstream.algorithms.algorithm import dump_plans
from pddlstream.utils import topological_sort
from pddlstream.algorithms.scheduling.simultaneous import get_plan_cost
from pddlstream.algorithms.scheduling.utils import evaluations_from_stream_plan, apply_streams
from pddlstream.algorithms.scheduling.relaxed import recover_stream_plan
from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.algorithms.refine_shared import optimistic_process_streams
from pddlstream.algorithms.visualization import create_visualizations, log_plans
from pddlstream.language.conversion import evaluation_from_fact, pddl_from_object
from pddlstream.language.synthesizer import SynthStreamResult, get_synthetic_stream_plan


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

def recover_opt_stream_plan(evaluations, action_plan, task):
    plan_instances = get_action_instances(task, action_plan) + [get_goal_instance(task.goal)]
    replace_derived(task, set(), plan_instances)
    preimage = filter(lambda a: not a.negated, plan_preimage(plan_instances, []))

    opt_stream_plan = []
    opt_from_obj = {}
    for stream_result in extract_order(evaluations, preimage):
        input_objects = tuple(opt_from_obj.get(o, o) for o in stream_result.instance.input_objects)
        instance = stream_result.instance.external.get_instance(input_objects)
        #instance.opt_index = stream_result.instance.opt_index
        #assert(instance.opt_index == 0)
        instance.opt_index = 0
        #assert(not instance.disabled)
        instance.disabled = False
        opt_results = instance.next_optimistic()
        if not opt_results:
            continue
        opt_result = opt_results[0]
        opt_stream_plan.append(opt_result)
        for obj, opt in zip(stream_result.output_objects, opt_result.output_objects):
            opt_from_obj[obj] = opt
    return opt_stream_plan, opt_from_obj

def locally_optimize(evaluations, store, goal_expression, domain, functions, negative,
                     dynamic_streams, visualize, sampling_time=0):
    action_plan = store.best_plan
    if action_plan is None:
        return None
    print('\nPostprocessing | Cost: {} | Total Time: {:.3f}'.format(store.best_cost, store.elapsed_time()))
    # TODO: postprocess current skeletons as well

    task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs=False))
    opt_stream_plan, opt_from_obj = recover_opt_stream_plan(evaluations, action_plan, task)
    opt_stream_plan += optimistic_process_streams(evaluations_from_stream_plan(evaluations, opt_stream_plan), functions)
    opt_action_plan = [(name, tuple(opt_from_obj.get(o, o) for o in args)) for name, args in action_plan]
    pddl_plan = [(name, tuple(map(pddl_from_object, args))) for name, args in opt_action_plan]
    stream_plan = recover_stream_plan(evaluations, goal_expression, domain,
                                      opt_stream_plan, pddl_plan, negative, unit_costs=False)
    stream_plan = get_synthetic_stream_plan(reorder_stream_plan(stream_plan), dynamic_streams)


    # TODO: need to make this just streams
    opt_evaluations = apply_streams(evaluations, stream_plan)
    opt_cost = get_plan_cost(opt_evaluations, opt_action_plan, domain, unit_costs=False)
    dump_plans(stream_plan, opt_action_plan, opt_cost)
    if visualize:
        log_plans(stream_plan, action_plan, None)
        create_visualizations(evaluations, stream_plan, None)

    store.start_time = time.time()
    store.max_cost = store.best_cost
    queue = SkeletonQueue(store, evaluations, domain)
    queue.new_skeleton(stream_plan, opt_action_plan, opt_cost)
    queue.greedily_process()
    queue.timed_process(sampling_time)
    # TODO: compare solution cost and validity?
    # TODO: select which binding
    # TODO: repeatedly do this
    # TODO: can do this during the search & with all queued plan skeletons
