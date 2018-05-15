from collections import deque

from pddlstream.conversion import evaluation_from_fact, pddl_from_object
from pddlstream.fast_downward import task_from_domain_problem, get_problem, fact_from_fd
from pddlstream.macro_stream import MacroResult, get_macro_stream_plan
from pddlstream.stream_plan import populate_results, process_stream_plan
from pddlstream.reorder import get_action_instances, replace_derived, topological_sort, reorder_stream_plan
from pddlstream.scheduling.relaxed import get_goal_instance, plan_preimage, recover_stream_plan
from pddlstream.scheduling.simultaneous import evaluations_from_stream_plan


def locally_optimize(evaluations, action_plan, goal_expression, domain, functions, negative, dynamic_streams, verbose):
    print('\nPostprocessing') # TODO: postprocess current skeleton as well

    task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs=False))
    plan_instances = get_action_instances(task, action_plan) + [get_goal_instance(task.goal)]
    replace_derived(task, set(), plan_instances)
    preimage = filter(lambda a: not a.negated, plan_preimage(plan_instances, []))

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
        if isinstance(result, MacroResult):
            ordered_results.extend(result.decompose())
        else:
            ordered_results.append(result)

    opt_stream_plan = []
    opt_from_obj = {}
    for stream_result in ordered_results:
        input_objects = tuple(opt_from_obj.get(o, o) for o in stream_result.instance.input_objects)
        instance = stream_result.instance.external.get_instance(input_objects)
        assert(not instance.disabled)
        opt_results = instance.next_optimistic()
        if not opt_results:
            continue
        opt_result = opt_results[0]
        opt_stream_plan.append(opt_result)
        for obj, opt in zip(stream_result.output_objects, opt_result.output_objects):
            opt_from_obj[obj] = opt
    opt_stream_plan += populate_results(evaluations_from_stream_plan(evaluations, opt_stream_plan), functions)
    opt_action_plan = [(name, tuple(opt_from_obj.get(o, o) for o in args)) for name, args in action_plan]

    opt_evaluations = evaluations_from_stream_plan(evaluations, opt_stream_plan)
    problem = get_problem(opt_evaluations, goal_expression, domain, unit_costs=False)
    pddl_plan = [(name, map(pddl_from_object, args)) for name, args in opt_action_plan]
    stream_plan = recover_stream_plan(evaluations, opt_stream_plan, domain, problem, pddl_plan, negative, unit_costs=False)

    stream_plan = reorder_stream_plan(stream_plan)  # TODO: is this strictly redundant?
    stream_plan = get_macro_stream_plan(stream_plan, dynamic_streams)
    print('Stream plan: {}\n'
          'Action plan: {}'.format(stream_plan, opt_action_plan))

    disabled = []
    _, obj_from_opt = process_stream_plan(evaluations, stream_plan, disabled, verbose)
    if obj_from_opt is None:
        return action_plan
    # TODO: compare solution cost and validity?
    # TODO: select which binding
    # TODO: repeatedly do this
    new_action_plan = [(name, tuple(obj_from_opt.get(o, [o])[0] for o in args)) for name, args in opt_action_plan]
    return new_action_plan