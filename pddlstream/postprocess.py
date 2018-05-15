from collections import deque, defaultdict
from itertools import product

from pddlstream.conversion import evaluation_from_fact, pddl_from_object, substitute_expression
from pddlstream.fast_downward import task_from_domain_problem, get_problem, fact_from_fd
from pddlstream.function import PredicateResult
from pddlstream.instantiation import Instantiator
from pddlstream.macro_stream import MacroResult, get_macro_stream_plan
from pddlstream.object import Object
from pddlstream.reorder import get_action_instances, replace_derived, topological_sort, reorder_stream_plan
from pddlstream.scheduling.relaxed import get_goal_instance, plan_preimage, recover_stream_plan
from pddlstream.scheduling.simultaneous import evaluations_from_stream_plan
from pddlstream.stream import StreamResult
from pddlstream.utils import INF, implies

def optimistic_process_stream_queue(instantiator):
    stream_instance = instantiator.stream_queue.popleft()
    stream_results = stream_instance.next_optimistic()
    for stream_result in stream_results:
        for fact in stream_result.get_certified():
            instantiator.add_atom(evaluation_from_fact(fact))
    return stream_results


def populate_results(evaluations, streams, initial_instances=[]):
    instantiator = Instantiator(evaluations, streams)
    for instance in initial_instances:
        instantiator.stream_queue.append(instance)
    stream_results = []
    while instantiator.stream_queue:
        stream_results += optimistic_process_stream_queue(instantiator)
    return stream_results

##################################################

def ground_stream_instances(stream_instance, bindings, evaluations, opt_evaluations):
    # TODO: combination for domain predicates
    evaluation_set = set(evaluations)
    combined_evaluations = evaluation_set | opt_evaluations
    real_instances = []
    opt_instances = []
    #input_objects = [[i] if isinstance(i, Object) else bindings[i]
    #                for i in stream_instance.input_objects]
    input_objects = [bindings.get(i, [i]) for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = dict(zip(stream_instance.input_objects, combo))
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping)))
        if domain <= combined_evaluations:
            instance = stream_instance.external.get_instance(combo)
            if domain <= evaluation_set:
                real_instances.append(instance)
            else:
                opt_instances.append(instance)
    return real_instances, opt_instances

def disable_stream_instance(stream_instance, disabled):
    disabled.append(stream_instance)
    stream_instance.disabled = True

##################################################

def process_stream_plan(evaluations, stream_plan, disabled, verbose,
                        quick_fail=True, layers=False, max_values=INF):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    streams_from_output = defaultdict(list)
    for result in stream_plan:
        if isinstance(result, StreamResult):
            for obj in result.output_objects:
                streams_from_output[obj].append(result)
    shared_output_streams = {s for streams in streams_from_output.values() if 1 < len(streams) for s in streams}
    #shared_output_streams = {}

    opt_bindings = defaultdict(list)
    opt_evaluations = set()
    opt_results = []
    failed = False
    stream_queue = deque(stream_plan)
    while stream_queue and implies(quick_fail, not failed):
        opt_result = stream_queue.popleft()
        real_instances, opt_instances = ground_stream_instances(opt_result.instance, opt_bindings,
                                                                evaluations, opt_evaluations)
        first_step = all(isinstance(o, Object) for o in opt_result.instance.input_objects)
        num_instances = min(len(real_instances), max_values) \
            if (layers or first_step or (opt_result not in shared_output_streams)) else 0
        opt_instances += real_instances[num_instances:]
        real_instances = real_instances[:num_instances]
        new_results = []
        local_failure = False
        for instance in real_instances:
            results = instance.next_results(verbose=verbose)
            for result in results:
                for fact in result.get_certified():
                    evaluation = evaluation_from_fact(fact)
                    if evaluation not in evaluations:
                        evaluations[evaluation] = result
            disable_stream_instance(instance, disabled)
            local_failure |= not results
            if isinstance(opt_result, PredicateResult) and not any(opt_result.value == r.value for r in results):
                local_failure = True # TODO: check for instance?
            new_results += results
        for instance in opt_instances:
            results = instance.next_optimistic()
            opt_evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            opt_results += results
            local_failure |= not results
            new_results += results
        for result in new_results:
            if isinstance(result, StreamResult): # Could not add if same value
                for opt, obj in zip(opt_result.output_objects, result.output_objects):
                    opt_bindings[opt].append(obj)
        if local_failure and isinstance(opt_result, MacroResult):
            stream_queue.extendleft(reversed(opt_result.decompose()))
            failed = False # TODO: check if satisfies target certified
        else:
            failed |= local_failure

    if verbose:
        print('Success: {}'.format(not failed))
    if failed:
        return None, None
    return opt_results, opt_bindings

##################################################

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
