import time
from collections import deque, defaultdict, namedtuple
from heapq import heappush, heappop
from itertools import product

from pddlstream.algorithm import add_certified
from pddlstream.conversion import evaluation_from_fact, substitute_expression
from pddlstream.function import FunctionResult, PredicateResult
from pddlstream.instantiation import Instantiator
from pddlstream.macro_stream import MacroResult
from pddlstream.object import Object
from pddlstream.stream import StreamResult
from pddlstream.utils import INF, implies, elapsed_time

def is_double_bound(stream_instance, double_bindings):
    if double_bindings is None:
        return True
    bindings = [double_bindings[o] for o in stream_instance.input_objects if o in double_bindings]
    return len(set(bindings)) != len(bindings)

def populate_results(evaluations, streams, double_bindings=None):
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

def ground_stream_instances(stream_instance, bindings, evaluations, opt_evaluations, plan_index):
    # TODO: combination for domain predicates
    evaluation_set = set(evaluations)
    combined_evaluations = evaluation_set | opt_evaluations
    real_instances = []
    opt_instances = []
    input_objects = [bindings.get(i, [i]) for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = dict(zip(stream_instance.input_objects, combo))
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping)))
        if domain <= combined_evaluations:
            instance = stream_instance.external.get_instance(combo)
            immediate = False
            if immediate:
                if domain <= evaluation_set:
                    if instance.opt_index == 0:
                        real_instances.append(instance)
                    else:
                        instance.opt_index -= 1
                        opt_instances.append(instance)
                else:
                    opt_instances.append(instance)
            else:
                #if (instance.opt_index == 0) and (domain <= evaluation_set):
                if (plan_index == 0) and (domain <= evaluation_set):
                    real_instances.append(instance)
                else:
                   if instance.opt_index != 0:
                       instance.opt_index -= 1
                   opt_instances.append(instance)
    return real_instances, opt_instances

##################################################

def reset_disabled(disabled):
    for stream_instance in disabled:
        stream_instance.disabled = False
    disabled[:] = []

def disable_stream_instance(stream_instance, disabled):
    disabled.append(stream_instance)
    stream_instance.disabled = True

##################################################

def get_stream_plan_index(stream_plan):
    if not stream_plan:
        return 0
    return max(r.opt_index for r in stream_plan)

def process_stream_plan(evaluations, stream_plan, disabled, verbose,
                        quick_fail=True, layers=False, max_values=INF):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    plan_index = get_stream_plan_index(stream_plan)
    streams_from_output = defaultdict(list)
    for result in stream_plan:
        if isinstance(result, StreamResult):
            for obj in result.output_objects:
                streams_from_output[obj].append(result)
    shared_output_streams = {s for streams in streams_from_output.values() if 1 < len(streams) for s in streams}
    #shared_output_streams = {}
    print(shared_output_streams)
    print(plan_index)

    opt_bindings = defaultdict(list)
    opt_evaluations = set()
    opt_results = []
    failed = False
    stream_queue = deque(stream_plan)
    while stream_queue and implies(quick_fail, not failed):
        opt_result = stream_queue.popleft()
        real_instances, opt_instances = ground_stream_instances(opt_result.instance, opt_bindings,
                                                                evaluations, opt_evaluations, plan_index)
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
                add_certified(evaluations, result)
            disable_stream_instance(instance, disabled)
            local_failure |= not results
            if isinstance(opt_result, PredicateResult) and not any(opt_result.value == r.value for r in results):
                local_failure = True # TODO: check for instance?
            new_results += results
        for instance in opt_instances:
            #print(instance, instance.opt_index)
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
    # TODO: just return binding
    # TODO: could also immediately switch to binding if plan_index == 0 afterwards
    return opt_results, opt_bindings


##################################################

# TODO: can either entirely replace arguments on plan or just pass bindings
# TODO: handle this in a partially ordered way
# TODO: no point not doing all at once if unique
# TODO: store location in history in the sampling history
# TODO: alternatively store just preimage and reachieve

SkeletonKey = namedtuple('SkeletonKey', ['attempts', 'length'])
Skeleton = namedtuple('Skeleton', ['instance', 'num_processed', 'bindings',
                                   'stream_plan', 'action_plan', 'cost'])

def instantiate_first(bindings, stream_plan):
    if not stream_plan:
        return None
    opt_result = stream_plan[0] # TODO: could do several at once but no real point
    input_objects = [bindings.get(i, i) for i in opt_result.instance.input_objects]
    instance = opt_result.instance.external.get_instance(input_objects)
    instance.disabled = True
    return instance

def bind_plan(bindings, plan):
    return [(name, tuple(bindings.get(o, o) for o in args)) for name, args in plan]

def pop_stream_queue(key, sampling_problem, queue, evaluations, store):
    instance, num_processed, bindings, stream_plan, action_plan, cost = sampling_problem
    if not stream_plan:
        store.add_plan(bind_plan(bindings, action_plan), cost)
        return
    if store.best_cost <= cost:
        instance.disabled = False # TODO: only disable if not used elsewhere
        # TODO: could just hash instances
        return
    opt_result = stream_plan[0] # TODO: could do several at once but no real point
    assert(not any(evaluation_from_fact(f) not in evaluations for f in instance.get_domain()))
    # TODO: hash combinations to prevent repeats

    results = []
    for i in range(num_processed, len(instance.results_history)):
        results.extend(instance.results_history[i])
    if not results and not instance.enumerated:
        #print(key.attempts, key.length)
        results = instance.next_results(verbose=store.verbose)
    for result in results:
        add_certified(evaluations, result)
        if (type(result) is PredicateResult) and (opt_result.value != result.value):
            continue # TODO: check if satisfies target certified
        new_bindings = bindings.copy()
        if isinstance(result, StreamResult):
            for opt, obj in zip(opt_result.output_objects, result.output_objects):
                assert(opt not in new_bindings) # TODO: return failure if conflicting bindings
                new_bindings[opt] = obj
        new_stream_plan = stream_plan[1:]
        new_cost = cost
        if type(result) is FunctionResult:
            new_cost += (result.value - opt_result.value)
        new_key = SkeletonKey(0, len(new_stream_plan))
        new_skeleton = Skeleton(instantiate_first(new_bindings, new_stream_plan), 0,
                                new_bindings, new_stream_plan, action_plan, new_cost)
        heappush(queue, (new_key, new_skeleton))

    if (key.attempts == 0) and isinstance(opt_result, MacroResult):
        new_stream_plan = opt_result.decompose() + stream_plan[1:]
        new_key = SkeletonKey(0, len(new_stream_plan))
        new_skeleton = Skeleton(instantiate_first(bindings, new_stream_plan), 0,
                                bindings, new_stream_plan, action_plan, cost)
        heappush(queue, (new_key, new_skeleton))
    if not instance.enumerated:
        new_key = SkeletonKey(key.attempts + 1, len(stream_plan)) # TODO: compute expected sampling effort required
        new_skeleton = Skeleton(instance, len(instance.results_history),
                               bindings, stream_plan, action_plan, cost)
        heappush(queue, (new_key, new_skeleton))

##################################################

def greedily_process_queue(queue, evaluations, store, max_time):
    # TODO: search until new disabled or new evaluation?
    start_time = time.time()
    while queue and not store.is_terminated():
        key, sampling_problem = queue[0]
        if (key.attempts != 0) and (max_time <= elapsed_time(start_time)):
            break
        heappop(queue)
        pop_stream_queue(key, sampling_problem, queue, evaluations, store)

def fairly_process_queue(queue, evaluations, store):
    # TODO: max queue attempts?
    old_queue = list(queue)
    queue[:] = []
    for key, sampling_problem in old_queue:
        if store.is_terminated():
            break
        pop_stream_queue(key, sampling_problem, queue, evaluations, store)
        if store.is_terminated():
            break
        greedily_process_queue(queue, evaluations, store, 0)


##################################################

def ground_stream_instances_2(stream_instance, bindings, evaluations, opt_evaluations, immediate=False):
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

def process_stream_plan_2(evaluations, stream_plan):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    evaluations = set(evaluations)
    opt_evaluations = set(evaluations)
    opt_bindings = defaultdict(list)
    opt_results = []
    for opt_result in stream_plan:
        # TODO: could just do first step
        for instance in ground_stream_instances_2(opt_result.instance, opt_bindings, evaluations, opt_evaluations):
            results = instance.next_optimistic()
            opt_evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            opt_results += results
            for result in results:
                if isinstance(result, StreamResult): # Could not add if same value
                    for opt, obj in zip(opt_result.output_objects, result.output_objects):
                        opt_bindings[opt].append(obj)
    return opt_results, opt_bindings