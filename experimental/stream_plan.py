from collections import defaultdict, deque
from itertools import product

from pddlstream.function import PredicateResult
from pddlstream.skeleton import get_stream_plan_index

from pddlstream.algorithms.algorithm import add_certified
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.object import Object
from pddlstream.language.stream import StreamResult
from pddlstream.language.synthesizer import SynthStreamResult
from pddlstream.utils import INF, implies


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


def reset_disabled(disabled):
    for stream_instance in disabled:
        stream_instance.disabled = False
    disabled[:] = []


def disable_stream_instance(stream_instance, disabled):
    disabled.append(stream_instance)
    stream_instance.disabled = True


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
        if local_failure and isinstance(opt_result, SynthStreamResult):
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