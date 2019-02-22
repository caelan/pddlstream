from pddlstream.algorithms.common import add_facts, add_certified, is_instance_ready
from pddlstream.algorithms.algorithm import remove_blocked
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult
from pddlstream.language.constants import INFEASIBLE, is_plan
from pddlstream.utils import INF, safe_zip, get_mapping, apply_mapping

from collections import defaultdict, deque, namedtuple
from itertools import product

# TODO: disabled isn't quite like complexity. Stream instances below the complexity threshold might be called again
# Well actually, if this was true wouldn't it have already been sampled on a lower level?

def update_bindings(bindings, opt_result, result):
    new_bindings = bindings.copy()
    if isinstance(result, StreamResult):
        for opt, obj in safe_zip(opt_result.output_objects, result.output_objects):
            assert (opt not in new_bindings)  # TODO: return failure if conflicting bindings
            new_bindings[opt] = obj
    return new_bindings

def update_cost(cost, opt_result, result):
    # TODO: recompute optimistic costs to attempt to produce a tighter bound
    new_cost = cost
    if type(result) is FunctionResult:
        new_cost += (result.value - opt_result.value)
    return new_cost

def bind_action_plan(action_plan, mapping):
    return [(name, apply_mapping(args, mapping)) for name, args in action_plan]

##################################################

def push_disabled(instantiator, disabled):
    for instance in list(disabled):
        if instance.enumerated:
            disabled.remove(instance)
        else:
            # TODO: only add if not already queued
            instantiator.push_instance(instance)

def reenable_disabled(evaluations, domain, disabled):
    for instance in disabled:
        instance.enable(evaluations, domain)
    disabled.clear()

def process_instance(store, domain, instance, disable=True):
    if instance.enumerated:
        return []
    new_results, new_facts = instance.next_results(verbose=store.verbose)
    if disable:
        instance.disable(store.evaluations, domain)
    for result in new_results:
        add_certified(store.evaluations, result) # TODO: only add if the fact is actually new?
    if disable:
        remove_blocked(store.evaluations, instance, new_results)
    add_facts(store.evaluations, new_facts, result=None, complexity=0) # TODO: record the instance
    return new_results

##################################################

def get_free_objects(stream_plan):
    return {out for result in stream_plan if isinstance(result, StreamResult) for out in result.output_objects}

def process_stream_plan_branch(store, domain, disabled, stream_plan, action_plan, cost):
    if not is_plan(stream_plan):
        return
    stream_plan = [result for result in stream_plan if result.optimistic]
    if not stream_plan:
        store.add_plan(action_plan, cost)
        return
    free_objects = get_free_objects(stream_plan)
    bindings = defaultdict(set)
    for opt_result in stream_plan:
        opt_inputs = [inp for inp in opt_result.instance.input_objects if inp in free_objects]
        inp_bindings = [bindings[inp] for inp in opt_inputs]
        for combo in product(*inp_bindings):
            bound_result = opt_result.remap_inputs(get_mapping(opt_inputs, combo))
            bound_instance = bound_result.instance
            if bound_instance.enumerated or not is_instance_ready(store.evaluations, bound_instance):
                continue # Disabled
            new_results = process_instance(store, domain, bound_instance)
            if not bound_instance.enumerated:
                disabled.add(bound_instance)
            if isinstance(opt_result, StreamResult):
                for new_result in new_results:
                    for out, obj in safe_zip(opt_result.output_objects, new_result.output_objects):
                        bindings[out].add(obj)
    #Binding = namedtuple('Binding', ['index', 'mapping'])
    # TODO: after querying, search over all bindings of the produced sampled

##################################################

def process_stream_plan(store, domain, disabled, stream_plan, action_plan, cost, bind=True, max_failures=0):
    # Bad old implementation of this method
    # The only advantage of this vs skeleton is that this can avoid the combinatorial growth in bindings
    if not is_plan(stream_plan):
        return
    stream_plan = [result for result in stream_plan if result.optimistic]
    free_objects = get_free_objects(stream_plan)
    bindings = {}
    bound_plan = []
    for i, opt_result in enumerate(stream_plan):
        if (store.best_cost <= cost) or (max_failures < (i - len(bound_plan))):
            break
        opt_inputs = [inp for inp in opt_result.instance.input_objects if inp in free_objects]
        if (not bind and opt_inputs) or not all(inp in bindings for inp in opt_inputs):
            continue
        bound_result = opt_result.remap_inputs(bindings)
        bound_instance = bound_result.instance
        if bound_instance.enumerated or not is_instance_ready(store.evaluations, bound_instance):
            continue
        # TODO: could remove disabled and just use complexity_limit
        new_results = process_instance(store, domain, bound_instance)
        if not bound_instance.enumerated:
            disabled.add(bound_instance)
        if not new_results:
            continue
        bound_plan.append(new_results[0])
        bindings = update_bindings(bindings, bound_result, bound_plan[-1])
        cost = update_cost(cost, opt_result, bound_plan[-1])
    if len(stream_plan) == len(bound_plan):
        store.add_plan(bind_action_plan(action_plan, bindings), cost)
    # TODO: report back whether to try w/o optimistic values in the event that wild