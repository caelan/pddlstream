from pddlstream.algorithms.common import add_facts, add_certified, is_instance_ready
from pddlstream.algorithms.algorithm import remove_blocked
from pddlstream.language.constants import INFEASIBLE, is_plan
from pddlstream.utils import INF

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

def process_instance(store, domain, instance):
    success = False
    if instance.enumerated:
        return success
    new_results, new_facts = instance.next_results(accelerate=1, verbose=store.verbose)
    instance.disable(store.evaluations, domain)
    for result in new_results:
        success |= bool(add_certified(store.evaluations, result))
    remove_blocked(store.evaluations, instance, new_results)
    success |= bool(add_facts(store.evaluations, new_facts, result=None, complexity=0)) # TODO: record the instance
    return success

##################################################

def process_stream_plan(store, domain, disabled, stream_plan, max_failures=INF):
    # TODO: had old implementation of these
    # TODO: could do version that allows bindings and is able to return
    # The only advantage of this vs skeleton is that this can avoid the combinatorial growth in bindings
    if not is_plan(stream_plan):
        return
    failures = 0
    for result in stream_plan:
        if max_failures < failures:
            break
        instance = result.instance
        if instance.enumerated:
            raise RuntimeError(instance)
        if is_instance_ready(store.evaluations, instance):
            # TODO: could remove disabled and just use complexity_limit
            failures += not process_instance(store, domain, instance)
            if not instance.enumerated:
                disabled.add(instance)
    # TODO: report back whether to try w/o optimistic values
