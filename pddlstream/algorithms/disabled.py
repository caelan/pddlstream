from pddlstream.algorithms.algorithm import add_certified, add_facts
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.utils import INF


def reenable_disabled(evaluations, domain, disabled):
    for instance in disabled:
        instance.enable(evaluations, domain)
    disabled.clear()

def process_instance(evaluations, instance, verbose=True):
    success = False
    if instance.enumerated:
        return success
    new_results, new_facts = instance.next_results(verbose=verbose)
    for result in new_results:
        success |= bool(add_certified(evaluations, result))
    # TODO: delete blocked fact if successful (for fluent problems)
    bool(add_facts(evaluations, new_facts, result=None))
    return success

def process_all_disabled(store, evaluations, disabled):
    # TODO: use max_sample_time to process the queue for some amount of time
    success = False
    while not success and disabled:
        for instance in list(disabled):
            # if success: # TODO: to greedily break, need to keep the order around
            #    break
            if instance.enumerated:
                disabled.remove(instance)
            else:
                success |= process_instance(evaluations, instance, verbose=store.verbose)
    return success

def process_stream_plan(evaluations, domain, stream_plan, disabled, max_failures=INF, **kwargs):
    # TODO: had old implementation of these
    # TODO: could do version that allows bindings and is able to return
    # effort_weight is None # Keep in a queue
    failures = 0
    for result in stream_plan:
        if max_failures < failures:
            break
        instance = result.instance
        assert not instance.enumerated
        if all(evaluation_from_fact(f) in evaluations for f in instance.get_domain()):
            failures += not process_instance(evaluations, instance, **kwargs)
            instance.disable(evaluations, domain)
            if not instance.enumerated:
                disabled.add(instance)
    # TODO: indicate whether should resolve w/o disabled
    return not failures

def process_disabled(store, evaluations, domain, disabled, stream_plan, action_plan, cost, reenable):
    if stream_plan is None:
        if not disabled:
            return False
        if reenable:
            reenable_disabled(evaluations, domain, disabled)
        else:
            process_all_disabled(store, evaluations, disabled)
    elif not stream_plan:
        store.add_plan(action_plan, cost)
    else:
        # TODO: option to run just the first option
        process_stream_plan(evaluations, domain, stream_plan, disabled, verbose=store.verbose)
    # TODO: report back whether to try w/o stream values
    return True
