from pddlstream.algorithms.skeleton import process_instance, process_stream_plan

def reenable_disabled(evaluations, domain, disabled):
    for instance in disabled:
        instance.enable(evaluations, domain)
    disabled.clear()

def process_all_disabled(store, evaluations, disabled, max_sample_time):
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

def process_disabled(store, evaluations, domain, disabled, stream_plan, action_plan, cost,
                     max_sample_time, reenable):
    if stream_plan is None:
        if not disabled:
            return False
        if reenable:
            reenable_disabled(evaluations, domain, disabled)
        else:
            process_all_disabled(store, evaluations, disabled, max_sample_time)
    elif not stream_plan:
        store.add_plan(action_plan, cost)
    else:
        # TODO: option to run just the first option
        process_stream_plan(evaluations, domain, stream_plan, disabled, verbose=store.verbose)
    # TODO: report back whether to try w/o stream values
    return True
