from pddlstream.language.constants import is_plan
from pddlstream.language.statistics import geometric_cost
from pddlstream.utils import INF

DEFAULT_SEARCH_OVERHEAD = 1e-2
EFFORT_OP = sum # max | sum
# Can also include the overhead to process skeletons


def get_unit_effort(effort):
    if (effort == 0) or (effort == INF):
        return effort
    return 1


def compute_external_effort(external, unit_efforts=False, search_overhead=DEFAULT_SEARCH_OVERHEAD):
    effort_fn = external.info.effort_fn
    if effort_fn is None:
        p_success = external.get_p_success()
        effort = geometric_cost(external.get_overhead(), p_success) + \
                 (1-p_success)*geometric_cost(search_overhead, p_success)
    elif callable(effort_fn):
        effort = 0 # This really is a bound on the effort
    else:
        effort = float(effort_fn)
    return get_unit_effort(effort) if unit_efforts else effort


def compute_instance_effort(instance, unit_efforts=False, search_overhead=DEFAULT_SEARCH_OVERHEAD):
    # TODO: handle case where resampled several times before the next search (search every ith time)
    effort = instance.opt_index * search_overhead # By linearity of expectation
    effort_fn = instance.external.info.effort_fn
    if effort_fn is None:
        effort += compute_external_effort(
            instance.external, unit_efforts=False, search_overhead=search_overhead)
    elif callable(effort_fn):
        effort += effort_fn(*instance.get_input_values())
    else:
        effort += float(effort_fn)
    return get_unit_effort(effort) if unit_efforts else effort


def compute_result_effort(result, **kwargs):
    if not result.optimistic:
        return 0 # Unit efforts?
    # TODO: this should be the min of all instances
    return compute_instance_effort(result.instance, **kwargs)


def compute_plan_effort(stream_plan, **kwargs):
    if not is_plan(stream_plan):
        return INF
    if not stream_plan:
        return 0
    return sum(compute_result_effort(result, **kwargs) for result in stream_plan)
