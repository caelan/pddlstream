import time

from pddlstream.language.constants import is_plan, Certificate
from pddlstream.utils import elapsed_time, INF

def restart(problem, planner_fn, max_time=INF, max_restarts=0):
    # TODO: iteratively lower the cost bound
    # TODO: a sequence of different planner configurations
    # TODO: reset objects and/or streams

    assert max_restarts >= 0
    start_time = time.time()
    for attempt in range(1+max_restarts):
        if elapsed_time(start_time) > max_time:
            break
        solution = planner_fn(problem) # Or include the lambda in the planner
        plan, cost, certificate = solution
        if is_plan(plan):
            return solution

    certificate = Certificate(all_facts=[], preimage_facts=[]) # TODO: aggregate
    return None, INF, certificate
