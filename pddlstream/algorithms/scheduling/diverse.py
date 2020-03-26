from itertools import combinations

import numpy as np

from pddlstream.utils import INF


def p_conjunction(stream_plans):
    return np.product([result.external.get_p_success()
                       for result in set.union(*stream_plans)])


def p_disjunction(stream_plans):
    # Inclusion exclusion
    # TODO: dynamic programming method for this?
    # TODO: incorporate overhead
    p = 0.
    for i in range(len(stream_plans)):
        r = i + 1
        for subset_plans in combinations(stream_plans, r=r):
            sign = -1 if r % 2 == 0 else +1
            p += sign*p_conjunction(subset_plans)
    return p


def select_diverse_subset(combined_plans, r=2):
    best_plans, best_p = None, -INF
    for i, subset_plans in enumerate(combinations(combined_plans, r=r)):
        stream_plans = [set(stream_plan) for stream_plan, _, _, in subset_plans]
        intersection = set.intersection(*stream_plans)
        p = p_disjunction(stream_plans)
        assert 0 <= p <= 1
        print('\nGroup: {} | Intersection: {} | p={:.3f}'.format(i, len(intersection), p))  # , intersection)
        for stream_plan, opt_plan, cost in subset_plans:
            print(len(stream_plan), stream_plan)
            print(len(opt_plan.action_plan), cost, opt_plan.action_plan)
        if p > best_p:
            best_plans, best_p = subset_plans, p

    print('\nBest (p={:.3f}):'.format(best_p))
    for stream_plan, opt_plan, cost in best_plans:
        print(len(stream_plan), stream_plan)
        print(len(opt_plan.action_plan), cost, opt_plan.action_plan)
    return best_plans