from __future__ import print_function

from itertools import combinations

import numpy as np
import time
import random

from pddlstream.utils import INF, elapsed_time
from pddlstream.language.constants import str_from_plan

DEFAULT_K = 2

def p_conjunction(stream_plans):
    return np.product([result.external.get_p_success()
                       for result in set.union(*stream_plans)])


def p_disjunction(stream_plans, n=INF):
    # Inclusion exclusion
    # TODO: incorporate overhead
    n = min(len(stream_plans), n)
    p = 0.
    for i in range(n):
        r = i + 1
        for subset_plans in combinations(stream_plans, r=r):
            sign = -1 if r % 2 == 0 else +1
            p += sign*p_conjunction(subset_plans)
    return p

#def str_from_plan(action_plan):
#    return '[{}]'.format(', '.join('{}{}'.format(*action) for action in action_plan))

##################################################

def random_subset(combined_plans, k=None):
    if k is None:
        k = DEFAULT_K
    if len(combined_plans) <= k:
        return combined_plans
    return random.sample(combined_plans, k=k)

def exact_diverse_subset(combined_plans, k=None, verbose=False):
    # TODO: dynamic programming method across multisets
    # TODO: ILP over selections
    # TODO: submodular
    start_time = time.time()
    if k is None:
        k = DEFAULT_K
    if len(combined_plans) <= k:
        return combined_plans
    best_plans, best_p = None, -INF
    for i, subset_plans in enumerate(combinations(combined_plans, r=k)):
        # Weight by effort
        stream_plans = [set(stream_plan) for stream_plan, _, _, in subset_plans]
        intersection = set.intersection(*stream_plans)
        p = p_disjunction(stream_plans)
        assert 0 <= p <= 1
        if verbose:
            print('\nTime: {:.2f} | Group: {} | Intersection: {} | p={:.3f}'.format(
                elapsed_time(start_time), i, len(intersection), p))  # , intersection)
            for stream_plan, opt_plan, cost in subset_plans:
                print(len(stream_plan), stream_plan)
                print(len(opt_plan.action_plan), cost, str_from_plan(opt_plan.action_plan))
        if p > best_p:
            best_plans, best_p = subset_plans, p

    # TODO: sort plans (or use my reordering technique)
    print('\nTime: {:.2f} | Best (p={:.3f})'.format(
        elapsed_time(start_time), best_p))
    for i, (stream_plan, opt_plan, cost) in enumerate(best_plans):
        print(i, len(opt_plan.action_plan), cost, str_from_plan(opt_plan.action_plan))
        print(i, len(stream_plan), stream_plan)

    print('\n'+'-'*50+'\n')
    return best_plans