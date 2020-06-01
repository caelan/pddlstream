from __future__ import print_function

from itertools import combinations

import numpy as np
import time
import random

from pddlstream.utils import INF, elapsed_time, find_unique
from pddlstream.language.constants import str_from_plan, StreamAction

def p_conjunction(stream_plans):
    return np.product([result.external.get_p_success(*result.get_input_values())
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

# Comparisons
# 1) Random
# 2) Diverse order
# 3) Greedy + Approx
# 4) Optimal + Exact
# 5) Increased K

def random_subset(externals, combined_plans, diverse):
    #if k is None:
    #    k = DEFAULT_K
    k = diverse['k']
    if len(combined_plans) <= k:
        return combined_plans
    return random.sample(combined_plans, k=k)

def exact_diverse_subset(externals, combined_plans, diverse, verbose=False):
    # TODO: dynamic programming method across multisets
    # TODO: ILP over selections
    # TODO: lazy greedy submodular maximimization
    start_time = time.time()
    k = diverse['k']
    if len(combined_plans) <= k:
        return combined_plans
    # TODO: pass results instead of externals
    best_plans, best_p = None, -INF
    for i, subset_plans in enumerate(combinations(combined_plans, r=k)):
        # Weight by effort
        #stream_plans = [set(stream_plan) for stream_plan, _, _, in subset_plans]
        stream_plans = []
        for _, opt_plan, _, in subset_plans:
            stream_actions = {action for action in opt_plan.action_plan if isinstance(action, StreamAction)}
            stream_plan = {find_unique(lambda e: e.name == name, externals).get_instance(inputs)
                           for name, inputs, _ in stream_actions}
            stream_plans.append(stream_plan)

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
    print('\nCandidates: {} | k={} | Best (p={:.3f}) | Time: {:.2f}'.format(
        len(combined_plans), k, best_p, elapsed_time(start_time)))
    for i, (stream_plan, opt_plan, cost) in enumerate(best_plans):
        print(i, len(opt_plan.action_plan), cost, str_from_plan(opt_plan.action_plan))
        print(i, len(stream_plan), stream_plan)

    print('\n'+'-'*50+'\n')
    return best_plans