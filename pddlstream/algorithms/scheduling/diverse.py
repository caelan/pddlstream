from __future__ import print_function

from itertools import combinations

import numpy as np
import time
import random

from pddlstream.utils import INF, elapsed_time, find_unique, randomize
from pddlstream.language.constants import str_from_plan, StreamAction

def p_conjunction(stream_plans):
    return np.product([result.external.get_p_success(*result.get_input_values())
                       for result in set.union(*stream_plans)])


def p_disjunction(stream_plans, diverse):
    # Inclusion exclusion
    # TODO: incorporate overhead
    # TODO: approximately compute by sampling outcomes
    # TODO: separate into connected components
    # TODO: compute for low k and increment, prunning if upper bound is less than best lower bound
    d = diverse.get('d', INF)
    d = min(len(stream_plans), d)
    assert (d % 2 == 1) or (d == len(stream_plans))
    p = 0.
    for i in range(d):
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

# TODO: toggle behavior based on k

def random_subset(externals, combined_plans, diverse, **kwargs):
    #if k is None:
    #    k = DEFAULT_K
    k = diverse['k']
    if len(combined_plans) <= k:
        return combined_plans
    return random.sample(combined_plans, k=k)

def extract_stream_plan(externals, combined_plan):
    stream_plan, opt_plan, _ = combined_plan
    if stream_plan:
        return opt_plan.stream_plan
    stream_actions = {action for action in opt_plan.action_plan if isinstance(action, StreamAction)}
    return {find_unique(lambda e: e.name == name, externals).get_instance(inputs)
            for name, inputs, _ in stream_actions}

def greedy_diverse_subset(externals, combined_plans, diverse, max_time=INF, verbose=False):
    # TODO: warm start using exact_diverse_subset for small k
    # TODO: lazy greedy submodular maximimization
    start_time = time.time()
    k = diverse['k']
    assert 1 <= k
    if len(combined_plans) <= k:
        return combined_plans
    best_indices = set()
    for i in range(k):
        best_index, best_p = None, -INF
        for index in set(range(len(combined_plans))) - best_indices:
            stream_plans = [extract_stream_plan(externals, combined_plans[i]) for i in best_indices | {index}]
            p = p_disjunction(stream_plans, diverse)
            if p > best_p:
                best_index, best_p = index, p
        best_indices.add(best_index)
        if verbose:
            print('i) p={:.3f} | Time: {:.2f}'.format(i, best_p, elapsed_time(start_time)))
        #if max_time < elapsed_time(start_time): # Randomly select the rest
        #    break

    best_plans = [combined_plans[i] for i in best_indices]
    print('\nCandidates: {} | k={} | Time: {:.2f}'.format(
        len(combined_plans), k, elapsed_time(start_time)))
    for i, (stream_plan, opt_plan, cost) in enumerate(best_plans):
        print(i, len(opt_plan.action_plan), cost, str_from_plan(opt_plan.action_plan))
        print(i, len(stream_plan), stream_plan)
    print('\n'+'-'*50+'\n')
    return best_plans

def exact_diverse_subset(externals, combined_plans, diverse, verbose=False):
    # TODO: dynamic programming method across multisets
    # TODO: ILP over selections
    # TODO: pass results instead of externals
    start_time = time.time()
    k = diverse['k']
    assert 1 <= k
    if len(combined_plans) <= k:
        return combined_plans
    max_time = diverse.get('max_time', INF)
    # TODO: version that randomly samples until timeout
    num_considered = 0
    best_plans, best_p = None, -INF
    for i, subset_plans in enumerate(combinations(randomize(combined_plans), r=k)):
        num_considered += 1
        # Weight by effort
        #stream_plans = [set(stream_plan) for stream_plan, _, _, in subset_plans]
        stream_plans = [extract_stream_plan(externals, combined_plan) for combined_plan in subset_plans]
        p = p_disjunction(stream_plans, diverse)
        #assert 0 <= p <= 1 # May not hold if not exact
        if verbose:
            intersection = set.intersection(*stream_plans)
            print('\nTime: {:.2f} | Group: {} | Intersection: {} | p={:.3f}'.format(
                elapsed_time(start_time), i, len(intersection), p))  # , intersection)
            for stream_plan, opt_plan, cost in subset_plans:
                print(len(stream_plan), stream_plan)
                print(len(opt_plan.action_plan), cost, str_from_plan(opt_plan.action_plan))
        if p > best_p:
            best_plans, best_p = subset_plans, p
        if max_time < elapsed_time(start_time):
            break

    # TODO: sort plans (or use my reordering technique)
    print('\nCandidates: {} | k={} | Considered: {} | Best (p={:.3f}) | Time: {:.2f}'.format(
        len(combined_plans), k, num_considered, best_p, elapsed_time(start_time)))
    for i, (stream_plan, opt_plan, cost) in enumerate(best_plans):
        print(i, len(opt_plan.action_plan), cost, str_from_plan(opt_plan.action_plan))
        print(i, len(stream_plan), stream_plan)
    print('\n'+'-'*50+'\n')
    return best_plans

def diverse_subset(externals, combined_plans, diverse, **kwargs):
    selector = diverse['selector']
    if selector == 'random':
        return random_subset(externals, combined_plans, diverse, **kwargs)
    if selector == 'greedy':
        return greedy_diverse_subset(externals, combined_plans, diverse, **kwargs)
    if selector == 'exact':
        return exact_diverse_subset(externals, combined_plans, diverse, **kwargs)
    raise ValueError(selector)