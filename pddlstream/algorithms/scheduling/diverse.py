from __future__ import print_function

from itertools import combinations, permutations

import numpy as np
import time
import random

from pddlstream.utils import INF, elapsed_time, find_unique, randomize
from pddlstream.language.constants import str_from_plan, StreamAction


def p_conjunction(stream_plans):
    return np.product([result.external.get_p_success(*result.get_input_values())
                       for result in set.union(*stream_plans)])

def prune_dominated(externals, combined_plans):
    # TODO: could hash instead
    dominated = set()
    indices = list(range(len(combined_plans)))
    for idx1, idx2 in permutations(indices, r=2):
        if (idx1 in dominated) or (idx2 in dominated):
            continue
        _, _, cost1 = combined_plans[idx1]
        stream_plans1 = extract_stream_plan(externals, combined_plans[idx1])
        _, _, cost2 = combined_plans[idx2]
        stream_plans2 = extract_stream_plan(externals, combined_plans[idx2])
        if (stream_plans1 < stream_plans2) or (stream_plans1 == stream_plans2 and cost1 <= cost2):
            dominated.add(idx2)
        #print(len(stream_plans1), len(stream_plans2), len(stream_plans1 & stream_plans2), cost1, cost2)
    print('{}/{} pruned'.format(len(dominated), len(indices)))
    return [combined_plans[idx] for idx in indices if idx not in dominated]

##################################################

# https://hub.docker.com/r/ctpelok77/ibmresearchaiplanningsolver
# https://zenodo.org/record/3404122#.XtZg8J5KjAI
# Diversity metrics: stability, state, uniqueness
# Linear combinations of these

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

# Linear combinations of these
# Minimize similarity
# dDISTANTkSET: requires the distance between every pair of plans in the solution to be of bounded diversity
# average over the pairwise dissimilarity of the set
def stability(stream_plans, diverse, r=2, op=np.average): # min | np.average
    # the ratio of the number of actions that appear on both plans
    # to the total number of actions on these plans, ignoring repetitions
    assert stream_plans
    if len(stream_plans) == 1:
        return 1.
    values = []
    for combo in combinations(stream_plans, r=r):
        # TODO: weighted intersection
        sim = float(len(set.intersection(*combo))) / len(set.union(*combo))
        values.append(1. - sim)
    return op(values)

def state(stream_plans, diverse):
    # measures similarity between two plans based on representing the plans as as sequence of states
    # maybe adapt using the certified conditions?
    raise NotImplementedError()

def uniqueness(stream_plans, diverse, op=np.average):
    # measures whether two plans are permutations of each other, or one plan is a subset of the other plan
    assert stream_plans
    if len(stream_plans) == 1:
        return 1.
    values = []
    for stream_plan1, stream_plan2 in combinations(stream_plans, r=2):
        sim = (stream_plan1 <= stream_plan2) or (stream_plan2 <= stream_plan1)
        values.append(1 - sim)
    return op(values)

def score(stream_plans, diverse, **kwargs):
    metric = diverse.get('metric', 'p_success')
    if metric == 'stability':
        return stability(stream_plans, diverse, **kwargs)
    if metric == 'state':
        return state(stream_plans, diverse, **kwargs)
    if metric == 'uniqueness':
        return uniqueness(stream_plans, diverse, **kwargs)
    if metric == 'p_success':
        return p_disjunction(stream_plans, diverse, **kwargs)
    raise ValueError(metric)

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

def first_k(externals, combined_plans, diverse, **kwargs):
    k = diverse['k']
    return combined_plans[:k]

##################################################

def extract_stream_plan(externals, combined_plan):
    stream_plan, opt_plan, _ = combined_plan
    if stream_plan:
        return opt_plan.stream_plan
    stream_actions = {action for action in opt_plan.action_plan if isinstance(action, StreamAction)}
    return {find_unique(lambda e: e.name == name, externals).get_instance(inputs)
            for name, inputs, _ in stream_actions}

def greedy_diverse_subset(externals, combined_plans, diverse, max_time=INF, verbose=False):
    # TODO: lazy greedy submodular maximimization
    start_time = time.time()
    k = diverse['k']
    assert 1 <= k
    if len(combined_plans) <= k:
        return combined_plans
    best_indices = set()
    # TODO: warm start using exact_diverse_subset for small k
    for i in range(len(best_indices), k):
        best_index, best_p = None, -INF
        for index in set(range(len(combined_plans))) - best_indices:
            stream_plans = [extract_stream_plan(externals, combined_plans[i]) for i in best_indices | {index}]
            # TODO: could prune dominated candidates given the selected set
            p = score(stream_plans, diverse)
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
    num_considered = 0
    best_plans, best_p = None, -INF
    for i, subset_plans in enumerate(combinations(randomize(combined_plans), r=k)):
        num_considered += 1
        # Weight by effort
        #stream_plans = [set(stream_plan) for stream_plan, _, _, in subset_plans]
        stream_plans = [extract_stream_plan(externals, combined_plan) for combined_plan in subset_plans]
        p = score(stream_plans, diverse)
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

##################################################

def diverse_subset(externals, combined_plans, diverse, **kwargs):
    # TODO: report back other statistics (possibly in kwargs)
    if not diverse:
        return combined_plans[:1]
    combined_plans = prune_dominated(externals, combined_plans)
    selector = diverse['selector']
    if selector == 'random':
        return random_subset(externals, combined_plans, diverse, **kwargs)
    if selector == 'first':
        return first_k(externals, combined_plans, diverse, **kwargs)
    if selector == 'greedy':
        return greedy_diverse_subset(externals, combined_plans, diverse, **kwargs)
    if selector == 'exact':
        return exact_diverse_subset(externals, combined_plans, diverse, **kwargs)
    raise ValueError(selector)
