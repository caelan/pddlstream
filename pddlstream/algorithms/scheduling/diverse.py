from __future__ import print_function

from itertools import combinations, permutations
from collections import OrderedDict

import numpy as np
import time
import random

from pddlstream.utils import INF, elapsed_time, find_unique, randomize, get_connected_components
from pddlstream.language.constants import str_from_plan, StreamAction, print_plan, OptPlan

# TODO: include costs/efforts
# TODO: costs as a function of k number of failures

def generic_intersection(*collections):
    items = generic_union(*collections)
    return {item for item in items if all(item in collection for collection in collections)}

def generic_union(*collections):
    intersection = set()
    for collection in collections:
        intersection.update(collection)
    return intersection

##################################################

def extract_action_plan(opt_plan):
    action_plan = opt_plan.action_plan if isinstance(opt_plan, OptPlan) else opt_plan
    return {action for action in action_plan if not isinstance(action, StreamAction)}

def extract_stream_plan(externals, combined_plan):
    stream_plan, opt_plan, _ = combined_plan
    if stream_plan:
        return set(stream_plan)
    stream_actions = {action for action in opt_plan.action_plan if isinstance(action, StreamAction)}
    return {find_unique(lambda e: e.name == name, externals).get_instance(inputs)
            for name, inputs, _ in stream_actions}

##################################################

def prune_dominated_action_plans(combined_plans):
    # TODO: consider multi-sets
    # TODO: more efficient trie-like data structure
    print('Attempting to prune using {} action plans'.format(len(combined_plans)))
    dominated = set()
    indices = list(range(len(combined_plans)))

    start_time = time.time()
    best_action_sets = {}
    for idx in indices:
        opt_plan, cost = combined_plans[idx][-2:]
        action_set = frozenset(extract_action_plan(opt_plan))
        if (action_set not in best_action_sets) or (cost < best_action_sets[action_set][0]):
            best_action_sets[action_set] = (cost, idx)
    best_indices = {idx for _, idx in best_action_sets.values()}
    dominated.update(set(indices) - best_indices)
    print('Pruned {}/{} reordered action plans in {:.3f} seconds'.format(
        len(dominated), len(combined_plans), elapsed_time(start_time)))

    start_time = time.time()
    for idx1, idx2 in permutations(best_indices, r=2):
        if (idx1 in dominated) or (idx2 in dominated):
            continue
        opt_plan1, cost1 = combined_plans[idx1][-2:]
        action_set1 = extract_action_plan(opt_plan1)
        opt_plan2, cost2 = combined_plans[idx2][-2:]
        action_set2 = extract_action_plan(opt_plan2)
        if (action_set1 < action_set2) or (action_set1 == action_set2 and cost1 <= cost2):
            dominated.add(idx2)
    print('Pruned {}/{} dominated action plans in {:.3f} seconds'.format(
        len(dominated), len(combined_plans), elapsed_time(start_time)))
    return [combined_plans[idx] for idx in indices if idx not in dominated]

def prune_dominated_stream_plans(externals, combined_plans):
    # TODO: prune certain multi-set subsets in the candidate generator
    print('Attempting to prune using {} stream plans'.format(len(combined_plans)))
    start_time = time.time()
    dominated = set()
    indices = list(range(len(combined_plans)))
    for idx1, idx2 in permutations(indices, r=2):
        if (idx1 in dominated) or (idx2 in dominated):
            continue
        _, _, cost1 = combined_plans[idx1]
        stream_set1 = extract_stream_plan(externals, combined_plans[idx1])
        _, _, cost2 = combined_plans[idx2]
        stream_set2 = extract_stream_plan(externals, combined_plans[idx2])
        if (stream_set1 < stream_set2) or (stream_set1 == stream_set2 and cost1 <= cost2):
            #print(idx1, stream_set1, idx2, stream_set2)
            dominated.add(idx2)
        #print(len(stream_set1), len(stream_plans2), len(stream_set1 & stream_plans2), cost1, cost2)
    print('Pruned {}/{} stream plans in {:.3f} seconds'.format(
        len(dominated), len(indices), elapsed_time(start_time)))
    return [combined_plans[idx] for idx in indices if idx not in dominated]

##################################################

def extract_components(portfolio):
    vertices = list(range(len(portfolio)))
    edges = [(i1, i2) for i1, i2 in combinations(vertices, r=2) if portfolio[i1] & portfolio[i2]]
    clusters = get_connected_components(vertices, edges)
    #print(clusters)
    return [[portfolio[i] for i in cluster] for cluster in clusters]

def p_conjunction(results, probabilities=None):
    union = generic_union(*results)
    if probabilities is None:
        # TODO: add to diverse instead?
        probabilities = {result: result.external.get_p_success(*result.get_input_values()) for result in union}
    else:
        assert all(result in probabilities for result in union)
    return np.product([probabilities[result] for result in union])

def p_combinations(portfolio, r, max_time=INF, **kwargs):
    # TODO: extract_components
    start_time = time.time()
    p = 0.
    for plans in combinations(portfolio, r=r):
        if elapsed_time(start_time) >= max_time:
            break
        p += p_conjunction(plans, **kwargs)
    sign = -1 if r % 2 == 0 else +1
    return sign*p

def is_overestimate(portfolio, d):
    return (d % 2 == 1) or (d == len(portfolio))

def p_disjunction_helper(portfolio, diverse={}, **kwargs):
    # Inclusion exclusion
    # TODO: incorporate cost/overhead
    # TODO: prune if upper bound is less than best lower bound according to the Bonferroni inequalities
    # TODO: iteratively increase d and prune
    # TODO: weight using costs
    start_time = time.time()
    max_time = diverse.get('d', INF)
    d = min(len(portfolio), diverse.get('d', INF))
    assert is_overestimate(portfolio, d)
    p_list = []
    for i in range(d):
        #print(i, sum(p_list), elapsed_time(start_time))
        p = p_combinations(portfolio, r=(i + 1), max_time=(max_time - elapsed_time(start_time)), **kwargs)
        if p is None:
            break
        p_list.append(p)
    if not is_overestimate(portfolio, len(p_list)):
        p_list = p_list[:-1]
    return sum(p_list)

def p_disjunction(portfolio, diverse={}, **kwargs):
    # TODO: cache subproblems
    return p_disjunction_helper(portfolio, diverse=diverse, **kwargs)
    # p_fail = 1.
    # for cluster in extract_components(portfolio):
    #     p_fail *= (1. - p_disjunction_helper(cluster, diverse=diverse, **kwargs))
    # return 1. - p_fail

##################################################

# https://hub.docker.com/r/ctpelok77/ibmresearchaiplanningsolver
# https://zenodo.org/record/3404122#.XtZg8J5KjAI
# Diversity metrics: stability, state, uniqueness
# Linear combinations of these

#def str_from_plan(action_plan):
#    return '[{}]'.format(', '.join('{}{}'.format(*action) for action in action_plan))

# Linear combinations of these
# Minimize similarity
# dDISTANTkSET: requires the distance between every pair of plans in the solution to be of bounded diversity
# average over the pairwise dissimilarity of the set
def stability(portfolio, diverse, r=2, op=np.average): # min | np.average
    # the ratio of the number of actions that appear on both plans
    # to the total number of actions on these plans, ignoring repetitions
    # TODO: infer r from diverse
    assert portfolio
    if len(portfolio) == 1:
        return 1.
    values = []
    for combo in combinations(portfolio, r=r):
        # TODO: weighted intersection
        sim = float(len(generic_intersection(*combo))) / len(generic_union(*combo))
        values.append(1. - sim)
    return op(values)

def state(portfolio, diverse):
    # measures similarity between two plans based on representing the plans as as sequence of states
    # maybe adapt using the certified conditions?
    raise NotImplementedError()

def uniqueness(portfolio, diverse, op=np.average):
    # measures whether two plans are permutations of each other, or one plan is a subset of the other plan
    assert portfolio
    if len(portfolio) == 1:
        return 1.
    values = []
    for stream_plan1, stream_plan2 in combinations(portfolio, r=2):
        sim = (stream_plan1 <= stream_plan2) or (stream_plan2 <= stream_plan1)
        values.append(1 - sim)
    return op(values)

def score(portfolio, diverse, **kwargs):
    metric = diverse.get('metric', 'p_success')
    if metric == 'stability':
        return stability(portfolio, diverse) #, **kwargs)
    if metric == 'state':
        return state(portfolio, diverse) #, **kwargs)
    if metric == 'uniqueness':
        return uniqueness(portfolio, diverse) #, **kwargs)
    if metric == 'p_success':
        # TODO: Monte Carlo simulation
        return p_disjunction(portfolio, diverse, **kwargs)
    raise ValueError(metric)

##################################################

# Comparisons
# 1) Random
# 2) Diverse order
# 3) Greedy + Approx
# 4) Optimal + Exact
# 5) Increased K

# TODO: toggle behavior based on k

def random_subset(portfolio, diverse, **kwargs):
    #if k is None:
    #    k = DEFAULT_K
    k = diverse['k']
    return random.sample(portfolio, k=k)

def first_k(portfolio, diverse, **kwargs):
    k = diverse['k']
    return portfolio[:k]

##################################################

def greedy_diverse_subset(candidates, diverse, verbose=True, **kwargs):
    # TODO: lazy greedy submodular maximization
    # TODO: greedy Bayesian update metric (like the one I used during the search)
    # TODO: convergence relative threshold
    start_time = time.time()
    k = diverse['k']
    max_time = diverse.get('max_time', INF)
    best_indices = set()
    # TODO: warm start using exact_diverse_subset for small k
    for i in range(len(best_indices), k):
        if elapsed_time(start_time) >= max_time: # Randomly select the rest
            break
        best_index, best_p = None, -INF
        for index in set(range(len(candidates))) - best_indices:
            portfolio = [candidates[i] for i in best_indices | {index}]
            # TODO: could prune dominated candidates given the selected set
            p = score(portfolio, diverse, **kwargs)
            if p > best_p:
                best_index, best_p = index, p
        best_indices.add(best_index)
        if verbose:
            print('{}) p={:.3f} | Time: {:.2f}'.format(i, best_p, elapsed_time(start_time)))

    best_portfolio = [candidates[i] for i in best_indices]
    if verbose:
        print('\nCandidates: {} | k={} | Time: {:.2f}'.format(
            len(candidates), k, elapsed_time(start_time)))
    return best_portfolio

##################################################

def exact_diverse_subset(candidates, diverse, verbose=True, **kwargs):
    # TODO: dynamic programming method across multi-sets
    # TODO: ILP over selections
    start_time = time.time()
    k = diverse['k']
    max_time = diverse.get('max_time', INF)
    num_considered = 0
    best_portfolio, best_p = None, -INF
    for i, portfolio in enumerate(combinations(randomize(candidates), r=k)):
        assert len(portfolio) == k
        num_considered += 1
        # Weight by effort
        #stream_plans = [set(stream_plan) for stream_plan, _, _, in portfolio]
        #stream_plans = [extract_stream_plan(externals, combined_plan) for combined_plan in portfolio]
        stream_plans = portfolio
        p = score(stream_plans, diverse, **kwargs)
        #assert 0 <= p <= 1 # May not hold if not exact
        if verbose:
            intersection = generic_intersection(*stream_plans)
            print('\nTime: {:.2f} | Group: {} | Intersection: {} | p={:.3f}'.format(
                elapsed_time(start_time), i, len(intersection), p))  # , intersection)
            for stream_plan in stream_plans:
                print(len(stream_plan), stream_plan)
            # for stream_plan, opt_plan, cost in portfolio:
            #     #print(len(stream_plan), stream_plan)
            #     print(len(opt_plan.action_plan), cost, str_from_plan(opt_plan.action_plan))
        if p > best_p:
            best_portfolio, best_p = portfolio, p
        if max_time < elapsed_time(start_time):
            break

    if verbose:
        print('\nCandidates: {} | k={} | Considered: {} | Best (p={:.3f}) | Time: {:.2f}'.format(
            len(candidates), k, num_considered, best_p, elapsed_time(start_time)))
    return best_portfolio

##################################################

def select_portfolio(candiates, diverse, **kwargs):
    k = diverse['k']
    assert 1 <= k
    selector = diverse['selector']
    if len(candiates) <= k:
        portfolio = candiates
    elif selector == 'random':
        portfolio = random_subset(candiates, diverse, **kwargs)
    elif selector == 'first':
        portfolio = first_k(candiates, diverse, **kwargs)
    elif selector == 'greedy':
        portfolio = greedy_diverse_subset(candiates, diverse, **kwargs)
    elif selector == 'exact':
        portfolio = exact_diverse_subset(candiates, diverse, **kwargs)
    else:
        raise ValueError(selector)
    assert len(portfolio) <= k
    return portfolio

def diverse_subset(externals, candidate_plans, diverse, verbose=True, **kwargs):
    # TODO: report back other statistics (possibly in kwargs)
    # TODO: pass results instead of externals
    if not diverse:
        return candidate_plans[:1]
    # for i, combined_plan in enumerate(combined_plans):
    #     stream_plan, opt_plan, cost = combined_plan
    #     print('\n{}) cost={:.0f}, length={}'.format(i, cost, len(opt_plan.action_plan)))
    #     print_plan(opt_plan.action_plan)
    #     print(extract_stream_plan(externals, combined_plan))
    start_time = time.time()
    #pruned_plans = candidate_plans
    pruned_plans = prune_dominated_action_plans(candidate_plans)
    pruned_plans = prune_dominated_stream_plans(externals, pruned_plans)
    #stream_plans = [extract_stream_plan(externals, plan) for plan in pruned_plans]
    #stream_dict = {frozenset(extract_stream_plan(externals, plan)): i
    #               for i, plan in enumerate(pruned_plans)} # Seems to retain ordering somehow
    stream_dict = OrderedDict((frozenset(extract_stream_plan(externals, plan)), i)
                              for i, plan in enumerate(pruned_plans))
    stream_plans = list(stream_dict.keys())
    subset_plans = select_portfolio(stream_plans, diverse, verbose=verbose, **kwargs)

    # TODO: sort plans (or use my reordering technique)
    stream_plans = subset_plans
    subset_plans = [pruned_plans[stream_dict[stream_plan]] for stream_plan in subset_plans]
    #subset_plans = [pruned_plans[stream_plans.index(candidate)] for candidate in subset_plans] # TODO: incorrect?
    #stream_plans = [extract_stream_plan(externals, plan) for plan in subset_plans]
    total_cost = sum(cost for _, _, cost in subset_plans) # normalize, length
    p = score(stream_plans, diverse, **kwargs)
    print('c={} | k={} | n={} | p={:.3f} | cost={:.3f} | {:.3f} seconds'.format(
        len(candidate_plans), diverse['k'], len(subset_plans), p, total_cost, elapsed_time(start_time)))
    if verbose:
        for i, (stream_plan, opt_plan, cost) in enumerate(subset_plans):
            print()
            print(i, len(opt_plan.action_plan), cost, str_from_plan(opt_plan.action_plan))
            print(i, len(stream_plan), stream_plan)
        print('\n'+'-'*50+'\n')
    return subset_plans
