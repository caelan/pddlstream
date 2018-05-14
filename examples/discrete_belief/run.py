from __future__ import print_function

import os
from collections import namedtuple

import numpy as np
import cProfile
import pstats
import math

from pddlstream.conversion import And
from pddlstream.incremental import solve_incremental
from pddlstream.stream import from_fn, from_test, StreamInfo, from_gen_fn
from pddlstream.utils import print_solution, user_input, read, INF

from examples.discrete_belief.dist import DDist, MixtureDD, DeltaDist, UniformDist, totalProbability

BeliefProblem = namedtuple('BeliefProblem', ['initial', 'goal', 'locations',
                                             'p_move_s', 'p_look_fp', 'p_look_fn'])

def get_belief_problem(num_locs, deterministic, observable):
    if observable:
        initial = [
          ('o1', 'l1', 1.0),
          ('o2', 'l2', 1.0),
        ]
    else:
        initial = [
          ('o1', 'l1', 0.6),
          ('o2', 'l2', 0.9),
        ]

    #goal = [('o1', 'l1', 0.95)]
    goal = [('o1', 'l0', 0.95)]
    #goal = [('o1', 'l2', 0.95)]

    locations = {'l0'}

    if deterministic:
        p_move_s = 1.0
        p_look_fp = p_look_fn = 0.0
    else:
        p_move_s = 0.8
        p_look_fp = p_look_fn = 0.1

    return BeliefProblem(initial, goal, locations,
                         p_move_s, p_look_fn, p_look_fp)

##################################################

SCALE_COST = 1000
MAX_COST = 1e8

def scale_cost(cost):
    # Unfortunately, FastDownward only supports nonnegative, integer functions
    # This function scales all costs, so decimals can be factored into the cost
    return min(int(math.ceil(SCALE_COST * cost)), MAX_COST)

MIN_P = 1e-6

def log_cost(p, min_p=MIN_P):
    return -math.log(max(p, min_p))


def geometric_cost(cost, p, min_p=MIN_P):
    return cost / max(p, min_p)


def revisit_mdp_cost(success_cost, failure_cost, p, min_p=MIN_P):
    # Cost for MDP where failures remain in the same state
    # Equals geometric_cost(cost, p) == revisit_mdp_cost(cost, cost, p)
    # failure_cost and success_cost can be about the same
    return success_cost + failure_cost * (1. / max(p, min_p) - 1.)


def continue_mdp_cost(success_cost, failure_cost, p):
    # Cost for MDP where failures move to the intended state
    # failure_cost should be much larger than success_cost
    return p * success_cost + failure_cost * (1 - p)

##################################################

def clip_p(p, min_p=1e-3, max_p=1-1e-3):
#def clip_p(p, min_p=0, max_p=1):
    # TODO: round p for clarity as well?
    return min(max(min_p, p), max_p)


def ge_fn(d, l, p):
    return bool(d.prob(l) >= p) # TODO: numpy boolean

##################################################


def get_transition_fn(control_loc1, control_loc2, p_move_s):
    def fn(loc1):
        # P(s2=loc2 | s1=loc1, a=(control_loc1, control_loc2))
        perfect_d = DeltaDist(control_loc2)
        fail_d = DeltaDist(loc1)
        # fail_d = UniformDist(locations)
        # return MixtureDD(perfect_d, fail_d, p_move_s)
        if control_loc1 == loc1:
            return MixtureDD(perfect_d, fail_d, p_move_s)
        return fail_d
    return fn

def move_cost_fn(l1, l2):
    action_cost = 1
    return scale_cost(action_cost)

def get_move_fn(belief_problem):
    def fn(control_loc1, control_loc2, d1):
        d2 = d1.copy()
        d2.transitionUpdate(get_transition_fn(control_loc1, control_loc2,
                                              belief_problem.p_move_s))
        return (d2,)
    return fn

##################################################

def get_observation_fn(control_loc, p_look_fp, p_look_fn):
    def fn(loc1):
        # P(o=hit | s1=loc1, a=control_loc)
        if loc1 == control_loc:
            return DDist({True: 1 - p_look_fn,
                          False: p_look_fn})
        return DDist({True: p_look_fp,
                      False: 1 - p_look_fp})
    return fn

def get_look_cost_fn(belief_problem):
    action_cost = 1
    def fn(control_loc, d):
        d_obs = totalProbability(d, get_observation_fn(control_loc, belief_problem.p_look_fp,
                                                       belief_problem.p_look_fn))
        p_obs = float(d_obs.prob(True))
        expected_cost = revisit_mdp_cost(action_cost, action_cost, p_obs)
        return scale_cost(expected_cost)
    return fn

def get_look_fn(belief_problem):
    def fn(control_loc, d1):
        if d1.prob(control_loc) == 0:  # Not possible to generate observation
            return None
        d2 = d1.copy()
        obs = True
        d2.obsUpdate(get_observation_fn(control_loc, belief_problem.p_look_fp,
                                        belief_problem.p_look_fn), obs)
        return (d2,)
    return fn

##################################################

def to_pddlstream(belief_problem):
    objects = {o for (o, _, _) in belief_problem.initial}
    locations = {l for (_, l, _) in belief_problem.initial + belief_problem.goal} | \
                set(belief_problem.locations)
    uniform = UniformDist(locations)
    initial_bel = {o: MixtureDD(DeltaDist(l), uniform, p) for o, l, p in belief_problem.initial}

    # TODO: separate pick and place for move
    init = \
        [('Obj', o) for o in objects] + \
        [('Location', l) for l in locations]
    for o, d in initial_bel.items():
        init += [('BLoc', o, d), ('Dist', d)]
    for (o, l, p) in belief_problem.goal:
        init += [('Obj', o), ('Location', l), ('Prob', p)]
    goal_literals = [('BLocGE', o, l, p) for (o, l, p) in belief_problem.goal]
    goal = And(*goal_literals)

    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))
    constant_map = {}
    stream_map = {
        'MoveCost': move_cost_fn,
        'LookCost': get_look_cost_fn(belief_problem),
        'ge': from_test(ge_fn),
        'prob-after-move': from_fn(get_move_fn(belief_problem)),
        'prob-after-look': from_fn(get_look_fn(belief_problem)),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

def main(num_locs=5, deterministic=False, observable=False, focused=True, custom_bound=True):

    belief_problem = get_belief_problem(num_locs, deterministic, observable)
    pddlstream_problem = to_pddlstream(belief_problem)

    solution = solve_incremental(pddlstream_problem, unit_costs=False)
    print_solution(solution)
    plan, cost, init = solution
    print('Real cost:', float(cost)/SCALE_COST)

if __name__ == '__main__':
    main(deterministic=False, observable=False, focused=True, custom_bound=True)
