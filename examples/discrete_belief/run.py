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

from examples.discrete_belief.dist import DDist, MixtureDD, DeltaDist, UniformDist, totalProbability, JDist

"""
  (:derived (Unsafe2 ?l ?p1)
    (exists (?o ?d ?p2) (and (Obj ?o) (Prob ?p1) (PCollision ?l ?d ?p2) (GE ?p1 ?p2) ;Strict?
                         (BLoc ?o ?d)))
  )
"""

# TODO: would be helpful if I could use <= here
# TODO: could use fixed threshold or the max of the ones met
# TODO: could ensure tight, but I bet FD prunes dominated actions anyways
# TODO: conditional effects on movements/collapsing belief (can either rest or just make zero)

##################################################

BeliefProblem = namedtuple('BeliefProblem', ['initial', 'goal', 'locations',
                                             'p_move_s', 'p_look_fp', 'p_look_fn'])

def get_belief_problem(num_locs, deterministic, observable):
    if observable:
        p1, p2 = 1.0, 1.0
    else:
        p1, p2 = 0.6, 0.9

    initial = [
        ('o1', 'l1', p1),
        ('o2', 'l2', p2),
    ]

    #goal = [('o1', 'l1', 0.95)]
    #goal = [('o1', 'l0', 0.95)]
    goal = [('o1', 'l2', 0.95)]

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

def clip_p(p, min_p=1e-3, max_p=1-1e-3):
#def clip_p(p, min_p=0, max_p=1):
    # TODO: round p for clarity as well?
    return min(max(min_p, p), max_p)

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

def prob_occupied(l, d):
    return float(d.prob(l))

def prob_collision(d1, d2):
    # TODO: collision options
    # 1) Constraint that cannot collide above threshold (infinite collision cost)
    # 2) Choose threshold but incorporate into cost
    # 3) Choose threshold but incorporate into transitions (most general but might use collisions)
    d_joint = JDist(d1, lambda _: d2)
    d_collision = d_joint.project(lambda (l1, l2): l1 == l2)
    return float(d_collision.prob(True))

def ge_fn(d, l, p):
    return bool(p <= prob_occupied(l, d)) # TODO: numpy boolean

def get_collision_test(max_p_collision):
    # TODO: could include the threshold as a parameter
    # TODO: function that computes this threshold and then test?
    def test(l, d):
        return ge_fn(d, l, max_p_collision)
        #return prob_collision(..., d) <= max_p_collision
        #return False
    return test

##################################################


def get_transition_fn(control_loc1, control_loc2, p_move_s):
    # TODO: can factor in collisions within
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
        d_obs = totalProbability(d, get_observation_fn(control_loc,
                                                       belief_problem.p_look_fp,
                                                       belief_problem.p_look_fn))
        p_obs = float(d_obs.prob(True))
        expected_cost = revisit_mdp_cost(action_cost, action_cost, p_obs)
        return scale_cost(expected_cost)
    return fn

def get_look_fn(belief_problem):
    def fn(control_loc, d1):
        if prob_occupied(control_loc, d1) == 0:  # Not possible to generate observation
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
        'BCollision': get_collision_test(0.25),
        'ge': from_test(ge_fn),
        'prob-after-move': from_fn(get_move_fn(belief_problem)),
        'prob-after-look': from_fn(get_look_fn(belief_problem)),
        #'PCollision': from_fn(prob_occupied), # Then can use GE
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

def main(num_locs=5, deterministic=False, observable=False, focused=True, custom_bound=True):

    belief_problem = get_belief_problem(num_locs, deterministic, observable)
    pddlstream_problem = to_pddlstream(belief_problem)

    # TODO: cap the cost at MAX_COST?
    solution = solve_incremental(pddlstream_problem, planner='ff-wastar1', max_cost=MAX_COST, unit_costs=False)
    print_solution(solution)
    plan, cost, init = solution
    print('Real cost:', float(cost)/SCALE_COST)

if __name__ == '__main__':
    main(deterministic=False, observable=False, focused=True, custom_bound=True)
