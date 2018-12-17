from __future__ import print_function

import cProfile
import math
import os
import pstats
from collections import namedtuple

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_fn, from_test

from examples.discrete_belief.dist import DDist, MixtureDD, DeltaDist, UniformDist, totalProbability, JDist
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.downward import get_cost_scale, set_cost_scale, MAX_FD_COST

from pddlstream.language.constants import And, print_solution
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, INF

# TODO: would be helpful if I could use <= here
# TODO: could use fixed threshold or the max of the ones met
# TODO: could ensure tight, but I bet FD prunes dominated actions anyways
# TODO: conditional effects on movements/collapsing belief (can either rest or just make zero)
# TODO: mixture of Gaussian's
# TODO: do we want to control other modalities by this?
# TODO: reduce to Gaussian around position in approximate belief
# TODO: factor belief to independent objects being updated. I think the multimodal won't work otheriwse

##################################################

BeliefProblem = namedtuple('BeliefProblem', ['initial', 'goal', 'locations',
                                             'p_move_s', 'p_look_fp', 'p_look_fn'])

def get_belief_problem(deterministic, observable):
    if observable:
        p1, p2 = 1.0, 1.0
    else:
        p1, p2 = 0.6, 0.9

    l0 = 'l0'
    l1 = 'l1'
    l2 = 'l2'

    initial = [
        ('o1', l1, p1),
        ('o2', l2, p2),
    ]

    #goal = [('o1', l1, 0.95)]
    #goal = [('o1', l0, 0.95)]
    goal = [('o1', l2, 0.95)]

    locations = {l0}

    if deterministic:
        p_move_s = 1.0
        p_look_fp = p_look_fn = 0.0
    else:
        p_move_s = 0.8
        p_look_fp = p_look_fn = 0.1

    return BeliefProblem(initial, goal, locations,
                         p_move_s, p_look_fn, p_look_fp)

##################################################

SCALE_COST = 1e3
MAX_COST = MAX_FD_COST / (10 * SCALE_COST)

def clip_cost(cost): # TODO: move this to downward?
    if cost == INF:
        return MAX_COST
    return min(cost, MAX_COST)


#def clip_p(p, min_p=1e-3, max_p=1-1e-3):
#    return min(max(min_p, p), max_p)


#def log_cost(p):
#    if p == 0:
#        return INF
#    return -math.log(p)


def geometric_cost(cost, p):
    if p == 0:
        return INF
    return cost / p


def revisit_mdp_cost(success_cost, failure_cost, p):
    """
    Cost for MDP where failures remain in the same state
    Equals geometric_cost(cost, p) == revisit_mdp_cost(cost, cost, p)
    failure_cost and success_cost can be about the same
    """
    return (success_cost - failure_cost) + geometric_cost(failure_cost, p)


def continue_mdp_cost(success_cost, failure_cost, p):
    """
    Cost for MDP where failures move to the intended state
    failure_cost should be much larger than success_cost
    """
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
    d_collision = d_joint.project(lambda pair: pair[0] == pair[1])
    return float(d_collision.prob(True))

def ge_fn(o, d, l, p):
    return bool(p <= prob_occupied(l, d)) # TODO: numpy boolean

def get_collision_test(max_p_collision):
    # TODO: could include the threshold as a parameter
    # TODO: function that computes this threshold and then test?
    def test(l, o, d):
        return max_p_collision < prob_occupied(l, d)
        #return ge_fn(d, l, max_p_collision)
        #return max_p_collision < prob_collision(..., d)
        #return max_p_collision < prob_collision(..., d)
    return test

##################################################

def get_transition_fn(loc1, loc2, p_move_s):
    # TODO: can factor in collisions within
    def fn(l):
        # P(s2 | s1=loc1, a=(control_loc1, control_loc2))
        perfect_d = DeltaDist(loc2)
        fail_d = DeltaDist(l)
        # fail_d = UniformDist(locations)
        # return MixtureDD(perfect_d, fail_d, p_move_s)
        if loc1 == l:
            return MixtureDD(perfect_d, fail_d, p_move_s)
        return fail_d
    return fn

def move_cost_fn():
#def move_cost_fn(l1, l2):
    action_cost = 1
    return clip_cost(action_cost)

def get_move_fn(p_move_s):
    def fn(o, d1, loc1, loc2):
        d2 = d1.copy()
        d2.transitionUpdate(get_transition_fn(loc1, loc2, p_move_s))
        return (d2,)
    return fn

##################################################

def get_observation_fn(loc, p_look_fp, p_look_fn):
    def fn(l):
        # P(obs | s1=loc1, a=control_loc)
        if l == loc:
            return DDist({True: 1 - p_look_fn,
                          False: p_look_fn})
        return DDist({True: p_look_fp,
                      False: 1 - p_look_fp})
    return fn

def get_look_cost_fn(p_look_fp, p_look_fn):
    action_cost = 1
    def fn(o, d, loc, obs):
        d_obs = totalProbability(d, get_observation_fn(loc, p_look_fp, p_look_fn))
        p_obs = float(d_obs.prob(obs))
        expected_cost = revisit_mdp_cost(action_cost, action_cost, p_obs)
        return clip_cost(expected_cost)
    return fn

def get_look_fn(p_look_fp, p_look_fn):
    def fn(o, d1, loc, obs):
        d2 = d1.copy()
        d2.obsUpdate(get_observation_fn(loc, p_look_fp, p_look_fn), obs)
        if not d2.support():
            return None
        return (d2,)
    return fn

##################################################

OTHER = 'other'

def concentrate_belief(loc, d):
    """
    Projects Multinoulli distribution d into a Bernoulli distribution with probability mass concentrated
    at either a given element or a "generic" element named OTHER
    """
    return d.project(lambda l1: l1 if (l1 == loc) else OTHER)

def get_opt_move_fn(factor):
    """
    Optimistic move function
    :param factor: Boolean that when true, factors d1 before performing updates
    """
    perfect_move_fn = get_move_fn(1)
    def fn(o, d1, loc1, loc2):
        """
        Move function with perfect transitions
        """
        if factor:
            d1 = concentrate_belief(loc1, d1)
        return perfect_move_fn(o, d1, loc1, loc2)
    return fn

def get_opt_obs_fn(factor):
    """
    Optimistic observation function
    :param factor: Boolean that when true, factors d1 before performing updates
    """
    perfect_look_fn = get_look_fn(0, 0)
    def fn(o, d1, loc, obs):
        """
        Look function with perfect observations
        """
        if factor:
            d1 = concentrate_belief(loc, d1)
        return perfect_look_fn(o, d1, loc, obs)
    return fn

##################################################

def to_pddlstream(belief_problem, collisions=True):
    locations = {l for (_, l, _) in belief_problem.initial + belief_problem.goal} | \
                set(belief_problem.locations)
    observations = [True, False]
    uniform = UniformDist(locations)
    initial_bel = {o: MixtureDD(DeltaDist(l), uniform, p) for o, l, p in belief_problem.initial}
    max_p_collision = 0.25 if collisions else 1.0

    # TODO: separate pick and place for move
    init = [('Obs', obs) for obs in observations] + \
           [('Location', l) for l in locations]
    for o, d in initial_bel.items():
        init += [('Dist', o, d), ('BLoc', o, d)]
    for (o, l, p) in belief_problem.goal:
        init += [('Location', l), ('GoalProb', l, p)]
    goal_literals = [('BLocGE', o, l, p) for (o, l, p) in belief_problem.goal]
    goal = And(*goal_literals)

    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))
    constant_map = {}
    stream_map = {
        'BCollision': get_collision_test(max_p_collision),
        'GE': from_test(ge_fn),
        'prob-after-move': from_fn(get_move_fn(belief_problem.p_move_s)),
        'MoveCost': move_cost_fn,
        'prob-after-look': from_fn(get_look_fn(belief_problem.p_look_fp, belief_problem.p_look_fn)),
        'LookCost': get_look_cost_fn(belief_problem.p_look_fp, belief_problem.p_look_fn),
        #'PCollision': from_fn(prob_occupied), # Then can use GE
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def main(deterministic=False, observable=False, collisions=True, focused=True, factor=True):
    # TODO: global search over the state
    belief_problem = get_belief_problem(deterministic, observable)
    pddlstream_problem = to_pddlstream(belief_problem, collisions)
    set_cost_scale(SCALE_COST)
    print('Cost scale:', get_cost_scale())

    pr = cProfile.Profile()
    pr.enable()
    planner = 'ff-wastar1'
    if focused:
        stream_info = {
            'GE': StreamInfo(from_test(ge_fn), eager=False),
            'prob-after-move': StreamInfo(from_fn(get_opt_move_fn(factor=factor))),
            'MoveCost': FunctionInfo(move_cost_fn),
            'prob-after-look': StreamInfo(from_fn(get_opt_obs_fn(factor=factor))),
            'LookCost': FunctionInfo(get_look_cost_fn(p_look_fp=0, p_look_fn=0)),
        }
        solution = solve_focused(pddlstream_problem, stream_info=stream_info, planner=planner, debug=False,
                                 success_cost=0, unit_costs=False, max_time=30)
    else:
        solution = solve_incremental(pddlstream_problem, planner=planner, debug=True,
                                     success_cost=MAX_COST, unit_costs=False, max_time=30)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)

    print_solution(solution)
    plan, cost, init = solution
    print('Real cost:', cost)

if __name__ == '__main__':
    main(deterministic=False, observable=False, collisions=True, focused=True)
