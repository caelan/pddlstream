#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats

import numpy as np
from pddlstream.algorithms.focused import solve_focused

from examples.continuous_tamp.primitives import get_pose_gen, inverse_kin_fn, get_region_test, plan_motion, \
    get_tight_problem, draw_state, \
    get_random_seed, TAMPState
from examples.continuous_tamp.viewer import ContinuousTMPViewer, GROUND_NAME
from examples.discrete_tamp.viewer import COLORS
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF, get_file_path
from pddlstream.language.constants import print_solution

R = 'r'
H = 'h'

def at_conf(s, q):
    return np.allclose(s[R], q)

def at_pose(s, b, p):
    return (s[b] is not None) and np.allclose(s[b], p)

def holding(s, b):
    return s[H] == b

def hand_empty(s):
    return s[H] is None

##################################################

def move_fn(s1, q1, t, q2):
    if not at_conf(s1, q1):
        return None
    s2 = s1.copy()
    s2[R] = q2
    return (s2,)

def pick_fn(s1, b, q, p):
    if not (at_pose(s1, b, p) and at_conf(s1, q)):
        return None
    s2 = s1.copy()
    s2[b] = None
    s2[H] = b
    return (s2,)

def place_fn(s1, b, q, p):
    if not (holding(s1, b) and at_conf(s1, q)):
        return None
    s2 = s1.copy()
    s2[b] = p
    s2[H] = None
    return (s2,)

def get_goal_test(tamp_problem):
    region_test = get_region_test(tamp_problem.regions)
    def test(s):
        if not at_conf(s, tamp_problem.goal_conf):
            return False
        return holding(s, 'block0')

        if not hand_empty(s):
            return False
        for b, r in tamp_problem.goal_regions.items():
            if (s[b] is None) or not region_test(b, s[b], r):
                return False
        return True
    return test

##################################################

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    # TODO: can always make the state the set of fluents
    #state = tamp_problem.initial
    state = {
        R: tamp_problem.initial.conf,
        H: tamp_problem.initial.holding,
    }
    for b, p in tamp_problem.initial.block_poses.items():
        state[b] = p

    init = [
        ('State', state),
        ('AtState', state),
        ('Conf', initial.conf)] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Pose', b, p) for b, p in initial.block_poses.items()] + \
           [('Region', r) for r in tamp_problem.regions.keys()] + \
           [('AtPose', b, p) for b, p in initial.block_poses.items()] + \
           [('Placeable', b, GROUND_NAME) for b in initial.block_poses.keys()] + \
           [('Placeable', b, r) for b, r in tamp_problem.goal_regions.items()]
    goal = ('AtGoal',)

    stream_map = {
        'plan-motion': from_fn(plan_motion),
        'sample-pose': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        'test-region': from_test(get_region_test(tamp_problem.regions)),
        'inverse-kinematics':  from_fn(inverse_kin_fn),
        #'posecollision': collision_test,
        #'trajcollision': lambda *args: False,

        'forward-move': from_fn(move_fn),
        'forward-pick': from_fn(pick_fn),
        'forward-place': from_fn(place_fn),
        'test-goal': from_test(get_goal_test(tamp_problem)),
    }
    #stream_map = 'debug'

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def main(focused=False, deterministic=False, unit_costs=True):
    np.set_printoptions(precision=2)
    if deterministic:
        seed = 0
        np.random.seed(seed)
    print('Seed:', get_random_seed())

    problem_fn = get_tight_problem  # get_tight_problem | get_blocked_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    stream_info = {
        #'test-region': StreamInfo(eager=True, p_success=0), # bound_fn is None
        #'plan-motion': StreamInfo(p_success=1),  # bound_fn is None
        #'trajcollision': StreamInfo(p_success=1),  # bound_fn is None
        #'cfree': StreamInfo(eager=True),
    }

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    pr = cProfile.Profile()
    pr.enable()
    if focused:
        solution = solve_focused(pddlstream_problem, stream_info=stream_info,
                                 max_time=10, success_cost=INF, debug=False,
                                 effort_weight=None, unit_costs=unit_costs,
                                 visualize=False)
    else:
        solution = solve_incremental(pddlstream_problem, complexity_step=1,
                                     unit_costs=unit_costs, verbose=False)
    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    if plan is None:
        return

    colors = dict(zip(sorted(tamp_problem.initial.block_poses.keys()), COLORS))
    viewer = ContinuousTMPViewer(tamp_problem.regions, title='Continuous TAMP')
    state = tamp_problem.initial
    print()
    print(state)
    draw_state(viewer, state, colors)
    for i, (action, args) in enumerate(plan):
        user_input('Continue?')
        print(i, action, args)
        s2 = args[-1]
        state = TAMPState(s2[R], s2[H], {b: s2[b] for b in state.block_poses if s2[b] is not None})
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')


if __name__ == '__main__':
    main()
