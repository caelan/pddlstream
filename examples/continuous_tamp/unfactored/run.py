#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats

import numpy as np

from examples.continuous_tamp.constraint_solver import cfree_motion_fn
from examples.continuous_tamp.primitives import get_pose_gen, collision_test, \
    distance_fn, inverse_kin_fn, get_region_test, plan_motion, get_blocked_problem, get_tight_problem, draw_state, \
    apply_action, get_random_seed
from examples.discrete_tamp.viewer import COLORS
from pddlstream.conversion import And, Equal
from pddlstream.downward import TOTAL_COST
from pddlstream.focused import solve_focused
from pddlstream.incremental import solve_incremental
from pddlstream.stream import from_fn, from_test, from_gen_fn
from pddlstream.synthesizer import StreamSynthesizer
from pddlstream.utils import print_solution, user_input, read, INF, get_file_path
from examples.continuous_tamp.viewer import ContinuousTMPViewer, GROUND

R = 'r'
H = 'h'

def at_conf(s, q):
    return np.allclose(s['r'], q)

def at_pose(s, b, p):
    return (s[b] is not None) and np.allclose(s[b], p)

def holding(s, b):
    return s['h'] == b

def hand_empty(s):
    return s['h'] is None

def get_move_fn(regions):
    def fn(s1, t):
        q1, q2 = t
        if not at_conf(s1, q1):
            return None
        s2 = s1.copy()
        s2['r'] = q2
        return (s2,)
    return fn

def get_pick_fn(regions):
    def fn(s1, b, q, p):
        if (s1['h'] is not None) or (s1[b] is None) or (not np.allclose(s1[b], p)):
            return None
        s2 = s1.copy()
        s2[b] = None
        s2['h'] = b
        return (s2,)
    return fn

def get_place_fn(regions):
    def fn(s1, b, q, p):
        if (s1['h'] != b) or (s1[b] is not None):
            return None
        #if s1[b] != p:
        #    return None
        s2 = s1.copy()
        s2[b] = p
        s2['h'] = None
        return (s2,)
    return fn

def get_goal_test(tamp_problem):
    region_test = get_region_test(tamp_problem.regions)
    def test(s):
        if not at_conf(s, tamp_problem.goal_conf):
            return False
        if not hand_empty(s):
            return False
        for b, r in tamp_problem.goal_regions.items():
            if (s[b] is None) or not region_test(b, s[b], r):
                return False
        return True
    return test


def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    #state = tamp_problem.initial
    state = {
        'r': tamp_problem.initial.conf,
        'h': tamp_problem.initial.holding,
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
           [('Placeable', b, GROUND) for b in initial.block_poses.keys()] + \
           [('Placeable', b, r) for b, r in tamp_problem.goal_regions.items()]
    goal = ('AtGoal',)

    stream_map = {
        'plan-motion': from_fn(plan_motion),
        'sample-pose': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        'test-region': from_test(get_region_test(tamp_problem.regions)),
        'inverse-kinematics':  from_fn(inverse_kin_fn),
        #'posecollision': collision_test,
        #'trajcollision': lambda *args: False,

        'forward-move': from_fn(get_move_fn(tamp_problem.regions)),
        'forward-pick': from_fn(get_pick_fn(tamp_problem.regions)),
        'forward-place': from_fn(get_place_fn(tamp_problem.regions)),
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
                                 max_time=10, max_cost=INF, debug=False,
                                 effort_weight=None, unit_costs=unit_costs, postprocess=False,
                                 visualize=False)
    else:
        solution = solve_incremental(pddlstream_problem, layers=1, unit_costs=unit_costs, verbose=False)
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
    for i, action in enumerate(plan):
        user_input('Continue?')
        print(i, *action)
        state = apply_action(state, action)
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')


if __name__ == '__main__':
    main()
