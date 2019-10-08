#!/usr/bin/env python

from __future__ import print_function

import numpy as np

from examples.continuous_tamp.primitives import get_pose_gen, inverse_kin_fn, get_region_test, plan_motion, \
    tight, blocked, draw_state, get_random_seed, TAMPState, GROUND_NAME, GRASP, SUCTION_HEIGHT
from examples.continuous_tamp.viewer import ContinuousTMPViewer
from examples.discrete_tamp.viewer import COLORS
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF, get_file_path, Profiler
from pddlstream.language.constants import print_solution

from copy import deepcopy

def at_conf(s, r, q):
    return np.allclose(s.robot_confs[r], q)

def at_pose(s, b, p):
    #if s[b] is None:
    if b not in s.block_poses:
        return False
    return np.allclose(s.block_poses[b], p)

def hand_empty(s, r):
    return r not in s.holding
    #return s[r] is None

def holding(s, r, b):
    if hand_empty(s, r):
        return False
    return s.holding[r][0] == b

def at_grasp(s, r, b, g):
    return holding(s, r, b) and (np.allclose(s.holding[r][1], g))

##################################################

def move_fn(s1, r, q1, t, q2):
    if not at_conf(s1, r, q1):
        return None
    s2 = deepcopy(s1)
    s2.robot_confs[r] = q2
    return (s2,)

def pick_fn(s1, r, b, q, p, g):
    if not (at_pose(s1, b, p) and at_conf(s1, r, q)):
        return None
    s2 = deepcopy(s1)
    del s2.block_poses[b]
    s2.holding[r] = (b, g)
    return (s2,)

def place_fn(s1, r, b, q, p, g):
    if not (at_grasp(s1, r, b, g) and at_conf(s1, r, q)):
        return None
    s2 = deepcopy(s1)
    s2.block_poses[b] = p
    del s2.holding[r]
    return (s2,)

def get_goal_test(tamp_problem):
    region_test = get_region_test(tamp_problem.regions)
    def test(s):
        for r in tamp_problem.initial.robot_confs:
            if not at_conf(s, r, tamp_problem.goal_conf): # or not hand_empty(s, r):
                return False
        #return holding(s, 'r0', 'A')
        for b, r in tamp_problem.goal_regions.items():
            if not (b in s.block_poses and region_test(b, s.block_poses[b], r)):
                return False
        return True
    return test

##################################################

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(not initial.holding) # is None

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    # TODO: can always make the state a set of fluents
    init = [
        ('State', initial),
        ('AtState', initial)] + \
           [('Robot', r) for r in initial.robot_confs.keys()] + \
           [('Conf', q) for q in initial.robot_confs.values()] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Pose', b, p) for b, p in initial.block_poses.items()] + \
           [('Grasp', b, GRASP) for b in initial.block_poses.keys()] + \
           [('Region', r) for r in tamp_problem.regions.keys()] + \
           [('Placeable', b, GROUND_NAME) for b in initial.block_poses.keys()] + \
           [('Placeable', b, r) for b, r in tamp_problem.goal_regions.items()]
    # TODO: could populate AtConf, AtPose, HandEmpty, Holding
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



def main(deterministic=False, unit_costs=True):
    np.set_printoptions(precision=2)
    if deterministic:
        seed = 0
        np.random.seed(seed)
    print('Seed:', get_random_seed())

    problem_fn = tight  # get_tight_problem | get_blocked_problem
    tamp_problem = problem_fn(n_blocks=1, n_goals=1, n_robots=1)
    print(tamp_problem)

    stream_info = {
        #'test-region': StreamInfo(eager=True, p_success=0), # bound_fn is None
        #'plan-motion': StreamInfo(p_success=1),  # bound_fn is None
        #'trajcollision': StreamInfo(p_success=1),  # bound_fn is None
        #'cfree': StreamInfo(eager=True),
    }

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    with Profiler():
        solution = solve_incremental(pddlstream_problem, complexity_step=1, max_time=30,
                                     unit_costs=unit_costs, verbose=False)
        print_solution(solution)
        plan, cost, evaluations = solution
    if plan is None:
        return

    colors = dict(zip(sorted(tamp_problem.initial.block_poses.keys()), COLORS))
    viewer = ContinuousTMPViewer(SUCTION_HEIGHT, tamp_problem.regions, title='Continuous TAMP')
    state = tamp_problem.initial
    print()
    print(state)
    draw_state(viewer, state, colors)
    for i, (action, args) in enumerate(plan):
        user_input('Continue?')
        print(i, action, args)
        state = args[-1]
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')


if __name__ == '__main__':
    main()
