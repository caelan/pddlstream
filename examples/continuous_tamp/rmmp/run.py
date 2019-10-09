#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import random

from itertools import combinations
from collections import namedtuple
from copy import deepcopy

from examples.continuous_tamp.primitives import get_pose_gen, inverse_kin_fn, get_region_test, plan_motion, \
    tight, blocked, get_random_seed, TAMPState, GROUND_NAME, GRASP, SUCTION_HEIGHT, sample_region, \
    inverse_kin, forward_kin, collision_test, draw_block, draw_robot
from examples.continuous_tamp.viewer import ContinuousTMPViewer
from examples.discrete_tamp.viewer import COLORS
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF, get_file_path, Profiler, randomize
from pddlstream.language.constants import print_solution, Exists, And

Mode = namedtuple('Mode', ['pose', 'frame'])

# TODO: could attach to surface
GROUND = None # GROUND_NAME

def test_state(problem, state):
    for block1, block2 in combinations(problem.initial.block_poses, r=2):
        if collision_test(block1, state[block1], block2, state[block2]):
            return False
    return True

def sample_forward(problem):
    def gen(mode1):
        unattached = {block for block in mode1 if mode1[block].frame == GROUND}
        attached = set(mode1) - unattached
        holding = {mode1[block].frame for block in attached}

        # TODO: sample combinations
        [robot] = problem.initial.robot_confs
        if robot in holding:
            [block] = attached
            mode2 = mode1.copy()
            while True:
                region = random.choice(problem.regions.values())
                pose = sample_region(block, region)
                if pose is not None:
                    mode2[block] = Mode(pose, GROUND)
                    yield (mode2,)
        else:
            for block in randomize(unattached):
                mode2 = mode1.copy()
                mode2[block] = Mode(GRASP, robot)
                yield (mode2,)
        # TODO: reject inconsistent modes?
    return gen

def sample_intersection(problem):
    def gen(mode1, mode2):
        assert set(mode1) == set(mode2)
        unchanged = {block for block in mode1 if mode1[block].frame == mode2[block].frame} # TODO: pose as well?
        changed = set(mode1) - unchanged

        [block] = changed
        transit_mode, transfer_mode = mode1, mode2
        if mode2[block].frame == GROUND:
            transit_mode, transfer_mode = mode2, mode1
        grasp, robot = transfer_mode[block]
        pose, _ = transit_mode[block]
        conf = inverse_kin(pose, grasp)

        state = {robot: conf}
        for block in transfer_mode: # Should be equivalent when using mode2
            pose, frame = transfer_mode[block]
            if frame == GROUND:
                state[block] = pose
            else:
                state[block] = forward_kin(state[frame], pose)
        # No kinematic chain => only one state in the intersection
        if test_state(problem, state):
            yield (state,)
    return gen

def sample_connection(problem):
    def gen(mode, conf1, conf2):
        traj = [conf1, conf2]
        yield (traj,)
    return gen

def test_goal_state(problem):
    region_test = get_region_test(problem.regions)
    def test(mode, conf):
        return all(region_test(block, conf[block], region)
                   for block, region in problem.goal_regions.items())
    return test

##################################################

def pddlstream_from_tamp(tamp_problem):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    initial = tamp_problem.initial
    mode = {b: Mode(p, None) for b, p in initial.block_poses.items()}
    conf = conf_from_state(initial)

    init = [
        ('CanMove',),
        ('Mode', mode),
        ('AtMode', mode),
        ('Conf', mode, conf),
        ('AtConf', conf),
    ]

    goal = Exists(['?m', '?q'], And(('GoalState', '?m', '?q'),
        ('AtMode', '?m'), ('AtConf', '?q')))

    stream_map = {
        's-forward': from_gen_fn(sample_forward(tamp_problem)),
        's-intersection': from_gen_fn(sample_intersection(tamp_problem)),
        's-connection': from_gen_fn(sample_connection(tamp_problem)),
        't-goal': from_test(test_goal_state(tamp_problem)),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def conf_from_state(initial):
    assert(not initial.holding) # is None
    conf = {}
    conf.update(initial.robot_confs)
    conf.update(initial.block_poses)
    return conf

def draw_conf(viewer, problem, conf):
    viewer.clear_state()
    colors = dict(zip(sorted(problem.initial.block_poses.keys()), COLORS))
    for robot in problem.initial.robot_confs:
        draw_robot(viewer, robot, conf[robot])
    for block in problem.initial.block_poses:
        draw_block(viewer, block, conf[block], color=colors[block])

##################################################

# http://motion.cs.illinois.edu/papers/ijrr2011-MultiModal-preprint.pdf

def main(deterministic=False, unit_costs=True):
    np.set_printoptions(precision=2)
    if deterministic:
        seed = 0
        np.random.seed(seed)
    print('Seed:', get_random_seed())

    problem_fn = tight  # get_tight_problem | get_blocked_problem
    tamp_problem = problem_fn(n_blocks=2, n_goals=2, n_robots=1)
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    with Profiler():
        solution = solve_incremental(pddlstream_problem, complexity_step=1, max_time=30, planner='dijkstra',
                                     unit_costs=unit_costs, verbose=False)
        print_solution(solution)
        plan, cost, evaluations = solution
    if plan is None:
        return

    # TODO: might still be a planning bug
    viewer = ContinuousTMPViewer(SUCTION_HEIGHT, tamp_problem.regions, title='Continuous TAMP')
    conf = conf_from_state(tamp_problem.initial)
    print()
    print(conf)
    draw_conf(viewer, tamp_problem, conf)
    user_input('Start?')
    for i, (action, args) in enumerate(plan):
        print(i, action, args)
        if action == 'switch':
            continue
        traj = args[-1]
        for conf in traj[1:]:
            print(conf)
            draw_conf(viewer, tamp_problem, conf)
            user_input('Continue?')
    user_input('Finish?')


if __name__ == '__main__':
    main()
