#!/usr/bin/env python

from __future__ import print_function

import math
import os
from collections import namedtuple

import numpy as np

from examples.continuous_tamp.constraint_solver import BLOCK_WIDTH, BLOCK_HEIGHT, GRASP, get_constraint_solver
from examples.discrete_tamp.viewer import COLORS
from pddlstream.conversion import And, Equal
from pddlstream.fast_downward import TOTAL_COST
from pddlstream.focused import solve_focused
from pddlstream.committed import solve_committed
from pddlstream.incremental import solve_incremental
from pddlstream.stream import from_gen_fn, from_fn, from_test, Generator
from pddlstream.utils import print_solution, user_input
from pddlstream.utils import read
from viewer import ContinuousTMPViewer, GROUND

SCALE_COST = 1.

def interval_contains(i1, i2):
    """
    :param i1: The container interval
    :param i2: The possibly contained interval
    :return:
    """
    return (i1[0] <= i2[0]) and (i2[1] <= i1[1])

def interval_overlap(i1, i2):
    return not ((i1[1] < i2[0]) or (i2[1] < i1[0]))

class PoseGenerator(Generator):
    def __init__(self, *inputs):
        super(PoseGenerator, self).__init__()
        self.p, = inputs
    def generate(self, context=None): # TODO: context
        raise NotImplementedError()

def collision_test(b1, p1, b2, p2):
    return np.linalg.norm(p2 - p1) <= BLOCK_WIDTH

def scale_cost(cost):
    return int(math.ceil(SCALE_COST*cost))

def distance_fn(q1, q2):
    ord = 1  # 1 | 2
    return scale_cost(np.linalg.norm(q2 - q1, ord=ord))

def inverse_kin_fn(b, p):
    return (p - GRASP,)

def get_boundary(b, p):
    return p + np.array([-BLOCK_WIDTH, +BLOCK_WIDTH]) / 2.

def sample_pose(b, region):
    x1, x2 = np.array(region, dtype=float) - get_boundary(b, np.zeros(2))
    if x2 < x1:
        return None
    x = np.random.uniform(x1, x2)
    return np.array([x, 0])

def get_region_test(regions):
    def test(b, p, r):
        r1, r2 = regions[r]
        p1, p2 = get_boundary(b, p)
        return (r1 <= p1) and (p2 <= r2)
    return test

def get_pose_gen(regions):
    def gen_fn(b, r):
        while True:
            p = sample_pose(b, regions[r])
            if p is None:
                break
            yield (p,)
    return gen_fn

##################################################

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))
    constant_map = {}

    init = [
        ('CanMove',),
        ('Conf', initial.conf),
        ('AtConf', initial.conf),
        ('HandEmpty',),
        Equal((TOTAL_COST,), 0)] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Pose', b, p) for b, p in initial.block_poses.items()] + \
           [('Region', r) for r in tamp_problem.regions.keys()] + \
           [('AtPose', b, p) for b, p in initial.block_poses.items()]

    goal_literals = [('In', b, r) for b, r in tamp_problem.goal_regions.items()]
    if tamp_problem.goal_conf is not None:
        goal_literals += [('AtConf', tamp_problem.goal_conf)]
    goal = And(*goal_literals)

    stream_map = {
        'sample-pose': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        'test-region': from_test(get_region_test(tamp_problem.regions)),
        'inverse-kinematics':  from_fn(inverse_kin_fn),
        'collision-free': from_test(lambda *args: not collision_test(*args)),
        #'constraint-solver': get_constraint_solver(tamp_problem.regions),
        'distance': distance_fn,
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

TAMPState = namedtuple('TAMPState', ['conf', 'holding', 'block_poses'])
TAMPProblem = namedtuple('TAMPProblem', ['initial', 'regions', 'goal_conf', 'goal_regions'])

def get_red_problem(n_blocks=1):
    regions = {
        GROUND: (-15, 15),
        'red': (5, 10)
    }

    conf = np.array([0, 5])
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    #poses = [np.array([(BLOCK_WIDTH + 1)*x, 0]) for x in range(n_blocks)]
    poses = [np.array([-(BLOCK_WIDTH + 1) * x, 0]) for x in range(n_blocks)]
    #poses = [sample_pose(regions[GROUND]) for _ in range(n_blocks)]

    initial = TAMPState(conf, None, dict(zip(blocks, poses)))
    goal_regions = {block: 'red' for block in blocks}

    return TAMPProblem(initial, regions, conf, goal_regions)

##################################################

def draw_state(viewer, state, colors):
    viewer.clear_state()
    viewer.draw_environment()
    viewer.draw_robot(*state.conf)
    for block, pose in state.block_poses.items():
        viewer.draw_block(pose[0], BLOCK_WIDTH, BLOCK_HEIGHT, color=colors[block])
    if state.holding is not None:
        viewer.draw_block(state.conf[0], BLOCK_WIDTH, BLOCK_HEIGHT, color=colors[state.holding])


def apply_action(state, action):
    conf, holding, block_poses = state
    # TODO: don't mutate block_poses?
    name = action[0]
    if name == 'move':
        _, conf = action[1:]
    elif name == 'pick':
        holding, _, _ = action[1:]
        del block_poses[holding]
    elif name == 'place':
        block, pose, _ = action[1:]
        holding = None
        block_poses[block] = pose
    else:
        raise ValueError(name)
    return TAMPState(conf, holding, block_poses)

##################################################

def main(deterministic=False):
    np.set_printoptions(precision=2)
    if deterministic:
        np.random.seed(0)

    problem_fn = get_red_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    stream_info = {}

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    solution = solve_incremental(pddlstream_problem, unit_costs=False)
    #solution = solve_focused(pddlstream_problem, unit_costs=True, visualize=False)
    #solution = solve_committed(pddlstream_problem, stream_info=stream_info, unit_costs=False, visualize=False)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return

    colors = dict(zip(tamp_problem.initial.block_poses, COLORS))
    viewer = ContinuousTMPViewer(tamp_problem.regions, title='Continuous TAMP')
    state = tamp_problem.initial
    print(state)
    draw_state(viewer, state, colors)
    for action in plan:
        user_input('Continue?')
        state = apply_action(state, action)
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')


if __name__ == '__main__':
    main()
