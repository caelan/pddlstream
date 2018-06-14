#!/usr/bin/env python

from __future__ import print_function

import math
import os
from collections import namedtuple

import numpy as np
from pddlstream.downward import TOTAL_COST
from pddlstream.focused import solve_focused

from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.conversion import And, Equal
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import print_solution, user_input, read
from viewer import DiscreteTAMPViewer, COLORS

# TODO: Can infer domain from usage or from specification

GRASP = np.array([0, 0])

# class IKGenerator(Generator):
#     def __init__(self, *inputs):
#         super(IKGenerator, self).__init__()
#         self.p, = inputs
#     def generate(self, *args):
#         self.enumerated = True
#         return [(self.p + GRASP,)]
#
# class IKFactGenerator(FactGenerator):
#     def __init__(self, *inputs):
#         super(IKFactGenerator, self).__init__()
#         self.p, = inputs
#     def generate(self, context=None):
#         self.enumerated = True
#         q = self.p + GRASP
#         # TODO: include a formula instead?
#         return [('conf', q), ('kin', q, self.p)]

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    known_poses = list(initial.block_poses.values()) + \
                  list(tamp_problem.goal_poses.values())

    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))

    q100 = np.array([100, 100])
    constant_map = {
        'q100': q100,
    }

    init = [
        #Type(q100, 'conf'),
        ('CanMove',),
        ('Conf', q100),
        ('Conf', initial.conf),
        ('AtConf', initial.conf),
        ('HandEmpty',),
        Equal((TOTAL_COST,), 0)] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Pose', p) for p in known_poses] + \
           [('AtPose', b, p) for b, p in initial.block_poses.items()]
    # [('Pose', p) for p in known_poses + tamp_problem.poses] + \

    goal = And(*[
        ('AtPose', b, p) for b, p in tamp_problem.goal_poses.items()
    ])

    def collision_test(p1, p2):
        return  np.linalg.norm(p2-p1) <= 1e-1

    def distance_fn(q1, q2):
        ord = 1 # 1 | 2
        return int(math.ceil(np.linalg.norm(q2 - q1, ord=ord)))

    # TODO: convert to lower case
    stream_map = {
        #'sample-pose': from_gen_fn(lambda: ((np.array([x, 0]),) for x in range(len(poses), n_poses))),
        'sample-pose': from_gen_fn(lambda: ((p,) for p in tamp_problem.poses)),
        'inverse-kinematics':  from_fn(lambda p: (p + GRASP,)),
        #'inverse-kinematics': IKGenerator,
        #'inverse-kinematics': IKFactGenerator,
        'collision-free': from_test(lambda *args: not collision_test(*args)),
        'collision': collision_test,
        #'constraint-solver': None,
        'distance': distance_fn,
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

DiscreteTAMPState = namedtuple('DiscreteTAMPState', ['conf', 'holding', 'block_poses'])
DiscreteTAMPProblem = namedtuple('DiscreteTAMPProblem', ['initial', 'poses', 'goal_poses'])

def get_shift_one_problem(n_blocks=2, n_poses=9):
    assert(2 <= n_blocks <= n_poses)
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]
    conf = np.array([0, -5])

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(conf, None, block_poses)
    goal_poses = {blocks[0]: poses[1]}

    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)
    #return DiscreteTAMPProblem(initial, poses, goal_poses)

def get_shift_all_problem(n_blocks=2, n_poses=9):
    assert(n_blocks + 1 <= n_poses)
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]
    conf = np.array([0, -5])

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(conf, None, block_poses)
    goal_poses = dict(zip(blocks, poses[1:]))

    return DiscreteTAMPProblem(initial, poses[n_blocks+1:], goal_poses)

##################################################

def draw_state(viewer, state, colors):
    viewer.clear()
    viewer.draw_environment()
    viewer.draw_robot(*state.conf[::-1])
    for block, pose in state.block_poses.items():
        r, c = pose[::-1]
        viewer.draw_block(r, c, color=colors[block])
    if state.holding is not None:
        pose = state.conf - GRASP
        r, c = pose[::-1]
        viewer.draw_block(r, c, color=colors[state.holding])


def apply_action(state, action):
    conf, holding, block_poses = state
    # TODO: don't mutate block_poses?
    name, args = action
    if name == 'move':
        _, conf = args
    elif name == 'pick':
        holding, _, _ = args
        del block_poses[holding]
    elif name == 'place':
        block, pose, _ = args
        holding = None
        block_poses[block] = pose
    else:
        raise ValueError(name)
    return DiscreteTAMPState(conf, holding, block_poses)

##################################################

def main(focused=True, unit_costs=False):
    problem_fn = get_shift_one_problem # get_shift_one_problem | get_shift_all_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=unit_costs)
    else:
        #solution = solve_exhaustive(pddlstream_problem, unit_costs=unit_costs)
        solution = solve_incremental(pddlstream_problem, unit_costs=unit_costs)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return
    print(evaluations)

    colors = dict(zip(tamp_problem.initial.block_poses, COLORS))
    viewer = DiscreteTAMPViewer(1, len(tamp_problem.poses), title='Initial')
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
