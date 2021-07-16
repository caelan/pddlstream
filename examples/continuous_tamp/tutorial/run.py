#!/usr/bin/env python

from __future__ import print_function

import os
import random

from numpy import array, random

from examples.continuous_tamp.primitives import get_random_seed, TAMPProblem, TAMPState
from examples.continuous_tamp.run import set_deterministic, dump_pddlstream, display_plan
from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import Equal, PDDLProblem, TOTAL_COST, print_solution
from pddlstream.language.generator import from_test
from pddlstream.language.stream import StreamInfo
from pddlstream.language.temporal import retime_plan
from pddlstream.utils import read

REGIONS = {
    'ground': array([-10., 5.]),
    'stove': array([5., 10.]),
}

BLOCK_WIDTH = 2.
HAND_HEIGHT = 1.

GRASPS = [
    -array([0., BLOCK_WIDTH + HAND_HEIGHT/2.]), # Transformation from hand to block
]

##################################################

def pose_sampler(b, r):
    x1, x2 = REGIONS[r]
    while True:
        x = random.uniform(x1 + BLOCK_WIDTH/2.,
                           x2 - BLOCK_WIDTH/2.)
        p = array([x, 0.])
        yield [(p,)]

def grasp_generator(b):
    yield [(g,) for g in GRASPS]

def ik_solver(b, p, g):
    q = p - g
    yield [(q,)]

def motion_planner(q1, q2):
    x1, _ = q1
    x2, _ = q2
    y = 2*BLOCK_WIDTH + HAND_HEIGHT/2.
    w1 = array([x1, y])
    w2 = array([x2, y])
    t = [q1, w1, w2, q2]
    yield [(t,)]

def distance(q1, q2):
    x1, _ = q1
    x2, _ = q2
    return abs(x2 - x1)

##################################################

q0 = array([-5.,  6.])
initial_poses = {
    'b1': array([0., 0.]),
    'b2': array([-3.,  0.]),
}

initial = [
    ('Conf', q0),
    ('AtConf', q0),
    ('HandEmpty',),

    ('Stove', 'stove'),

    Equal((TOTAL_COST,), 0)
]
for r in REGIONS:
    initial += [
        ('Region', r),
    ]

for b, p0 in initial_poses.items():
    initial += [
        ('Block', b),
        ('Pose', b, p0),
        ('AtPose', b, p0),
    ]

##################################################

goal = ('and', ('Cooked', 'b1'),
               ('Holding', 'b1'),
               ('AtConf', q0),
        )

##################################################

parent_dir = os.path.dirname(__file__)
domain_pddl = read(os.path.join(parent_dir, 'domain.pddl'))
stream_pddl = read(os.path.join(parent_dir, 'stream.pddl'))

constant_map = {}
stream_map = {
    's-grasp': grasp_generator,
    's-region': pose_sampler,
    's-ik': ik_solver,
    's-motion': motion_planner,
    't-cfree': from_test(lambda *args: True),
    #'t-region': from_test(get_region_test(tamp_problem.regions)),
    'dist': distance,
}
#stream_map = 'debug'

problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, initial, goal)

##################################################

import numpy as np

# stripstream
# https://caelan.github.io/stripstream/tutorial.html
# https://github.com/caelan/stripstream/blob/master/scripts/run_tutorial.py
# https://github.com/caelan/stripstream/blob/master/scripts/run_pddl_tutorial.py

# ss
# https://github.com/caelan/ss/tree/master/examples/tutorial

args = create_parser().parse_args()
np.set_printoptions(precision=2)
set_deterministic(seed=0)
print('Random seed:', get_random_seed())

stream_info = {
    't-region': StreamInfo(eager=True, p_success=0),
}
#stream_info = {}

dump_pddlstream(problem)
solution = solve(problem, algorithm=args.algorithm, stream_info=stream_info,
                 planner='ff-wastar1', verbose=True)
print_solution(solution)
plan, cost, evaluations = solution

if plan is not None:
    initial_state = TAMPState(robot_confs={'': q0}, holding={}, block_poses=initial_poses)
    tamp_problem = TAMPProblem(initial=initial_state, regions=REGIONS,
                               goal_conf=q0, goal_regions={}, goal_cooked=['b1'])
    display_plan(tamp_problem, retime_plan(plan), time_step=0.01)
