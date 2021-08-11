#!/usr/bin/env python

from __future__ import print_function

##################################################

import os
from numpy import array, random

# Global constants
BLOCK_WIDTH = 2.
HAND_HEIGHT = 1.
GY = BLOCK_WIDTH + HAND_HEIGHT/2.
GRASPS = [-array([0., GY])] # Transformation from hand to block
REGIONS = {'ground': array([-10., 5.]), 'stove': array([5., 10.])}

##################################################

# Generator streams

def pose_sampler(b, r):
    x1, x2 = REGIONS[r]
    while True:
        x = random.uniform(x1 + BLOCK_WIDTH/2., x2 - BLOCK_WIDTH/2.)
        p = array([x, 0.])
        yield [(p,)]

def grasp_generator(b):
    yield [(g,) for g in GRASPS]

def ik_solver(b, p, g):
    q = p - g # q + g = p
    yield [(q,)]

def motion_planner(q1, q2):
    x1, _ = q1
    x2, _ = q2
    y = 2*BLOCK_WIDTH + HAND_HEIGHT/2.
    w1 = array([x1, y]) # waypoint1
    w2 = array([x2, y]) # waypoint2
    t = [q1, w1, w2, q2] # path
    yield [(t,)]

##################################################

# Test streams

def collision_free_checker(b1, p1, b2, p2):
    assert b1 != b2
    x1, _ = p1
    x2, _ = p2
    collision = abs(x2 - x1) <= BLOCK_WIDTH
    if not collision:
        yield [tuple()] # empty tuple certifies

##################################################

# External functions

def distance(q1, q2):
    x1, _ = q1
    x2, _ = q2
    return abs(x2 - x1)

##################################################

# Initial state

q0 = array([-5.,  6.])
poses0 = {'b1': array([0., 0.]), 'b2': array([7.5, 0.])}

initial = [
    ('Conf', q0), # Static type property

    ('AtConf', q0),     # Fluent initial fact
    ('HandEmpty',),     # Fluent initial fact
    ('Stove', 'stove'), # Fluent initial fact

    ('=', ('total-cost',), 0.) # Fluent initial plan cost (optional)
]
for r in REGIONS:
    initial += [
        ('Region', r), # Static type property
    ]

for b, p0 in poses0.items():
    initial += [
        ('Body', b),       # Static type property
        ('Pose', b, p0),   # Static type property
        ('AtPose', b, p0), # Fluent initial fact
    ]

##################################################

# Goal formula
goal = ('and', ('Cooked', 'b1'),
               ('Holding', 'b1'),
               ('AtConf', q0),    # Return to initial conf
       )

##################################################

# Reading PDDL text files
parent_dir = os.path.dirname(__file__)
domain_pddl = open(os.path.join(parent_dir, 'domain.pddl')).read()
stream_pddl = open(os.path.join(parent_dir, 'stream.pddl')).read()

constant_map = {}
stream_map = {
    's-grasp': grasp_generator,
    's-region': pose_sampler,
    's-ik': ik_solver,
    's-motion': motion_planner,
    't-cfree': collision_free_checker,
    'dist': distance,
}

# The input to a PDDLStream planner
problem = (domain_pddl, constant_map, stream_pddl, stream_map, initial, goal)

##################################################

# ('CanMove',),   # Fluent initial fact (prevents the robot double moving)

# def test_contained(b, p, r):
#     x, y = p
#     x1, x2 = REGIONS[r]
#     contained = (x1 + BLOCK_WIDTH/2.) <= x <= (x2 - BLOCK_WIDTH/2.)
#     if contained:
#         yield [tuple()] # empty tuple certifies
#
# stream_map = {
#     't-region': test_contained,
# }

##################################################

import numpy as np

from examples.continuous_tamp.primitives import get_random_seed, TAMPProblem, TAMPState
from examples.continuous_tamp.run import set_deterministic, dump_pddlstream, display_plan
from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import print_solution, PDDLProblem
from pddlstream.language.temporal import retime_plan

# stripstream
# https://caelan.github.io/stripstream/tutorial.html
# https://github.com/caelan/stripstream/blob/master/scripts/run_tutorial.py
# https://github.com/caelan/stripstream/blob/master/scripts/run_pddl_tutorial.py

# ss
# https://github.com/caelan/ss/tree/master/examples/tutorial

args = create_parser().parse_args()
np.set_printoptions(precision=3)
set_deterministic(seed=0)
print('Random seed:', get_random_seed())
dump_pddlstream(PDDLProblem(*problem))

##################################################

# Solve the problem

solution = solve(problem, algorithm=args.algorithm, planner='ff-astar', verbose=True)
print_solution(solution)
plan, cost, evaluations = solution

##################################################

# Visualize the plan

if plan is not None:
    initial_state = TAMPState(robot_confs={'': q0}, holding={}, block_poses=poses0)
    tamp_problem = TAMPProblem(initial=initial_state, regions=REGIONS,
                               goal_conf=q0, goal_regions={}, goal_cooked=['b1'])
    display_plan(tamp_problem, retime_plan(plan), time_step=0.025)
