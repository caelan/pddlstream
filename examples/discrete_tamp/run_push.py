#!/usr/bin/env python

from __future__ import print_function

import math
import os
from collections import namedtuple

import numpy as np
from pddlstream.algorithms.downward import TOTAL_COST
from pddlstream.algorithms.focused import solve_focused

from pddlstream.algorithms.incremental import solve_exhaustive
from pddlstream.language.conversion import And, Equal
from pddlstream.language.generator import from_gen_fn, from_fn, from_test, from_list_fn
from pddlstream.utils import print_solution, user_input, read
from examples.discrete_tamp.viewer import DiscreteTAMPViewer, COLORS
from examples.discrete_tamp.run import DiscreteTAMPState, DiscreteTAMPProblem, \
    draw_state, apply_action, apply_plan, GRASP

# Unit pushes
DIRECTIONS = [
    np.array([+1, 0]),
    #np.array([-1, 0]),
]
MAX_PUSH = 5

# TODO: dynamically introduce object

def collision_test(p1, p2):
    return np.linalg.norm(p2 - p1) <= 1e-1

def distance_fn(q1, q2):
    ord = 1  # 1 | 2
    return int(math.ceil(np.linalg.norm(q2 - q1, ord=ord)))

def ik_fn(p):
    q = p + GRASP
    return (q,)

def push_direction_fn(p1):
    q1, = ik_fn(p1)
    p2 = p1 + DIRECTIONS[0] # TODO: ensure legal
    q2, = ik_fn(p2)
    outputs = [(q1, p2, q2)]
    #facts = [('Kin', q, p1)]
    facts = []
    return outputs, facts

def push_target_fn(p1, p2):
    q1, = ik_fn(p1)
    q2, = ik_fn(p2)
    outputs = [(q1, q2)]
    facts = []
    return outputs, facts

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    known_poses = list(initial.block_poses.values()) + \
                  list(tamp_problem.goal_poses.values())

    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain_push.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream_push.pddl'))
    constant_map = {}

    init = [
        ('CanMove',),
        ('Conf', initial.conf),
        ('AtConf', initial.conf),
        ('HandEmpty',),
        Equal((TOTAL_COST,), 0)] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Pose', p) for p in known_poses] + \
           [('AtPose', b, p) for b, p in initial.block_poses.items()]

    goal = And(('AtConf', initial.conf), *[
        ('AtPose', b, p) for b, p in tamp_problem.goal_poses.items()
    ])

    # TODO: convert to lower case
    stream_map = {
        'push-target': from_list_fn(push_target_fn),
        #'push-direction': from_list_fn(push_direction_fn),
        'test-cfree': from_test(lambda *args: not collision_test(*args)),
        'distance': distance_fn,
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def get_shift_one_problem(n_blocks=1, n_poses=9):
    assert(1 <= n_blocks <= n_poses)
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]
    conf = np.array([0, -5])

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(conf, None, block_poses)
    goal_poses = {blocks[0]: poses[1]}

    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)
    #return DiscreteTAMPProblem(initial, poses, goal_poses)

##################################################

def main(focused=False, unit_costs=False):
    problem_fn = get_shift_one_problem # get_shift_one_problem | get_shift_all_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=unit_costs)
    else:
        solution = solve_exhaustive(pddlstream_problem, unit_costs=unit_costs)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return
    apply_plan(tamp_problem, plan)

if __name__ == '__main__':
    main()
