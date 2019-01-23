#!/usr/bin/env python

from __future__ import print_function

import math
import os
import numpy as np

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution, Fact
from pddlstream.language.generator import from_test, from_list_fn
from pddlstream.language.stream import WildOutput
from pddlstream.utils import read
from examples.discrete_tamp.run import DiscreteTAMPState, DiscreteTAMPProblem, apply_plan, GRASP, \
    distance_fn, collision_test, get_length, get_difference, is_valid
from examples.discrete_tamp.viewer import MAX_COLS

#array = np.array
#array = tuple
array = lambda args: tuple(map(int, args))

# Unit pushes
DIRECTIONS = [
    np.array([+1, 0]),
    np.array([-1, 0]),
]
MAX_PUSH = 3

# TODO: dynamically introduce object

def ik_fn(p):
    q = array(GRASP + p)
    return (q,)

def get_push_poses(p1, p2):
    push_difference = get_difference(p1, p2)
    push_distance = get_length(push_difference)
    if push_distance == 0:
        return [p1, p2]
    push_unit = push_difference / push_distance
    num_pushes = int(math.ceil(push_distance))
    waypoint_poses = [array(i*push_unit + p1) for i in range(1, num_pushes)]
    all_poses = [p1] + waypoint_poses + [p2]
    return all_poses
    #return list(filter(is_valid, all_poses))

def get_push_confs(poses):
    return [ik_fn(p)[0] for p in poses]

def get_push_facts(poses, confs):
    return [Fact('Push', args) for args in zip(poses, confs, poses[1:], confs[1:])]

def push_target_fn(p1, p2):
    poses = get_push_poses(p1, p2)
    confs = get_push_confs(poses)
    facts = get_push_facts(poses, confs)
    if 2 <= len(poses):
        return WildOutput([], facts)
    return WildOutput([tuple(confs)], facts)

def push_direction_gen_fn(p1):
    # TODO: if any movable objects collide with the route, activate them by adding a predicate
    for direction in DIRECTIONS:
        p2 = array(MAX_PUSH*direction + p1)
        poses = get_push_poses(p1, p2)
        confs = get_push_confs(poses)
        #facts = get_push_facts(poses, confs)
        facts = []
        #yield [], facts
        yield WildOutput([(confs[0], poses[1], confs[1])], facts)

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
           [('AtPose', b, p) for b, p in initial.block_poses.items()] + \
           [('GoalPose', p) for p in tamp_problem.goal_poses.values()]

    goal = And(('AtConf', initial.conf), *[
        ('AtPose', b, p) for b, p in tamp_problem.goal_poses.items()
    ])

    # TODO: convert to lower case
    stream_map = {
        'push-target': from_list_fn(push_target_fn),
        'push-direction': push_direction_gen_fn,
        'test-cfree': from_test(lambda *args: not collision_test(*args)),
        'distance': distance_fn,
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def get_shift_one_problem(n_blocks=2, n_poses=MAX_COLS):
    assert(1 <= n_blocks <= n_poses)
    blocks = ['b{}'.format(i) for i in range(n_blocks)]
    poses = [array([x, 0]) for x in range(n_poses)]
    conf = array([0, -5])

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(conf, None, block_poses)
    goal_poses = {blocks[0]: poses[2]}

    return DiscreteTAMPProblem(initial, poses, goal_poses)

##################################################

def main(focused=True, unit_costs=False):
    problem_fn = get_shift_one_problem # get_shift_one_problem | get_shift_all_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=unit_costs)
    else:
        solution = solve_incremental(pddlstream_problem, unit_costs=unit_costs)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return
    apply_plan(tamp_problem, plan)

if __name__ == '__main__':
    main()
