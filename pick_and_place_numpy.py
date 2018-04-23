#!/usr/bin/env python

from __future__ import print_function

from pddlstream.conversion import AND, EQ
from pddlstream.fast_downward import TOTAL_COST
from pddlstream.incremental import solve_exhaustive, solve_current, solve_incremental
from pddlstream.stream import from_gen_fn, from_fn
from pick_and_place_countable import DOMAIN_PDDL, STREAM_PDDL
from pddlstream.utils import print_solution
import numpy as np

def get_problem1(n_blocks=2, n_poses=5):
    assert(n_blocks + 1 <= n_poses)
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_blocks+1)]
    conf = np.array([0, 5])
    grasp = np.array([0, 1])

    #objects = []
    init = [
        ('Conf', conf),
        ('AtConf', conf),
        ('HandEmpty',),
        (EQ, (TOTAL_COST,), 0),
    ]
    init += [('Block', b) for b in blocks]
    init += [('Pose', p) for p in poses]
    init += [('AtPose', b, p) for b, p in zip(blocks, poses)]

    #goal = (AND,
    #        ('AtPose', blocks[0], poses[1]),
    #        ('AtConf', conf))
    goal = ('AtPose', blocks[0], poses[1])

    domain_pddl = DOMAIN_PDDL
    stream_pddl = STREAM_PDDL
    stream_map = {
        'sample-pose': from_gen_fn(lambda: ((np.array([x, 0]),) for x in range(len(poses), n_poses))),
        'inverse-kinematics':  from_fn(lambda p: (p + grasp,)),
    }
    constant_map = {}

    return init, goal, domain_pddl, stream_pddl, stream_map, constant_map

def main():
    problem = get_problem1()
    solution = solve_exhaustive(problem, verbose=True, debug=False)
    #solution = solve_incremental(problem)
    print_solution(solution)

if __name__ == '__main__':
    main()
