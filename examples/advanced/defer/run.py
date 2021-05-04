#!/usr/bin/env python

from __future__ import print_function

import numpy as np

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.generator import from_fn, from_constant, from_test, universe_test, empty_test
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, get_file_path, Profiler
from pddlstream.language.constants import print_solution, PDDLProblem
from pddlstream.language.external import defer_unique, never_defer
#from examples.advanced.exogenous.run import ik_fn, motion_fn

GRASP = np.array([0, 1])

def ik_fn(p):
    q = p + GRASP
    return (q,)

def motion_fn(q1, q2):
    times = [0, 1] # linspace, arange
    t = [w * (q2 - q1) + q1 for w in times]
    return (t,)

##################################################

def get_pddlstream():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    stream_map = {
        'test-pose': from_test(empty_test), # universe_test | empty_test
        'sample-pose': from_constant((np.array([2, 0]),)),
        'inv-kin': from_fn(ik_fn),
        'motion': from_fn(motion_fn),
    }

    block = 'block1'
    region = 'region1'
    pose = np.array([1, 0])
    conf = np.array([0, 0])

    init = [
        ('Conf', conf),
        ('AtConf', conf),
        ('HandEmpty',),

        ('Block', block),
        ('Pose', pose),
        ('AtPose', block, pose),
        ('Region', region),
    ]

    goal = ('In', block, region)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

##################################################

def main(replan=True, defer=True):
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    problem = get_pddlstream()

    stream_info = {
        #'test-pose': StreamInfo(eager=True, p_success=0),
        'motion': StreamInfo(defer_fn=defer_unique if defer else never_defer),
    }
    replan_actions = {'pick'} if replan else set()

    with Profiler():
        solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit,
                         stream_info=stream_info, replan_actions=replan_actions)
    print_solution(solution)


if __name__ == '__main__':
    main()
