#!/usr/bin/env python

from __future__ import print_function

import os
import numpy as np
import argparse
from collections import namedtuple

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_current, solve_exhaustive, solve_incremental
#from pddlstream.algorithms.execution import solve_execution
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_gen_fn, from_fn, from_test, from_list_fn, outputs_from_boolean
from pddlstream.utils import user_input, read
from examples.discrete_tamp.viewer import DiscreteTAMPViewer, COLORS, MAX_COLS, MAX_ROWS

# TODO: Can infer domain from usage or from specification

GRASP = np.array([0, 0])

def is_valid(p):
    return np.greater_equal(p, [0, 0]) and np.greater([MAX_COLS, MAX_ROWS], p)

def get_length(vec, ord=1):
    return np.linalg.norm(vec, ord=ord)

def get_difference(p1, p2):
    return np.array(p2) - p1

def collision_test(p1, p2):
    return get_length(get_difference(p1, p2)) < 1e-3

def noisy_collision_gen_fn(*args):
    while True:
        if np.random.random() < 0.75:
            yield outputs_from_boolean(False)
        else:
            yield outputs_from_boolean(not collision_test(*args))

def distance_fn(q1, q2):
    return get_length(get_difference(q1, q2))

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
        'q100': q100, # As an example
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

    # TODO: convert to lower case
    stream_map = {
        #'sample-pose': from_gen_fn(lambda: ((np.array([x, 0]),) for x in range(len(poses), n_poses))),
        'sample-pose': from_gen_fn(lambda: ((p,) for p in tamp_problem.poses)),
        'inverse-kinematics':  from_fn(lambda p: (p + GRASP,)),
        'test-cfree': from_test(lambda *args: not collision_test(*args)),
        #'test-cfree': from_gen_fn(noisy_collision_gen_fn),
        'collision': collision_test,
        #'constraint-solver': None,
        'distance': distance_fn,
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

DiscreteTAMPState = namedtuple('DiscreteTAMPState', ['conf', 'holding', 'block_poses'])
DiscreteTAMPProblem = namedtuple('DiscreteTAMPProblem', ['initial', 'poses', 'goal_poses'])

BLOCK_TEMPLATE = 'b{}'
INITIAL_CONF = np.array([0, -1])

def get_shift_one_problem(n_blocks=2, n_poses=9):
    assert(1 <= n_blocks <= n_poses)
    blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    goal_poses = {blocks[0]: poses[1]}

    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)
    #return DiscreteTAMPProblem(initial, poses, goal_poses)

def get_shift_all_problem(n_blocks=2, n_poses=9):
    assert(n_blocks + 1 <= n_poses)
    blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    goal_poses = dict(zip(blocks, poses[1:]))

    return DiscreteTAMPProblem(initial, poses[n_blocks+1:], goal_poses)

##################################################

def draw_state(viewer, state, colors):
    viewer.clear()
    viewer.draw_environment()
    viewer.draw_robot(*state.conf[::-1])
    for block, pose in state.block_poses.items():
        r, c = pose[::-1]
        viewer.draw_block(r, c, name=block, color=colors[block])
    if state.holding is not None:
        pose = state.conf - GRASP
        r, c = pose[::-1]
        viewer.draw_block(r, c, name=state.holding, color=colors[state.holding])


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
    elif name == 'push':
        block, _, _, pose, conf = args
        holding = None
        block_poses[block] = pose
    else:
        raise ValueError(name)
    return DiscreteTAMPState(conf, holding, block_poses)

def apply_plan(tamp_problem, plan):
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

##################################################

def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument('-p', '--problem', default='blocked', help='The name of the problem to solve')
    parser.add_argument('-a', '--algorithm', default='focused', help='Specifies the algorithm')
    parser.add_argument('-u', '--unit', action='store_true', help='Uses unit costs')
    args = parser.parse_args()
    print('Arguments:', args)

    problem_fn = get_shift_one_problem # get_shift_one_problem | get_shift_all_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    stream_info = {
        'test-cfree': StreamInfo(negate=True),
    }

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    if args.algorithm == 'focused':
        #solution = solve_execution(pddlstream_problem, unit_costs=unit_costs, stream_info=stream_info)
        solution = solve_focused(pddlstream_problem, unit_costs=args.unit, stream_info=stream_info, debug=False)
    elif args.algorithm == 'current':
        # Should fail to find a solution
        solution = solve_current(pddlstream_problem, unit_costs=args.unit)
    elif args.algorithm == 'exhaustive':
        solution = solve_exhaustive(pddlstream_problem, unit_costs=args.unit)
    elif args.algorithm == 'incremental':
        solution = solve_incremental(pddlstream_problem, unit_costs=args.unit)
    else:
        raise ValueError(args.algorithm)

    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return
    apply_plan(tamp_problem, plan)


if __name__ == '__main__':
    main()
