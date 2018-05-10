#!/usr/bin/env python

from __future__ import print_function

import os
from collections import namedtuple

import numpy as np

from examples.continuous_tamp.constraint_solver import get_constraint_solver
from examples.continuous_tamp.constraint_solver import get_optimize_fn, get_cfree_pose_fn, cfree_motion_fn
from examples.continuous_tamp.primitives import BLOCK_WIDTH, BLOCK_HEIGHT, get_pose_generator, collision_test, \
    distance_fn, inverse_kin_fn, get_region_test, rejection_sample_placed, plan_motion
from examples.discrete_tamp.viewer import COLORS
from pddlstream.macro_stream import DynamicStream
from pddlstream.conversion import And, Equal
from pddlstream.fast_downward import TOTAL_COST
from pddlstream.focused import solve_focused
from pddlstream.incremental import solve_incremental
from pddlstream.stream import from_fn, from_test, StreamInfo
from pddlstream.utils import print_solution, user_input, read, INF
from viewer import ContinuousTMPViewer, GROUND


def pddlstream_from_tamp(tamp_problem, constraint_solver=False):
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
           [('AtPose', b, p) for b, p in initial.block_poses.items()] + \
           [('Placeable', b, GROUND) for b in initial.block_poses.keys()] + \
           [('Placeable', b, r) for b, r in tamp_problem.goal_regions.items()]

    goal_literals = [('In', b, r) for b, r in tamp_problem.goal_regions.items()]
    if tamp_problem.goal_conf is not None:
        goal_literals += [('AtConf', tamp_problem.goal_conf)]
    goal = And(*goal_literals)

    stream_map = {
        #'sample-pose': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        'plan-motion': from_fn(plan_motion),
        'sample-pose': get_pose_generator(tamp_problem.regions),
        'test-region': from_test(get_region_test(tamp_problem.regions)),
        'inverse-kinematics':  from_fn(inverse_kin_fn),
        #'collision-free': from_test(lambda *args: not collision_test(*args)),
        'cfree': lambda *args: not collision_test(*args),
        'posecollision': collision_test,
        'trajcollision': lambda *args: False,
        'distance': distance_fn,
    }
    if constraint_solver:
        stream_map['constraint-solver'] = get_constraint_solver(tamp_problem.regions)

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

TAMPState = namedtuple('TAMPState', ['conf', 'holding', 'block_poses'])
TAMPProblem = namedtuple('TAMPProblem', ['initial', 'regions', 'goal_conf', 'goal_regions'])

def get_tight_problem(n_blocks=2, n_goals=2):
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
    goal_regions = {block: 'red' for block in blocks[:n_goals]}

    return TAMPProblem(initial, regions, conf, goal_regions)

##################################################

def get_blocked_problem():
    goal = 'red'
    regions = {
        GROUND: (-15, 15),
        goal: (5, 10)
    }

    conf = np.array([0, 5])
    blocks = ['block{}'.format(i) for i in range(2)]
    #poses = [np.zeros(2), np.array([7.5, 0])]
    #block_poses = dict(zip(blocks, poses))

    block_regions = {
        blocks[0]: GROUND,
        blocks[1]: goal,
    }
    block_poses = rejection_sample_placed(block_regions=block_regions, regions=regions)

    initial = TAMPState(conf, None, block_poses)
    goal_regions = {blocks[0]: 'red'}

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
    name, args = action
    if name == 'move':
        _, _, conf = args
    elif name == 'pick':
        holding, _, _ = args
        del block_poses[holding]
    elif name == 'place':
        block, pose, _ = args
        holding = None
        block_poses[block] = pose
    else:
        raise ValueError(name)
    return TAMPState(conf, holding, block_poses)

##################################################

def main(focused=True, deterministic=False):
    np.set_printoptions(precision=2)
    if deterministic:
        np.random.seed(0)

    problem_fn = get_tight_problem # get_tight_problem | get_blocked_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    action_info = {
        #'move': ActionInfo(terminal=True),
        #'pick': ActionInfo(terminal=True),
        #'place': ActionInfo(terminal=True),
    }
    stream_info = {
        'test-region': StreamInfo(eager=True, p_success=0), # bound_fn is None
        'plan-motion': StreamInfo(p_success=1),  # bound_fn is None
        #'cfree': StreamInfo(eager=True),
    }

    dynamic = [
        DynamicStream('cfree-motion', {'plan-motion': 1, 'trajcollision': 0},
                      gen_fn=from_fn(cfree_motion_fn)),
        #DynamicStream('cfree-pose', {'sample-pose': 1, 'posecollision': 0},
        #              gen_fn=from_fn(get_cfree_pose_fn(tamp_problem.regions))),
        DynamicStream('optimize', {'sample-pose': 1, 'inverse-kinematics': 1,
                                   'posecollision': 0, 'distance': 0},
                      gen_fn=from_fn(get_optimize_fn(tamp_problem.regions))),
    ]

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    if focused:
        solution = solve_focused(pddlstream_problem, action_info=action_info, stream_info=stream_info,
                                 dynamic_streams=dynamic,
                                 max_time=10, max_cost=INF, debug=False,
                                 commit=True, effort_weight=None, unit_costs=False, visualize=False)
    else:
        solution = solve_incremental(pddlstream_problem, layers=1, unit_costs=False)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return

    colors = dict(zip(sorted(tamp_problem.initial.block_poses.keys()), COLORS))
    viewer = ContinuousTMPViewer(tamp_problem.regions, title='Continuous TAMP')
    state = tamp_problem.initial
    print()
    print(state)
    draw_state(viewer, state, colors)
    for i, action in enumerate(plan):
        user_input('Continue?')
        print(i, action)
        state = apply_action(state, action)
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')


if __name__ == '__main__':
    main()
