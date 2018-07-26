#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats

import numpy as np
from pddlstream.algorithms.downward import TOTAL_COST
from pddlstream.algorithms.focused import solve_focused

from examples.continuous_tamp.constraint_solver import cfree_motion_fn, get_optimize_fn
from examples.continuous_tamp.primitives import get_pose_gen, collision_test, \
    distance_fn, inverse_kin_fn, get_region_test, plan_motion, \
    get_blocked_problem, draw_state, get_random_seed, \
    TAMPState, get_tight_problem
from examples.continuous_tamp.viewer import ContinuousTMPViewer, GROUND
from examples.discrete_tamp.viewer import COLORS
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.conversion import And, Equal
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.language.synthesizer import StreamSynthesizer
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import print_solution, user_input, read, INF, get_file_path


#def valid_state_fn(fluents, parameters):
#    new_fluents = set(fluents)
#    # TODO: could return action descriptions or could just return fluents
#    return

def reachable_test(q1, q2, fluents=[]):
    placed_blocks = {}
    holding_block = None
    for fluent in fluents:
        if fluent[0] == 'atpose':
            b, p = fluent[1:]
            placed_blocks[b] = p
        if fluent[0] == 'holding':
            b, = fluent[1:]
            holding_block = b
    print(q1, q2, holding_block, placed_blocks)
    #return True
    return False

##################################################

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    init = [
        ('CanMove',),
        ('Conf', initial.conf),
        ('AtConf', initial.conf),
        ('HandEmpty',),
        Equal((TOTAL_COST,), 0)] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Pose', b, p) for b, p in initial.block_poses.items()] + \
           [('AtPose', b, p) for b, p in initial.block_poses.items()] + \
           [('Placeable', b, GROUND) for b in initial.block_poses.keys()] + \
           [('Placeable', b, r) for b, r in tamp_problem.goal_regions.items()]

    goal_literals = [('HandEmpty',)] + \
                    [('In', b, r) for b, r in tamp_problem.goal_regions.items()]
    if tamp_problem.goal_conf is not None:
        goal_literals += [('AtConf', tamp_problem.goal_conf)]
    goal = And(*goal_literals)

    stream_map = {
        'plan-motion': from_fn(plan_motion),
        'sample-pose': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        'test-region': from_test(get_region_test(tamp_problem.regions)),
        'inverse-kinematics':  from_fn(inverse_kin_fn),
        'collision-free': from_test(lambda *args: not collision_test(*args)),
        'cfree': lambda *args: not collision_test(*args),
        'posecollision': collision_test,
        'trajcollision': lambda *args: False,
        'distance': distance_fn,
        'reachable': from_test(reachable_test),
        #'Valid': valid_state_fn,
    }
    #stream_map = 'debug'

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

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

def main(focused=True, deterministic=False, unit_costs=False, use_synthesizers=True):
    np.set_printoptions(precision=2)
    if deterministic:
        seed = 0
        np.random.seed(seed)
    print('Seed:', get_random_seed())

    problem_fn = get_tight_problem  # get_tight_problem | get_blocked_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    action_info = {
        #'move': ActionInfo(terminal=True),
        #'pick': ActionInfo(terminal=True),
        #'place': ActionInfo(terminal=True),
    }
    stream_info = {
        'test-region': StreamInfo(eager=True, p_success=0), # bound_fn is None
        #'cfree': StreamInfo(eager=True),
    }

    synthesizers = [
        #StreamSynthesizer('cfree-motion', {'plan-motion': 1, 'trajcollision': 0},
        #                  gen_fn=from_fn(cfree_motion_fn)),
        StreamSynthesizer('optimize', {'sample-pose': 1, 'inverse-kinematics': 1,
                                       'posecollision': 0, 'distance': 0},
                          gen_fn=from_fn(get_optimize_fn(tamp_problem.regions))),
    ] if use_synthesizers else []

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    pr = cProfile.Profile()
    pr.enable()
    if focused:
        solution = solve_focused(pddlstream_problem, action_info=action_info, stream_info=stream_info,
                                 synthesizers=synthesizers,
                                 max_time=10, max_cost=INF, debug=False,
                                 effort_weight=None, unit_costs=unit_costs, postprocess=False,
                                 visualize=False)
    else:
        solution = solve_incremental(pddlstream_problem, layers=1, unit_costs=unit_costs)
    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
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
        print(i, *action)
        state = apply_action(state, action)
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')


if __name__ == '__main__':
    main()
