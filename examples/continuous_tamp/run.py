#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats
import os
import numpy as np
import argparse

from pddlstream.algorithms.downward import TOTAL_COST
from pddlstream.algorithms.focused import solve_focused
from pddlstream.utils import clear_dir, ensure_dir

from examples.continuous_tamp.constraint_solver import cfree_motion_fn, get_optimize_fn, has_gurobi
from examples.continuous_tamp.primitives import get_pose_gen, collision_test, \
    distance_fn, inverse_kin_fn, get_region_test, plan_motion, PROBLEMS, \
    draw_state, get_random_seed, TAMPState, GROUND_NAME, SUCTION_HEIGHT

from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.search import ABSTRIPSLayer
from pddlstream.algorithms.visualization import VISUALIZATIONS_DIR
from pddlstream.language.constants import And, Equal, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.language.synthesizer import StreamSynthesizer
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo
from pddlstream.language.optimizer import OptimizerInfo
from pddlstream.utils import print_solution, user_input, read, INF, get_file_path, str_from_object

def pddlstream_from_tamp(tamp_problem, use_stream=True, use_optimizer=False):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    external_paths = []
    if use_stream:
        external_paths.append(get_file_path(__file__, 'stream.pddl'))
    if use_optimizer:
        external_paths.append(get_file_path(__file__, 'optimizer.pddl'))
    external_pddl = [read(path) for path in external_paths]

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
           [('Placeable', b, GROUND_NAME) for b in initial.block_poses.keys()] + \
           [('Placeable', b, r) for b, r in tamp_problem.goal_regions.items()] + \
           [('Region', r) for r in tamp_problem.goal_regions.values() + [GROUND_NAME]]

    goal_literals = [('In', b, r) for b, r in tamp_problem.goal_regions.items()] #+ [('HandEmpty',)]

    if tamp_problem.goal_conf is not None:
        goal_literals += [('AtConf', tamp_problem.goal_conf)]
    goal = And(*goal_literals)

    stream_map = {
        's-motion': from_fn(plan_motion),
        's-region': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        't-region': from_test(get_region_test(tamp_problem.regions)),
        's-ik': from_fn(inverse_kin_fn),
        'distance': distance_fn,

        't-cfree': from_test(lambda *args: not collision_test(*args)),
        'posecollision': collision_test, # Redundant
        'trajcollision': lambda *args: False,
    }
    if use_optimizer:
        stream_map.update({
            'gurobi': from_fn(get_optimize_fn(tamp_problem.regions)),
            'rrt': from_fn(cfree_motion_fn),
        })
    #stream_map = 'debug'

    return PDDLProblem(domain_pddl, constant_map, external_pddl, stream_map, init, goal)

##################################################

def apply_action(state, action):
    conf, holding, block_poses = state
    # TODO: don't mutate block_poses?
    name, args = action
    if name == 'move':
        _, traj, _ = args
        for conf in traj[1:]:
            yield TAMPState(conf, holding, block_poses)
    elif name == 'pick':
        holding, _, _ = args
        del block_poses[holding]
        yield TAMPState(conf, holding, block_poses)
    elif name == 'place':
        block, pose, _ = args
        holding = None
        block_poses[block] = pose
        yield TAMPState(conf, holding, block_poses)
    else:
        raise ValueError(name)

##################################################

def display_plan(tamp_problem, plan, display=True):
    from examples.continuous_tamp.viewer import ContinuousTMPViewer
    from examples.discrete_tamp.viewer import COLORS

    example_name = os.path.basename(os.path.dirname(__file__))
    directory = os.path.join(VISUALIZATIONS_DIR, example_name + '/')
    ensure_dir(directory)

    colors = dict(zip(sorted(tamp_problem.initial.block_poses.keys()), COLORS))
    viewer = ContinuousTMPViewer(SUCTION_HEIGHT, tamp_problem.regions, title='Continuous TAMP')
    state = tamp_problem.initial
    print()
    print(state)
    draw_state(viewer, state, colors)
    if display:
        user_input('Continue?')
    if plan is not None:
        for i, action in enumerate(plan):
            print(i, *action)
            for j, state in enumerate(apply_action(state, action)):
                print(i, j, state)
                draw_state(viewer, state, colors)
                viewer.save(os.path.join(directory, '{}_{}'.format(i, j)))
                if display:
                    user_input('Continue?')
    if display:
        user_input('Finish?')


def main(unit_costs=False, use_synthesizers=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='blocked', help='The name of the problem to solve')
    parser.add_argument('-d', '--deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-a', '--algorithm', default='focused', help='Specifies the algorithm')
    args = parser.parse_args()
    print('Arguments:', args)
    print('Costs: {} | Synthesizers: {}'.format(not unit_costs, use_synthesizers))

    np.set_printoptions(precision=2)
    if args.deterministic:
        seed = 0
        np.random.seed(seed)
    print('Random seed:', get_random_seed())
    if use_synthesizers and not has_gurobi():
        use_synthesizers = False
        print('Warning! use_synthesizers=True requires gurobipy. Setting use_synthesizers=False.')

    if args.problem not in PROBLEMS:
        raise ValueError(args.problem)
    print('Problem:', args.problem)
    problem_fn = PROBLEMS[args.problem]
    tamp_problem = problem_fn()
    print(tamp_problem)

    action_info = {
        #'move': ActionInfo(terminal=True),
        #'pick': ActionInfo(terminal=True),
        #'place': ActionInfo(terminal=True),
    }
    stream_info = {
        't-region': StreamInfo(eager=True, p_success=0), # bound_fn is None
        't-cfree': StreamInfo(eager=False, negate=True),
        #'distance': FunctionInfo(opt_fn=lambda *args: 1),
        #'gurobi': OptimizerInfo(p_success=0),
        #'rrt': OptimizerInfo(p_success=0),
    }
    hierarchy = [
        #ABSTRIPSLayer(pos_pre=['atconf']), #, horizon=1),
    ]

    synthesizers = [
        #StreamSynthesizer('cfree-motion', {'s-motion': 1, 'trajcollision': 0},
        #                  gen_fn=from_fn(cfree_motion_fn)),
        StreamSynthesizer('optimize', {'s-region': 1, 's-ik': 1,
                                       'posecollision': 0, 't-cfree': 0, 'distance': 0},
                          gen_fn=from_fn(get_optimize_fn(tamp_problem.regions))),
    ] if use_synthesizers else []

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    print('Initial:', str_from_object(pddlstream_problem.init))
    print('Goal:', str_from_object(pddlstream_problem.goal))
    pr = cProfile.Profile()
    pr.enable()
    if args.algorithm == 'focused':
        solution = solve_focused(pddlstream_problem, action_info=action_info, stream_info=stream_info,
                                 planner='ff-wastar1', max_planner_time=10, synthesizers=synthesizers, verbose=True,
                                 max_time=300, max_cost=INF, debug=False, hierarchy=hierarchy,
                                 effort_weight=1, search_sampling_ratio=0, # TODO: run with search_sampling_ratio=1
                                 unit_costs=unit_costs, postprocess=False, visualize=False)
    elif args.algorithm == 'incremental':
        solution = solve_incremental(pddlstream_problem, layers=1, hierarchy=hierarchy,
                                     unit_costs=unit_costs, verbose=False)
    else:
        raise ValueError(args.algorithm)

    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    if plan is not None:
        display_plan(tamp_problem, plan)

if __name__ == '__main__':
    main()
