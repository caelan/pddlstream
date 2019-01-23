#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import os
import pstats

import numpy as np

from examples.continuous_tamp.constraint_solver import cfree_motion_fn, get_optimize_fn, has_gurobi
from examples.continuous_tamp.primitives import get_pose_gen, collision_test, unreliable_ik_fn, \
    distance_fn, inverse_kin_fn, get_region_test, plan_motion, PROBLEMS, \
    draw_state, get_random_seed, TAMPState, GROUND_NAME, SUCTION_HEIGHT, MOVE_COST
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.visualization import VISUALIZATIONS_DIR
from pddlstream.algorithms.constraints import PlanConstraints, WILD
from pddlstream.language.constants import And, Equal, PDDLProblem, TOTAL_COST, print_solution
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo
from pddlstream.language.synthesizer import StreamSynthesizer
from pddlstream.utils import ensure_dir
from pddlstream.utils import user_input, read, INF, get_file_path, str_from_object, implies


def pddlstream_from_tamp(tamp_problem, use_stream=True, use_optimizer=False, collisions=True):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    external_paths = []
    if use_stream:
        external_paths.append(get_file_path(__file__, 'stream.pddl'))
    if use_optimizer:
        external_paths.append(get_file_path(__file__, 'optimizer2.pddl')) # optimizer1 | optimizer2
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
        #'s-ik': from_gen_fn(unreliable_ik_fn),
        'distance': distance_fn,

        't-cfree': from_test(lambda *args: implies(collisions, not collision_test(*args))),
        #'posecollision': collision_test, # Redundant
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


def main(use_synthesizers=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='blocked', help='The name of the problem to solve')
    parser.add_argument('-a', '--algorithm', default='focused', help='Specifies the algorithm')
    parser.add_argument('-c', '--cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-d', '--deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-u', '--unit', action='store_true', help='Uses unit costs')
    parser.add_argument('-o', '--optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-s', '--skeleton', action='store_true', help='Enforces skeleton plan constraints')
    parser.add_argument('-t', '--max_time', default=20, type=int, help='The max time')
    args = parser.parse_args()
    print('Arguments:', args)
    print('Synthesizers: {}'.format(use_synthesizers))

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
        'distance': FunctionInfo(opt_fn=lambda q1, q2: MOVE_COST),
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


    skeleton = [
        ('move', ['?q0', WILD, '?q1']),
        ('pick', ['b1', '?p0', '?q1']),
        ('move', ['?q1', WILD, '?q2']),
        ('place', ['b1', '?p1', '?q2']),

        ('move', ['?q2', WILD, '?q3']),
        ('pick', ['b0', '?p2', '?q3']),
        ('move', ['?q3', WILD, '?q4']),
        ('place', ['b0', '?p3', '?q4']),
    ]
    skeletons = [skeleton] if args.skeleton else None
    max_cost = INF # 8*MOVE_COST
    constraints = PlanConstraints(skeletons=skeletons,
                                  #skeletons=[],
                                  #skeletons=[skeleton, []],
                                  exact=True,
                                  max_cost=max_cost)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem, collisions=not args.cfree, use_optimizer=True)
    print('Initial:', str_from_object(pddlstream_problem.init))
    print('Goal:', str_from_object(pddlstream_problem.goal))
    pr = cProfile.Profile()
    pr.enable()
    success_cost = 0 if args.optimal else INF
    if args.algorithm == 'focused':
        solution = solve_focused(pddlstream_problem, constraints=constraints,
                                 action_info=action_info, stream_info=stream_info, synthesizers=synthesizers,
                                 planner='ff-wastar1', max_planner_time=10, hierarchy=hierarchy, debug=False,
                                 max_time=args.max_time, max_iterations=INF, verbose=True,
                                 unit_costs=args.unit, success_cost=success_cost,
                                 unit_efforts=False, effort_weight=0,
                                 #search_sample_ratio=1,
                                 #max_skeletons=None,
                                 visualize=False)
    elif args.algorithm == 'incremental':
        solution = solve_incremental(pddlstream_problem, constraints=constraints,
                                     complexity_step=2, hierarchy=hierarchy,
                                     unit_costs=args.unit, success_cost=success_cost,
                                     max_time=args.max_time, verbose=False)
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
