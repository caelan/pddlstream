#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import os
import pstats
import random
import numpy as np

from examples.continuous_tamp.constraint_solver import cfree_motion_fn, get_optimize_fn
from examples.continuous_tamp.primitives import get_pose_gen, collision_test, distance_fn, inverse_kin_fn, \
    get_region_test, plan_motion, PROBLEMS, unreliable_ik_fn, \
    draw_state, get_random_seed, GROUND_NAME, SUCTION_HEIGHT, MOVE_COST, apply_action, GRASP
from pddlstream.algorithms.constraints import PlanConstraints, WILD
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.visualization import VISUALIZATIONS_DIR
from pddlstream.language.constants import And, Equal, PDDLProblem, TOTAL_COST, print_solution, Not, Exists
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_test, from_fn
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import ensure_dir, safe_rm_dir, user_input, read, INF, get_file_path, str_from_object, \
    sorted_str_from_list, implies


def pddlstream_from_tamp(tamp_problem, use_stream=True, use_optimizer=False, collisions=True):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    domain_pddl = read(get_file_path(__file__, 'temporal_domain.pddl'))
    external_paths = []
    if use_stream:
        external_paths.append(get_file_path(__file__, 'stream.pddl'))
    if use_optimizer:
        external_paths.append(get_file_path(__file__, 'optimizer.pddl')) # optimizer | optimizer_hard
    external_pddl = [read(path) for path in external_paths]

    robots = ['r{}'.format(r) for r in range(2)]

    constant_map = {'{}'.format(name).lower(): name
                    for name in initial.block_poses.keys() + tamp_problem.regions.keys()}
    #constant_map = {}
    init = [
        ('Safe',),
        ('Stove', 'red'),
        #Equal((TOTAL_COST,), 0), TODO: cannot do TOTAL_COST
    ]
    for r in robots:
        init += [
            ('Robot', r),
            ('CanMove', r),
            ('Conf', initial.conf),
            ('AtConf', r, initial.conf),
            ('HandEmpty', r),
        ]
    for b, p in initial.block_poses.items():
        init += [
            ('Block', b),
            ('Pose', b, p),
            ('Grasp', b, GRASP),
            ('AtPose', b, p),
            ('Placeable', b, GROUND_NAME),
        ]

    goal_literals = [
        #('Safe',), # Segfault for OPTIC
    ]
    for b, s in tamp_problem.goal_regions.items():
        if isinstance(s, str):
            init += [('Region', s), ('Placeable', b, s)]
            goal_literals += [('Cooked', b)]
            #goal_literals += [('On', b, s)]
            #goal_literals += [Exists(['?p'], And(('Contain', b, '?p', s), ('AtPose', b, '?p')))]
        else:
            init += [('Pose', b, s)]
            goal_literals += [('AtPose', b, s)]

    #goal_literals += [Not(('Unsafe',))] # ('HandEmpty',)
    #if tamp_problem.goal_conf is not None:
    #    goal_literals += [('AtConf', tamp_problem.goal_conf)]
    goal = And(*goal_literals)

    stream_map = {
        's-motion': from_fn(plan_motion),
        's-region': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        't-region': from_test(get_region_test(tamp_problem.regions)),
        's-ik': from_fn(inverse_kin_fn),
        #'s-ik': from_gen_fn(unreliable_ik_fn),
        'dist': distance_fn,

        't-cfree': from_test(lambda *args: implies(collisions, not collision_test(*args))),
    }
    if use_optimizer:
        # To avoid loading gurobi
        stream_map.update({
            'gurobi': from_list_fn(get_optimize_fn(tamp_problem.regions, collisions=collisions)),
            'rrt': from_fn(cfree_motion_fn),
        })
    #stream_map = 'debug'

    return PDDLProblem(domain_pddl, constant_map, external_pddl, stream_map, init, goal)

##################################################

def display_plan(tamp_problem, plan, display=True):
    from examples.continuous_tamp.viewer import ContinuousTMPViewer
    from examples.discrete_tamp.viewer import COLORS

    example_name = os.path.basename(os.path.dirname(__file__))
    directory = os.path.join(VISUALIZATIONS_DIR, example_name + '/')
    safe_rm_dir(directory)
    ensure_dir(directory)

    colors = dict(zip(sorted(tamp_problem.initial.block_poses.keys()), COLORS))
    viewer = ContinuousTMPViewer(SUCTION_HEIGHT, tamp_problem.regions, title='Continuous TAMP')
    state = tamp_problem.initial
    print()
    print(state)
    draw_state(viewer, state, colors)
    if display:
        user_input('Continue?')
    return
    if plan is not None:
        for i, action in enumerate(plan):
            print(i, *action)
            #action = (action[0], action[1][1:])
            for j, state in enumerate(apply_action(state, action)):
                print(i, j, state)
                draw_state(viewer, state, colors)
                viewer.save(os.path.join(directory, '{}_{}'.format(i, j)))
                if display:
                    user_input('Continue?')
    if display:
        user_input('Finish?')

##################################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='mirror', help='The name of the problem to solve')
    parser.add_argument('-a', '--algorithm', default='incremental', help='Specifies the algorithm')
    parser.add_argument('-c', '--cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-d', '--deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-g', '--gurobi', action='store_true', help='Uses gurobi')
    parser.add_argument('-n', '--number', default=1, type=int, help='The number of blocks')
    parser.add_argument('-o', '--optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-s', '--skeleton', action='store_true', help='Enforces skeleton plan constraints')
    parser.add_argument('-t', '--max_time', default=30, type=int, help='The max time')
    parser.add_argument('-u', '--unit', action='store_true', help='Uses unit costs')
    parser.add_argument('-v', '--visualize', action='store_true', help='Visualizes graphs')
    args = parser.parse_args()
    print('Arguments:', args)

    np.set_printoptions(precision=2)
    if args.deterministic:
        random.seed(seed=0)
        np.random.seed(seed=0)
    print('Random seed:', get_random_seed())

    problem_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if args.problem not in problem_from_name:
        raise ValueError(args.problem)
    print('Problem:', args.problem)
    problem_fn = problem_from_name[args.problem]
    tamp_problem = problem_fn(args.number)
    print(tamp_problem)

    #stream_info = {
    #    't-region': StreamInfo(eager=False, p_success=0), # bound_fn is None
    #    't-cfree': StreamInfo(eager=False, negate=True),
    #    'distance': FunctionInfo(opt_fn=lambda q1, q2: MOVE_COST),
    #    'gurobi-cfree': StreamInfo(eager=False, negate=True),
    #    #'gurobi': OptimizerInfo(p_success=0),
    #    #'rrt': OptimizerInfo(p_success=0),
    #}

    pddlstream_problem = pddlstream_from_tamp(tamp_problem, collisions=not args.cfree,
                                              use_stream=not args.gurobi, use_optimizer=args.gurobi)
    print('Constants:', str_from_object(pddlstream_problem.constant_map))
    print('Initial:', sorted_str_from_list(pddlstream_problem.init))
    print('Goal:', str_from_object(pddlstream_problem.goal))
    pr = cProfile.Profile()
    pr.enable()
    success_cost = 0 if args.optimal else INF
    planner = 'max-astar'
    #planner = 'ff-wastar1'
    if args.algorithm == 'focused':
        raise NotImplementedError()
        #solution = solve_focused(pddlstream_problem, constraints=constraints,
        #                         action_info=action_info, stream_info=stream_info,
        #                         planner=planner, max_planner_time=10, hierarchy=hierarchy, debug=False,
        #                         max_time=args.max_time, max_iterations=INF, verbose=True,
        #                         unit_costs=args.unit, success_cost=success_cost,
        #                         unit_efforts=False, effort_weight=0,
        #                         search_sample_ratio=0,
        #                         #max_skeletons=None, bind=True,
        #                         visualize=args.visualize)
    elif args.algorithm == 'incremental':
        solution = solve_incremental(pddlstream_problem,
                                     complexity_step=1, planner=planner,
                                     unit_costs=args.unit, success_cost=success_cost,
                                     max_time=args.max_time, verbose=False)
    else:
        raise ValueError(args.algorithm)

    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('cumtime').print_stats(20)
    if plan is not None:
        display_plan(tamp_problem, plan)

if __name__ == '__main__':
    main()
