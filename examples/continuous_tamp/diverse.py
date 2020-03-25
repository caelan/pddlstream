#!/usr/bin/env python

from __future__ import print_function

from itertools import permutations, combinations

import argparse
import cProfile
import os
import pstats
import random
import numpy as np
import time

from examples.continuous_tamp.optimizer.optimizer import cfree_motion_fn, get_optimize_fn
from examples.continuous_tamp.primitives import get_pose_gen, collision_test, distance_fn, inverse_kin_fn, \
    get_region_test, plan_motion, PROBLEMS, draw_state, get_random_seed, \
    GROUND_NAME, SUCTION_HEIGHT, MOVE_COST, GRASP, update_state, dual, GROUND_NAME
from pddlstream.algorithms.constraints import PlanConstraints, WILD
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.visualization import VISUALIZATIONS_DIR
from pddlstream.language.external import never_defer, defer_unique, defer_shared, get_defer_all_unbound, get_defer_any_unbound
from pddlstream.language.constants import And, Equal, PDDLProblem, TOTAL_COST, print_solution, Or
from pddlstream.language.conversion import objects_from_values
from pddlstream.language.function import FunctionInfo
from pddlstream.language.object import DebugValue
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_test, from_fn
from pddlstream.language.stream import StreamInfo, DEBUG
from pddlstream.language.temporal import get_end, compute_duration, retime_plan
from pddlstream.utils import ensure_dir, safe_rm_dir, user_input, read, INF, get_file_path, str_from_object, \
    sorted_str_from_list, implies, inclusive_range, flatten
from pddlstream.language.statistics import load_data

from pddlstream.algorithms.serialized import SEPARATOR

from examples.continuous_tamp.run import initialize, create_problem, dump_pddlstream
from examples.continuous_tamp.viewer import ContinuousTMPViewer
from examples.discrete_tamp.viewer import COLORS

def create_skeleton(robot, blocks, home=False):
    # TODO: constrain the placement region
    skeleton = []
    for block in blocks:
        skeleton.extend([
            ('move', [robot, WILD, WILD, WILD]),
            ('pick', [robot, block, WILD, WILD, WILD]),
            ('move', [robot, WILD, WILD, WILD]),
            ('place', [robot, block, WILD, WILD, WILD]),
        ])
    if home:
        skeleton.append(('move', [robot, WILD, WILD, WILD]))
    return skeleton


def dump_statistics(name='continuous-tamp'):
    data = load_data(name)
    for name, statistics in data.items():
        attempts = statistics['calls']
        if not attempts:
            continue
        p_success = float(statistics['successes']) / attempts
        overhead = float(statistics['overhead']) / attempts
        print('External: {} | n: {:d} | p_success: {:.3f} | overhead: {:.3f}'.format(
            name, attempts, p_success, overhead))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--algorithm', default='focused', help='Specifies the algorithm')
    parser.add_argument('-s', '--skeleton', action='store_true', help='Enforces skeleton plan constraints')

    tamp_problem, args = initialize(parser)

    colors = dict(zip(sorted(tamp_problem.initial.block_poses.keys()), COLORS))
    viewer = ContinuousTMPViewer(SUCTION_HEIGHT, tamp_problem.regions, title='Continuous TAMP')
    draw_state(viewer, tamp_problem.initial, colors)

    [robot] = tamp_problem.initial.robot_confs
    blocks = sorted(tamp_problem.initial.block_poses)
    regions = sorted(set(tamp_problem.regions) - {GROUND_NAME})
    print(robot)
    print(blocks)
    print(regions)
    print(tamp_problem)

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}
    stream_map = DEBUG

    stream_info = {
        't-region': StreamInfo(eager=False, p_success=0),  # bound_fn is None
    }

    streams_from_problem = {}
    for order in permutations(blocks):
        for region in regions: # Objects could also have separate goals
            print(SEPARATOR)
            print('Block order:', order)
            print('Goal region:', region)
            tamp_problem = dual(n_blocks=args.number, goal_regions=[region])

            init, goal = create_problem(tamp_problem)
            problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
            dump_pddlstream(problem)

            skeleton = create_skeleton(robot, order, home=tamp_problem.goal_conf is not None)
            print(skeleton)
            skeletons = [skeleton]
            #skeletons = None
            constraints = PlanConstraints(skeletons=skeletons, exact=True)

            solution = solve_focused(problem, constraints=constraints, stream_info=stream_info,
                                     planner='max-astar', max_planner_time=10, debug=False,
                                     max_time=args.max_time, verbose=True,
                                     unit_costs=True, unit_efforts=True, effort_weight=1)
            print_solution(solution)
            plan, cost, evaluations = solution
            assert plan is not None
            params = {param for name, params in plan for param in params if isinstance(param, DebugValue)}
            print(params)

            streams = set()
            for param in params:
                input_ids = tuple(map(id, param.input_values))
                key = (param.stream, input_ids)
                streams.add(key)
            streams_from_problem[order, region] = streams
            print(streams)
            #user_input()

    # TODO: kin should be the same for both
    print(SEPARATOR)
    for problem1, problem2 in combinations(streams_from_problem, r=2): # Make r a parameter
        print(problem1, problem2, streams_from_problem[problem1] & streams_from_problem[problem2])



if __name__ == '__main__':
    main()
