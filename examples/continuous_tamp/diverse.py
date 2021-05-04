#!/usr/bin/env python

from __future__ import print_function

import argparse
from collections import namedtuple
from itertools import permutations, combinations

import numpy as np

from examples.continuous_tamp.primitives import draw_state, SUCTION_HEIGHT, GROUND_NAME
from examples.continuous_tamp.run import initialize, create_problem, dump_pddlstream
from examples.continuous_tamp.viewer import ContinuousTMPViewer
from examples.discrete_tamp.viewer import COLORS
from pddlstream.algorithms.constraints import PlanConstraints, WILD
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.language.object import DebugValue
from pddlstream.language.statistics import load_data
from pddlstream.language.stream import StreamInfo, DEBUG
from pddlstream.utils import read, get_file_path, value_or_id, INF, SEPARATOR


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

##################################################

Stats = namedtuple('Stats', ['p_success', 'overhead'])

STATS_PER_STREAM = {
    's-grasp':  Stats(p_success=1.0, overhead=0.1),
    's-region': Stats(p_success=0.5, overhead=0.1),
    't-cfree':  Stats(p_success=0.1, overhead=0.1),
    's-ik':     Stats(p_success=0.9, overhead=0.1),
    's-motion': Stats(p_success=0.99, overhead=0.1),
}

def dump_statistics(name='continuous-tamp'):
    data = load_data(name)
    stats_from_stream = {}
    for name, statistics in data.items():
        attempts = statistics['calls']
        if not attempts:
            continue
        p_success = float(statistics['successes']) / attempts
        overhead = float(statistics['overhead']) / attempts
        print('External: {} | n: {:d} | p_success: {:.3f} | overhead: {:.3f}'.format(
            name, attempts, p_success, overhead))
        stats_from_stream[name] = Stats(p_success, overhead)
    print(stats_from_stream)
    return stats_from_stream

StreamValue = namedtuple('StreamValue', ['stream', 'inputs', 'output'])

def get_stream_value(value):
    # TODO: dynamic programming version
    if not isinstance(value, DebugValue):
        return value_or_id(value)
    return StreamValue(value.stream, tuple(map(get_stream_value, value.input_values)), value.output_parameter)

def p_conjunction(stream_plans, stats_from_stream):
    return np.product([stats_from_stream[stream].p_success
                       for stream, _ in set.union(*stream_plans)])

def p_disjunction(stream_plans, stats_from_stream):
    # Inclusion exclusion
    # TODO: dynamic programming method for this?
    # TODO: incorporate overhead
    p = 0.
    for i in range(len(stream_plans)):
        r = i + 1
        for subset_plans in combinations(stream_plans, r=r):
            sign = -1 if r % 2 == 0 else +1
            p += sign*p_conjunction(subset_plans, stats_from_stream)
    return p

##################################################

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
    stream_map = DEBUG # TODO: reuse same streams

    stream_info = {
        't-region': StreamInfo(eager=False, p_success=0),  # bound_fn is None
    }

    streams_from_problem = {}
    for order in permutations(blocks):
        for region in regions: # Objects could also have separate goals
            print(SEPARATOR)
            print('Block order:', order)
            print('Goal region:', region)
            #tamp_problem = dual(n_blocks=args.number, goal_regions=[region])
            tamp_problem.goal_regions.update({block: region for block in order})

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

            # TODO: params not sufficient if no output stream or output not in plan
            streams = set()
            for name, params in plan:
                for param in params:
                    value = get_stream_value(param)
                    if isinstance(value, StreamValue):
                        # TODO: propagate recursively
                        streams.add((value.stream, value.inputs))
            streams_from_problem[order, region] = streams
            #print(streams)
            #user_input()

    print(SEPARATOR)
    dump_statistics()

    print(SEPARATOR)
    best_problems, best_p = None, -INF
    for problems in combinations(streams_from_problem, r=2): # Make r a parameter
        stream_plans = [streams_from_problem[problem] for problem in problems]
        intersection = set.intersection(*stream_plans)
        p = p_disjunction(stream_plans, STATS_PER_STREAM)
        assert 0 <= p <= 1
        print('Problems: {} | Intersection: {} | p={:.3f}'.format(problems, len(intersection), p)) #, intersection)
        if p > best_p:
            best_problems, best_p = problems, p

    print('\nBest: {} (p={:.3f})'.format(best_problems, best_p))


if __name__ == '__main__':
    main()
