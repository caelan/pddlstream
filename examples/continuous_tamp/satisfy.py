#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import pstats

import numpy as np
from numpy import array

from examples.continuous_tamp.primitives import get_random_seed, tight, MOVE_COST, GRASP
from examples.continuous_tamp.run import pddlstream_from_tamp, display_plan, set_deterministic
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo
from pddlstream.language.constants import Not, Minimize, is_parameter
from pddlstream.retired.satisfaction import solve_pddlstream_satisfaction
from pddlstream.algorithms.satisfaction import constraint_satisfaction, dump_assignment
from pddlstream.language.temporal import retime_plan

# Be careful about uniqueness here
#CONF0 = array([-7.5, 5.])
CONF0 = array([-5., 6.])
POSE0 = array([0, 0])
POSE1 = array([-3, 0])

INIT = [
    # TODO: use problem.init instead
    ('block', 'A'),
    ('block', 'B'),
    ('conf', CONF0),
    ('contain', 'A', POSE0, 'grey'),
    ('contain', 'B', POSE1, 'grey'),
    ('placeable', 'A', 'grey'),
    ('placeable', 'A', 'red'),
    ('placeable', 'B', 'grey'),
    ('placeable', 'B', 'red'),
    ('pose', 'A', POSE0),
    ('pose', 'B', POSE1),
    ('grasp', 'A', GRASP),
    ('grasp', 'B', GRASP),
    ('region', 'grey'),
    ('region', 'red'),
]

# TODO: predicate (Not) constraints as well
CONSTRAINTS = [
    ('cfree', 'A', '?p0', 'B', POSE1),
    ('cfree', 'B', '?p1', 'A', '?p0'),

    ('contain', 'A', '?p0', 'red'),
    ('contain', 'B', '?p1', 'red'),

    ('kin', 'A', '?q0', POSE0, '?g0'),
    ('kin', 'A', '?q1', '?p0', '?g0'),
    ('kin', 'B', '?q3', POSE1, '?g1'),
    ('kin', 'B', '?q2', '?p1', '?g1'),

    ('motion', CONF0, '?t0', '?q0'),
    ('motion', '?q0', '?t1', '?q1'),
    ('motion', '?q1', '?t3', '?q3'),
    ('motion', '?q3', '?t2', '?q2'),

    #('conf', '?q0'),
    #('conf', '?q1'),
    #('conf', '?q2'),
    #('conf', '?q3'),
    ('pose', 'A', '?p0'),
    ('pose', 'B', '?p1'),
    #('traj', '?t0'),
    #('traj', '?t1'),
    #('traj', '?t2'),
    #('traj', '?t3'),
]

SKELETON = [
    ('move', ['r0', CONF0, '?t0', '?q0']),
    ('pick', ['r0', 'A', POSE0, '?g0', '?q0']),
    ('move', ['r0', '?q0', '?t1', '?q1']),
    ('place', ['r0', 'A', '?p0', '?g0', '?q1']),

    ('move', ['r0', '?q1', '?t3', '?q3']),
    ('pick', ['r0', 'B', POSE1, '?g1', '?q3']),
    ('move', ['r0', '?q3', '?t2', '?q2']),
    ('place', ['r0', 'B', '?p1', '?g1', '?q2']),
]

OBJECTIVES = [
    Minimize(('dist', CONF0, '?q0')),
    Minimize(('dist', '?q0', '?q1')),
    Minimize(('dist', '?q1', '?q3')),
    Minimize(('dist', '?q3', '?q2')),
]

##################################################

def main(success_cost=0):
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-a', '--algorithm', default='', help='Specifies the algorithm')
    parser.add_argument('-g', '--gurobi', action='store_true', help='Uses gurobi')
    parser.add_argument('-t', '--max_time', default=30, type=int, help='The max time')
    parser.add_argument('-u', '--unit', action='store_true', help='Uses unit costs')
    args = parser.parse_args()
    print('Arguments:', args)

    np.set_printoptions(precision=2)
    if args.deterministic:
        set_deterministic()
    print('Random seed:', get_random_seed())
    tamp_problem = tight(n_robots=1, n_blocks=2, n_goals=2)
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem, use_stream=not args.gurobi,
                                              use_optimizer=args.gurobi)
    stream_pddl, stream_map = pddlstream_problem[2:4]
    stream_info = {
        't-region': StreamInfo(eager=True, p_success=0), # bound_fn is None
        #'t-cfree': StreamInfo(eager=False, negate=True),
        #'distance': FunctionInfo(opt_fn=lambda q1, q2: MOVE_COST), # Doesn't make a difference
    }
    print('Constraints:', CONSTRAINTS)
    print('Objectives:', OBJECTIVES)
    terms = CONSTRAINTS # + OBJECTIVES
    pr = cProfile.Profile()
    pr.enable()
    if args.algorithm == 'focused':
        solution = solve_pddlstream_satisfaction(stream_pddl, stream_map, INIT, terms,
                                                 incremental=False, stream_info=stream_info,
                                                 #search_sample_ratio=1,
                                                 #max_skeletons=1,
                                                 success_cost=success_cost, max_time=args.max_time)
    elif args.algorithm == 'incremental':
        solution = solve_pddlstream_satisfaction(stream_pddl, stream_map, INIT, terms, incremental=True,
                                                 success_cost=success_cost, max_time=args.max_time,
                                                 verbose=False, debug=False)
    else:
        # TODO: likely need to make GRASP a stream to solve
        solution = constraint_satisfaction(stream_pddl, stream_map, INIT, terms, stream_info=stream_info,
                                           costs=not args.unit, success_cost=success_cost,
                                           max_time=args.max_time, search_sample_ratio=1,
                                           debug=False)
        #raise ValueError(args.algorithm)

    dump_assignment(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    bindings, cost, evaluations = solution
    if bindings is None:
        return
    plan = []
    for name, args in SKELETON:
        new_args = [bindings[a] if is_parameter(a) else a for a in args]
        plan.append((name, new_args))
    display_plan(tamp_problem, retime_plan(plan))

if __name__ == '__main__':
    main()
