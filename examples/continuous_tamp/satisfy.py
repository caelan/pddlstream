#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import pstats

import numpy as np
from numpy import array

from examples.continuous_tamp.primitives import get_random_seed, get_tight_problem, MOVE_COST
from examples.continuous_tamp.run import pddlstream_from_tamp, display_plan
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo
from pddlstream.language.constants import Not, Minimize, is_parameter
from pddlstream.algorithms.satisfaction import dump_assignment, solve_pddlstream_satisfaction
from pddlstream.algorithms.satisfaction2 import constraint_satisfaction

# Be careful about uniqueness here
CONF0 = array([-7.5, 5.])
POSE0 = array([0, 0])
POSE1 = array([-3, 0])

INIT = [
    # TODO: use problem.init instead
    ('block', 'b0'),
    ('block', 'b1'),
    ('conf', CONF0),
    ('contained', 'b0', POSE0, 'grey'),
    ('contained', 'b1', POSE1, 'grey'),
    ('placeable', 'b0', 'grey'),
    ('placeable', 'b0', 'red'),
    ('placeable', 'b1', 'grey'),
    ('placeable', 'b1', 'red'),
    ('pose', 'b0', POSE0),
    ('pose', 'b1', POSE1),
    ('region', 'grey'),
    ('region', 'red'),
]

# TODO: predicate (Not) constraints as well
CONSTRAINTS = [
    ('cfree', 'b0', '?p0', 'b1', POSE1),
    ('cfree', 'b1', '?p1', 'b0', '?p0'),

    ('contained', 'b0', '?p0', 'red'),
    ('contained', 'b1', '?p1', 'red'),

    ('kin', 'b0', '?q0', POSE0),
    ('kin', 'b0', '?q1', '?p0'),
    ('kin', 'b1', '?q2', '?p1'),
    ('kin', 'b1', '?q3', POSE1),

    ('motion', CONF0, '?t0', '?q0'),
    ('motion', '?q0', '?t1', '?q1'),
    ('motion', '?q1', '?t3', '?q3'),
    ('motion', '?q3', '?t2', '?q2'),

    #('conf', '?q0'),
    #('conf', '?q1'),
    #('conf', '?q2'),
    #('conf', '?q3'),
    #('pose', 'b0', '?p0'),
    #('pose', 'b1', '?p1'),
    #('traj', '?t0'),
    #('traj', '?t1'),
    #('traj', '?t2'),
    #('traj', '?t3'),
]

SKELETON = [
    ('move', [CONF0, '?t0', '?q0']),
    ('pick', ['b0', POSE0, '?q0']),
    ('move', ['?q0', '?t1', '?q1']),
    ('place', ['b0', '?p0', '?q1']),

    ('move', ['?q1', '?t3', '?q3']),
    ('pick', ['b1', POSE1, '?q3']),
    ('move', ['?q3', '?t2', '?q2']),
    ('place', ['b1', '?p1', '?q2']),
]

OBJECTIVES = [
    Minimize(('distance', CONF0, '?q0')),
    Minimize(('distance', '?q0', '?q1')),
    Minimize(('distance', '?q1', '?q3')),
    Minimize(('distance', '?q3', '?q2')),
]

def set_deterministic(seed=0):
    np.random.seed(seed)

def main(success_cost=0):
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-a', '--algorithm', default='', help='Specifies the algorithm')
    parser.add_argument('-o', '--optimizer', action='store_true', help='Uses the optimizers')
    parser.add_argument('-t', '--max_time', default=30, type=int, help='The max time')
    parser.add_argument('-u', '--unit', action='store_true', help='Uses unit costs')
    args = parser.parse_args()
    print('Arguments:', args)

    np.set_printoptions(precision=2)
    if args.deterministic:
        set_deterministic()
    print('Random seed:', get_random_seed())
    tamp_problem = get_tight_problem(n_blocks=2, n_goals=2)
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem, use_stream=not args.optimizer,
                                              use_optimizer=args.optimizer)
    stream_pddl, stream_map = pddlstream_problem[2:4]
    stream_info = {
        't-region': StreamInfo(eager=True, p_success=0), # bound_fn is None
        #'t-cfree': StreamInfo(eager=False, negate=True),
        #'distance': FunctionInfo(opt_fn=lambda q1, q2: MOVE_COST), # Doesn't make a difference
    }

    terms = CONSTRAINTS + OBJECTIVES
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
    display_plan(tamp_problem, plan)

if __name__ == '__main__':
    main()
