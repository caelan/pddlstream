#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import pstats

import numpy as np
from numpy import array

from examples.continuous_tamp.primitives import get_random_seed, get_tight_problem
from examples.continuous_tamp.run import pddlstream_from_tamp
from pddlstream.algorithms.satisfaction import constraint_satisfaction, dump_assignment

# Be careful about uniqueness here
conf0 = array([-7.5,  5. ])
pose0 = array([0, 0])
pose1 = array([-3,  0])

init = [
    # TODO: use problem.init instead
    #('=', ('total-cost',), 0),
    #('atconf', conf0),
    #('atpose', 'b0', pose0),
    #('atpose', 'b1', pose1),
    ('block', 'b0'),
    ('block', 'b1'),
    #('canmove',),
    ('conf', conf0),
    ('contained', 'b0', pose0, 'grey'),
    ('contained', 'b1', pose1, 'grey'),
    #('handempty',),
    ('placeable', 'b0', 'grey'),
    ('placeable', 'b0', 'red'),
    ('placeable', 'b1', 'grey'),
    ('placeable', 'b1', 'red'),
    ('pose', 'b0', pose0),
    ('pose', 'b1', pose1),
    ('region', 'grey'),
    ('region', 'red'),
]

constraints = [
    ('cfree', 'b0', '?p0', 'b1', pose1),
    ('cfree', 'b1', '?p1', 'b0', '?p0'),
    ('conf', '?q0'),
    ('conf', '?q1'),
    ('conf', '?q2'),
    ('conf', '?q3'),
    ('contained', 'b0', '?p0', 'red'),
    ('contained', 'b1', '?p1', 'red'),
    ('kin', 'b0', '?q0', pose0),
    ('kin', 'b0', '?q1', '?p0'),
    ('kin', 'b1', '?q2', '?p1'),
    ('kin', 'b1', '?q3', pose1),
    ('motion', '?q0', '?t1', '?q1'),
    ('motion', '?q1', '?t3', '?q3'),
    ('motion', '?q3', '?t2', '?q2'),
    ('motion', conf0, '?t0', '?q0'),
    ('pose', 'b0', '?p0'),
    ('pose', 'b1', '?p1'),
    ('traj', '?t0'),
    ('traj', '?t1'),
    ('traj', '?t2'),
    ('traj', '?t3'),
]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-o', '--optimizer', action='store_true', help='Uses the optimizers')
    args = parser.parse_args()
    print('Arguments:', args)

    np.set_printoptions(precision=2)
    if args.deterministic:
        seed = 0
        np.random.seed(seed)
    print('Random seed:', get_random_seed())
    tamp_problem = get_tight_problem()
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem, use_stream=not args.optimizer,
                                              use_optimizer=args.optimizer)
    stream_pddl, stream_map = pddlstream_problem[2:4]
    pr = cProfile.Profile()
    pr.enable()
    solution = constraint_satisfaction(stream_pddl, stream_map, init, constraints)
    dump_assignment(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    #if plan is not None:
    #    display_plan(tamp_problem, plan)


if __name__ == '__main__':
    main()

