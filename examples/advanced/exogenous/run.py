#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats
import numpy as np

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_fn
from pddlstream.utils import read, get_file_path
from pddlstream.language.constants import print_solution


def pddlstream_from_belief():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    stream_map = {
        'inv-kin': from_fn(lambda p: (p + np.array([0, 1]),)),
        'motion': from_fn(lambda q1, q2: ([t*(q2-q1) + q1 for t in [0, 1]],)), # linspace, arange
    }

    # Options
    # - observation produces one of several poses
    # - always at the pose, observation just makes it observable
    # - object has a unobserved fluent

    block = 'block1'
    pose = None # Unknown
    #pose = np.array([1, 0])
    conf = np.array([0, 1])
    booth = np.array([0, 2])

    init = [
        ('Initial',), # Forces move first
        ('Conf', conf),
        ('Conf', booth),
        ('Booth', booth),
        ('AtConf', conf),
        ('HandEmpty',),

        ('Block', block),
        ('Pose', pose),
        ('AtPose', block, pose),
        #('Observable', pose),
        #('Latent', pose),
    ]

    goal = ('Holding', block)

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def main(focused=True):
    # TODO: maybe load problems as a domain explicitly
    pddlstream_problem = pddlstream_from_belief()
    _, _, _, _, init, goal = pddlstream_problem
    print('Init:', sorted(init, key=lambda f: f[0]))
    print('Goal:', goal)
    pr = cProfile.Profile()
    pr.enable()
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=False)
    else:
        #solution = solve_exhaustive(pddlstream_problem, unit_costs=False)
        solution = solve_incremental(pddlstream_problem, unit_costs=False)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(5)
    print_solution(solution)


if __name__ == '__main__':
    main()
