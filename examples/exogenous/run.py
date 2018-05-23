#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats

from pddlstream.incremental import solve_incremental
from pddlstream.utils import print_solution, read, get_file_path
from pddlstream.stream import from_fn

def pddlstream_from_belief():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    stream_map = {
        'inverse-kinematics': from_fn(lambda p: (p,)), # TODO: make crash
    }

    # Options
    # - observation produces one of several poses
    # - always at the pose, observation just makes it observable
    # - object has a unobserved fluent


    block = 'block1'
    pose = None # Unknown
    conf = 0

    init = [
        ('Initial',), # Forces move first
        ('Conf', conf),
        ('AtConf', conf),
        ('HandEmpty',),

        ('Block', block),
        ('Pose', pose),
        ('AtPose', block, pose),
        ('Latent', pose),
    ]

    goal = ('Holding', block)

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def main():
    pddlstream_problem = pddlstream_from_belief()
    _, _, _, _, init, goal = pddlstream_problem
    print(sorted(init))
    print(goal)
    pr = cProfile.Profile()
    pr.enable()
    solution = solve_incremental(pddlstream_problem, unit_costs=False)
    print_solution(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)

if __name__ == '__main__':
    main()
