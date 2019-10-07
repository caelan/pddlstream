#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats
import numpy as np
import argparse

import pddlstream.language.exogenous as exogenous
exogenous.EXOGENOUS_AXIOMS = True
exogenous.REPLACE_STREAM = False

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_fn
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, get_file_path
from pddlstream.language.constants import print_solution
#from pddlstream.language.exogenous import FutureValue

class Latent(object):
    pass

class Uncertain(Latent):
    def __init__(self, mode=None, confidence=None):
        # Point estimate, interval estimate
        self.mode = mode
        self.confidence = confidence
    def __repr__(self):
        #return 'N(mu={})'.format(self.mode)
        return 'L({})'.format(self.mode)

class Derived(Latent):
    def __init__(self, fn, inputs):
        self.fn = fn
        self.inputs = tuple(inputs)
    def __repr__(self):
        return '{}({})'.format(self.fn.__name__, ', '.join(map(repr, self.inputs)))

##################################################

def ik_fn(*inputs):
    if any(isinstance(inp, Latent) for inp in inputs):
        q = Derived(ik_fn, inputs)
        return (q,)
    p, = inputs
    q = p + np.array([0, 1])
    return (q,)

def motion_fn(*inputs):
    if any(isinstance(inp, Latent) for inp in inputs):
        t = Derived(motion_fn, inputs)
        return (t,)
    q1, q2 = inputs
    # linspace, arange
    t = [y * (q2 - q1) + q1 for y in [0, 1]]
    return (t,)

##################################################

def pddlstream_from_belief():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    stream_map = {
        'inv-kin': from_fn(ik_fn),
        'motion': from_fn(motion_fn),
    }

    # Options
    # - observation produces one of several poses
    # - always at the pose, observation just makes it observable
    # - object has a unobserved fluent

    block = 'block1'
    #pose = None # Unknown
    pose = Uncertain()
    #pose = np.array([1, 0])
    conf = np.array([0, 1])
    booth = np.array([0, 2])

    init = [
        ('Initial',), # Forces move first
        ('Conf', conf),
        #('Booth', conf),
        #('Conf', booth),
        #('Booth', booth),
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

def main(planner='max-astar', unit_costs=True):
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--algorithm', default='focused', help='Specifies the algorithm')
    args = parser.parse_args()
    print('Arguments:', args)

    pddlstream_problem = pddlstream_from_belief()
    _, _, _, _, init, goal = pddlstream_problem
    print('Init:', sorted(init, key=lambda f: f[0]))
    print('Goal:', goal)
    replan_actions = set()
    #replan_actions = {'phone'}
    stream_info = {
        'motion': StreamInfo(defer=True),
    }

    pr = cProfile.Profile()
    pr.enable()
    if args.algorithm == 'focused':
        solution = solve_focused(pddlstream_problem, replan_actions=replan_actions,
                                 stream_info=stream_info, planner=planner, unit_costs=unit_costs)
    elif args.algorithm == 'incremental':
        solution = solve_incremental(pddlstream_problem, planner=planner, unit_costs=unit_costs)
    else:
        raise NotImplementedError(args.algorithm)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(5)
    print_solution(solution)


if __name__ == '__main__':
    main()
