#!/usr/bin/env python

from __future__ import print_function

import numpy as np

import pddlstream.language.exogenous as exogenous
exogenous.EXOGENOUS_AXIOMS = True
exogenous.REPLACE_STREAM = False

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.generator import from_fn
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, get_file_path, Profiler
from pddlstream.language.constants import print_solution, PDDLProblem
from pddlstream.language.external import defer_shared, never_defer
#from pddlstream.language.exogenous import FutureValue
#from examples.advanced.defer.run import ik_fn, motion_fn

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

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

##################################################

def main(planner='max-astar', defer=False):
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    problem = pddlstream_from_belief()
    _, _, _, _, init, goal = problem
    print('Init:', sorted(init, key=lambda f: f[0]))
    print('Goal:', goal)

    stream_info = {
        'motion': StreamInfo(defer_fn=defer_shared if defer else never_defer),
    }

    replan_actions = set()
    #replan_actions = {'phone'}

    with Profiler(field='tottime', num=5):
        solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit, planner=planner,
                         stream_info=stream_info, replan_actions=replan_actions)
    print_solution(solution)


if __name__ == '__main__':
    main()
