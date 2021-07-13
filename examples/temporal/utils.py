import numpy as np

from pddlstream.language.constants import Output, Solution
from pddlstream.language.function import Function, FunctionInfo
from pddlstream.language.generator import from_test, from_fn
from pddlstream.language.stream import Stream, StreamInfo
from pddlstream.language.temporal import Time, GE, Duration, StartTime, Sum, DURATION_TEMPLATE, Difference, Elapsed, \
    AtTime, temporal_from_sequential, compute_makespan
from pddlstream.utils import INF

GE_STREAM = 'ge'
ADD_STREAM = 'add'


def create_inequality_stream():
    # TODO: refactor
    #from pddlstream.algorithms.downward import IDENTICAL
    return Stream(name=GE_STREAM,
                  gen_fn=from_test(lambda t1, t2: t1 >= t2),
                  inputs=['?t1', '?t2'],
                  domain=[(Time, '?t1'), (Time, '?t2')],
                  outputs=[],
                  certified=[(GE, '?t1', '?t2')],
                  info=StreamInfo(eager=True))


class BoundedTime(object):
    def __init__(self, t=0., num=0):
        # Limits the number of sequential actions
        # TODO: exploit the fact that robots can only do one thing at time
        self.t = t
        self.num = num # TODO: countdown from zero instead?
    def __add__(self, duration):
        return BoundedTime(self.t + duration, self.num + 1)


def create_add_function(max_t=INF, propagate_start=True):
    # TODO: could make one of these per action to only propagate for a specific duration
    return Stream(name=ADD_STREAM,
                  gen_fn=from_fn(lambda t1, dt: Output(t1 + dt) if (t1 + dt <= max_t) else None),
                  inputs=['?t1', '?dt'],
                  domain=[(Time, '?t1'), (Duration, '?dt'),
                          (StartTime, '?t1'),
                          ],
                  outputs=['?t2'],
                  certified=[(Time, '?t2'), (Sum, '?t1', '?dt', '?t2')]
                            # TODO: toggle depending on if can start from any stop time
                            + ([(StartTime, '?t2')] if propagate_start else []),
                  info=StreamInfo(eager=True))


def create_duration_stream(action, parameters, durations=[]):
    # TODO: lower and upper generator
    # TODO: interval generator
    ActionDuration = DURATION_TEMPLATE.format(action)  # TODO: relate to duration
    # TODO: load from duration functions
    return Stream(name='{}-duration',
                  gen_fn=from_list_fn(lambda *args, **kwar: Output(t1 + dt) if (t1 + dt <= max_t) else None),
                  inputs=['?t1', '?dt'],
                  domain=[(Time, '?t1'), (Duration, '?dt')],
                  outputs=['?dt'],
                  certified=[(Duration, '?dt'), (ActionDuration, '?t1', '?dt', '?t2')],
                  info=StreamInfo(eager=True))

##################################################

def create_difference_function(): #scale=1.):
    # TODO: min_dt, max_dt, scale_dt?
    return Function(head=(Difference, '?t2', '?t1'),
                    fn=lambda t2, t1: (t2 - t1), # scale
                    domain=[(GE, '?t2', '?t1')],
                    info=FunctionInfo(eager=True)) # TODO: eager function


def create_duration_function(): #scale=1.):
    return Function(head=(Elapsed, '?dt'),
                    fn=lambda dt: dt, # scale
                    domain=[(Duration, '?dt')],
                    info=FunctionInfo(eager=True)) # TODO: eager function

##################################################

def initialize_time(t0=0.):
    return [
        # (Advancable,), # TODO: add automatically
        (Time, t0),
        (StartTime, t0),
        (AtTime, t0),
    ]


def discretize_time(t0, max_t, dt=None):
    # TODO: stream for increasing max_t
    # TODO: stream for decreasing dt
    init = []
    if dt is None:
        return init # TODO: include t0?
    for t in np.arange(t0, max_t, step=dt):
        t = round(t, 3)
        init.extend([
            (Time, t),
            (StartTime, t),
        ])
    return init


def convert_solution(solution):
    if not solution:
        return solution
    plan, cost, certificate = solution
    plan = temporal_from_sequential(plan)
    cost = compute_makespan(plan)
    return Solution(plan, cost, certificate)