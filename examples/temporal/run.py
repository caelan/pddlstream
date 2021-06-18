#!/usr/bin/env python

from __future__ import print_function

from itertools import product
import numpy as np

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, Or, Exists, print_solution, Output, And
from pddlstream.utils import read_pddl, INF
#from pddlstream.algorithms.downward import print_search_options
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_fn, from_test

# TODO: 2 modes - check for possible inconsistency before or fail if inconsistency is detected after

# TODO: see counting, satisfy, cashpoint, discrete_belief, ...
STREAM_PDDL = """
(define (stream temporal)
  ; TODO: could make one of these per action to only propagate for a specific duration
  (:stream add
    :inputs (?t1 ?dt)
    :domain (and (Time ?t1)
                 (StartTime ?t1)
                 (Duration ?dt))
    :outputs (?t2)
    :certified (and (Sum ?t1 ?dt ?t2)
                    (Time ?t2)
                    (StartTime ?t2) ; TODO: toggle depending on if can start from any stop time
               )
  )
  
  (:stream ge
    :inputs (?t1 ?t2)
    :domain (and (Time ?t1) (Time ?t2))
    :certified (GE ?t1 ?t2)
  )
  (:function (Elapsed ?dt) 
             (Duration ?dt))
  (:function (Difference ?t2 ?t1)
             (GE ?t2 ?t1))
)
"""

##################################################

def create_problem(max_t=20., n_foods=3, n_stoves=2):
    constant_map = {}
    stream_map = {
        # TODO: compute the sequence of times and/or length
        'add': from_fn(lambda t1, dt: Output(t1 + dt) if (t1 + dt <= max_t) else None),
        'ge': from_test(lambda t1, t2: t1 >= t2),
        'Elapsed': lambda dt: dt,
        'Difference': lambda t2, t1: t2 - t1,
    }

    foods = ['f{}'.format(i) for i in range(n_foods)]
    stoves = ['s{}'.format(i) for i in range(n_stoves)]

    t0 = 0.
    goal_t = 5
    #wait_dt = 0.5
    #discretize_dt = 1./3
    discretize_dt = None # TODO: set to None if not all times are StartTimes
    # TODO: iteratively increase max_t and/or discretize_dt

    # TODO: min_dt is the min of all the durations
    # TODO: sample the duration for actions
    # TODO: add epsilon after every action start

    init = [
        ('CanWait',),
        ('Time', t0),
        ('StartTime', t0),
        ('AtTime', t0),
        #('Duration', wait_dt), # T
        ('Time', goal_t),
    ]
    if discretize_dt is not None:
        for t in np.arange(t0, max_t, step=1./3):
            t = round(t, 3)
            init.extend([
                ('Time', t),
                ('StartTime', t),
            ])

    # TODO: extract all initial times as important times
    # TODO: support timed initial literals

    cook_dt = 2.
    for food, stove in product(foods, stoves):
        init.extend([
            ('Food', food),
            ('Stove', stove),
            ('CookDuration', cook_dt, food, stove),
            ('Duration', cook_dt),
        ])

    goal_expressions = [
        #Exists(['?t'], And(('GE', '?t', goal_t),
        #                   ('AtTime', '?t'))),
    ]
    for food in foods:
        goal_expressions.append(('Cooked', food))
    goal = And(*goal_expressions)

    domain_pddl = read_pddl(__file__, 'domain.pddl')
    stream_pddl = STREAM_PDDL

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

##################################################

def main():
    # TODO: previous domains with dynamics: rocket domain, car domain, ..
    # stripstream
    # https://github.mit.edu/caelan/stripstream/tree/master/fts_scripts
    # https://github.mit.edu/caelan/stripstream/tree/master/lis_scripts
    # https://github.mit.edu/caelan/stripstream/tree/master/robotics/openrave
    # https://github.mit.edu/caelan/stripstream/tree/master/scripts
    # https://github.mit.edu/caelan/stripstream/tree/master/scripts/metric/run_car.py
    # https://github.mit.edu/caelan/stripstream/tree/master/scripts/metric/run_rocket.py
    # https://github.mit.edu/caelan/stripstream/tree/master/scripts/metric/run_tsiolkovsky.py
    # https://github.mit.edu/caelan/stripstream/tree/master/scripts/openrave
    # https://github.mit.edu/caelan/stripstream/tree/master/stripstream/pddl/examples
    # https://github.mit.edu/caelan/stripstream/tree/master/stripstream/fts/examples

    # ss
    # https://github.mit.edu/caelan/ss/tree/master/belief
    # https://github.mit.edu/caelan/ss/tree/master/openrave
    # https://github.mit.edu/caelan/ss/tree/master/examples

    # http://gki.informatik.uni-freiburg.de/papers/eyerich-etal-icaps09.pdf

    # TODO: exogenous agents such as intercepting a ball
    # examples/pybullet/turtlebots/domain.pddl
    # examples/continuous_tamp/temporal/domain.pddl
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    problem = create_problem()
    print('Init:', problem.init)
    print('Goal:', problem.goal)

    info = {
        'add': StreamInfo(eager=True, verbose=True),
        'ge': StreamInfo(eager=True, verbose=False),
        'Duration': FunctionInfo(eager=True, verbose=False),
        'Difference': FunctionInfo(eager=True, verbose=False),
    }

    # TODO: eager for incremental
    solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit,
                     stream_info=info, planner='dijkstra', initial_complexity=INF,
                     effort_weight=None, debug=True, verbose=False)
    print_solution(solution)
    #print_search_options()

if __name__ == '__main__':
    main()