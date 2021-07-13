#!/usr/bin/env python

from __future__ import print_function

from itertools import product
import os

from examples.temporal.utils import create_inequality_stream, create_add_function, create_difference_function, \
    create_duration_function, initialize_time, discretize_time
from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, print_solution, And, Not, Equal, Solution
from pddlstream.utils import read_pddl, INF
from pddlstream.language.temporal import Time, ENV_VAR, DURATION_TEMPLATE, Duration, temporal_from_sequential

#from pddlstream.algorithms.downward import print_search_options

os.environ[ENV_VAR] = '/Users/caelan/Programs/external/planners/TemporalFastDownward'

# TODO: 2 modes - check for possible inconsistency before or fail if inconsistency is detected after

# TODO: see counting, satisfy, cashpoint, discrete_belief, ...
STREAM_PDDL = """
(define (stream temporal)
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
  
)
"""

##################################################

def create_problem(max_t=20., n_foods=1, n_stoves=1):
    constant_map = {}

    foods = ['f{}'.format(i) for i in range(n_foods)]
    stoves = ['s{}'.format(i) for i in range(n_stoves)]

    t0 = 0.
    goal_t = 5
    #wait_dt = 0.5
    #discretize_dt = 1./3
    discretize_dt = None # TODO: set to None if not all times are StartTimes
    # TODO: iteratively increase max_t and/or discretize_dt

    # TODO: min_dt is the min of all the durations
    # TODO: add epsilon (epsilonize) after every action start

    init = [
        #(Duration, wait_dt), # T
        (Time, goal_t),
        (Duration, 0),
    ]
    init.extend(initialize_time())
    if discretize_dt is not None:
        init.extend(discretize_time(t0, max_t, dt=discretize_dt))

    # TODO: extract all initial times as important times
    # TODO: support timed initial literals # e.g (at 10 (train-not-in-use t1))

    cook_dt = 2.
    for food, stove in product(foods, stoves):
        CookDuration = DURATION_TEMPLATE.format('cook')
        init.extend([
            (Duration, cook_dt),
            (CookDuration, cook_dt, food, stove),
            ('Food', food),
            ('Stove', stove),
            Equal(('GasCost', stove), 0),
        ])

    ##################################################

    goal_expressions = [
        #Exists(['?t'], And((GE, '?t', goal_t),
        #                   (AtTime, '?t'))),
    ]
    for stove in stoves:
        goal_expressions.append(Not(('On', stove)))
    for food in foods:
        goal_expressions.append(('Cooked', food))
    goal = And(*goal_expressions)

    ##################################################

    #path = '/Users/caelan/Programs/pddlstream/examples/continuous_tamp/temporal/domain.pddl'
    #path = 'sequential_domain.pddl'
    path = 'durative_domain.pddl'
    domain_pddl = read_pddl(__file__, path)

    #domain = parse_domain(domain_pddl)
    #domain = parse_temporal_domain(domain_pddl)
    #domain_pddl = domain

    # for action in domain.actions:
    #     action.dump()

    ##################################################

    stream_pddl = [
        create_inequality_stream(),
        create_add_function(max_t=max_t),

        create_duration_function(),
        create_difference_function(),

        #STREAM_PDDL,
    ]

    stream_map = {
        # TODO: compute the sequence of times and/or length
        #ADD_STREAM: from_fn(lambda t1, dt: Output(t1 + dt) if (t1 + dt <= max_t) else None),
        #GE_STREAM: from_test(lambda t1, t2: t1 >= t2),
        #Elapsed: lambda dt: dt,
        #Difference: lambda t2, t1: t2 - t1,
    }

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
        # ADD_STREAM: StreamInfo(eager=True, verbose=True),
        # GE_STREAM: StreamInfo(eager=True, verbose=False),
        # Elapsed: FunctionInfo(eager=True, verbose=False),
        # Difference: FunctionInfo(eager=True, verbose=False),
    }

    # TODO: eager for incremental
    solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit,
                     stream_info=info, planner='dijkstra', initial_complexity=INF,
                     effort_weight=None, debug=True, verbose=False)
    plan, cost, certificate = solution
    plan = temporal_from_sequential(plan)
    solution = Solution(plan, cost, certificate)
    print_solution(solution)
    #print_search_options()

if __name__ == '__main__':
    main()
