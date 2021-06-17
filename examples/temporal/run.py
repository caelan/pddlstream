#!/usr/bin/env python

from __future__ import print_function

from itertools import product

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, Or, Exists, print_solution, Output, And
from pddlstream.utils import read_pddl
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_fn, from_test

# TODO: 2 modes - check for possible inconsistency before or fail if inconsistency is detected after

# TODO: see counting, satisfy, cashpoint, discrete_belief, ...
STREAM_PDDL = """
(define (stream temporal)
  (:stream add
    :inputs (?t1 ?dt)
    :domain (and (Time ?t1) (Duration ?dt))
    :outputs (?t2)
    :certified (and (Sum ?t1 ?dt ?t2) (Time ?t2))
  )
  (:stream ge
    :inputs (?t1 ?t2)
    :domain (and (Time ?t1) (Time ?t2))
    :certified (GE ?t1 ?t2)
  )
  (:function (Elapsed ?dt) 
             (Duration ?dt))
)
"""

##################################################

def create_problem(max_time=20., n_foods=3, n_stoves=2):
    constant_map = {}
    stream_map = {
        'add': from_fn(lambda t1, dt: Output(t1 + dt) if (t1 + dt <= max_time) else None),
        'ge': from_test(lambda t1, t2: t1 >= t2),
        'Elapsed': lambda dt: dt,
    }

    foods = ['f{}'.format(i) for i in range(n_foods)]
    stoves = ['s{}'.format(i) for i in range(n_stoves)]

    t0 = 0.
    goal_t = 5
    wait_dt = 0.5

    init = [
        ('Time', t0),
        ('AtTime', t0),
        #('Duration', wait_dt),
        ('Time', goal_t),
    ]

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
    # TODO: exogenous agents such as intercepting a ball
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    problem = create_problem()
    print('Init:', problem.init)
    print('Goal:', problem.goal)

    info = {
        'add': StreamInfo(eager=True, verbose=True),
        'ge': StreamInfo(eager=True, verbose=False),
    }

    # TODO: eager for incremental
    solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit,
                     stream_info=info, planner='max-astar',
                     effort_weight=None, debug=True, verbose=False)
    print_solution(solution)

if __name__ == '__main__':
    main()