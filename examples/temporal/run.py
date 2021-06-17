#!/usr/bin/env python

from __future__ import print_function

from itertools import product

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, Or, Exists, print_solution, Output, And
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_fn, from_test

DOMAIN_PDDL = """
(define (domain debug)
  (:predicates 
    (Time ?t)
    (Duration ?dt)
    (Sum ?t1 ?dt ?t2)
    (GE ?t1 ?t2)
    
    (Food ?f)
    (Stove ?s) ; TODO: Oven
    
    (CookDuration ?dt ?f ?s)
    (Cooking ?t2 ?f ?s)

    (AtTime ?t)
    (Cooked ?f)
    (Locked ?s)
  )
  (:functions
    (Elapsed ?dt)
  )
  (:action wait
    :parameters (?t1 ?dt ?t2)
    :precondition (and (Sum ?t1 ?dt ?t2) 
                       (AtTime ?t1))
    :effect (and (AtTime ?t2) 
                 (not (AtTime ?t1))
                 (increase (total-cost) (Elapsed ?dt)))
  )
  (:action start-cooking
    :parameters (?t1 ?dt ?t2 ?f ?s)
    :precondition (and ;(Food ?f) (Stove ?s)
                       (CookDuration ?dt ?f ?s)
                       (Sum ?t1 ?dt ?t2)
                       (AtTime ?t1) 
                       (not (Locked ?f)) (not (Locked ?s))
                   )
    :effect (and (Cooking ?t2 ?f ?s)
                 (Locked ?f) (Locked ?s)
                 (increase (total-cost) 0))
  )
  (:action stop-cooking
    :parameters (?t2 ?f ?s)
    :precondition (and (Time ?t2) (Food ?f) (Stove ?s)
                       (Cooking ?t2 ?f ?s)
                       (AtTime ?t2))
    :effect (and (Cooked ?f)
                 (not (Cooking ?t2 ?f ?s))
                 (not (Locked ?f)) (not (Locked ?s))
                 (increase (total-cost) 0))
  )
)
"""

# TODO: see counting, satisfy, cashpoint, discrete_belief, ...
STREAM_PDDL = """
(define (stream exogenous)
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
  
  ;(:stream decrement
  ;  :inputs (?x1)
  ;  :domain (Integer ?x1)
  ;  :outputs (?x2)
  ;  :certified (Integer ?x2)
  ;)
  ;(:stream test-large
  ;  :inputs (?x)
  ;  :domain (Integer ?x)
  ;  :certified (Large ?x)
  ;)
  ;(:stream test-small
  ;  :inputs (?x)
  ;  :domain (Integer ?x)
  ;  :certified (Small ?x)
  ;)

  (:function (Elapsed ?dt) 
             (Duration ?dt))
)
"""

##################################################

def create_problem(max_time=20., n_foods=2, n_stoves=1):
    constant_map = {}
    stream_map = {
        'add': from_fn(lambda t1, dt: Output(t1 + dt) if (t1 + dt <= max_time) else None),
        'decrement': from_fn(lambda x: (x - 1,)),
        'ge': from_test(lambda t1, t2: t1 >= t2),
        # 'test-small': from_test(lambda x: x <= -n),
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
        ('Duration', wait_dt),
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

    return PDDLProblem(DOMAIN_PDDL, constant_map, STREAM_PDDL, stream_map, init, goal)

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
        # Intentionally, misleading the stream
        'increment': StreamInfo(p_success=0.01, overhead=1),
        'decrement': StreamInfo(p_success=1, overhead=1),
    }

    solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit,
                     stream_info=info, planner='max-astar', effort_weight=1)
    print_solution(solution)

if __name__ == '__main__':
    main()