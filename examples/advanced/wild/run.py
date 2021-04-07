#!/usr/bin/env python

from __future__ import print_function

import random

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, And, Exists, print_solution, Imply
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_fn, from_test, from_sampler, fn_from_constant

# TODO:
# Push wild streams
# Selectively exposing objects
# Cutting
# Objects in a bag
# Open world models and object counts
# Active segmentation
# Collisions
# Universal quantification (over infinite domains
# Conditional effects
# Unknown number of pieces

DOMAIN_PDDL = """
(define (domain wild)
  (:predicates 
    (Carrot ?c)
    (Cut ?c0 ?s ?c1 ?c2)
    (Loc ?l)
    (AtLoc ?c ?l)
  )
  (:functions 
    (SplitCost ?c0 ?s)
  )
  (:action move
    :parameters (?c ?l1 ?l2)
    :precondition (and (Carrot ?c) (Loc ?l1) (Loc ?l2)
                       ; (forall (?c2) (imply (Carrot ?c2) (Carrot ?c2)))
                       (AtLoc ?c ?l1))
    :effect (and (AtLoc ?c ?l2)
                 (not (AtLoc ?c ?l1))
                 (increase (total-cost) 1))
  )  
  (:action cut
    :parameters (?l ?c0 ?s ?c1 ?c2)
    :precondition (and (Loc ?l) (Cut ?c0 ?s ?c1 ?c2) 
                       (AtLoc ?c0 ?l))
    :effect (and (AtLoc ?c1 ?l) (AtLoc ?c2 ?l)
                 (not (AtLoc ?c0 ?l)) 
                 (increase (total-cost) (SplitCost ?c0 ?s)))
  )
)
"""

STREAM_PDDL = """
(define (stream wild)
  (:stream sample-split
    :outputs (?s)
    :certified (Split ?s)
  )
  (:stream compute-split
    :inputs (?c0 ?s)
    :domain (and (Carrot ?c0) (Split ?s))
    :outputs (?c1 ?c2)
    :certified (and (Carrot ?c1) (Carrot ?c2) 
                    (Cut ?c0 ?s ?c1 ?c2))
  )
  (:function (SplitCost ?c0 ?s)
    (and (Carrot ?c0) (Split ?s))
  )
)
"""
#STREAM_PDDL = None

##################################################

# TODO: compose functions
def to_tuple(*x):
    return tuple(x)

def convex_combo(x1, x2, lamb=0.5):
    return lamb*x1 + (1-lamb)*x2

def compute_split(pair, s):
    x1, x2 = pair
    return [(x1, convex_combo(x1, x2, s)),
            (convex_combo(x1, x2, s), x2)]

def compute_cost(pair, s):
    x1, x2 = pair
    return round(1./(min(convex_combo(x1, x2, s) - x1,
                         x2 - convex_combo(x1, x2, s))), 3)

def get_problem(num=3):
    constant_map = {}
    stream_map = {
        'sample-split': from_sampler(lambda: to_tuple(round(random.random(), 2))),
        'compute-split': from_fn(compute_split),
        #'SplitCost': fn_from_constant(2),
        'SplitCost': compute_cost,
    }

    locations = ['loc{}'.format(i) for i in range(num)]
    init_carrot = (0., 1.)
    init = [
        #('Split', 0.5),
        ('Carrot', init_carrot),
        ('AtLoc', init_carrot, 'loc0'),
    ] + [('Loc', loc) for loc in locations]


    goal = And(*[Exists(['?c'], And(('Loc', loc), ('AtLoc', '?c', loc)))
                 for loc in locations])

    return PDDLProblem(DOMAIN_PDDL, constant_map, STREAM_PDDL, stream_map, init, goal)

##################################################

def main():
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    problem = get_problem()
    print('Init:', problem.init)
    print('Goal:', problem.goal)

    info = {
        # Intentionally, misleading the stream
        'increment': StreamInfo(p_success=0.01, overhead=1),
        'decrement': StreamInfo(p_success=1, overhead=1),
    }
    solution = solve(problem, algorithm=args.algorithm, planner='max-astar', unit_costs=args.unit,
                     stream_info=info, effort_weight=1, #success_cost=0., max_iterations=3, max_time=5,
                     debug=False, verbose=True)
    print_solution(solution)

if __name__ == '__main__':
    main()