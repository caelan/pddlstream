#!/usr/bin/env python2.7

from __future__ import print_function

import argparse
import random

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import PDDLProblem, And, Exists, print_solution
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_fn, from_test, from_sampler

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
    (Cut ?c0 ?c1 ?c2)
    (Loc ?l)
    (AtLoc ?c ?l)
  )
  (:action move
    :parameters (?c ?l1 ?l2)
    :precondition (and (Carrot ?c) (Loc ?l1) (Loc ?l2)
                       (AtLoc ?c ?l1))
    :effect (and (AtLoc ?c ?l2)
                 (not (AtLoc ?c ?l1)))
  )  
  (:action cut
    :parameters (?l ?c0 ?c1 ?c2)
    :precondition (and (Loc ?l) (Cut ?c0 ?c1 ?c2) 
                       (AtLoc ?c0 ?l))
    :effect (and (AtLoc ?c1 ?l) (AtLoc ?c2 ?l)
                 (not (AtLoc ?c0 ?l)))
  )
)
"""

# TODO: cost as a function of the split
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
                    (Cut ?c0 ?c1 ?c2)) ; ?s
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

def get_problem():
    constant_map = {}
    stream_map = {
        'sample-split': from_sampler(lambda: to_tuple(random.random())),
        'compute-split': from_fn(lambda (x1, x2), s: [(x1, convex_combo(x1, x2, s)),
                                                        (convex_combo(x1, x2, s), x2)]),
    }

    locations = ['loc0', 'loc1', 'loc2']
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
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--algorithm', default='incremental', help='Specifies the algorithm')
    args = parser.parse_args()
    print('Arguments:', args)

    problem_fn = get_problem # get_problem1 | get_problem2
    pddlstream_problem = problem_fn()
    print('Init:', pddlstream_problem.init)
    print('Goal:', pddlstream_problem.goal)

    info = {
        # Intentionally, misleading the stream
        'increment': StreamInfo(p_success=0.01, overhead=1),
        'decrement': StreamInfo(p_success=1, overhead=1),
    }
    if args.algorithm == 'focused':
        solution = solve_focused(pddlstream_problem, stream_info=info,
                                 planner='max-astar', effort_weight=1)
    elif args.algorithm == 'incremental':
        solution = solve_incremental(pddlstream_problem, debug=False, verbose=True)
    else:
        raise ValueError(args.algorithm)
    print_solution(solution)

if __name__ == '__main__':
    main()