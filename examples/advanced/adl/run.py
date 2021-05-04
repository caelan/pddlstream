#!/usr/bin/env python

from __future__ import print_function

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, Or, print_solution

DOMAIN_PDDL = """
(define (domain debug)
  (:requirements :strips :equality :adl)
  ;(:constants c1)
  (:predicates 
    (A)
    (B)
    (P1 ?o)
    (P2 ?o)
    (Goal)
    (Unachievable)
  )
  ;(:action or-act
  ;  :parameters ()
  ;  :precondition (or (A) (B))
  ;  :effect (Goal)
  ;)
  ;(:action conditional-act
  ;  :parameters ()
  ;  :precondition ()
  ;  :effect (when (B) (Goal))
  ;)
  ;(:action forall-cond-act
  ;  :parameters ()
  ;  :precondition ()
  ;  :effect (forall (?o) (when (P1 ?o) (P2 ?o)))
  ;)
  ;(:action exists-act
  ;  :parameters ()
  ;  :precondition (exists (?o) (P1 ?o))
  ;  :effect (Goal)
  ;)
  ;(:action forall-act
  ;  :parameters ()
  ;  :precondition (forall (?o) (P1 ?o))
  ;  :effect (Goal)
  ;)
  (:action imply-act
    :parameters ()
    :precondition (imply (A) (B))
    :effect (Goal)
  )
)
"""

##################################################

def get_problem1():
    value = 10
    constant_map = {
        #'c1': value,
    }
    stream_pddl = None
    stream_map = {}

    init = [
        #('Goal',),
        #('A',),
        ('B',),
        ('P1', value),
        #('P2', value),
    ]
    goal = Or(
        ('Goal',),
        #Exists([], ('P2', value)),
        #Exists(['?p'], ('P2', '?p')),
        ('Unachievable',),
    )

    return PDDLProblem(DOMAIN_PDDL, constant_map, stream_pddl, stream_map, init, goal)

##################################################

PROBLEM_PDDL = """
(define (problem debug)
   (:domain debug)
   (:objects c1)
   (:init 
     (P1 c1)
     ;(P2 c1)
   )
   (:goal (exists (?p) (P2 ?p)))
)
"""

##################################################

def solve_pddlstream():
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    problem = get_problem1()
    print('Init:', problem.init)
    print('Goal:', problem.goal)
    solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit)
    print_solution(solution)
    #print(*solve_from_pddl(DOMAIN_PDDL, PROBLEM_PDDL))

##################################################

def main():
    solve_pddlstream()

if __name__ == '__main__':
    main()