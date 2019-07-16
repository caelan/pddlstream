#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.algorithms.focused import solve_focused

from pddlstream.algorithms.incremental import solve_incremental
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
    goal =  Or(
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

def solve_pddlstream(focused=True):
    problem_fn = get_problem1 # get_problem1 | get_problem2
    pddlstream_problem = problem_fn()
    print('Init:', pddlstream_problem.init)
    print('Goal:', pddlstream_problem.goal)
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=True)
    else:
        solution = solve_incremental(pddlstream_problem, unit_costs=True)
    print_solution(solution)
    #print(*solve_from_pddl(DOMAIN_PDDL, PROBLEM_PDDL))

##################################################

def main():
    solve_pddlstream()

if __name__ == '__main__':
    main()