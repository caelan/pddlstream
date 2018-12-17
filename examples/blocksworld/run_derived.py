#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.algorithms.focused import solve_focused

from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import print_solution

DOMAIN_PDDL = """
(define (domain blocksworld)
  (:requirements :strips :equality)
  (:predicates (on-table ?x)
               (arm-empty)
               (holding ?x)
               (on ?x ?y)
               (unsafe ?y)
  )
  (:action pickup
    :parameters (?ob)
    :precondition (and (on-table ?ob) (arm-empty) (not (unsafe ?ob)))
    :effect (and (holding ?ob) (not (on-table ?ob))
                 (not (arm-empty))))
  (:action putdown
    :parameters  (?ob)
    :precondition (and (holding ?ob))
    :effect (and (arm-empty) (on-table ?ob)
                 (not (holding ?ob))))
  (:action stack
    :parameters  (?ob ?underob)
    :precondition (and (holding ?ob) (not (unsafe ?underob)))
    :effect (and (arm-empty) (on ?ob ?underob)
                 (not (holding ?ob))))
  (:action unstack
    :parameters  (?ob ?underob)
    :precondition (and (on ?ob ?underob) (arm-empty) (not (unsafe ?ob)))
    :effect (and (holding ?ob)
                 (not (on ?ob ?underob)) (not (arm-empty))))
  (:derived (unsafe ?underob) 
    (exists (?ob) (on ?ob ?underob)))
)
"""

PROBLEM_PDDL = """
(define (problem pb2)
   (:domain blocksworld)
   (:objects a b)
   (:init 
     (on b a)
     (on-table a)
     (arm-empty))
   (:goal (and (on a b))))
"""

##################################################

def solve_pddl():
    plan, cost = solve_from_pddl(DOMAIN_PDDL, PROBLEM_PDDL)
    print('Plan:', plan)
    print('Cost:', cost)

##################################################

def get_problem():
    constant_map = {}
    stream_pddl = None
    stream_map = {}

    init = [
        ('on-table', 'a'),
        ('on', 'b', 'a'),
        ('arm-empty',),
    ]
    goal =  ('on', 'a', 'b')

    return DOMAIN_PDDL, constant_map, stream_pddl, stream_map, init, goal

def solve_pddlstream(focused=True):
    pddlstream_problem = get_problem()
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=True)
    else:
        #solution = solve_exhaustive(pddlstream_problem, unit_costs=True)
        solution = solve_incremental(pddlstream_problem, unit_costs=True)
    print_solution(solution)

##################################################

def main():
    #solve_pddl()
    solve_pddlstream()

if __name__ == '__main__':
    main()