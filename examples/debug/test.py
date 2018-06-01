#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.incremental import solve_incremental, solve_exhaustive
from pddlstream.focused import solve_focused
from pddlstream.utils import print_solution

DOMAIN_PDDL = """
(define (domain debug)
  (:requirements :strips :equality)
  (:predicates 
    (A)
    (B)
    (Goal)
  )
  (:action action
    :parameters ()
    :precondition (or (A) (B))
    :effect (Goal)
  )
)
"""

##################################################

def get_problem():
    constant_map = {}
    stream_pddl = None
    stream_map = {}

    init = [
        ('A',),
        #('B',),
    ]
    goal =  ('Goal',)

    return DOMAIN_PDDL, constant_map, stream_pddl, stream_map, init, goal

def solve_pddlstream(focused=True):
    pddlstream_problem = get_problem()
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=True)
    else:
        solution = solve_incremental(pddlstream_problem, unit_costs=True)
    print_solution(solution)

##################################################

def main():
    solve_pddlstream()

if __name__ == '__main__':
    main()