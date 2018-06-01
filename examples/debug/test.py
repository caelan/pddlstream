#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.incremental import solve_incremental, solve_exhaustive
from pddlstream.focused import solve_focused
from pddlstream.utils import print_solution
from pddlstream.conversion import Or

DOMAIN_PDDL = """
(define (domain debug)
  (:requirements :strips :equality)
  (:predicates 
    (A)
    (B)
    (Goal)
    (Unachievable)
  )
  (:action action
    :parameters ()
    :precondition (or (A) (B))
    :effect (Goal)
  )
)
"""

##################################################

def get_problem1():
    constant_map = {}
    stream_pddl = None
    stream_map = {}

    init = [
        #('A',),
        ('B',),
    ]
    goal =  ('Goal',)

    return DOMAIN_PDDL, constant_map, stream_pddl, stream_map, init, goal

def get_problem2():
    constant_map = {}
    stream_pddl = None
    stream_map = {}

    init = [
        #('A',),
        ('B',),
    ]
    goal =  Or(
        ('Goal',),
        ('Unachievable',),
    )

    return DOMAIN_PDDL, constant_map, stream_pddl, stream_map, init, goal

##################################################

def solve_pddlstream(focused=True):
    problem_fn = get_problem2 # get_problem1 | get_problem2
    pddlstream_problem = problem_fn()
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