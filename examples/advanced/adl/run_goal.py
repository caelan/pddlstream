#!/usr/bin/env python

from __future__ import print_function

from pddlstream.algorithms.search import solve_from_pddl
from examples.ipc.rovers.run import dump_plan

DOMAIN_PDDL = """
(define (domain sanity)
    (:requirements :strips)
    (:predicates
        (isA ?obj)
        (isB ?obj)
    )
)
"""

PROBLEM_PDDL = """
(define (problem check) 
 (:domain sanity)
    (:objects
        A
        B
    )
    (:init
        (isA A)
        (isB B)
    )
    (:goal 
        (or (isA A) (isB B))
    )
)
"""

def main():
    plan, cost = solve_from_pddl(DOMAIN_PDDL, PROBLEM_PDDL, planner='dijkstra')
    dump_plan(plan, cost)

if __name__ == '__main__':
    main()
