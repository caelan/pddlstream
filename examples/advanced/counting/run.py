#!/usr/bin/env python2.7

from __future__ import print_function

import argparse

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import PDDLProblem, Or, Exists, print_solution
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_fn, from_test

DOMAIN_PDDL = """
(define (domain debug)
  (:predicates 
    (Integer ?x)
    (Large ?x)
    (Small ?x)
    (Goal)
  )
  (:action achieve
    :parameters (?x)
    :precondition (Large ?x)
    ;:precondition (Small ?x)
    :effect (Goal)
  )
)
"""
# Need a goal to include the effort

STREAM_PDDL = """
(define (stream exogenous)
  (:stream increment
    :inputs (?x1)
    :domain (Integer ?x1)
    :outputs (?x2)
    :certified (Integer ?x2)
  )
  (:stream decrement
    :inputs (?x1)
    :domain (Integer ?x1)
    :outputs (?x2)
    :certified (Integer ?x2)
  )
  (:stream test-large
    :inputs (?x)
    :domain (Integer ?x)
    :certified (Large ?x)
  )
  (:stream test-small
    :inputs (?x)
    :domain (Integer ?x)
    :certified (Small ?x)
  )
)
"""

##################################################

def get_problem1(n=5):
    # TODO: consider even/odd and version with infinite generator
    # TODO: replicate the effect where a stream fails
    constant_map = {}
    stream_map = {
        'increment': from_fn(lambda x: (x + 1,)), # Increment by different amounts
        'decrement': from_fn(lambda x: (x - 1,)),
        'test-large': from_test(lambda x: n <= x),
        'test-small': from_test(lambda x: x <= -n),
    }

    init = [
        ('Integer', 0),
    ]
    goal = Or(
        ('Goal',)
        #Exists(['?x'], ('Large', '?x')),
        #Exists(['?x'], ('Small', '?x')),
    )
    # TODO: sort actions when renaming to make deterministic

    return PDDLProblem(DOMAIN_PDDL, constant_map, STREAM_PDDL, stream_map, init, goal)

##################################################

def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument('-p', '--problem', default='blocked', help='The name of the problem to solve')
    parser.add_argument('-a', '--algorithm', default='focused', help='Specifies the algorithm')
    args = parser.parse_args()
    print('Arguments:', args)

    problem_fn = get_problem1 # get_problem1 | get_problem2
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
        solution = solve_incremental(pddlstream_problem)
    else:
        raise ValueError(args.algorithm)
    print_solution(solution)

if __name__ == '__main__':
    main()