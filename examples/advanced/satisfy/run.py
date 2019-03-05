#!/usr/bin/env python2.7

from __future__ import print_function

import argparse

from pddlstream.retired.satisfaction import solve_pddlstream_satisfaction
from pddlstream.algorithms.satisfaction import constraint_satisfaction, dump_assignment
from pddlstream.language.generator import from_test, from_gen_fn, from_fn
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import INF

STREAM_PDDL = """
(define (stream satisfy)
  (:stream positive
    :outputs (?x)
    :certified (and (Positive ?x) (Integer ?x))
  )
  (:stream negative
    :outputs (?x)
    :certified (and (Negative ?x) (Integer ?x))
  )
  (:stream test-small
    :inputs (?x)
    :domain (Integer ?x)
    :certified (Small ?x)
  )
  (:stream test-large
    :inputs (?x)
    :domain (Integer ?x)
    :certified (Large ?x)
  )
  (:stream addition
    :inputs (?x1 ?x2)
    :domain (and (Integer ?x1) (Integer ?x2))
    :outputs (?x3)
    :certified (and (Sum ?x1 ?x2 ?x3) (Integer ?x3))
  )
  
  (:function (Cost ?x) 
             (Integer ?x)
  )
)
"""

LARGE = 5
SMALL = -1
#MAX_INT = 100000
MAX_INT = 5

pairs = set()

def addition(x1, x2):
    pairs.add((x1, x2))
    x2 = x1 + x2
    return (x2,)

STREAM_MAP = {
    'positive': from_gen_fn(lambda: ((x,) for x in range(1, MAX_INT + 1))),
    'negative': from_gen_fn(lambda: ((-x,) for x in range(1, MAX_INT + 1))),
    'addition': from_fn(addition),
    'test-large': from_test(lambda x: LARGE <= x),
    'test-small': from_test(lambda x: x <= SMALL),
    'cost': lambda x: 1. / (abs(x) + 1),
}

def problem1():
    #constant_map = {} # TODO: constant_map
    init = []
    terms = [('Integer', '?x1'), ('Large', '?x1'), ('Integer', '?x2'),
             ('minimize', ('Cost', '?x1')), ('minimize', ('Cost', '?x2'))]
    return STREAM_PDDL, STREAM_MAP, init, terms

def problem2():
    init = []
    terms = [('Sum', '?x1', '?x2', '?x3'), ('Positive', '?x1'), ('Negative', '?x2'),
             ('minimize', ('Cost', '?x3'))]
    return STREAM_PDDL, STREAM_MAP, init, terms

def problem3():
    init = []
    terms = [('Negative', '?x1'), ('Small', '?x1'), ('Positive', '?x2'), ('Large', '?x2')]
    return STREAM_PDDL, STREAM_MAP, init, terms

##################################################

def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument('-p', '--problem', default='problem1', help='The name of the problem to solve')
    parser.add_argument('-a', '--algorithm', default=None, help='Specifies the algorithm')
    parser.add_argument('-o', '--optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-t', '--max_time', default=2, type=int, help='The max time')
    args = parser.parse_args()
    print('Arguments:', args)

    problem_fn = problem3 # problem1 | problem2 | problem3
    stream_pddl, stream_map, init, terms = problem_fn()
    #print('Init:', pddlstream_problem.init)
    #print('Goal:', pddlstream_problem.goal)

    info = {
        # Intentionally, misleading the stream
        'positive': StreamInfo(overhead=2),
        'negative': StreamInfo(overhead=1),
        'test-small': StreamInfo(p_success=1e-1),
        # Alternatively, can make the second stream called work
    }
    success_cost = 0 if args.optimal else INF
    if args.algorithm == 'focused':
        solution = solve_pddlstream_satisfaction(stream_pddl, stream_map, init, terms, incremental=False,
                                                 stream_info=info, max_time=args.max_time, success_cost=success_cost)
    elif args.algorithm == 'incremental':
        solution = solve_pddlstream_satisfaction(stream_pddl, stream_map, init, terms, incremental=True,
                                                 max_time=args.max_time, success_cost=success_cost)
    else:
        solution = constraint_satisfaction(stream_pddl, stream_map, init, terms, stream_info=info, #unit_efforts=True,
                                           max_time=args.max_time, success_cost=success_cost)
    dump_assignment(solution)
    #print(len(sorted(pairs)))

if __name__ == '__main__':
    main()