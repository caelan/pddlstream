#!/usr/bin/env python2.7

from __future__ import print_function

import argparse

from pddlstream.algorithms.satisfaction import dump_assignment, solve_pddlstream_satisfaction
from pddlstream.algorithms.satisfaction2 import constraint_satisfaction
from pddlstream.language.generator import from_test, from_gen_fn
from pddlstream.language.stream import StreamInfo


def problem1(n=5):
    stream_pddl = """
    (define (stream satisfy)
      (:stream positive
        :outputs (?x)
        :certified (Integer ?x)
      )
      (:stream negative
        :outputs (?x)
        :certified (Integer ?x)
      )
      (:stream test-large
        :inputs (?x)
        :domain (Integer ?x)
        :certified (Large ?x)
      )
    )
    """

    #constant_map = {} # TODO: constant_map
    stream_map = {
        'positive': from_gen_fn(lambda: ((x,) for x in range(100000))),
        'negative': from_gen_fn(lambda: ((-x,) for x in range(100000))),
        'test-large': from_test(lambda x: n <= x),
    }
    init = []
    terms = [('Integer', '?x'), ('Large', '?x')]

    return stream_pddl, stream_map, init, terms

##################################################

def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument('-p', '--problem', default='problem1', help='The name of the problem to solve')
    parser.add_argument('-a', '--algorithm', default=None, help='Specifies the algorithm')
    args = parser.parse_args()
    print('Arguments:', args)

    problem_fn = problem1 # get_problem1 | get_problem2
    stream_pddl, stream_map, INIT, terms = problem_fn()
    #print('Init:', pddlstream_problem.init)
    #print('Goal:', pddlstream_problem.goal)

    info = {
        # Intentionally, misleading the stream
        'positive': StreamInfo(overhead=2),
        'negative': StreamInfo(overhead=1),
        # Alternatively, can make the second stream called work
    }
    if args.algorithm == 'focused':
        solution = solve_pddlstream_satisfaction(stream_pddl, stream_map, INIT, terms,
                                                 incremental=False, stream_info=info)
    elif args.algorithm == 'incremental':
        solution = solve_pddlstream_satisfaction(stream_pddl, stream_map, INIT, terms, incremental=True,
                                                 success_cost=info)
    else:
        solution = constraint_satisfaction(stream_pddl, stream_map, INIT, terms, stream_info=info,
                                           planner='ff-astar')
    dump_assignment(solution)

if __name__ == '__main__':
    main()