#!/usr/bin/env python2.7

from __future__ import print_function

import os
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import argparse
import random

from itertools import combinations, count

import pddlstream.algorithms.scheduling.diverse
pddlstream.algorithms.scheduling.diverse.DEFAULT_K = 1

import pddlstream.algorithms.downward
pddlstream.algorithms.downward.USE_FORBID = False

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.scheduling.diverse import p_disjunction
from pddlstream.language.generator import from_test, universe_test, empty_test, fn_from_constant
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, get_file_path, INF, hash_or_id
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream, is_plan
from examples.fault_tolerant.logistics.run import test_from_bernoulli_fn, CachedFn

P_SUCCESS = 1

# TODO: parse problem.pddl directly

OBJECTS = """
data-0-3 data-0-5 data-1-2 data-1-4 data-2-1 - data
script1 script2 script3 script4 script5 script6 script7 script8 script9 script10 - script
server1 server2 server3 - server
number0 number1 number2 number3 number4 number5 number6 number7 number8 number9 number10 number11 number12 number13 number14 number15 number16 - numbers
"""

INIT = """
(SCRIPT-IO script1 data-0-3 data-0-5 data-1-4)
(SCRIPT-IO script2 data-0-5 data-0-3 data-1-2)
(SCRIPT-IO script3 data-1-4 data-0-5 data-2-1)
(SCRIPT-IO script4 data-0-3 data-0-5 data-1-4)
(SCRIPT-IO script5 data-1-2 data-0-5 data-2-1)
(SCRIPT-IO script6 data-1-2 data-0-3 data-2-1)
(SCRIPT-IO script7 data-0-5 data-0-3 data-1-2)
(SCRIPT-IO script8 data-1-4 data-1-2 data-2-1)
(SCRIPT-IO script9 data-0-3 data-0-5 data-1-4)
(SCRIPT-IO script10 data-1-2 data-1-4 data-2-1)
(CONNECTED server1 server2)
(CONNECTED server2 server1)
(CONNECTED server1 server3)
(CONNECTED server3 server1)
(DATA-SIZE data-0-3 number4)
(DATA-SIZE data-0-5 number5)
(DATA-SIZE data-1-2 number4)
(DATA-SIZE data-1-4 number1)
(DATA-SIZE data-2-1 number4)
(CAPACITY server1 number16)
(CAPACITY server2 number8)
(CAPACITY server3 number8)
(saved data-0-3 server3)
(saved data-0-5 server1)
(usage server1 number0)
(usage server2 number0)
(usage server3 number0)
"""
# Removed functions for now

def object_facts_from_str(s):
    objs, ty = s.strip().rsplit(' - ', 1)
    return [(ty, obj) for obj in objs.split(' ')]

def fact_from_str(s):
    return tuple(s.strip('( )').split(' '))

def get_problem():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))

    # TODO: compare statistical success and the actual success
    bernoulli_fns = {
        #'test-open': fn_from_constant(P_SUCCESS),
    }

    # universe_test | empty_test
    stream_map = {name: from_test(CachedFn(test_from_bernoulli_fn(fn)))
                  for name, fn in bernoulli_fns.items()}

    init = [fact_from_str(s) for s in INIT.split('\n') if s]
    for line in OBJECTS.split('\n'):
        if line:
            init.extend(object_facts_from_str(line))

    goal_literals = [
        'saved data-2-1 server2',
    ]

    goal = And(*map(fact_from_str, goal_literals))

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def solve_pddlstream(num=1, **kwargs):
    # TODO: make a simulator that randomizes these probabilities
    # TODO: include local correlation
    stream_info = {
        'test-open': StreamInfo(p_success=P_SUCCESS), # TODO: p_success=lambda x: 0.5
    }

    problem = get_problem(**kwargs)
    dump_pddlstream(problem)

    successes = 0.
    for _ in range(num):
        print('\n'+'-'*5+'\n')
        #problem = get_problem(**kwargs)
        #solution = solve_incremental(problem, unit_costs=True, debug=True)
        solution = solve_focused(problem, stream_info=stream_info, unit_costs=True, unit_efforts=False, debug=True,
                                 #initial_complexity=1, max_iterations=1, max_skeletons=1
                                 )
        plan, cost, certificate = solution
        print_solution(solution)
        successes += is_plan(plan)
    print('Fraction {:.3f}'.format(successes / num))

##################################################

def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument('-v', '--visualize', action='store_true')
    args = parser.parse_args()
    solve_pddlstream()

if __name__ == '__main__':
    main()


# https://github.com/AI-Planning/classical-domains/tree/master/classical/data-network-opt18
# TODO: load the initial state from a problem file
# Packet sizes
# https://github.com/tomsilver/pddlgym/blob/master/rendering/tsp.py
# https://networkx.github.io/
# https://pypi.org/project/graphviz/
