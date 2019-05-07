#!/usr/bin/env python2.7

from __future__ import print_function

import os

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.algorithms.focused import solve_focused

from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.utils import read
from pddlstream.language.constants import print_solution


def read_pddl(filename):
    directory = os.path.dirname(os.path.abspath(__file__))
    return read(os.path.join(directory, filename))

##################################################

def solve_pddl():
    domain_pddl = read_pddl('domain.pddl')
    problem_pddl = read_pddl('problem.pddl')

    plan, cost = solve_from_pddl(domain_pddl, problem_pddl)
    print('Plan:', plan)
    print('Cost:', cost)

##################################################

def get_problem():
    domain_pddl = read_pddl('domain.pddl')
    constant_map = {}
    stream_pddl = None
    stream_map = {}

    init = [
        ('on-table', 'a'),
        ('on', 'b', 'a'),
        ('clear', 'b'),
        ('arm-empty',),
    ]
    goal =  ('on', 'a', 'b')

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

def solve_pddlstream(focused=False):
    pddlstream_problem = get_problem()
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=True)
    else:
        solution = solve_incremental(pddlstream_problem, unit_costs=True, planner='cerberus', debug=False)
    print_solution(solution)

##################################################

def main():
    #solve_pddl()
    solve_pddlstream()

if __name__ == '__main__':
    main()
