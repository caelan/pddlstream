#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats

from pddlstream.conversion import And
from pddlstream.incremental import solve_incremental
from pddlstream.utils import print_solution, read, get_file_path

def get_pddlstream():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = None
    constant_map = {}

    table = 'table'
    soup = 'soup'
    left = 'left'
    right = 'right'

    init = [
        ('CanMove',),
        ('Arm', left),
        ('Arm', right),
        ('HandEmpty', left),
        ('HandEmpty', right),

        ('Found', 'table0'),
        ('Found', 'table1'),
        ('Class', 'table0', table),
        ('Class' 'table1', table),
        ('Fixed', 'table0'),
        ('Fixed', 'table1'),

        ('Stackable', 'soup0', 'table0'),
        ('Stackable', 'soup0', 'table1'),
        ('Class', 'soup0', soup),
        ('Found', 'soup0'),
        ('On', 'soup0', 'table0'),

        ('Stackable', 'green0', 'table0'),
        ('Stackable', 'green0', 'table1'),
        ('Class', 'green0', 'green'),
        ('Found', 'green0'),
        ('On', 'green0', 'table0'),
    ]

    #goal_literals = [('On', 'soup0', 'table1'), ('Localized', 'soup0')]
    #goal_literals = [('Holding', 'left', 'soup0')]
    #goal_literals = [('HoldingClass', soup)]
    #goal_literals = [Nearby('table1')]
    goal_literals = [('HoldingClass', 'green'), ('HoldingClass', 'soup')]
    goal = And(*goal_literals)

    stream_map = {}

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def main(unit_costs=False):
    pddlstream_problem = get_pddlstream()
    pr = cProfile.Profile()
    pr.enable()
    solution = solve_incremental(pddlstream_problem, layers=1, unit_costs=unit_costs)
    print_solution(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)

if __name__ == '__main__':
    main()
