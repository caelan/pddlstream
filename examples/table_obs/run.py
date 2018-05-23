#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats

from pddlstream.conversion import And
from pddlstream.incremental import solve_incremental
from pddlstream.utils import print_solution, read, get_file_path

def is_class(item, cl):
    return cl in item

def get_pddlstream():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = None
    constant_map = {}

    init = [
        #('CanMove',),
    ]

    room = 'room'
    table = 'table'
    soup = 'soup'
    green = 'green'
    classes = [room, table, soup, green]

    initial_stackings = {
        'room0': None,
        'table0': 'room0',
        'table1': 'room0',
        'soup0': 'table0',
        'green0': 'table0',
    }

    problem = 'objects'
    if problem == 'observable':
        registered = [room, table, soup, green]
        localized = registered
    elif problem == 'objects':
        registered = [room, table]
        localized = registered + [soup, green]
    elif problem == 'tables':
        registered = ['room']
        localized = registered + ['table']
    elif problem == 'room':
        registered = []
        localized = registered + ['room']
    else:
        raise ValueError(problem)

    graspable_classes = [soup, green]
    for item in initial_stackings:
        init += [('Item', item)]
        for cl in filter(lambda c: is_class(item, c), classes):
            init += [('Class', item, cl)]
            if cl in graspable_classes:
                init += [('Graspable', item)] # TODO: include hand?
            if cl in localized:
                init += [('On', item, initial_stackings[item]), ('Localized', item)]
            else:
                init += [('Unknown', item)]
            if cl in registered:
                init += [('Registered', item)]

    stackable_classes = [(table, room), (soup, table), (green, table)]
    for cl1, cl2 in stackable_classes:
        for i1 in filter(lambda i: is_class(i, cl1), initial_stackings):
            for i2 in filter(lambda i: is_class(i, cl2), initial_stackings):
                init += [('Stackable', i1, i2)]

    arms = ['left', 'right']
    for arm in arms:
        init += [('Arm', arm), ('HandEmpty', arm)]

    goal_literals = [('On', 'soup0', 'table1'), ('Registered', 'soup0'), ('HoldingClass', 'green')]
    #goal_literals = [('Holding', 'left', 'soup0')]
    #goal_literals = [('HoldingClass', soup)]
    #goal_literals = [Nearby('table1')]
    #goal_literals = [('HoldingClass', 'green'), ('HoldingClass', 'soup')]
    goal = And(*goal_literals)

    stream_map = {}

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def main():
    pddlstream_problem = get_pddlstream()
    pr = cProfile.Profile()
    pr.enable()
    solution = solve_incremental(pddlstream_problem, unit_costs=False)
    print_solution(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)

if __name__ == '__main__':
    main()
