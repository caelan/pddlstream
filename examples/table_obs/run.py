#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats

from examples.discrete_belief.dist import DeltaDist, MixtureDist, UniformDist
from examples.discrete_belief.run import revisit_mdp_cost
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, Equal, print_solution
from pddlstream.utils import read, get_file_path, INF

ROOM = 'room'
TABLE = 'table'
SOUP = 'soup'
GREEN = 'green'
CLASSES = [ROOM, TABLE, SOUP, GREEN]

def is_class(item, cl):
    assert(cl in CLASSES)
    return cl in item

def pddlstream_from_belief(initial_belief):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = None
    stream_map = {}

    init = [
        #('CanMove',),
        Equal(('RegisterCost',), 1),
        Equal(('PickCost',), 1), # TODO: imperfect transition model
        Equal(('PlaceCost',), 1),
    ]

    for item, dist in initial_belief.items():
        support = dist.support()
        if len(support) == 1:
            init += [('On', item, support[0]), ('Localized', item)]
        else:
            init += [('Unknown', item)]
            for i2 in support:
                p_obs = dist.prob(i2)
                cost = revisit_mdp_cost(1, 1, p_obs) # TODO: imperfect observation model
                if cost == INF:
                    continue
                if i2 in initial_belief:
                    init += [('FiniteScanCost', i2, item),
                             Equal(('ScanCost', i2, item), cost)]

    graspable_classes = [SOUP, GREEN]
    for item in initial_belief:
        for cl in filter(lambda c: is_class(item, c), CLASSES):
            init += [('Class', item, cl)]
            if cl in graspable_classes:
                init += [('Graspable', item)] # TODO: include hand?

    stackable_classes = [(TABLE, ROOM), (SOUP, TABLE), (GREEN, TABLE)]
    for cl1, cl2 in stackable_classes:
        for i1 in filter(lambda i: is_class(i, cl1), initial_belief):
            for i2 in filter(lambda i: is_class(i, cl2), initial_belief):
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

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

WORLD = 'world'
OTHER = 'other'

def get_room_belief(uniform_rooms, uniform_tables, uncertainty):
    mix = 1 - uncertainty
    return {
        'room0': DeltaDist(WORLD),
        'table0': MixtureDist(DeltaDist('room0'), uniform_rooms, mix),
        'table1': MixtureDist(DeltaDist('room0'), uniform_rooms, mix),
        'soup0': MixtureDist(DeltaDist('table0'), uniform_tables, mix),
        'green0': MixtureDist(DeltaDist('table0'), uniform_tables, mix),
    }

def get_table_belief(uniform_tables, uncertainty):
    mix = 1 - uncertainty
    return {
        'room0': DeltaDist(WORLD),
        'table0': DeltaDist('room0'),
        'table1': DeltaDist('room0'),
        'soup0': MixtureDist(DeltaDist('table0'), uniform_tables, mix),
        'green0': MixtureDist(DeltaDist('table0'), uniform_tables, mix),
    }

def get_item_belief():
    return {
        'room0': DeltaDist(WORLD),
        'table0': DeltaDist('room0'),
        'table1': DeltaDist('room0'),
        'soup0': DeltaDist('table0'),
        'green0': DeltaDist('table0'),
    }

##################################################

def main():
    uniform_rooms = UniformDist(['room0', OTHER])
    #uniform_tables = UniformDist(['table0', 'table1'])
    #uniform_tables = UniformDist(['table0', OTHER])
    uniform_tables = UniformDist(['table0', 'table1', OTHER])

    #initial_belief = get_room_belief(uniform_rooms, uniform_tables, 1.0)
    initial_belief = get_room_belief(uniform_rooms, uniform_tables, 0.2)
    #initial_belief = get_table_belief(uniform_tables, 1.0)
    #initial_belief = get_table_belief(uniform_tables, 0.2)
    #initial_belief = get_item_belief()

    pddlstream_problem = pddlstream_from_belief(initial_belief)
    _, _, _, _, init, goal = pddlstream_problem
    print(sorted(init))
    print(goal)
    pr = cProfile.Profile()
    pr.enable()
    planner = 'max-astar'
    #solution = solve_incremental(pddlstream_problem, planner=planner, unit_costs=False)
    solution = solve_focused(pddlstream_problem, planner=planner, unit_costs=False)
    print_solution(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)

if __name__ == '__main__':
    main()
