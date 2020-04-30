#!/usr/bin/env python2.7

from __future__ import print_function

import os
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl

from itertools import combinations

from pddlstream.algorithms.focused import solve_focused

from pddlstream.algorithms.downward import USE_FORBID
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_test, universe_test
from pddlstream.utils import read, get_file_path, INF
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream

MAX_DISTANCE = 15 # 15 | INF

DEFAULT_SIZE = mpl.rcParams['lines.markersize'] ** 2
BLACK = 'k'

def read_pddl(filename):
    directory = os.path.dirname(os.path.abspath(__file__))
    return read(os.path.join(directory, filename))

class Location(object):
    # Alternatively a set of properties
    def __init__(self, value, retailer=False, warehouse=False):
        self.value = np.array(value)
        self.retailer = retailer
        self.warehouse = warehouse
    def draw(self):
        marker = 'o'
        size = DEFAULT_SIZE
        color = BLACK
        if self.warehouse:
            marker = 's'
            size *= 2
            color = 'r'
        if self.retailer:
            marker = 's'
            size *= 2
            color = 'g'
        plt.scatter(*self.value, s=size, c=color, marker=marker, alpha=1)
    def __str__(self):
        return str(self.value)

##################################################

def draw_network(roads):
    # https://matplotlib.org/tutorials/introductory/pyplot.html
    locations = {location for road in roads for location in road}
    for location in locations:
        location.draw()
    for location1, location2 in roads:
        x1, y1 = location1.value
        x2, y2 = location2.value
        plt.plot([x1, x2], [y1, y2], c=BLACK)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()

def get_problem(draw=False):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    stream_map = {
        'test-open': from_test(universe_test),
    }

    # Trucks return to depots?
    trucks = [
        'truck1',
    ]
    packages = [
        'package1',
    ]

    retailers = [
        Location([10, 0], retailer=True),
    ]
    warehouses = [
        Location([-10, 0], warehouse=True),
    ]
    intersections = [
        Location([0, 10]),
        Location([0, -10]),
    ]
    locations = retailers + warehouses + intersections

    roads = set()
    for location1, location2 in combinations(locations, r=2):
        distance = np.linalg.norm(location2.value - location1.value, ord=2)
        if distance <= MAX_DISTANCE:
            roads.update({
                (location1, location2),
                (location2, location1),
            })

    if draw:
        draw_network(roads)
        # Draw trucks / packages

    init = []
    init += [('Location', location) for location in locations]
    init += [('Retailer', location) for location in retailers]
    init += [('Warehouse', location) for location in warehouses]
    init += [('Road', location1, location2) for location1, location2 in roads]
    for truck in trucks:
        init += [
            ('Truck', truck),
            ('AtLocation', truck, retailers[0]),
        ]
    for package in packages:
        init += [
            ('Package', package),
            ('AtLocation', package, warehouses[0]),
        ]

    goal_literals = [('AtLocation', package, retailers[0]) for package in packages]
    #goal_literals = [('Carrying', trucks[0], packages[0])]
    #goal_literals = [('AtLocation', trucks[0], warehouses[0])]
    goal = And(*goal_literals)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def solve_pddlstream():
    problem = get_problem()
    dump_pddlstream(problem)
    solution = solve_incremental(problem, unit_costs=True, debug=True)
    #solution = solve_focused(problem, unit_costs=True)
    print_solution(solution)

##################################################

def main():
    solve_pddlstream()

if __name__ == '__main__':
    main()
