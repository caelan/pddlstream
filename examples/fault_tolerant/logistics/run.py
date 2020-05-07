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
pddlstream.algorithms.scheduling.diverse.DEFAULT_K = 2

import pddlstream.algorithms.downward
pddlstream.algorithms.downward.USE_FORBID = True

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.scheduling.diverse import p_disjunction
from pddlstream.language.generator import from_test, universe_test, empty_test, fn_from_constant
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, get_file_path, INF, hash_or_id
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream, is_plan

MAX_DISTANCE = 15 # 15 | INF
P_SUCCESS = 0.75

DEFAULT_SIZE = mpl.rcParams['lines.markersize'] ** 2
BLACK = 'k'

def test_from_bernoulli_fn(bernoulli_fn):
    return lambda *args, **kwargs: random.random() < bernoulli_fn(*args, **kwargs)

class CachedFn(object):
    def __init__(self, fn):
        self.fn = fn
        self.cache = {}
    def __call__(self, *args): #, **kwargs):
        key = tuple(map(hash_or_id, args)) # Add the type
        if key not in self.cache:
            self.cache[key] = self.fn(*args)
        return self.cache[key]
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.fn)

##################################################

class Location(object):
    # Alternatively a set of properties
    counter = count()
    def __init__(self, value, retailer=False, warehouse=False):
        self.value = np.array(value)
        self.retailer = retailer
        self.warehouse = warehouse
        self.num = next(Location.counter)
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
    def __repr__(self):
        return 'l{}'.format(self.num)
        #return str(self.value)

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
    plt.tight_layout(pad=0)
    plt.show()

def get_problem(visualize=False):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))

    # TODO: compare statistical success and the actual success
    bernoulli_fns = {
        'test-open': fn_from_constant(P_SUCCESS),
    }

    # universe_test | empty_test
    stream_map = {name: from_test(CachedFn(test_from_bernoulli_fn(fn)))
                  for name, fn in bernoulli_fns.items()}

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

    if visualize:
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
        solution = solve_focused(problem, stream_info=stream_info, unit_costs=True, unit_efforts=False, debug=False,
                                 initial_complexity=1, max_iterations=1, max_skeletons=1)
        plan, cost, certificate = solution
        print_solution(solution)
        successes += is_plan(plan)
    print('Fraction {:.3f}'.format(successes / num))

##################################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--visualize', action='store_true')
    args = parser.parse_args()
    solve_pddlstream(visualize=args.visualize)

if __name__ == '__main__':
    main()
