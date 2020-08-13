#!/usr/bin/env python2.7

from __future__ import print_function

import os
import time

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.algorithms.focused import solve_focused

from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.utils import read, elapsed_time
from pddlstream.language.constants import print_solution

from pddlstream.language.generator import from_test, universe_test, from_gen_fn, from_gen
from pddlstream.language.constants import print_solution, PDDLProblem, And, Fact, get_args
from pddlstream.language.conversion import list_from_conjunction
from pddlstream.language.stream import WildOutput
from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, get_conjunctive_parts, get_literals


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

def get_problem(num=0):
    domain_pddl = read_pddl('domain.pddl')
    constant_map = {}
    stream_pddl = None
    stream_map = {}
    others = ['b{}'.format(i) for i in range(num)]

    init = [
        #('clear', others[0]),
        #('on', others[0], 'b'),
        ('on', 'b', 'a'),
        ('on-table', 'a'),
        #('on', 'a', others[0]),
        #('on-table', others[0]),
        ('clear', 'b'),
        ('arm-empty',),
        ('isblock', 'b'),
        ('isblock', 'a'),
    ]
    for block in others:
        init.extend([
            ('on-table', block),
            ('clear', block),
            ('isblock', block),
        ])

    #goal = ('on', 'a', 'b')
    goal = ('clear', 'a')

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def solve_pddlstream(focused=False):
    pddlstream_problem = get_problem()
    if focused:
        solution = solve_focused(pddlstream_problem, unit_costs=True)
    else:
        solution = solve_incremental(pddlstream_problem, unit_costs=True, debug=True)
    print_solution(solution)

def stuff():
    problem = get_problem()
    #stream_pddl = None
    stream_pddl = read_pddl('stream.pddl')

    all_objects = {o for fact in problem.init for o in get_args(fact)}
    goal_objects = set()
    for fact in list_from_conjunction(problem.goal):
        goal_objects.update(get_args(fact))
    unused_objects = all_objects - goal_objects

    new_init = []
    for fact in problem.init:
        #if set(get_args(fact)) & goal_objects:
        if set(get_args(fact)) <= goal_objects:
            new_init.append(fact)
    #new_init = problem.init
    print(problem.init)
    print(new_init)

    def test_clear(block):
        fact = ('clear', block)
        return fact in problem.init

    def sample_on(b1): # TODO: wild
        # TODO: return all other facts that are contained just to speed up the process
        for b2 in all_objects:
            fact = ('on', b2, b1)
            print(fact)
            if fact in problem.init:
                yield (b2,)

    stream_map = {
        'sample-block': from_gen((o,) for o in unused_objects),
        'test-block': from_test(universe_test),
        'test-arm-empty': from_test(universe_test),
        'test-clear': from_test(test_clear),
        'sample-on': from_gen_fn(sample_on),
    }

    start_time = time.time()
    pddlstream = PDDLProblem(problem.domain_pddl, problem.constant_map, stream_pddl, stream_map, new_init, problem.goal)
    solution = solve_incremental(pddlstream, unit_costs=True, verbose=True, debug=False)
    print_solution(solution)
    print(elapsed_time(start_time))

##################################################

def main():
    #solve_pddl()
    #solve_pddlstream()
    stuff()

if __name__ == '__main__':
    main()
