#!/usr/bin/env python2.7

from __future__ import print_function

import os
import time
import random

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.algorithms.focused import solve_focused

from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.utils import read
from pddlstream.language.constants import print_solution, And, Fact
from pddlstream.language.generator import from_fn, from_gen_fn, from_test


from pddlstream.language.generator import from_test, universe_test
from pddlstream.language.constants import print_solution, PDDLProblem, And
from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, get_conjunctive_parts, get_literals

def read_pddl(filename):
    directory = os.path.dirname(os.path.abspath(__file__))
    return read(os.path.join(directory, filename))

##################################################

def solve_pddl():
    domain_pddl = read_pddl('domain.pddl')
    problem_pddl = read_pddl('problem.pddl')

    start = time.time()
    plan, _ = solve_from_pddl(domain_pddl, problem_pddl, debug=True)
    print('Plan:', plan)
    print('Time taken:', time.time()-start)

##################################################

def get_problem():
    domain_pddl = read_pddl('domain.pddl')
    constant_map = {}
    stream_pddl = read_pddl('stream.pddl')

    goal_tower = [
        "block33",
        "block82",
        "block51",
        "block13",
    ]

    def _sample_block():
        num_blocks = 100
        ordered_lst = list(range(num_blocks))
        # random.shuffle(ordered_lst)
        for i in ordered_lst:
            block = "block{}".format(i)
            if block not in goal_tower:
                yield ("block{}".format(i),)

    def _sample_goal_block():
        for goal_block in goal_tower:
            yield (goal_block,)

    stream_map = {
        'sample-robot': from_fn(lambda: ("rohanrobot",)),
        'sample-block': from_gen_fn(_sample_block),
        'sample-goal-block': from_gen_fn(_sample_goal_block),
        'test-on': from_test(lambda x, y: False),  # all blocks start on table
        'test-ontable': from_test(lambda x: True),  # all blocks start on table
        'test-clear': from_test(lambda x: True),  # all blocks start on table
        'test-handempty': from_test(lambda robot: True),  # robot starts off empty-handed
        'test-handfull': from_test(lambda robot: False),  # robot starts off empty-handed
        'test-holding': from_test(lambda x: False),  # robot starts off empty-handed
        'test-pickup': from_test(lambda x: True),  # action literals are always true
        'test-putdown': from_test(lambda x: True),  # action literals are always true
        'test-stack': from_test(lambda x, y: True),  # action literals are always true
        'test-unstack': from_test(lambda x: True),  # action literals are always true
    }

    init = []
    goal_lits = []
    for i in range(len(goal_tower)-1):
        goal_lits.append(('on', goal_tower[i], goal_tower[i+1]))
    goal = And(*goal_lits)

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

def stuff():
    domain_pddl = read_pddl('domain.pddl')
    problem_pddl = read_pddl('problem.pddl')
    stream_pddl = read_pddl('stream.pddl')

    domain = parse_sequential_domain(domain_pddl)
    problem = parse_problem(domain, problem_pddl)
    print(problem)

    goal_literals = []
    goal_objects = set()
    for literal in get_literals(problem.goal):
        goal_objects.update(literal.args)
        assert not literal.negated
        goal_literals.append(Fact(literal.predicate, literal.args))


    new_init = []
    for fact in problem.init:
        #if set(fact.args) & goal_objects:
        new_init.append(Fact(fact.predicate, fact.args))

    goal = And(*goal_literals)

    constant_map = {}
    stream_pddl = None
    stream_map = None

    problem = PDDLProblem(domain_pddl, constant_map,stream_pddl, stream_map, new_init, goal)
    solution = solve_incremental(problem, unit_costs=True, debug=False)
    print_solution(solution)


def solve_pddlstream():
    pddlstream_problem = get_problem()
    start = time.time()
    solution = solve_incremental(pddlstream_problem, unit_costs=True, debug=False)
    # solution = solve_focused(pddlstream_problem, unit_costs=True, debug=False)
    print_solution(solution)
    print('Time taken:', time.time()-start)

##################################################

def main():
    #solve_pddl()
    #solve_pddlstream()
    stuff()

if __name__ == '__main__':
    main()
