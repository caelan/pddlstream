#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_fn, from_test
from pddlstream.utils import print_solution, read, get_file_path

def test_feasible(fluents=set()):
    print(fluents)
    return True


def pddlstream_from_belief():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    stream_map = {
        'test-feasible': from_test(test_feasible),
    }

    init = [
        ('OnTable', 'b1'),
        ('OnTable', 'b2'),
    ]

    goal = ('Holding', 'b1')

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def main():
    # TODO: maybe load problems as a domain explicitly
    pddlstream_problem = pddlstream_from_belief()
    _, _, _, _, init, goal = pddlstream_problem
    print(sorted(init, key=lambda f: f[0]))
    print(goal)
    pr = cProfile.Profile()
    pr.enable()
    solution = solve_focused(pddlstream_problem, unit_costs=False)
    print_solution(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)

if __name__ == '__main__':
    main()
