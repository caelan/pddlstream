#!/usr/bin/env python2.7

from __future__ import print_function

import os

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.utils import read
from pddlstream.language.constants import get_length


def read_pddl(filename):
    directory = os.path.dirname(os.path.abspath(__file__))
    return read(os.path.join(directory, filename))

def main():
    domain_pddl = read_pddl('domain.pddl')
    problem_pddl = read_pddl('problem.pddl')

    plan, cost = solve_from_pddl(domain_pddl, problem_pddl)
    solved = plan is not None
    print('Solved: {}'.format(solved))
    print('Cost: {}'.format(cost))
    print('Length: {}'.format(get_length(plan)))
    if not solved:
        return
    for i, action in enumerate(plan):
        print('{}) {}'.format(i+1, ' '.join(map(str, action))))

if __name__ == '__main__':
    main()
