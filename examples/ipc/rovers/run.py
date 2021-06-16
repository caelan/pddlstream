#!/usr/bin/env python

from __future__ import print_function

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.language.constants import get_length, read_pddl_pair

def dump_plan(plan, cost):
    solved = plan is not None
    print('Solved: {}'.format(solved))
    print('Cost: {}'.format(cost))
    print('Length: {}'.format(get_length(plan)))
    if not solved:
        return
    for i, action in enumerate(plan):
        print('{}) {}'.format(i+1, ' '.join(map(str, action))))

def main():
    # TODO: make a method that does this
    domain_pddl, problem_pddl = read_pddl_pair(__file__)
    plan, cost = solve_from_pddl(domain_pddl, problem_pddl)
    dump_plan(plan, cost)

if __name__ == '__main__':
    main()
