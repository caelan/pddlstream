#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.utils import read_pddl, irange, INF
from pddlstream.language.constants import get_length, PDDLProblem, print_solution
from pddlstream.language.generator import from_test, from_gen
from pddlstream.algorithms.incremental import solve_incremental

# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/domain0.pddl
# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/p0.pddl
# https://github.com/Emresav/ECAI16Domains/tree/master/experimental%20results

# TODO: rocket and car domains

def get_problem():
    domain_pddl = read_pddl(__file__, 'domain0.pddl')
    constant_map = {}
    stream_pddl = read_pddl(__file__, 'stream.pddl')
    #stream_pddl = None
    stream_map = {
        's-cash': from_gen((c,) for c in irange(INF)),
        't-geq': from_test(lambda c1, c2: c1 >= c2),
    }

    init = [
        ('person', 'Emre'),
        ('machine', 'atm1'),
        ('machine', 'atm2'),
        ('machine', 'atm3'),
        ('cash', 30),
        ('maxwithdraw', 'atm1', 30),
        ('maxwithdraw', 'atm2', 30),
        ('maxwithdraw', 'atm3', 30),
        ('inpocket', 'Emre', 2),
    ]
    goal = ('finished',)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def main():
    problem = get_problem()
    solution = solve_incremental(problem, unit_costs=True, debug=False)
    print_solution(solution)
    return

    domain_pddl = read_pddl(__file__, 'domain0.pddl')
    problem_pddl = read_pddl(__file__, 'problem0.pddl')

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
