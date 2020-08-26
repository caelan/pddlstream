#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.utils import read_pddl, irange, INF
from pddlstream.language.constants import get_length, PDDLProblem, print_solution, Exists, And
from pddlstream.language.generator import from_test, from_gen, from_fn
from pddlstream.algorithms.incremental import solve_incremental

# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/domain0.pddl
# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/p0.pddl
# https://github.com/Emresav/ECAI16Domains/tree/master/experimental%20results

# TODO: rocket and car domains

def get_problem():
    min_deposit = 0
    max_deposit = 10
    target = 3
    initial_atm = 30
    initial_person = 2

    domain_pddl = read_pddl(__file__, 'domain0.pddl')
    constant_map = {
        '@amount': target,
        '@min': min_deposit,
        '@max': max_deposit,
    }
    stream_pddl = read_pddl(__file__, 'stream.pddl')
    #stream_pddl = None
    stream_map = {
        's-cash': from_gen((c,) for c in irange(1, 2)),
        't-ge': from_test(lambda c1, c2: c1 >= c2),
        'add': from_fn(lambda c1, c2: (c1 + c2,)),
        'subtract': from_fn(lambda c3, c2: (c3 - c2,) if c3 - c2 >= 0 else None),
    }

    #initial_people = {'Emre': initial_person}
    #initial_atms = {'Emre': initial_person}
    amounts = [min_deposit, max_deposit, target, initial_atm, initial_person]

    init = [
        ('person', 'Emre'),
        ('machine', 'atm1'),
        #('machine', 'atm2'),
        #('machine', 'atm3'),
        ('maxwithdraw', 'atm1', initial_atm),
        #('maxwithdraw', 'atm2', initial_atm),
        #('maxwithdraw', 'atm3', initial_atm),
        ('inpocket', 'Emre', initial_person),
    ] + [('cash', amount) for amount in amounts]

    goal = Exists(['?c1'], And(('inpocket', 'Emre', '?c1'), ('ge', '?c1', target)))
    #goal = ('finished',)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def main():
    problem = get_problem()
    print(problem.constant_map)
    print(problem.init)
    print(problem.goal)
    solution = solve_incremental(problem, unit_costs=True, debug=False, verbose=True)
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
