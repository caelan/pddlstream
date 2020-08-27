#!/usr/bin/env python2.7

from __future__ import print_function

from random import uniform

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.utils import read_pddl, irange, INF, randomize, Profiler
from pddlstream.language.constants import get_length, PDDLProblem, print_solution, Exists, And
from pddlstream.language.generator import from_test, from_gen, from_fn, from_sampler
from pddlstream.algorithms.incremental import solve_incremental

# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/domain0.pddl
# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/p0.pddl
# https://github.com/Emresav/ECAI16Domains/tree/master/experimental%20results

# TODO: rocket and car domains

def get_problem():
    initial_atm = 30
    initial_person = 2

    min_take = 1
    #max_take = 10
    max_take = initial_atm
    target = 50

    domain_pddl = read_pddl(__file__, 'domain0.pddl')
    constant_map = {
        '@min': min_take,
        '@max': max_take,
        '@amount': target,
    }
    stream_pddl = read_pddl(__file__, 'stream.pddl')
    #stream_pddl = None
    stream_map = {
        #'s-cash': from_gen((c,) for c in randomize(irange(min_take, max_take + 1))),
        #'s-cash': from_gen((c,) for c in reversed(list(irange(min_take, max_take+1)))),
        's-cash': from_sampler(lambda: (round(uniform(min_take, max_take), 2),)),
        't-ge': from_test(lambda c1, c2: c1 >= c2),
        'add': from_fn(lambda c1, c2: (c1 + c2,)),
        'subtract': from_fn(lambda c3, c2: (c3 - c2,) if c3 - c2 >= 0 else None),
    }

    person = 'Emre'
    #initial_people = {'Emre': initial_person}
    #initial_atms = {'Emre': initial_person}
    #amounts = [min_take, max_take, target, initial_atm, initial_person]

    init = [
        ('machine', 'atm1'),
        ('machine', 'atm2'),
        ('machine', 'atm3'),
        ('mcash', initial_atm),
        ('maxwithdraw', 'atm1', initial_atm),
        ('maxwithdraw', 'atm2', initial_atm),
        ('maxwithdraw', 'atm3', initial_atm),

        ('person', person),
        ('pcash', initial_person),
        ('pcash', target),
        ('tcash', target),
        ('inpocket', person, initial_person),
    ] # + [('cash', amount) for amount in amounts]

    goal = Exists(['?c1'], And(('person', person), ('inpocket', person, '?c1'), ('ge', '?c1', target)))
    #goal = ('finished',)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def solve_pddlstream():
    problem = get_problem()
    print(problem.constant_map)
    print(problem.init)
    print(problem.goal)
    with Profiler(field='cumtime'):
        solution = solve_incremental(problem, planner='max-astar', unit_costs=True,
                                     hierarchy=None, debug=True, verbose=True)
        print_solution(solution)

def solve_pddl():
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

def main():
    #solve_pddl()
    solve_pddlstream()

if __name__ == '__main__':
    main()
