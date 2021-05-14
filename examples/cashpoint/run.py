#!/usr/bin/env python

from __future__ import print_function

from random import uniform

from pddlstream.algorithms.meta import solve, create_parser
from examples.cashpoint.optimizer import create_optimizer
from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.utils import read_pddl, Profiler
from pddlstream.language.constants import get_length, PDDLProblem, print_solution
from pddlstream.language.generator import from_test, from_fn, from_sampler
from pddlstream.language.stream import StreamInfo, PartialInputs
from examples.ipc.rovers.run import dump_plan

# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/domain0.pddl
# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/p0.pddl
# https://github.com/Emresav/ECAI16Domains/tree/master/experimental%20results

# TODO: rocket and car domains

def get_problem(optimize=True):
    initial_atm = 30
    initial_person = 2

    min_take = 1
    #max_take = 10
    max_take = initial_atm
    target = 50 # 1 | 3 | 50
    num_atms = 2

    domain_pddl = read_pddl(__file__, 'domain0.pddl')
    constant_map = {
        #'@min': min_take,
        #'@max': max_take,
        '@amount': target,
    }

    if optimize:
        stream_pddl = read_pddl(__file__, 'optimizer.pddl')
    else:
        stream_pddl = read_pddl(__file__, 'stream.pddl')

    # TODO: store history of withdraws to ensure not taking too much
    #stream_pddl = None
    stream_map = {
        #'s-cash': from_gen((c,) for c in randomize(irange(min_take, max_take + 1))),
        #'s-cash': from_gen((c,) for c in reversed(list(irange(min_take, max_take+1)))),
        's-cash': from_sampler(lambda: (round(uniform(min_take, max_take), 2),)),
        't-ge': from_test(lambda c1, c2: c1 >= c2),
        'add': from_fn(lambda c1, c2: (c1 + c2,)),
        'subtract': from_fn(lambda c3, c2: (c3 - c2,) if c3 - c2 >= 0 else None),
        'withdraw': from_fn(lambda wc, pc1, mc1: (pc1 + wc, mc1 - wc) if mc1 - wc >= 0 else None),
        'withdrawcost': lambda c: c, # TODO: withdraw fee
        'gurobi': create_optimizer(min_take, max_take),
    }

    person = 'Emre'
    #initial_people = {'Emre': initial_person}
    #initial_atms = {'Emre': initial_person}
    #amounts = [min_take, max_take, target, initial_atm, initial_person]

    init = [
        ('person', person),
        ('pcash', initial_person),
        ('pcash', target),
        ('tcash', target),
        ('inpocket', person, initial_person),
    ] # + [('cash', amount) for amount in amounts]

    for num in range(num_atms):
        name = 'atm{}'.format(num+1)
        init.extend([
            ('machine', name),
            ('mcash', initial_atm),
            ('maxwithdraw', name, initial_atm),
        ])

    #goal = Exists(['?c1'], And(('person', person), ('ge', '?c1', target),
    #                           ('inpocket', person, '?c1')))
    goal = ('finished',) # TODO: focused bug when goal initially satisfied

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

##################################################

def solve_pddlstream(planner='max-astar'):
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    problem = get_problem(optimize=True)
    print(problem.constant_map)
    print(problem.init)
    print(problem.goal)

    stream_info = {
        't-ge': StreamInfo(eager=True),
        'withdraw': StreamInfo(opt_gen_fn=PartialInputs(unique=True)),
    }
    with Profiler(field='cumtime'):
        solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit,
                         stream_info=stream_info, planner=planner,
                         initial_complexity=3, clean=False, debug=True, verbose=True)

    print_solution(solution)
    plan, cost, certificate = solution
    print('Certificate:', certificate.preimage_facts)

##################################################

def solve_pddl():
    domain_pddl = read_pddl(__file__, 'domain0.pddl')
    problem_pddl = read_pddl(__file__, 'problem0.pddl')
    plan, cost = solve_from_pddl(domain_pddl, problem_pddl)
    dump_plan(plan, cost)

def main():
    #solve_pddl()
    solve_pddlstream()

if __name__ == '__main__':
    main()
