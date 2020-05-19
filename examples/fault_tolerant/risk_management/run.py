#!/usr/bin/env python2.7

from __future__ import print_function

import argparse
import os

import pddlstream.algorithms.scheduling.diverse
pddlstream.algorithms.scheduling.diverse.DEFAULT_K = 1

import pddlstream.algorithms.downward
pddlstream.algorithms.downward.USE_FORBID = True

from collections import defaultdict

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_test
from pddlstream.language.stream import StreamInfo, DEBUG
from pddlstream.utils import read, get_file_path, safe_rm_dir
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream, \
    is_plan, get_prefix, get_args, Not
from examples.fault_tolerant.logistics.run import test_from_bernoulli_fn, CachedFn
from examples.fault_tolerant.data_network.run import fact_from_str
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, \
    task_from_domain_problem, is_literal, get_conjunctive_parts, TEMP_DIR

P_SUCCESS = 0.9

RISK_DIR = 'risk-pddl/risk/'

def fact_from_fd(literal):
    assert is_literal(literal)
    atom = (literal.predicate,) + literal.args
    if literal.negated:
        return Not(atom)
    return atom

def get_problem(*kwargs):
    safe_rm_dir(TEMP_DIR)
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    domain = parse_sequential_domain(domain_pddl)
    constant_map = {}

    problem_pddl = read(get_file_path(__file__, 'problem.pddl'))
    problem = parse_problem(domain, problem_pddl)
    #task = task_from_domain_problem(domain, problem) # Uses Object
    #print(problem.objects)

    risk_path = get_file_path(__file__, RISK_DIR)
    problem_paths = [os.path.join(risk_path, f) for f in os.listdir(risk_path)
                     if f.startswith('prob') and f.endswith('.pddl')]

    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    #stream_pddl = None

    # TODO: introduce object more generally
    # init = [fact_from_str(s) for s in INIT.split('\n') if s]
    # objects = {n for f in init for n in get_args(f)}
    # atoms_from_predicate = defaultdict(set)
    # for fact in init:
    #     atoms_from_predicate[get_prefix(fact)].add(get_args(fact))
    #
    # init = [f for f in init if get_prefix(f) not in ['CONNECTED']]
    # init.extend(('OBJECT', n) for n in objects)
    #
    # goal_literals = [fact_from_str(s) for s in GOAL.split('\n') if s]
    # goal = And(*goal_literals)

    init = list(map(fact_from_fd, filter(is_literal, problem.init)))
    goal = And(*map(fact_from_fd, get_conjunctive_parts(problem.goal)))

    # TODO: compare statistical success and the actual success
    bernoulli_fns = {
        #'test-open': fn_from_constant(P_SUCCESS),
    }

    # universe_test | empty_test
    stream_map = {
        'test-connected': from_test(lambda x, y: True),
        # TODO: make probabilities depend on the alphabetical/numeric distance
        #'test-connected': from_test(lambda *args: args in atoms_from_predicate['CONNECTED']),
    }
    stream_map.update({name: from_test(CachedFn(test_from_bernoulli_fn(fn)))
                       for name, fn in bernoulli_fns.items()})

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def solve_pddlstream(num=1, **kwargs):
    stream_info = {
        'test-connected': StreamInfo(p_success=P_SUCCESS),
    }

    problem = get_problem(**kwargs)
    dump_pddlstream(problem)

    successes = 0.
    for _ in range(num):
        print('\n'+'-'*5+'\n')
        #problem = get_problem(**kwargs)
        #solution = solve_incremental(problem, unit_costs=True, debug=True)
        solution = solve_focused(problem, stream_info=stream_info,
                                 unit_costs=True, unit_efforts=False, debug=False,
                                 initial_complexity=1, max_iterations=1, max_skeletons=1
                                 )
        plan, cost, certificate = solution
        print_solution(solution)
        successes += is_plan(plan)
    print('Fraction {:.3f}'.format(successes / num))

##################################################

def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument('-v', '--visualize', action='store_true')
    args = parser.parse_args()
    solve_pddlstream()

if __name__ == '__main__':
    main()

# https://github.com/IBM/risk-pddl
