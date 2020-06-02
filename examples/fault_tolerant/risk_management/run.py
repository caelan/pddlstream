#!/usr/bin/env python2.7

from __future__ import print_function

import argparse
import os
import time
import random

import pddlstream.algorithms.downward
pddlstream.algorithms.downward.USE_FORBID = True

from collections import defaultdict

from pddlstream.algorithms.scheduling.diverse import p_disjunction
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_test, fn_from_constant
from pddlstream.language.stream import StreamInfo, DEBUG
from pddlstream.language.external import defer_unique
from pddlstream.utils import read, get_file_path, safe_rm_dir, INF, Profiler, elapsed_time
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream, \
    is_plan, get_prefix, get_args, Not, StreamAction
from examples.fault_tolerant.logistics.run import test_from_bernoulli_fn, CachedFn
from examples.fault_tolerant.data_network.run import fact_from_str
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, \
    task_from_domain_problem, is_literal, get_conjunctive_parts, TEMP_DIR

P_SUCCESS = 0.75

RISK_DIR = 'risk-pddl/risk/'

class hashabledict(dict):
    # assumes immutable once hashed
    def __hash__(self):
        return hash(frozenset(self.items()))

def fact_from_fd(literal):
    assert is_literal(literal)
    atom = (literal.predicate,) + literal.args
    if literal.negated:
        return Not(atom)
    return atom

##################################################

def get_problem(*kwargs):
    safe_rm_dir(TEMP_DIR)
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    domain = parse_sequential_domain(domain_pddl)
    constant_map = {}

    risk_path = get_file_path(__file__, RISK_DIR)
    problem_paths = [os.path.join(risk_path, f) for f in sorted(os.listdir(risk_path))
                     if f.startswith('prob') and f.endswith('.pddl')]

    # Size increases every 20 problems
    # for problem_path in problem_paths:
    #     problem_pddl = read(problem_path)
    #     problem = parse_problem(domain, problem_pddl)
    #     print(os.path.basename(problem_path), len(problem.objects))

    #index = random.randint(0, 19)
    index = 0
    #index = None
    print('Problem index:', index)
    if index is None:
        problem_path = get_file_path(__file__, 'problem.pddl')
    else:
        problem_path = problem_paths[index]

    problem_pddl = read(problem_path)
    problem = parse_problem(domain, problem_pddl)
    #task = task_from_domain_problem(domain, problem) # Uses Object
    #print(problem.objects)

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

    # universe_test | empty_test
    stream_map = {
        #'test-connected': from_test(lambda x, y: True),
        # TODO: make probabilities depend on the alphabetical/numeric distance
        #'test-connected': from_test(lambda *args: args in atoms_from_predicate['CONNECTED']),
    }

    # TODO: visualize using networks
    def connected_bernoulli_fn(*args, **kwargs):
        if not args:
            return P_SUCCESS
        return P_SUCCESS

    bernoulli_fns = {name: CachedFn(fn) for name, fn in {
        'test-connected': connected_bernoulli_fn,
    }.items()}
    stream_map.update({name: from_test(CachedFn(test_from_bernoulli_fn(fn)))
                       for name, fn in bernoulli_fns.items()})

    pddl_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return pddl_problem, bernoulli_fns

def simulate_successes(stochastic_fns, solutions, n_simulations):
    successes = 0
    plans = [plan for plan, _, _ in solutions]
    if not plans:
        return successes
    for _ in range(n_simulations):
        streams = set()
        for plan in plans:
            streams.update(stream for stream in plan if isinstance(stream, StreamAction))

        # TODO: compare with exact computation from p_disjunction
        outcomes = {}
        for stream in streams:
            name, inputs, outputs = stream
            assert not outputs
            outcomes[stream] = stochastic_fns[name](*inputs)
            # total = sum(stochastic_fns[name](*inputs) for _ in range(n_simulations))
            # print(float(total) / n_simulations)

        for plan in plans:
            stream_plan = [stream for stream in plan if isinstance(stream, StreamAction)]
            if all(outcomes[stream] for stream in stream_plan):
                successes += 1
                break
    return successes

def solve_pddlstream(n_trials=1, n_simulations=10000, **kwargs):
    total_time = time.time()
    problem, bernoulli_fns = get_problem(**kwargs)
    #dump_pddlstream(problem)
    stream_info = {name: StreamInfo(p_success=fn, defer_fn=defer_unique)
                   for name, fn in bernoulli_fns.items()}

    stochastic_fns = {name: test_from_bernoulli_fn(cached)
                      for name, cached in bernoulli_fns.items()}

    configs = [
        {'candidates': 10, 'selector': 'greedy', 'k': 3, 'd': INF, 'max_time': INF},
    ]

    selectors = ['random', 'greedy', 'exact']
    ks = list(range(1, 1+10))
    configs = [{'selector': selector, 'k': k, 'max_time': 30}
               for selector in selectors for k in ks]

    configs = list(map(hashabledict, configs))
    successes = defaultdict(float)
    runtimes = defaultdict(float)
    for _ in range(n_trials):
        print('\n'+'-'*100+'\n')
        for config in configs:
            print(config)
            trial_time = time.time()
            #problem = get_problem(**kwargs)
            #solution = solve_incremental(problem, unit_costs=True, debug=True)
            # TODO: return the actual success rate of the portfolio (maybe in the cost)?
            solutions = solve_focused(problem, stream_info=stream_info,
                                      unit_costs=True, unit_efforts=False, debug=False,
                                      initial_complexity=1, max_iterations=1, max_skeletons=None,
                                      max_planner_time=10, replan_actions=['enter'],
                                      diverse=config)
            runtimes[config] += elapsed_time(trial_time)
            for solution in solutions:
                print_solution(solution)
            #successes += is_plan(plan)
            successes[config] += simulate_successes(stochastic_fns, solutions, n_simulations)

    attempts = n_trials*n_simulations
    print('Configs: {} | Attempts: {} | Total Time: {:.3f}'.format(
        len(configs), attempts, elapsed_time(total_time)))
    for i, config in enumerate(configs):
        print('{}) {} | Probability {:.3f} | Mean Time: {:.3f}'.format(
            i, config, successes[config] / attempts, runtimes[config] / n_trials))

    import matplotlib.pyplot as plt
    plt.figure()
    configs_from_name = defaultdict(list)
    for config in configs:
        configs_from_name[config['selector']].append(config)
    for name in configs_from_name:
        ks = [config['k'] for config in configs_from_name[name]]
        probabilities = [successes[config] / attempts for config in configs_from_name[name]]
        #plt.fill_between(train_sizes, test_scores_mean - width, test_scores_mean + width, alpha=0.1)
        plt.plot(ks, probabilities, 'o-', label=name)

    plt.title('Selector Probability of Success')
    #plt.ylim(0, 1)
    plt.xlabel('K')
    plt.ylabel('Probability of Success')
    plt.grid()
    plt.legend(loc='best')
    plt.tight_layout()

    # if figure_dir is not None:
    #     mkdir(figure_dir)
    #     figure_path = os.path.join(figure_dir, '{}.png'.format(y_label)) # pdf
    #     plt.savefig(figure_path, transparent=False, bbox_inches='tight') # dpi=1000,
    #     print('Saved', figure_path)
    plt.show()

##################################################

def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument('-v', '--visualize', action='store_true')
    args = parser.parse_args()
    with Profiler():
        solve_pddlstream()

if __name__ == '__main__':
    main()

# https://github.com/IBM/risk-pddl
