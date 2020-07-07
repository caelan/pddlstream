#!/usr/bin/env python2.7

from __future__ import print_function

import argparse
import os
import time
import random
import numpy as np

from collections import defaultdict
from itertools import product
from multiprocessing import cpu_count, Pool

from pddlstream.algorithms.scheduling.diverse import p_disjunction
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.constraints import PlanConstraints
#from pddlstream.algorithms.scheduling.plan_streams import *

from pddlstream.language.generator import from_test, fn_from_constant
from pddlstream.language.stream import StreamInfo, DEBUG
from pddlstream.language.external import defer_unique
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream, \
    is_plan, get_prefix, get_args, Not, StreamAction, Equal
from pddlstream.utils import read, get_file_path, safe_rm_dir, INF, Profiler, elapsed_time, read_pickle, write_pickle
from examples.fault_tolerant.logistics.run import test_from_bernoulli_fn, CachedFn
from examples.fault_tolerant.data_network.run import fact_from_str
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, \
    task_from_domain_problem, is_literal, is_assignment, get_conjunctive_parts, TEMP_DIR, set_cost_scale #, evaluation_from_fd, fact_from_fd
from examples.pybullet.utils.pybullet_tools.utils import is_darwin #, write_json, read_json

from matplotlib.ticker import MaxNLocator

P_SUCCESSES = [0.9]
#P_SUCCESSES = np.linspace(n=4, endpoint=False)

RISK_DIR = 'risk-pddl/risk/'

class hashabledict(dict):
    def __setitem__(self, key, value):
        raise RuntimeError()
    # assumes immutable once hashed
    def __hash__(self):
        return hash(frozenset(self.items()))

def fact_from_fd(literal):
    if is_literal(literal):
        atom = (literal.predicate,) + literal.args
        if literal.negated:
            return Not(atom)
        return atom
    if is_assignment(literal):
        func = (literal.fluent.symbol,) + literal.fluent.args
        return Equal(func, literal.expression.value)
    raise NotImplementedError(literal)

##################################################

def get_benchmarks(sizes=[0]):
    risk_path = get_file_path(__file__, RISK_DIR)
    problem_paths = [os.path.join(risk_path, f) for f in sorted(os.listdir(risk_path))
                     if f.startswith('prob') and f.endswith('.pddl')]
    # 100 problem instances
    # Size increases every 20 problems
    # print(len(problem_paths))
    # for problem_path in problem_paths:
    #     problem_pddl = read(problem_path)
    #     problem = parse_problem(domain, problem_pddl)
    #     print(os.path.basename(problem_path), len(problem.objects))
    size_paths = [] # TODO: set
    for size in sizes:
        size_paths.extend(problem_paths[20*size:20*(size+1)])
    return size_paths

def get_problem(problem_path, **kwargs):
    safe_rm_dir(TEMP_DIR) # TODO: fix re-running bug
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    domain = parse_sequential_domain(domain_pddl)
    constant_map = {}

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

    init = list(map(fact_from_fd, problem.init))
    goal = And(*map(fact_from_fd, get_conjunctive_parts(problem.goal)))

    # universe_test | empty_test
    stream_map = {
        #'test-connected': from_test(lambda x, y: True),
        # TODO: make probabilities depend on the alphabetical/numeric distance
        #'test-connected': from_test(lambda *args: args in atoms_from_predicate['CONNECTED']),
    }

    # TODO: visualize using networks
    def connected_bernoulli_fn(*args, **kwargs):
        #assert args
        if not args:
            return 1. # For pruning low probability = 0
        return random.choice(P_SUCCESSES)

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

def run_trial(inputs, candidate_time=10, n_simulations=10000):
    # TODO: randomize the seed
    problem, stream_info, stochastic_fns, constraints, config = inputs
    print('# Init:', len(problem.init))
    print('Goal:', problem.goal)
    print(config)
    trial_time = time.time()
    # problem = get_problem(**kwargs)
    # solution = solve_incremental(problem, unit_costs=True, debug=True)
    # TODO: return the actual success rate of the portfolio (maybe in the cost)?
    solutions = solve_focused(problem, stream_info=stream_info, constraints=constraints,
                              unit_costs=False, unit_efforts=False, effort_weight=None,
                              debug=True, clean=False,
                              initial_complexity=1, max_iterations=1, max_skeletons=None,
                              planner=config['planner'], max_planner_time=candidate_time,
                              replan_actions=['enter'], diverse=config)
    runtime = elapsed_time(trial_time)

    for solution in solutions:
        print_solution(solution)
    n_successes = simulate_successes(stochastic_fns, solutions, n_simulations)
    p_success = float(n_successes) / n_simulations
    outputs = (runtime, len(solutions), p_success)
    return inputs, outputs

def solve_pddlstream(n_trials=1, max_cost_multiplier=10,
                     diverse_time=60, **kwargs):
    total_time = time.time()
    set_cost_scale(1)
    n_problems = 10 # 1 | INF
    min_k, max_k = 2, 5 # Start with min_k >= 2
    max_k = min_k

    constraints = PlanConstraints(max_cost=max_cost_multiplier) # top_quality
    #constraints = PlanConstraints(max_cost=INF) # kstar

    problem_paths = get_benchmarks(sizes=range(0, 5)) # 0 | 1
    n_problems = min(len(problem_paths), n_problems)
    #indices = random.randint(0, 19)
    indices = range(n_problems)
    indices = [0] # 0 | -1
    #indices = None # problem.pddl

    print('Problem indices:', indices)
    if indices is None:
        problem_paths = [get_file_path(__file__, 'problem.pddl')]
    else:
        problem_paths = [problem_paths[index] for index in indices]

    # blind search is effective on these problems
    planners = ['forbid'] # dijkstra | forbid | kstar
    selectors = ['greedy'] # random | greedy | exact | first
    metrics = ['p_success'] # p_success | stability | uniqueness

    planners = ['forbid', 'kstar']
    # selectors = ['random', 'greedy'] #, 'exact']
    # metrics = ['p_success', 'stability', 'uniqueness']

    ks = list(range(min_k, 1+max_k))
    configs = [{'planner': planner, 'selector': selector, 'metric': metric, 'k': k, 'max_time': diverse_time}
               for planner, selector, metric, k in product(planners, selectors, metrics, ks)]
    #configs = [
    #    {'selector': 'greedy', 'k': 3, 'd': INF, 'max_time': INF},  # 'candidates': 10,
    #]
    # TODO: more random runs
    configs = list(map(hashabledict, configs))

    jobs = []
    for _ in range(n_trials):
        for problem_path in problem_paths:
            print(problem_path)
            problem, bernoulli_fns = get_problem(problem_path, **kwargs)
            #dump_pddlstream(problem)
            stream_info = {name: StreamInfo(p_success=cached, defer_fn=defer_unique)
                           for name, cached in bernoulli_fns.items()}
            stochastic_fns = {name: test_from_bernoulli_fn(cached)
                              for name, cached in bernoulli_fns.items()}
            # TODO: log problem_path
            # TODO: compare relative odds
            print('\n'+'-'*100+'\n')
            for config in configs:
                inputs = (problem, stream_info, stochastic_fns, constraints, config)
                jobs.append(inputs)

    serial = True # is_darwin()
    if serial:
        generator = map(run_trial, jobs)
    else:
        available_cores = cpu_count()
        #num_cores = max(1, min(1 if serial else available_cores - 4, len(jobs)))
        num_cores = 2
        pool = Pool(processes=num_cores)  # , initializer=mute)
        generator = pool.imap_unordered(run_trial, jobs, chunksize=1)

    max_solutions = defaultdict(int)
    num_solutions = defaultdict(list)
    successes = defaultdict(list)
    runtimes = defaultdict(list)
    for (problem, _, _, _, config), (runtime, num, p_success) in generator:
        runtimes[config].append(runtime)
        #max_solutions[problem_path] = max(max_solutions[problem_path], num)
        num_solutions[config].append(p_success)
        successes[config].append(p_success)

    n_iterations = len(problem_paths)*n_trials
    print('Configs: {} | Iterations: {} | Total Time: {:.3f}'.format(
        len(configs), n_iterations, elapsed_time(total_time)))
    for i, config in enumerate(configs):
        print('{}) {} | Num: {} | Mean Probability: {:.3f} | Mean Time: {:.3f}'.format(
            i, config, len(successes[config]), np.mean(successes[config]), np.mean(runtimes[config])))
        print('Probabilities:', successes[config])
        print('Runtimes:', runtimes[config])

    configs_from_name = defaultdict(list)
    for config in configs:
        name = config['planner'] + ' ' + config['selector']
        if (name not in ['random', 'first']) and ('metric' in config):
            name += ' ' + config['metric']
        configs_from_name[name].append(config)
    # TODO: store other metrics
    # TODO: pickle the results

    # TODO: combine these
    plot_successes(configs_from_name, successes)
    plot_runtimes(configs_from_name, runtimes)

def plot_successes(configs_from_name, successes, scale=1.):

    import matplotlib.pyplot as plt
    plt.figure()
    for name in configs_from_name:
        ks = [config['k'] for config in configs_from_name[name]]
        means = np.array([np.mean(successes[config]) for config in configs_from_name[name]])
        stds = np.array([np.std(successes[config]) for config in configs_from_name[name]])
        widths = scale * stds  # standard deviation
        # TODO: standard error (confidence interval)
        # from learn_tools.active_learner import tail_confidence
        # alpha = 0.95
        # scale = tail_confidence(alpha)
        # width = scale * test_scores_std / np.sqrt(train_sizes)
        plt.fill_between(ks, means - widths, means + widths, alpha=0.5)
        plt.plot(ks, means, 'o-', label=name)
    plt.title('Selector Probability of Success')
    #plt.ylim(0, 1)
    plt.xlabel('K')
    plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
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

def plot_runtimes(configs_from_name, runtimes, scale=1.):
    import matplotlib.pyplot as plt
    plt.figure()
    for name in configs_from_name:
        ks = [config['k'] for config in configs_from_name[name]]
        means = np.array([np.mean(runtimes[config]) for config in configs_from_name[name]])
        stds = np.array([np.std(runtimes[config]) for config in configs_from_name[name]])
        widths = scale * stds  # standard deviation
        plt.fill_between(ks, means - widths, means + widths, alpha=0.1)
        plt.plot(ks, means, 'o-', label=name)
    plt.title('Selector Runtime')
    plt.xlabel('K')
    plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
    plt.ylabel('Runtime (seconds)')
    plt.grid()
    plt.legend(loc='best')
    plt.tight_layout()
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
