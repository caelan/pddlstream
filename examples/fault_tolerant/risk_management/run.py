#!/usr/bin/env python2.7

from __future__ import print_function

import argparse
import os
import time
import random
import numpy as np
import sys
import datetime

from collections import defaultdict
from itertools import product
from multiprocessing import cpu_count, Pool

from pddlstream.algorithms.scheduling.diverse import p_disjunction
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.scheduling.plan_streams import *

from pddlstream.language.generator import from_test, fn_from_constant
from pddlstream.language.stream import StreamInfo, DEBUG
from pddlstream.language.external import defer_unique
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream, \
    is_plan, get_prefix, get_args, Not, StreamAction, Equal, Fact
from pddlstream.utils import read, get_file_path, safe_rm_dir, INF, Profiler, elapsed_time, \
    read_pickle, write_pickle, ensure_dir, get_python_version
from examples.fault_tolerant.logistics.run import test_from_bernoulli_fn, CachedFn
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, \
    task_from_domain_problem, is_literal, is_assignment, get_conjunctive_parts, TEMP_DIR, set_cost_scale #, evaluation_from_fd, fact_from_fd
from examples.pybullet.utils.pybullet_tools.utils import SEPARATOR, is_darwin, clip, DATE_FORMAT, \
    read_json, write_json #, write_json, read_json

SERIAL = is_darwin()
#SERIAL = False
P_SUCCESSES = [0.9]
#P_SUCCESSES = np.linspace(n=4, endpoint=False)

SMALL_RISK_DIR = 'smallprobs/'
LARGE_RISK_DIR = 'risk-pddl/risk/'
PARALLEL_DIR = 'temp_parallel/'
EXPERIMENTS_DIR = 'experiments/'

class hashabledict(dict):
    def __setitem__(self, key, value):
        raise RuntimeError()
    # assumes immutable once hashed
    def __hash__(self):
        return hash(frozenset(self.items()))

def fact_from_fd(literal):
    if is_literal(literal):
        atom = Fact(literal.predicate, literal.args)
        if literal.negated:
            return Not(atom)
        return atom
    if is_assignment(literal):
        func = (literal.fluent.symbol,) + literal.fluent.args
        return Equal(func, literal.expression.value)
    raise NotImplementedError(literal)

##################################################

def get_small_benchmarks():
    small_risk_path = get_file_path(__file__, SMALL_RISK_DIR)
    problem_paths = [os.path.join(small_risk_path, f) for f in sorted(os.listdir(small_risk_path))
                     if f.endswith('.pddl')]
    # 101 (20), 201 (20), 301 (20), 401 (20), 501 (20)
    return problem_paths

def get_large_benchmarks():
    large_risk_path = get_file_path(__file__, LARGE_RISK_DIR)
    problem_paths = [os.path.join(large_risk_path, f) for f in sorted(os.listdir(large_risk_path))
                     if f.startswith('prob') and f.endswith('.pddl')]
    # 1001 (20), 2501 (20), 5001 (20), 10001 (20), 20001 (20)
    # TODO: cannot solve the first 2501 instance and even some of the later 1001 instances
    return problem_paths

def extract_benchmarks(problem_paths, sizes=[0]):
    # print(len(problem_paths))
    # domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    # domain = parse_sequential_domain(domain_pddl)
    # for problem_path in problem_paths:
    #     problem_pddl = read(problem_path)
    #     problem = parse_problem(domain, problem_pddl)
    #     print(os.path.basename(problem_path), len(problem.objects))

    size_paths = [] # TODO: set
    for size in sizes:
        size_paths.extend(problem_paths[20*size:20*(size+1)])
    return size_paths

# def parse_strings():
#     # TODO: introduce object more generally
#     from examples.fault_tolerant.data_network.run import fact_from_str
#     init = [fact_from_str(s) for s in INIT.split('\n') if s]
#     objects = {n for f in init for n in get_args(f)}
#     atoms_from_predicate = defaultdict(set)
#     for fact in init:
#         atoms_from_predicate[get_prefix(fact)].add(get_args(fact))
#
#     init = [f for f in init if get_prefix(f) not in ['CONNECTED']]
#     init.extend(('OBJECT', n) for n in objects)
#
#     goal_literals = [fact_from_str(s) for s in GOAL.split('\n') if s]
#     goal = And(*goal_literals)
#     return init, goal

def get_problem(problem_path):
    #safe_rm_dir(TEMP_DIR) # TODO: fix re-running bug
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    domain = parse_sequential_domain(domain_pddl)
    constant_map = {}

    problem_pddl = read(problem_path)
    problem = parse_problem(domain, problem_pddl)
    #task = task_from_domain_problem(domain, problem) # Uses Object
    #print(problem.objects)

    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    #stream_pddl = None

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

def run_trial(inputs, candidate_time=10*60, n_simulations=10000):
    # TODO: randomize the seed
    pid = os.getpid()
    problem_path, constraints, config = inputs
    print(SEPARATOR)
    print('Process {}: {}, {}'.format(pid, problem_path, config))

    stdout = sys.stdout
    current_wd = os.getcwd()
    trial_wd = os.path.join(current_wd, PARALLEL_DIR, '{}/'.format(pid))
    if not SERIAL:
        sys.stdout = open(os.devnull, 'w')
        safe_rm_dir(trial_wd)
        ensure_dir(trial_wd)
        os.chdir(trial_wd)

    problem, bernoulli_fns = get_problem(problem_path)
    # dump_pddlstream(problem)
    stream_info = {name: StreamInfo(p_success=cached, defer_fn=defer_unique)
                   for name, cached in bernoulli_fns.items()}
    stochastic_fns = {name: test_from_bernoulli_fn(cached)
                      for name, cached in bernoulli_fns.items()}

    print('# Init:', len(problem.init))
    print('Goal:', problem.goal)
    print(config)
    trial_time = time.time()
    # problem = get_problem(**kwargs)
    # solution = solve_incremental(problem, unit_costs=True, debug=True)
    # TODO: return the actual success rate of the portfolio (maybe in the cost)?
    solutions = solve_focused(problem, stream_info=stream_info, constraints=constraints,
                              unit_costs=False, unit_efforts=False, effort_weight=None,
                              debug=True, clean=False, temp_dir=TEMP_DIR,
                              initial_complexity=1, max_iterations=1, max_skeletons=None,
                              planner=config['planner'], max_planner_time=candidate_time,
                              replan_actions=['enter'], diverse=config)

    runtime = elapsed_time(trial_time)
    for solution in solutions:
        print_solution(solution)
    n_successes = simulate_successes(stochastic_fns, solutions, n_simulations)
    p_success = float(n_successes) / n_simulations
    outputs = (runtime, len(solutions), p_success)

    if not SERIAL:
        os.chdir(current_wd)
        #safe_rm_dir(trial_wd)
        sys.stdout.close()
        sys.stdout = stdout
    # git status -u --ignored

    return inputs, outputs

def solve_pddlstream(n_trials=1, cost_multiplier=10, diverse_time=5*60, **kwargs):
    total_time = time.time()
    set_cost_scale(1)
    n_problems = INF # 1 | INF
    min_k, max_k = INF, INF # Start with min_k >= 2
    max_k = min_k # INF

    #constraints = PlanConstraints(max_cost=cost_multiplier) # top_quality
    constraints = PlanConstraints(max_cost=INF) # kstar

    problem_paths = get_small_benchmarks() + get_large_benchmarks()
    #problem_paths = extract_benchmarks(sizes=range(0, 5)) # 0 | 1
    n_problems = min(len(problem_paths), n_problems)
    #indices = random.randint(0, 19)
    indices = range(n_problems)
    #indices = [0] # 0 | -1
    #indices = None # problem.pddl

    print('Problem indices:', indices)
    if indices is None:
        problem_paths = [get_file_path(__file__, 'problem.pddl')]
    else:
        problem_paths = [problem_paths[index] for index in indices]

    # blind search is effective on these problems
    planners = ['forbid'] # dijkstra | forbid | kstar
    selectors = ['first'] # random | greedy | exact | first
    metrics = ['p_success'] # p_success | stability | uniqueness

    # planners = ['forbid', 'kstar']
    # selectors = ['random', 'greedy'] #, 'exact']
    # metrics = ['p_success', 'stability', 'uniqueness']

    ks = [min_k] if min_k == max_k else list(range(min_k, 1+max_k))
    configs = [{'planner': planner, 'selector': selector, 'metric': metric, 'k': k, 'max_time': diverse_time}
               for planner, selector, metric, k in product(planners, selectors, metrics, ks)]
    #configs = [
    #    {'selector': 'greedy', 'k': 3, 'd': INF, 'max_time': INF},  # 'candidates': 10,
    #]
    # TODO: more random runs

    jobs = [(problem_path, constraints, config) for _ in range(n_trials)
            for problem_path in problem_paths for config in configs]

    if SERIAL:
        generator = map(run_trial, jobs)
    else:
        num_cores = clip(int(cpu_count()-4), min_value=1, max_value=len(jobs))
        print('Using {}/{} cores'.format(num_cores, cpu_count()))
        pool = Pool(processes=num_cores)  # , initializer=mute)
        generator = pool.imap_unordered(run_trial, jobs, chunksize=1)

    ensure_dir(EXPERIMENTS_DIR)
    date_name = datetime.datetime.now().strftime(DATE_FORMAT)
    #file_name = os.path.join(EXPERIMENTS_DIR, '{}.pk3'.format(date_name))
    file_name = os.path.join(EXPERIMENTS_DIR, '{}.json'.format(date_name))

    # TODO: log problem_path
    # TODO: compare relative odds
    results = []
    for (problem_path, _, config), (runtime, num, p_success) in generator:
        result = {'problem': problem_path, 'config': config,
                  'runtime': runtime, 'num_plans': num, 'p_success': p_success}
        results.append(result)
        print('{}/{} | Plans: {} | Probability: {:.3f} | Runtime: {:.3f} | Total time: {:.3f}'.format(
            len(results), len(jobs), num, p_success, runtime, elapsed_time(total_time)))
        if not is_darwin():
            #write_pickle(file_name, results)
            write_json(file_name, results)
        print('Wrote {}'.format(file_name))
    print(SEPARATOR)
    analyze_results(results)

def analyze_results(results):
    problems = set()
    configs = set()
    max_solutions = defaultdict(int)
    runtimes = defaultdict(list)
    num_solutions = defaultdict(list)
    successes = defaultdict(list)
    for result in results:
        problems.add(result['problem'])
        config = hashabledict(result['config'])
        configs.add(config)
        runtimes[config].append(result['runtime'])
        num_solutions[config].append(result['num_plans'])
        max_solutions[result['problem']] = max(max_solutions[result['problem']], result['num_plans'])
        successes[config].append(result['p_success'])
    #safe_rm_dir(PARALLEL_DIR)

    print('Problems:', max_solutions)
    print('Configs:', configs)
    for i, config in enumerate(configs):
        print('{}) {} | Num: {} | Mean Probability: {:.3f} | Mean Time: {:.3f}'.format(
            i, config, len(successes[config]), np.mean(successes[config]), np.mean(runtimes[config])))
        print('Num solutions:', num_solutions[config])
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
    #plot_successes(configs_from_name, successes)
    #plot_runtimes(configs_from_name, runtimes)

def plot_successes(configs_from_name, successes, scale=1.):
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
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
    from matplotlib.ticker import MaxNLocator
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
    assert get_python_version() == 3
    parser = argparse.ArgumentParser()
    #parser.add_argument('-v', '--visualize', action='store_true')
    args = parser.parse_args()
    with Profiler():
        solve_pddlstream()

if __name__ == '__main__':
    main()

# https://github.com/IBM/risk-pddl
