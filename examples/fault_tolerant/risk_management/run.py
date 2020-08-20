#!/usr/bin/env python2.7

from __future__ import print_function

import argparse
import os
import time
import random
import numpy as np
import sys
import math
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
    read_pickle, write_pickle, ensure_dir, get_python_version, MockSet
from examples.fault_tolerant.logistics.run import test_from_bernoulli_fn, CachedFn
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, \
    task_from_domain_problem, is_literal, is_assignment, get_conjunctive_parts, TEMP_DIR, set_cost_scale #, evaluation_from_fd, fact_from_fd
from examples.pybullet.utils.pybullet_tools.utils import SEPARATOR, is_darwin, clip, DATE_FORMAT, \
    read_json, write_json, is_remote

SERIAL = is_darwin() # is_remote
#SERIAL = True
#SERIAL = False

P_SUCCESSES = [0.75] # 0.9 | 0.75
#P_SUCCESSES = np.linspace(n=4, endpoint=False)
CANDIDATE_TIME = 10*60

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

def get_benchmark_candidates():
    file_name = '20-07-08_22-44-40.json'
    results = read_json(os.path.join(EXPERIMENTS_DIR, file_name))
    #analyze_results(results)
    max_solutions = defaultdict(int)
    for result in results:
        max_solutions[result['problem']] = max(max_solutions[result['problem']], result['num_plans'])
    #for problem, num in sorted(max_solutions.items(), key=lambda pair: pair[1]):
    #    print(problem, num)
    return dict(max_solutions)

def get_good_benchmarks(min_solutions=15):
    max_solutions = get_benchmark_candidates()
    good_problems = sorted(os.path.basename(problem) for problem, num in max_solutions.items() if num >= min_solutions)
    #for problem in sorted(good_problems):
    #    print(problem, max_solutions[problem])
    small_risk_path = get_file_path(__file__, SMALL_RISK_DIR)
    large_risk_path = get_file_path(__file__, LARGE_RISK_DIR)
    good_paths = [os.path.join(small_risk_path if 'smallprob' in problem else large_risk_path, problem)
                  for problem in good_problems]
    return good_paths

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

##################################################

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

def run_trial(inputs, candidate_time=CANDIDATE_TIME, max_plans=INF, n_simulations=10000):
    # TODO: randomize the seed
    pid = os.getpid()
    problem_path, constraints, config = inputs
    planner = config['planner']
    # config = {}
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
    # from examples.fault_tolerant.data_network.run import visualize_graph
    # visualize_graph(problem.init, edge_predicates=[('CONNECTED', [0, 1])])
    # return inputs, (0., 0, 0.)

    # dump_pddlstream(problem)
    stream_info = {name: StreamInfo(p_success=cached, defer_fn=defer_unique)
                   for name, cached in bernoulli_fns.items()}
    stochastic_fns = {name: test_from_bernoulli_fn(cached)
                      for name, cached in bernoulli_fns.items()}

    prohibit_predicates = {'LINKED': P_SUCCESSES[0]}
    use_probabilities = config.get('probabilities', False)
    unit_costs = False

    print('# Init:', len(problem.init))
    print('Goal:', problem.goal)
    print(config)
    trial_time = time.time()
    # problem = get_problem(**kwargs)
    # solution = solve_incremental(problem, unit_costs=True, debug=True)
    # TODO: return the actual success rate of the portfolio (maybe in the cost)?
    solutions = solve_focused(problem, stream_info=stream_info, constraints=constraints,
                              unit_costs=unit_costs, unit_efforts=False, effort_weight=None,
                              debug=True, clean=False, temp_dir=TEMP_DIR,
                              initial_complexity=1, max_iterations=1, max_skeletons=None,
                              planner=planner, max_planner_time=candidate_time, max_plans=max_plans,
                              use_probabilities=use_probabilities, prohibit_predicates=prohibit_predicates,
                              replan_actions=True, diverse=config)

    runtime = elapsed_time(trial_time)
    for solution in solutions:
        print_solution(solution)
    n_successes = simulate_successes(stochastic_fns, solutions, n_simulations)
    p_success = float(n_successes) / n_simulations
    total_cost = sum(cost for _, cost, _  in solutions)
    outputs = (runtime, len(solutions), p_success, total_cost)
    # TODO: store other metrics (such as candidate time and probability of success

    if not SERIAL:
        os.chdir(current_wd)
        #safe_rm_dir(trial_wd)
        sys.stdout.close()
        sys.stdout = stdout
    # git status -u --ignored

    return inputs, outputs

def create_generator(fn, jobs):
    if SERIAL:
        generator = map(fn, jobs)
    else:
        num_cores = clip(int(cpu_count()/2), min_value=1, max_value=len(jobs))
        print('Using {}/{} cores for {} jobs'.format(num_cores, cpu_count(), len(jobs)))
        pool = Pool(processes=num_cores)  # , initializer=mute)
        generator = pool.imap_unordered(fn, jobs, chunksize=1)
    return generator

def solve_pddlstream(n_trials=1, cost_multiplier=10, diverse_time=10*60, **kwargs):
    total_time = time.time()
    set_cost_scale(1)
    n_problems = INF # 1 | INF
    min_k, max_k = 2, 10 # Start with min_k >= 2
    #max_k = min_k # INF

    #constraints = PlanConstraints(max_cost=cost_multiplier) # top_quality
    constraints = PlanConstraints(max_cost=INF) # kstar

    #problem_paths = get_small_benchmarks() + get_large_benchmarks()
    problem_paths = get_good_benchmarks()
    #problem_paths = extract_benchmarks(problem_paths, sizes=range(0, 5)) # 0 | 1

    n_problems = min(len(problem_paths), n_problems)
    #indices = random.randint(0, 19)
    indices = range(n_problems)
    #indices = [-1] # 0 | -1
    #indices = None # problem.pddl

    print('Problem indices:', indices)
    if indices is None:
        problem_paths = [get_file_path(__file__, 'problem.pddl')]
    else:
        problem_paths = [problem_paths[index] for index in indices]

    # blind search is effective on these problems
    #planners = ['forbid', 'symk', 'ff-wastar1'] # dijkstra | forbid | kstar | symk | ff-wastar1
    selectors = ['greedy', 'random', 'first'] # random | greedy | exact | first
    metrics = ['p_success'] # p_success | stability | uniqueness

    planners = ['ff-wastar1'] #, 'kstar'] # ff-wastar1 | max-astar | dijkstra | symk
    #selectors = ['random', 'greedy', 'exact'] #,'first']
    #metrics = ['p_success', 'stability', 'uniqueness']
    probs = [False, True]

    # TODO: prune repeated using hashabledict
    ks = [min_k] if min_k == max_k else list(range(min_k, 1+max_k))
    configs = [{'planner': planner, 'probabilities': prob,
                'selector': selector, 'metric': metric, 'k': k, 'max_time': diverse_time}
               for planner, prob, selector, metric, k in product(planners, probs, selectors, metrics, ks)]
    #configs = [
    #    {'selector': 'greedy', 'k': 3, 'd': INF, 'max_time': INF},  # 'candidates': 10,
    #]
    # TODO: more random runs

    jobs = [(problem_path, constraints, config) for _ in range(n_trials)
            for problem_path in problem_paths for config in configs]
    #jobs = jobs[:1]
    generator = create_generator(run_trial, jobs)

    ensure_dir(EXPERIMENTS_DIR)
    date_name = datetime.datetime.now().strftime(DATE_FORMAT)
    file_name = os.path.join(EXPERIMENTS_DIR, '{}.json'.format(date_name)) # pk3

    # TODO: log problem_path
    # TODO: compare relative odds
    results = []
    for (problem_path, _, config), (runtime, num, p_success, cost) in generator:
        result = {'problem': problem_path, 'config': config,
                  'runtime': runtime, 'num_plans': num, 'p_success': p_success, 'cost': cost}
        # TODO: record that risk management
        results.append(result)
        print('{}/{} | Plans: {} | Probability: {:.3f} | Cost: {:.3f} | Runtime: {:.3f} | Total time: {:.3f}'.format(
            len(results), len(jobs), num, p_success, cost, runtime, elapsed_time(total_time)))
        if not is_darwin():
            #write_pickle(file_name, results)
            write_json(file_name, results)
            print('Wrote {}'.format(file_name))
    print(SEPARATOR)
    analyze_results(results, visualize=not SERIAL)

##################################################

def get_named_configs(configs):
    configs_from_name = defaultdict(list)
    for config in configs:
        if (config['planner'] in DIVERSE_PLANNERS) and (config['selector'] in ['first']):
            continue
        name = config['planner'] + ' ' + config['selector']
        #name = config['selector']
        if (config['selector'] not in ['random', 'first']) and ('metric' in config):
            name += ' ' + config['metric']
        configs_from_name[name].append(config)
    return configs_from_name

def analyze_results(all_results, ratio=False, visualize=True, verbose=False):
    #planners = MockSet()
    planners = ['ff-wastar1'] #, 'forbid', 'kstar']

    #selectors = ['random', 'greedy', 'exact']
    selectors = ['first', 'greedy'] #, 'random', 'exact']

    metrics = ['p_success', 'stability', 'uniqueness']
    #metrics = ['p_success']#, 'stability', 'uniqueness']

    # TODO: compare to best before or after filtering
    best_successes = {}
    for result in all_results:
        best_successes[result['problem']] = max(result['p_success'], best_successes.get(result['problem'], 0))

    results = [result for result in all_results if
               (result['config']['planner'] in planners) and
               (result['config']['selector'] in selectors) and
               (result['config']['metric'] in metrics)]
    print('Results: {} | filtered: {}'.format(len(all_results), len(results)))

    configs = set()
    max_solutions = defaultdict(int)
    runtimes = defaultdict(list)
    num_solutions = defaultdict(list)
    successes = defaultdict(list)
    costs = defaultdict(list)
    for result in results:
        config = hashabledict(result['config'])
        configs.add(config)
        runtimes[config].append(result['runtime'])
        #runtimes[config].append(result['runtime'] - CANDIDATE_TIME)
        num_solutions[config].append(result['num_plans'])
        problem = os.path.basename(result['problem'])
        max_solutions[problem] = max(max_solutions[problem], result['num_plans'])
        p_success = result['p_success']
        #p_success = math.log(p_success)
        #p_success = 1/p_success
        if ratio:
            p_success /= best_successes[result['problem']]
        successes[config].append(p_success)
        if result['num_plans']:
            costs[config].append(result['cost'] / result['num_plans'])
    #safe_rm_dir(PARALLEL_DIR)

    print('Problems ({}):'.format(len(max_solutions)), max_solutions)
    print('Configs ({}):'.format(len(configs)), configs)
    if verbose:
        for i, config in enumerate(configs):
            print('{}) {} | Num: {} | Mean Probability: {:.3f} | Mean Time: {:.3f}'.format(
                i, config, len(successes[config]), np.mean(successes[config]), np.mean(runtimes[config])))
            print('Num solutions:', num_solutions[config])
            print('Probabilities:', successes[config])
            print('Runtimes:', runtimes[config])

    if visualize and not is_remote():
        # TODO: disable importing matplotlib when remote
        configs_from_name = get_named_configs(configs)
        plot_data(configs_from_name, successes, ratio=ratio, y_label='Probability of Success')
        plot_data(configs_from_name, costs, ratio=ratio, y_label='Average Cost')
        plot_data(configs_from_name, runtimes, y_label='Runtime (Seconds)')

def plot_data(configs_from_name, data, ratio=False, y_label=None, scale=0.5): # 0.0 | 0.25 | 1.0
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    plt.figure()
    for name in sorted(configs_from_name):
        config_from_k = defaultdict(list)
        for config in configs_from_name[name]:
            config_from_k[config['k']].append(data[config])
            #config_from_k[config['k']].append(list(map(math.log, successes[config]))
        ks = sorted(config_from_k)
        means = np.array([np.mean(config_from_k[k]) for k in ks])
        stds = np.array([np.std(config_from_k[k]) for k in ks])
        widths = scale * stds  # standard deviation
        # TODO: standard error (confidence interval)
        # from learn_tools.active_learner import tail_confidence
        # alpha = 0.95
        # scale = tail_confidence(alpha)
        # width = scale * test_scores_std / np.sqrt(train_sizes)
        plt.fill_between(ks, means - widths, means + widths, alpha=0.25)
        plt.plot(ks, means, 'o-', label=name)

    plt.title('Selector {}'
              '\nScale = {}$\sigma$'.format(y_label, scale))
    #plt.ylim(0, 1)
    plt.xlabel('K')
    plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
    if ratio:
        plt.ylabel('{} Performance Ratio'.format(y_label))
    else:
        plt.ylabel(y_label)
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

#ALPHA = None
#EDGES = ['face', 'face', 'g', 'b']
#COLORS = ['r', 'y', 'none', 'none']

MARKERS = ['x', 'o', '+']
EDGES = ['face', 'g', 'face', ] # C2
#COLORS = ['C0', 'C1', 'none']
#COLORS = ['c', 'y', 'none'] # m
COLORS = ['r', 'none', 'b']

# https://matplotlib.org/api/markers_api.html
# https://matplotlib.org/2.0.2/api/colors_api.html

def scatter_plot(results):
    # TODO: analyze for a particular problem
    import matplotlib.pyplot as plt
    min_candidates, max_candidates = 15, 100
    k = 6

    #selectors = ['random', 'greedy', 'exact']
    selectors = ['greedy'] #, 'exact']
    metrics = ['p_success', 'stability', 'uniqueness']
    #metrics = ['p_success']#, 'stability', 'uniqueness']

    problem_candidates = get_benchmark_candidates()
    print(problem_candidates)
    good_problems = {problem for problem, num in problem_candidates.items()
                     if min_candidates <= num <= max_candidates}
    all_sizes = sorted({problem_candidates[problem] for problem in good_problems}) # Overlay multiple?
    print('Sizes:', all_sizes)
    plt.scatter(all_sizes, np.zeros(len(all_sizes)), marker='|', color='k') # black

    results_from_config = defaultdict(list)
    for result in results:
        config = hashabledict(result['config'])
        if (config['selector'] not in selectors) or (config['metric'] not in metrics):
            continue
        if (result['problem'] in good_problems) and (config['k'] == k):
            results_from_config[config].append(result)

    configs_from_name = get_named_configs(results_from_config)
    for idx, name in enumerate(sorted(configs_from_name)):
        for config in configs_from_name[name]:
            sizes = []
            runtimes = []
            for result in results_from_config[config]:
                #if result['success']:
                sizes.append(problem_candidates[result['problem']])
                runtimes.append(result['p_success'])
            plt.scatter(sizes, runtimes, #marker=MARKERS[idx],
                        #color=COLORS[idx], edgecolors=EDGES[idx],
                        alpha=0.75, label=name)

    plt.title('Selector Success')
    #plt.xticks(range(1, max_size+1)) #, [get_index(problem) for problem in problems])
    #plt.xlim([1, 1000]) #max(all_sizes)])
    plt.xlabel('# candidates')
    plt.ylabel('Probability of Success')
    #plt.legend(loc='upper left')
    plt.legend(loc='upper center')
    #plt.savefig('test')
    plt.show()
    # logarithmic scale

##################################################

def main():
    assert get_python_version() == 3
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--experiment', default=None)
    args = parser.parse_args()
    if args.experiment is None:
        with Profiler():
            solve_pddlstream()
    else:
        results = read_json(args.experiment)
        #scatter_plot(results)
        analyze_results(results)

if __name__ == '__main__':
    main()

# https://github.com/IBM/risk-pddl

# python3 -m examples.fault_tolerant.risk_management.run 2>&1 | tee log.txt
