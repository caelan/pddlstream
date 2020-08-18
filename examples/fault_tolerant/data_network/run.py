#!/usr/bin/env python2.7

from __future__ import print_function

import argparse
import os
import time
import datetime
import sys
import traceback

from pddlstream.algorithms.scheduling.diverse import p_disjunction, diverse_subset, prune_dominated_action_plans
from collections import defaultdict, Counter
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_test, universe_test, fn_from_constant
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.language.stream import StreamInfo, DEBUG
from pddlstream.utils import read, get_file_path, elapsed_time, INF, ensure_dir, safe_rm_dir, str_from_object, \
    write_pickle, read_pickle
from pddlstream.language.external import defer_unique
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream, is_plan, Fact, OBJECT, \
    get_prefix, get_function
from pddlstream.algorithms.search import solve_from_pddl, diverse_from_pddl
from examples.fault_tolerant.logistics.run import test_from_bernoulli_fn, CachedFn
from examples.fault_tolerant.risk_management.run import EXPERIMENTS_DIR, PARALLEL_DIR, SERIAL, create_generator, \
    fact_from_fd, simulate_successes
from examples.pybullet.utils.pybullet_tools.utils import SEPARATOR, is_darwin, clip, DATE_FORMAT, \
    read_json, write_json
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, \
    task_from_domain_problem, get_conjunctive_parts, TEMP_DIR, set_cost_scale, make_predicate
#from pddlstream.language.write_pddl import get_problem_pddl

P_SUCCESS = 0.75 # 0.9 | 0.75

# TODO: handle more generically
if is_darwin():
    CLASSICAL_PATH = '/Users/caelan/Programs/domains/classical-domains/classical'
else:
    CLASSICAL_PATH = '/home/caelan/Programs/domains/classical-domains/classical'
# ls /Users/caelan/Programs/domains/classical-domains/classical/*-opt18

#TERMES_PATH = '/Users/caelan/Documents/IBM/termes-opt18-strips-untyped'

##################################################

def list_paths(directory):
    if not os.path.exists(directory):
        return []
    return [os.path.abspath(os.path.join(directory, f)) for f in sorted(os.listdir(directory))]

def list_pddl_problems(directory): # TODO: outlaw domain.pddl
    return [f for f in list_paths(directory) if f.endswith('.pddl')]

def get_optimal_benchmarks():
    directory = CLASSICAL_PATH
    return sorted({p for p in list_paths(directory) if os.path.isdir(p)}) #if f.endswith('-opt18')}

def get_benchmarks(directory):
    pddl_files = {f for f in list_paths(directory) if f.endswith('.pddl')}
    domain_files = {f for f in pddl_files if os.path.basename(f) == 'domain.pddl'}
    if len(domain_files) != 1:
        raise RuntimeError(directory, domain_files)
    [domain_file] = list(domain_files)
    problem_files = sorted(pddl_files - domain_files)
    return domain_file, problem_files

##################################################

def object_facts_from_str(s):
    objs, ty = s.strip().rsplit(' - ', 1)
    return [(ty, obj) for obj in objs.split(' ')]

def fact_from_str(s):
    return tuple(s.strip('( )').split(' '))

def int_from_str(s):
    return int(s.replace('number', ''))

##################################################

def get_problem(domain_name, index):
    # TODO: filter out problems that are reorderings
    import pddl
    #domain_path, problem_paths = get_benchmarks(DATA_NETWORK_PATH)
    #problem_path = problem_paths[index]

    #domain_dir = os.path.dirname(__file__ )
    #problem_path = os.path.join(domain_dir, 'problems/p{:02d}.pddl'.format(index+1))

    domain_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, domain_name))
    print(domain_dir)
    problem_paths = list_pddl_problems(os.path.join(domain_dir, 'problems'))
    print('# problems: {}'.format(len(problem_paths)))
    for idx, problem_path in enumerate(problem_paths):
        print(idx, problem_path)
    problem_path = problem_paths[index]

    #safe_rm_dir(TEMP_DIR) # TODO: fix re-running bug
    domain_path = os.path.join(domain_dir, 'domain.pddl')
    domain_pddl = read(domain_path)
    domain = parse_sequential_domain(domain_pddl)
    print(os.path.abspath(domain_path))
    print(os.path.abspath(problem_path))

    for action in domain.actions:
        new_parameters = []
        new_preconditions = []
        for parameter in action.parameters:
            new_parameters.append(pddl.TypedObject(parameter.name, OBJECT))
            new_preconditions.append(parameter.get_atom())
        action.parameters = new_parameters # Not necessary
        action.precondition = pddl.Conjunction([action.precondition] + new_preconditions).simplified()

    # from pddl.pddl_types import _get_type_predicate_name
    # for ty in domain.types:
    #     #pddl._get_type_predicate_name
    #     name = _get_type_predicate_name(ty.name)
    #     predicate = make_predicate(name, '?o')
    #     domain.predicates.append(predicate)
    #     domain.predicate_dict[name] = predicate

    domain.types.clear()
    domain.type_dict.clear()
    object_type = pddl.Type(OBJECT, basetype_name=None)
    object_type.supertype_names = []
    domain.types.append(object_type)
    domain.type_dict[object_type.name] = object_type

    assert not domain.axioms
    domain_pddl = domain

    assert not domain.constants
    constant_map = {}

    problem_pddl = read(problem_path)
    problem = parse_problem(domain, problem_pddl)
    #task = task_from_domain_problem(domain, problem) # Uses Object

    stream_pddl = read(os.path.join(domain_dir, 'stream.pddl'))
    #stream_pddl = None

    # TODO: compare statistical success and the actual success
    bernoulli_fns = {
        'test-online': fn_from_constant(P_SUCCESS),
        'test-open': fn_from_constant(P_SUCCESS),
    }

    # universe_test | empty_test
    stream_map = {
        'test-less_equal': from_test(lambda x, y: int_from_str(x) <= int_from_str(y)),
        'test-sum': from_test(lambda x, y, z: int_from_str(x) + int_from_str(y) == int_from_str(z)),
        #'test-online': from_test(universe_test),
        #'test-open': from_test(universe_test),
    }
    stream_map.update({name: from_test(CachedFn(test_from_bernoulli_fn(fn)))
                       for name, fn in bernoulli_fns.items()})
    #stream_map = DEBUG

    initial = problem.init + [obj.get_atom() for obj in problem.objects]
    init = list(map(fact_from_fd, initial))
    goal = And(*map(fact_from_fd, get_conjunctive_parts(problem.goal)))
    # TODO: throw error is not a conjunction
    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return problem, bernoulli_fns

def for_optimization(problem):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = problem
    # TODO: compile functions into a lookup table
    stream_predicates = ['less-equal', 'sum', ] # send-cost, io-cost
    new_init = [init for fact in init if get_function(fact) not in stream_predicates]
    raise NotImplementedError()

##################################################

def solve_pddlstream(n_trials=1, max_time=1*30, verbose=True):
    # TODO: make a simulator that randomizes these probabilities
    # TODO: include local correlation
    # TODO: combine with risk_management
    set_cost_scale(1)
    #constraints = PlanConstraints(max_cost=100) # kstar
    constraints = PlanConstraints(max_cost=INF)

    domain_name = 'rovers_02' # data_network | visit_all | rovers_02
    index = 1 # 0 | 10 | -1
    problem, bernoulli_fns = get_problem(domain_name, index)
    #for_optimization(problem)
    if verbose:
        dump_pddlstream(problem)
    stochastic_fns = {name: test_from_bernoulli_fn(cached)
                      for name, cached in bernoulli_fns.items()}

    # TODO: combine with the number of candidates
    planner = 'ff-wastar3' # forbid | kstar | symk | ff-astar | ff-wastar1 | ff-wastar3
    diverse = {'selector': 'greedy', 'metric': 'p_success', 'k': 10} # 5 | 10 | INF

    # TODO: sum sampling function
    stream_info = {
        # Also used for subtraction TODO: make two of these
        # Make a subtraction and addition one
        # Make a stream that makes ?size samplers bounding by the capacity of the problem
        'test-sum': StreamInfo(opt_gen_fn=None, eager=True, p_success=0),  # TODO: p_success=lambda x: 0.5
        # Better to use little space
        'test-less_equal': StreamInfo(opt_gen_fn=None, eager=True, p_success=0),
        'test-online': StreamInfo(p_success=P_SUCCESS, defer_fn=defer_unique),
        #'test-open': StreamInfo(p_success=P_SUCCESS, defer_fn=defer_unique),
    }
    stream_info.update({name: StreamInfo(p_success=cached, defer_fn=defer_unique)
                        for name, cached in bernoulli_fns.items()})

    #prohibit_actions = True
    prohibit_actions = []
    #prohibit_actions = {'send': P_SUCCESS}
    prohibit_predicates = {
        'ONLINE': P_SUCCESS,
        'open': P_SUCCESS, # TODO: make a function instead
    }
    costs = False

    successes = 0.
    for _ in range(n_trials):
        print('\n'+'-'*5+'\n')
        start_time = time.time()
        #problem = get_problem(**kwargs)
        #solution = solve_incremental(problem, unit_costs=True, debug=True)
        solutions = solve_focused(problem, constraints=constraints, stream_info=stream_info,
                                  unit_costs=False, unit_efforts=False, effort_weight=None,
                                  debug=verbose, clean=True,
                                  costs=costs, prohibit_actions=prohibit_actions, prohibit_predicates=prohibit_predicates,
                                  planner=planner, max_planner_time=max_time, diverse=diverse,
                                  initial_complexity=1, max_iterations=1, max_skeletons=None,
                                  replan_actions=True,
                                  )
        for solution in solutions:
            print_solution(solution)
            #plan, cost, certificate = solution
            #successes += is_plan(plan)

        n_simulations = 10000
        n_successes = simulate_successes(stochastic_fns, solutions, n_simulations)
        p_success = float(n_successes) / n_simulations
        print('Empirical success: {:.3f} | Runtime: {:.3f}'.format(p_success, elapsed_time(start_time)))
        successes += bool(solutions)
    print('Fraction solved: {:.3f}'.format(successes / n_trials))

##################################################

# https://bitbucket.org/ipc2018-classical/workspace/projects/GEN

def solve_pddl_trial(inputs, planner='ff-wastar3', max_time=1 * 10, max_printed=10): # TODO: too greedy causes local min
    # TODO: randomize the seed
    pid = os.getpid()
    domain_path, problem_path = inputs['domain_path'], inputs['problem_path']
    print(SEPARATOR)
    print('Process {}: {}'.format(pid, inputs))

    stdout = sys.stdout
    current_wd = os.getcwd()
    trial_wd = os.path.join(current_wd, PARALLEL_DIR, '{}/'.format(pid))
    if not SERIAL:
        sys.stdout = open(os.devnull, 'w')
        safe_rm_dir(trial_wd)
        ensure_dir(trial_wd)
        os.chdir(trial_wd)

    all_solutions = []
    outputs = dict(inputs)
    outputs.update({
        'planner': planner,
        'max_time': max_time,
        'error': True,
    })
    start_time = time.time()
    try:
        domain_pddl, problem_pddl = read(domain_path), read(problem_path)
        #all_solutions = solve_from_pddl(domain_pddl, problem_pddl, planner=planner,
        #                                max_planner_time=max_time, max_cost=INF, debug=True)
        prohibit_actions = True
        #prohibit_actions = ['send']
        all_solutions = diverse_from_pddl(domain_pddl, problem_pddl, planner=planner, prohibit_actions=prohibit_actions,
                                          max_planner_time=max_time, max_cost=INF, debug=True)
        outputs.update({
            'error': False,
        })
    except Exception:
        traceback.print_exc()
    solutions = prune_dominated_action_plans(all_solutions)
    outputs.update({
        'all_plans': len(all_solutions),
        'num_plans': len(solutions),
        'runtime': elapsed_time(start_time),
    })

    print('Overall runtime:', elapsed_time(start_time))
    evaluations = []
    for i, (plan, cost) in enumerate(solutions[:max_printed]):
        print('\nPlan {}/{}'.format(i + 1, len(solutions)), )
        solution = (plan, cost, evaluations)
        print_solution(solution)
        #break

    if not SERIAL:
        os.chdir(current_wd)
        #safe_rm_dir(trial_wd)
        sys.stdout.close()
        sys.stdout = stdout
    # git status -u --ignored

    return inputs, outputs

##################################################

def solve_pddl():
    # No restriction to be untyped here
    set_cost_scale(1)
    #constraints = PlanConstraints(max_cost=INF) # kstar

    directory_paths = get_optimal_benchmarks()
    #directory_paths = [TERMES_PATH] # create-block, destroy-block
    problems = []
    for directory_path in directory_paths:
        try:
            domain_path, problem_paths = get_benchmarks(directory_path)
        except RuntimeError:
            continue
        #problem_paths = problem_paths[:1] # 0, -1
        for problem_path in problem_paths:
            problems.append({'domain_path': domain_path, 'problem_path': problem_path})

    problems = [{
        'domain_path': 'data-network-opt18/domain.pddl',
        'problem_path': 'data-network-opt18/p11.pddl',
        #'domain_path': 'caldera-opt18/domain.pddl',
        #'problem_path': 'caldera-opt18/p01.pddl',
        # 'domain_path': 'blocks/domain.pddl',
        # 'problem_path': 'blocks/probBLOCKS-5-0.pddl',
    }]
    problems = [{key: os.path.join(CLASSICAL_PATH, path) for key, path in problem.items()} for problem in problems]

    for i, problem in enumerate(problems):
        print(i, problem)
    generator = create_generator(solve_pddl_trial, problems)

    ensure_dir(EXPERIMENTS_DIR)
    date_name = datetime.datetime.now().strftime(DATE_FORMAT)
    #file_name = os.path.join(EXPERIMENTS_DIR, '{}.pk3'.format(date_name))
    file_name = os.path.join(EXPERIMENTS_DIR, '{}.json'.format(date_name))
    results = []
    for inputs, outputs in generator:
        # TODO: pickle the solutions to reuse later
        results.append(outputs)
        if not SERIAL: # TODO: only write if is above a threshold
            #write_pickle(file_name, results)
            write_json(file_name, results)
            print('Wrote {}'.format(file_name))

##################################################

def extract_domain(path):
    #if path is None:
    #    return path
    #assert os.path.isfile(path)
    return os.path.basename(os.path.dirname(path)) # .decode(encoding='UTF-8')

def compare_histograms(results, n_bins=10, min_value=10, max_value=100):
    # https://matplotlib.org/examples/statistics/multiple_histograms_side_by_side.html
    # https://matplotlib.org/examples/statistics/histogram_demo_multihist.html
    import matplotlib.pyplot as plt

    results_from_planner = {}
    for result in results:
        planner = result.get('planner', None)
        results_from_planner.setdefault(planner, []).append(result)

    attribute = 'num_plans' # all_plans | num_plans
    planners = sorted(results_from_planner)
    print(planners)
    x = [[0 if result['error'] else clip(result[attribute], max_value=max_value)
          for result in results_from_planner[planner] if min_value <= result[attribute]] for planner in planners]
    print(zip(planners, map(len, x))) # Uneven sizes

    fig, axes = plt.subplots(nrows=1, ncols=2)
    ax0, ax1 = axes.flatten()
    plt.title('Min Plans={}'.format(min_value))

    # Make a multiple-histogram of data-sets with different length.
    ax0.hist(x, n_bins, histtype='bar', label=planners)
    ax0.legend(prop={'size': 10})
    ax0.set_title('Standard Histogram')

    ax1.hist(x, n_bins, normed=False, cumulative=-1, histtype='bar', stacked=False, label=planners) # range=(min_value, max_value),
    ax1.legend(prop={'size': 10})
    ax1.set_title('Reverse Cumulative Histogram')
    #ax0.ylim([0, 1000])

    # ax2.hist(x, n_bins, histtype='step', stacked=False, fill=False, label=planners) #, alpha=0.25)
    # ax2.legend(prop={'size': 10})
    # ax2.hist(x, n_bins, histtype='step', stacked=False, fill=True, label=planners, alpha=0.25)
    # ax2.set_title('stack step (unfilled)')

    fig.tight_layout()
    plt.show()

def analyze_experiment(results, min_plans=5, verbose=False): # 10 | 25
    problems = Counter(extract_domain(result['domain_path']) for result in results)
    print('Problems:', problems)
    planners = Counter(result.get('planner', None) for result in results)
    print('Planners:', planners)

    total_counter = defaultdict(int)
    success_counter = defaultdict(int)
    for result in sorted(results, key=lambda r: r['problem_path']):
        domain = extract_domain(result['domain_path'])
        planner = result.get('planner', None)
        total_counter[domain, planner] += 1
        if result.get('error', False):
            continue
        #if (planner != 'symk') and (result['runtime'] < result['max_time']):
        #    print('{}: {:.0f}/{:.0f}'.format(planner, result['runtime'], result['max_time']))
        if (result['num_plans'] >= min_plans) or ((planner != 'symk') and (result['runtime'] < 0.9*result['max_time'])):
            success_counter[domain, planner] += 1
            if verbose:
                print('c={:.3f}, n={}, t={:.0f}, {}'.format(
                    result.get('all_plans', None), result['num_plans'],
                    result['runtime'], result['problem_path']))  # result['domain_path']

    print(SEPARATOR)
    print('{} domains, {} problems'.format(len(success_counter), sum(success_counter.values())))
    #print(str_from_object(counter))

    problem_counter = Counter()
    for (problem, _), num in success_counter.items():
        problem_counter[problem] += num

    planner_counter = Counter()
    for (_, planner), num in success_counter.items():
        #planner_counter[planner] += 1 # Domains
        planner_counter[planner] += num # Problems
    print('Planners: ', '  '.join('{} ({})'.format(
        planner, planner_counter[planner]) for planner in sorted(planners)))

    #for domain in sorted(counter, key=counter.get):
    #for domain in sorted(problems, key=problems.get): # Num problems
    for domain in sorted(problem_counter, key=problem_counter.get): # Most successes
        print('{:<25} {:3}'.format(domain, problem_counter[domain]), end='')
        for planner in sorted(planners):
            fraction = float(success_counter[domain, planner]) / total_counter[domain, planner]
            print('  {:3}/{:3} ({:.3f})'.format(
                success_counter[domain, planner], total_counter[domain, planner], fraction), end='')
        print()

##################################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--experiments', nargs='*')
    args = parser.parse_args()

    if args.experiments:
        results = []
        for experiment_path in args.experiments:
            results.extend(read_json(experiment_path))
        #compare_histograms(results)
        analyze_experiment(results)
    else:
        solve_pddlstream()
        #solve_pddl()

if __name__ == '__main__':
    main()

# TODO: extend PDDLStream to support typing directly

# https://github.com/AI-Planning/classical-domains/tree/master/classical/data-network-opt18
# Packet sizes
# https://github.com/tomsilver/pddlgym/blob/master/rendering/tsp.py
# https://networkx.github.io/
# https://pypi.org/project/graphviz/

# ./FastDownward/fast-downward.py --show-aliases
# ./FastDownward/fast-downward.py --build release64 --alias lama examples/fault_tolerant/data_network/domain.pddl examples/fault_tolerant/data_network/problem.pddl
