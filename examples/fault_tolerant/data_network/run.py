#!/usr/bin/env python2.7

from __future__ import print_function

import argparse
import os
import time
import datetime
import sys
import traceback

from examples.fault_tolerant.utils import list_paths, int_from_str, extract_static, get_static_predicates, fact_from_fd, \
    extract_streams, simulate_successes, test_from_bernoulli_fn, CachedFn
from pddlstream.algorithms.scheduling.diverse import p_disjunction, prune_dominated_action_plans, generic_union, select_portfolio
from collections import defaultdict, Counter
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_test, fn_from_constant
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, elapsed_time, INF, ensure_dir, safe_rm_dir, implies
from pddlstream.language.external import defer_unique
from pddlstream.language.constants import print_solution, PDDLProblem, And, dump_pddlstream, OBJECT, \
    get_prefix, get_function, get_args, Action
from pddlstream.algorithms.search import diverse_from_pddl
from examples.fault_tolerant.risk_management.run import EXPERIMENTS_DIR, PARALLEL_DIR, SERIAL, create_generator
from examples.pybullet.utils.pybullet_tools.utils import SEPARATOR, is_darwin, clip, DATE_FORMAT, \
    read_json, write_json
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, \
    get_conjunctive_parts, set_cost_scale, Domain

#from pddlstream.language.write_pddl import get_problem_pddl

# TODO: make a simulator that randomizes these probabilities
P_SUCCESS = 0.9 # 0.9 | 0.75

# TODO: handle more generically
if is_darwin():
    CLASSICAL_PATH = '/Users/caelan/Programs/domains/classical-domains/classical'
else:
    CLASSICAL_PATH = '/home/caelan/Programs/domains/classical-domains/classical'
# ls /Users/caelan/Programs/domains/classical-domains/classical/*-opt18

#TERMES_PATH = '/Users/caelan/Documents/IBM/termes-opt18-strips-untyped'

##################################################

def list_pddl_problems(directory): # TODO: outlaw domain.pddl
    return [f for f in list_paths(directory) if f.endswith('.pddl')]

def get_domains(directory=CLASSICAL_PATH, optimal=False):
    return sorted({p for p in list_paths(directory) if os.path.isdir(p)
                   and implies(optimal, '-opt' in os.path.basename(p))}) #if f.endswith('-opt18')}

def get_benchmarks(directory):
    import importlib.machinery
    #api = importlib.import_module('{}.api'.format(directory))
    api = importlib.machinery.SourceFileLoader('api', os.path.join(directory, 'api.py')).load_module()

    # spec = importlib.util.spec_from_file_location('api', os.path.join(directory, 'api.py')) # >= 3.5
    # api = importlib.util.module_from_spec(spec)
    # spec.loader.exec_module(api)

    pairs = []
    for domain in api.domains:
        for domain_file, problem_file in domain['problems']:
            pairs.append((os.path.abspath(os.path.join(directory, os.pardir, domain_file)),
                          os.path.abspath(os.path.join(directory, os.pardir, problem_file))))
    return pairs
    # pddl_files = {f for f in list_paths(directory) if f.endswith('.pddl')}
    # domain_files = {f for f in pddl_files if os.path.basename(f) == 'domain.pddl'}
    # if len(domain_files) != 1:
    #     raise RuntimeError(directory, domain_files)
    # [domain_file] = list(domain_files)
    # pairs = [(domain_file, problem_file) for problem_file in sorted(pddl_files - domain_files)]
    # return pairs

##################################################

def get_problem(domain_name, index):
    # TODO: filter out problems that are reorderings
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
    print(os.path.abspath(domain_path))
    print(os.path.abspath(problem_path))

    domain, init, goal = types_to_predicates(domain_path, problem_path)
    return get_pddlstream(domain, domain_path, init, goal)

def types_to_predicates(domain_path, problem_path):
    import pddl
    domain_pddl = read(domain_path)
    domain = parse_sequential_domain(domain_pddl)
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

    domain.types[:] = []
    domain.type_dict.clear()
    object_type = pddl.Type(OBJECT, basetype_name=None)
    object_type.supertype_names = []
    domain.types.append(object_type)
    domain.type_dict[object_type.name] = object_type

    assert not domain.axioms
    assert not domain.constants

    problem_pddl = read(problem_path)
    domain, init, goal = convert_problem(domain, problem_pddl)

    return domain, init, goal

def convert_problem(domain_pddl, problem_pddl):
    if isinstance(domain_pddl, Domain):
        domain = domain_pddl
    else:
        domain = parse_sequential_domain(domain_pddl)
    problem = parse_problem(domain, problem_pddl)
    #task = task_from_domain_problem(domain, problem) # Uses Object

    initial = problem.init + [obj.get_atom() for obj in problem.objects]
    init = list(map(fact_from_fd, initial))
    goal = And(*map(fact_from_fd, get_conjunctive_parts(problem.goal)))
    # TODO: throw error is not a conjunction
    return domain, init, goal

def get_pddlstream(domain, domain_path, init, goal):
    assert not domain.constants
    constant_map = {}

    domain_dir = os.path.dirname(domain_path)
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

    problem = PDDLProblem(domain, constant_map, stream_pddl, stream_map, init, goal)

    return problem, bernoulli_fns

def visualize_graph(init, edge_predicates, title=None):
    # TODO: visualize causal graph and domain transition graph
    # TODO: metric space embedding using costs
    # TODO: highlight some important nodes
    # https://github.com/tomsilver/pddlgym/blob/master/rendering/tsp.py
    # https://networkx.github.io/
    # https://pypi.org/project/graphviz/
    # https://scikit-learn.org/stable/modules/generated/sklearn.manifold.SpectralEmbedding.html
    edges = []
    for fact in init:
        for name, (idx1, idx2) in edge_predicates:
            if get_prefix(fact).lower() == name.lower():
                args = get_args(fact)
                edges.append((args[idx1], args[idx2]))
    #print(edges)
    print('# edges = {}'.format(len(edges)))
    if len(edges) >= 500:
        return None

    import networkx
    #import graphviz
    #import pydot
    import matplotlib.pyplot as plt

    # options = {
    #     'node_color': 'black',
    #     'node_size': 100,
    #     'width': 3,
    # }
    graph = networkx.DiGraph()
    graph.add_edges_from(edges) # add_weighted_edges_from

    #pos = None
    pos = networkx.nx_agraph.graphviz_layout(graph)

    if pos is None:
        try:
            #draw = networkx.draw_spectral
            draw = networkx.draw_planar # Fails is not planar
            draw(graph, with_labels=True,) #, font_weight='bold'), **options)
        except networkx.exception.NetworkXException:
            networkx.draw_spectral(graph, with_labels=True,)
    else:
        networkx.draw(graph, pos=pos, with_labels=True,) #, font_weight='bold'), **options)
    #if title is not None:
    plt.title(title)
    plt.show()

    return graph

def for_optimization(problem):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = problem
    # TODO: compile functions into a lookup table
    stream_predicates = ['less-equal', 'sum',] # send-cost, io-cost
    new_init = [init for fact in init if get_function(fact) not in stream_predicates]
    raise NotImplementedError()

##################################################

def solve_trial(domain_name, index, planner, diverse, max_time=1*30, n_simulations=10000, visualize=False, verbose=True):
    set_cost_scale(1)
    #constraints = PlanConstraints(max_cost=100) # kstar
    constraints = PlanConstraints(max_cost=INF)

    problem, bernoulli_fns = get_problem(domain_name, index)
    stochastic_fns = {name: test_from_bernoulli_fn(cached)
                      for name, cached in bernoulli_fns.items()}
    #for_optimization(problem)

    if visualize and not SERIAL:
        edge_predicates = [
            ('can_traverse', [1, 2]), # rovers
            # ('visible', [0, 1]),  # rovers
            # ('visible_from', [0, 1]),  # rovers
            ('CONNECTED', [0, 1]), # data_network, visit_all
        ]
        for index in range(100):
            problem, _ = get_problem(domain_name, index)
            visualize_graph(problem.init, edge_predicates, title='{}[{}]'.format(domain_name, index))
        dump_pddlstream(problem)

    # TODO: sum sampling function
    stream_info = {
        # Also used for subtraction TODO: make two of these
        # Make a subtraction and addition one
        # Make a stream that makes ?size samplers bounding by the capacity of the problem
        'test-sum': StreamInfo(opt_gen_fn=None, eager=True, p_success=0),  # TODO: p_success=lambda x: 0.5
        # Better to use little space
        'test-less_equal': StreamInfo(opt_gen_fn=None, eager=True, p_success=0),
        #'test-online': StreamInfo(p_success=P_SUCCESS, defer_fn=defer_unique),
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
        #'visible': P_SUCCESS,
        #'visible_from': P_SUCCESS,
    }
    use_probabilities = True # TODO: toggle and see difference in performance

    print('\n'+'-'*5+'\n')
    start_time = time.time()
    #problem = get_problem(**kwargs)
    #solution = solve_incremental(problem, unit_costs=True, debug=True)
    solutions = solve_focused(problem, constraints=constraints, stream_info=stream_info,
                              unit_costs=False, unit_efforts=False, effort_weight=None,
                              debug=verbose, clean=True, use_probabilities=use_probabilities,
                              prohibit_actions=prohibit_actions, prohibit_predicates=prohibit_predicates,
                              planner=planner, max_planner_time=max_time, diverse=diverse,
                              initial_complexity=1, max_iterations=1, max_skeletons=None,
                              replan_actions=True)

    for solution in solutions:
        print_solution(solution)

    stream_plans = [extract_streams(plan) for plan, _, _ in solutions]
    probabilities = {stream: bernoulli_fns[stream.name](*stream.inputs)
                     for stream in generic_union(*stream_plans)}

    exact_success = p_disjunction(stream_plans, probabilities=probabilities)
    n_successes = simulate_successes(stochastic_fns, solutions, n_simulations)
    p_success = float(n_successes) / n_simulations
    print('Exact success: {:.3f} | Empirical success: {:.3f} | Runtime: {:.3f}'.format(
        exact_success, p_success, elapsed_time(start_time)))

def solve_pddlstream():
    # TODO: combine with risk_management

    domain_name = 'rovers_02' # data_network | visit_all | rovers_02 | no_mprime
    index = 0 # 0 | 10 | -1

    # TODO: sidestep using the focused algorithm
    # prune_dominated_action_plans

    # TODO: combine with the number of candidates
    planner = 'ff-wastar3' # forbid | kstar | symk | ff-astar | ff-wastar1 | ff-wastar3
    diverse = {'selector': 'greedy', 'metric': 'p_success', 'k': 10} # 5 | 10 | INF
    solve_trial(domain_name, index, planner, diverse)

##################################################

# https://bitbucket.org/ipc2018-classical/workspace/projects/GEN

PROBABILITIES = {
    'connect': P_SUCCESS,
    'connected': P_SUCCESS,
    'road': P_SUCCESS, # transport
    'link': P_SUCCESS, # driverlog
    'can_traverse': P_SUCCESS, # rovers
    'visible': P_SUCCESS, # rovers
    'visible_from': P_SUCCESS, # rovers
    'path': P_SUCCESS,
}

def solve_pddl_trial(inputs, max_time=5 * 60, max_printed=10, max_plans=5):
    # TODO: randomize the seed
    pid = os.getpid()
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

    set_cost_scale(1)
    #constraints = PlanConstraints(max_cost=INF) # kstar

    #prohibit_actions = ['send']
    prohibit_actions = []
    #prohibit_actions = True

    prohibit_predicates = PROBABILITIES
    #prohibit_predicates = []
    use_probabilities = True

    all_solutions = []
    outputs = dict(inputs)
    outputs.update({
        'prohibit_actions': prohibit_actions,
        'prohibit_predicates': list(prohibit_predicates),
        'max_time': max_time,
        'max_plans': max_plans,
        'error': True,
    })

    domain_pddl, problem_pddl = read(inputs['domain_path']), read(inputs['problem_path'])
    start_time = time.time()
    try:
        #all_solutions = solve_from_pddl(domain_pddl, problem_pddl, planner=planner,
        #                                max_planner_time=max_time, max_cost=INF, debug=True)
        all_solutions = diverse_from_pddl(domain_pddl, problem_pddl, planner=inputs['planner'],
                                          use_probabilities=use_probabilities,
                                          prohibit_actions=prohibit_actions,
                                          prohibit_predicates=prohibit_predicates,
                                          max_planner_time=max_time, max_cost=INF, max_plans=max_plans, debug=True)
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

    plans = [plan for plan, _, in solutions]
    static_sets = extract_static(domain_pddl, plans, PROBABILITIES)

    probabilities = {fact: PROBABILITIES[fact.predicate] for fact in generic_union(*static_sets)}
    all_p_success = p_disjunction(static_sets, probabilities=probabilities)
    outputs.update({
        'probabilities': PROBABILITIES,
        'all_p_success': all_p_success,
    })

    min_k, max_k = 2, 10 # Start with min_k >= 2
    #max_k = min_k # INF
    ks = list(range(min_k, 1+max_k))

    outputs_list = []
    diverse = inputs.get('diverse', None)
    if diverse:
        for k in ks:
            diverse = dict(diverse)
            diverse['k'] = k
            portfolio = select_portfolio(static_sets, diverse, probabilities=probabilities)
            p_success = p_disjunction(portfolio, probabilities=probabilities)
            outputs = dict(outputs)
            outputs.update({
                'diverse': diverse,
                'full_runtime': elapsed_time(start_time),
                'p_success': p_success,
            })
            outputs_list.append(outputs)
            print('All: {:.3f} | Portfolio: {:.3f} | Runtime: {:.3f}'.format(
                all_p_success, p_success, elapsed_time(start_time)))
    else:
        outputs_list = [outputs]

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

    return inputs, outputs_list

##################################################

def solve_pddl(visualize=False):
    # No restriction to be untyped here

    #directory_paths = get_domains()
    #directory_paths = [TERMES_PATH] # create-block, destroy-block

    directories = {
        1: 'data-network-opt18',
        2: 'visitall-opt11-strips',
        3: 'rovers-02',
        4: 'rovers',
        #5: 'pipesworld-notankage',
        6: 'no-mprime',
        7: 'no-mystery',
        8: 'storage',
        9: 'trucks',
        10: 'transport-opt11-strips',
        11: 'driverlog',
    }
    indices = sorted(directories.keys())
    indices = [4]
    directory_paths = [os.path.join(CLASSICAL_PATH, directories[idx]) for idx in indices]

    planner = 'ff-wastar3'
    diverse = {'k': 3, 'selector': 'greedy'} # exact
    #diverse = None

    problems = []
    for directory_path in directory_paths:
        for domain_path, problem_path in get_benchmarks(directory_path):
            problems.append({'domain_path': domain_path, 'problem_path': problem_path})

    # problems = [{
    #     'domain_path': 'data-network-opt18/domain.pddl',
    #     'problem_path': 'data-network-opt18/p11.pddl',
    #     #'domain_path': 'caldera-opt18/domain.pddl',
    #     #'problem_path': 'caldera-opt18/p01.pddl',
    #     # 'domain_path': 'blocks/domain.pddl',
    #     # 'problem_path': 'blocks/probBLOCKS-5-0.pddl',
    # }]
    problems = [{key: os.path.join(CLASSICAL_PATH, path)
                 for key, path in problem.items()} for problem in problems]
    for problem in problems:
        problem.update({'planner': planner, 'diverse': diverse})

    #problems = problems[:1]
    problems = problems[5:6]
    for i, problem in enumerate(problems):
        print(i, problem)
    generator = create_generator(solve_pddl_trial, problems)

    if visualize:
        edge_predicates = [
            ('connect', [0, 1]),
            ('connected', [0, 1]),
            ('road', [0, 1]), # transport
            ('link', [0, 1]), # driverlog
            #('can_traverse', [1, 2]), # rovers
            ('visible', [0, 1]),  # rovers
            #('visible_from', [0, 1]),  # rovers
        ]
        for problem in problems:
            print()
            print(problem['domain_path'], problem['problem_path'])
            domain, init, goal = convert_problem(read(problem['domain_path']), read(problem['problem_path']))
            print('Static predicates', get_static_predicates(domain))
            visualize_graph(init, edge_predicates)

    ensure_dir(EXPERIMENTS_DIR)
    date_name = datetime.datetime.now().strftime(DATE_FORMAT)
    #file_name = os.path.join(EXPERIMENTS_DIR, '{}.pk3'.format(date_name))
    file_name = os.path.join(EXPERIMENTS_DIR, '{}.json'.format(date_name))

    results = []
    for inputs, outputs_list in generator:
        # TODO: pickle the solutions to reuse later
        for outputs in outputs_list:
            #outputs.update(inputs)
            print(len(results), outputs)
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
    ax0.xlabel('K')
    ax0.ylabel('Frequency')
    ax0.set_title('Standard Histogram')

    ax1.hist(x, n_bins, normed=False, cumulative=-1, histtype='bar', stacked=False, label=planners) # range=(min_value, max_value),
    ax1.legend(prop={'size': 10})
    ax1.xlabel('K')
    ax1.ylabel('Cumulative Frequency')
    ax1.set_title('Reverse Cumulative Histogram')
    #ax0.ylim([0, 1000])

    # ax2.hist(x, n_bins, histtype='step', stacked=False, fill=False, label=planners) #, alpha=0.25)
    # ax2.legend(prop={'size': 10})
    # ax2.hist(x, n_bins, histtype='step', stacked=False, fill=True, label=planners, alpha=0.25)
    # ax2.set_title('stack step (unfilled)')

    fig.tight_layout()
    plt.show()

def analyze_experiment(results, min_plans=10, verbose=False): # 10 | 25
    # TODO: compare on just -opt
    problem_trials = Counter(extract_domain(result['domain_path']) for result in results)
    print('Domains ({}):'.format(len(problem_trials)), sorted(problem_trials.keys()))
    print('\nProblems ({}):'.format(sum(problem_trials.values())), problem_trials)
    planner_trials = Counter(result.get('planner', None) for result in results)
    print('\nPlanners ({}):'.format(len(planner_trials)), planner_trials)

    total_counter = defaultdict(int)
    success_counter = defaultdict(int)
    for result in sorted(results, key=lambda r: r['problem_path']):
        domain = extract_domain(result['domain_path'])
        planner = result.get('planner', None)
        total_counter[domain, planner] += 1
        if result.get('error', False):
            #print('Error:', result['problem_path'])
            # TODO: save the error
            continue
        #if (planner != 'symk') and (result['runtime'] < result['max_time']):
        #    print('{}: {:.0f}/{:.0f}'.format(planner, result['runtime'], result['max_time']))
        if (result['num_plans'] >= min_plans): # or ((planner != 'symk') and (result['runtime'] < 0.9*result['max_time'])):
            success_counter[domain, planner] += 1
            if verbose:
                print('c={:.3f}, n={}, t={:.0f}, {}'.format(
                    result.get('all_plans', None), result['num_plans'],
                    result['runtime'], result['problem_path']))  # result['domain_path']

    domain_frequencies = defaultdict(Counter)
    for result in results:
        domain = extract_domain(result['domain_path'])
        domain_frequencies[domain][result['num_plans']] += 1
    for domain in sorted(domain_frequencies):
        print(domain, sorted(domain_frequencies[domain].elements()))

    print(SEPARATOR)
    print('Solved {} domains, {} problems'.format(len(success_counter), sum(success_counter.values())))
    #print(str_from_object(counter))
    #print(sum(success_counter.values()), sum(total_counter.values()))

    problem_counter = Counter()
    for (problem, _), num in success_counter.items():
        problem_counter[problem] += num

    planner_counter = Counter()
    for (_, planner), num in success_counter.items():
        #planner_counter[planner] += 1 # Domains
        planner_counter[planner] += num # Problems
    print('Planners: ', '  '.join('{} ({}/{})'.format(
        planner, planner_counter[planner], planner_trials[planner]) for planner in sorted(planner_trials)))

    #for domain in sorted(counter, key=counter.get):
    #for domain in sorted(problems, key=problems.get): # Num problems
    for domain in sorted(problem_counter, key=problem_counter.get): # Most successes
        print('{:<25} {:3}'.format(domain, problem_counter[domain]), end='')
        for planner in sorted(planner_trials):
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
        #solve_pddlstream()
        solve_pddl()

if __name__ == '__main__':
    main()

# TODO: extend PDDLStream to support typing directly

# https://github.com/AI-Planning/classical-domains/tree/master/classical/data-network-opt18
# Packet sizes

# ./FastDownward/fast-downward.py --show-aliases
# ./FastDownward/fast-downward.py --build release64 --alias lama examples/fault_tolerant/data_network/domain.pddl examples/fault_tolerant/data_network/problem.pddl
