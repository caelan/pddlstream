from __future__ import print_function

import os
import sys
from time import time
from collections import namedtuple

from utils import read, write, ensure_dir, safe_rm_dir, INF

TEMP_DIR = 'temp/'
DOMAIN_INPUT = 'domain.pddl'
PROBLEM_INPUT = 'problem.pddl'
TRANSLATE_OUTPUT = 'output.sas'
SEARCH_OUTPUT = 'sas_plan'

ENV_VAR = 'FD_PATH'
FD_BIN = 'bin'
TRANSLATE_DIR = 'translate'
SEARCH_COMMAND = 'downward --internal-plan-file %s %s < %s'

# TODO: be careful when doing costs. Might not be admissible if use plus one for heuristic
# TODO: use goal_serialization / hierarchy on the inside loop of these algorithms

SEARCH_OPTIONS = {
    'dijkstra': '--heuristic "h=blind(transform=adapt_costs(cost_type=NORMAL))" '
                '--search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'max-astar': '--heuristic "h=hmax(transform=adapt_costs(cost_type=NORMAL))"'
                 ' --search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'ff-astar': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
                '--search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',

    'ff-wastar1': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=true,boost=100,w=1,'
                  'preferred_successors_first=true,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'ff-wastar3': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=100,w=3,'
                  'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',
    'ff-wastar5': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=100,w=5,'
                  'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',

    'cea-wastar1': '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" '
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w=1,'
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',
    'cea-wastar3': '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" '
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w=3,'
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',
    'cea-wastar5': '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" '
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w=5,'
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',

    'ff-eager': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                '--search "eager_greedy([hff],max_time=%s,bound=%s)"',
    'ff-eager-pref': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                     '--search "eager_greedy([hff],preferred=[hff],max_time=%s,bound=%s)"',
    'ff-lazy': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
               '--search "lazy_greedy([hff],preferred=[hff],max_time=%s,bound=%s)"',
}


##################################################


def get_fd_root():
    if ENV_VAR not in os.environ:
        raise RuntimeError('Environment variable %s is not defined.' % ENV_VAR)
    return os.environ[ENV_VAR]

def add_translate_path():
    translate_path = os.path.join(get_fd_root(), FD_BIN, TRANSLATE_DIR)
    if translate_path not in sys.path:
        sys.path.append(translate_path)

Domain = namedtuple('Domain', ['name', 'requirements', 'types', 'type_dict', 'constants',
                               'predicates', 'predicate_dict', 'functions', 'actions', 'axioms'])

def parse_lisp(lisp):
    add_translate_path()
    temp_argv = sys.argv[:]
    sys.argv = sys.argv[:1] + [DOMAIN_INPUT, PROBLEM_INPUT] # Arguments aren't used here
    from pddl_parser.lisp_parser import parse_nested_list, tokenize
    sys.argv = temp_argv
    lines = lisp.split()
    return parse_nested_list(lines)

#def parse_domain(domain_path):
def parse_domain(domain_pddl):
    add_translate_path()
    temp_argv = sys.argv[:]
    sys.argv = sys.argv[:1] + [DOMAIN_INPUT, PROBLEM_INPUT] # Arguments aren't used here
    from pddl_parser.pddl_file import parse_pddl_file
    from pddl_parser.parsing_functions import parse_domain_pddl
    sys.argv = temp_argv
    #return Domain(*parse_domain_pddl(parse_pddl_file('domain', domain_path) ))
    #domain_pddl = read(domain_path)
    return Domain(*parse_domain_pddl(parse_lisp(domain_pddl)))

def parse_problem(domain_path, problem_path):
    # TODO: requires domain_path
    raise NotImplementedError()

def translate_task(domain_path, problem_path):
    add_translate_path()

    temp_argv = sys.argv[:]
    sys.argv = sys.argv[:1] + [DOMAIN_INPUT, PROBLEM_INPUT] # Arguments aren't used here
    #import pddl_parser
    #from pddl_parser import open
    import normalize
    from pddl_parser.pddl_file import open
    from pddl_parser.pddl_file import parse_pddl_file
    sys.argv = temp_argv

    for line in parse_pddl_file('domain', domain_path):
        print(line)

    task = open(domain_filename=domain_path, task_filename=problem_path)
    #task = pddl_parser.open(
    #    domain_filename=domain_path, task_filename=problem_path)
    normalize.normalize(task)
    return task

def run_translate(temp_dir, verbose, use_negative=False):
    t0 = time()
    add_translate_path()
    translate_flags = []  # '--keep-unreachable-facts'
    if use_negative:
        translate_flags += ['--negative-axioms']

    temp_argv = sys.argv[:]
    sys.argv = sys.argv[:1] + translate_flags + [DOMAIN_INPUT, PROBLEM_INPUT]
    import translate
    sys.argv = temp_argv

    old_cwd = os.getcwd()
    tmp_cwd = os.path.join(old_cwd, temp_dir)
    if verbose:
        print('\nTranslate command: import translate; translate.main()')
        os.chdir(tmp_cwd)
        translate.main()
        os.chdir(old_cwd)
        print('Translate runtime:', time() - t0)
        return

    with open(os.devnull, 'w') as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        os.chdir(tmp_cwd)
        try:
            translate.main()
        finally:
            sys.stdout = old_stdout
            os.chdir(old_cwd)


def run_search(temp_dir, planner, max_time, max_cost, verbose):
    if max_time == INF:
        max_time = 'infinity'
    elif isinstance(max_time, float):
        max_time = int(max_time)
    if max_cost == INF:
        max_cost = 'infinity'
    elif isinstance(max_cost, float):
        max_cost = int(max_cost)

    t0 = time()
    search = os.path.join(get_fd_root(), FD_BIN, SEARCH_COMMAND)
    planner_config = SEARCH_OPTIONS[planner] % (max_time, max_cost)
    command = search % (temp_dir + SEARCH_OUTPUT, planner_config, temp_dir + TRANSLATE_OUTPUT)
    if verbose:
        print('\nSearch command:', command)
    p = os.popen(command)  # NOTE - cannot pipe input easily with subprocess
    output = p.read()
    if verbose:
        print(output[:-1])
        print('Search runtime:', time() - t0)
    if not os.path.exists(temp_dir + SEARCH_OUTPUT):
        return None
    return read(temp_dir + SEARCH_OUTPUT)


##################################################

def parse_solution(solution):
    if solution is None:
        return None
    # TODO: regex
    lines = solution.split('\n')[:-2]  # Last line is newline, second to last is cost
    plan = []
    for line in lines:
        entries = line.strip('( )').split(' ')
        plan.append((entries[0], list(entries[1:])))
    return plan

def write_pddl(domain_pddl=None, problem_pddl=None, temp_dir=TEMP_DIR):
    safe_rm_dir(temp_dir)
    ensure_dir(temp_dir)
    domain_path = os.path.join(temp_dir, DOMAIN_INPUT)
    if domain_pddl is not None:
        write(domain_path, domain_pddl)
    problem_path = os.path.join(temp_dir, PROBLEM_INPUT)
    if problem_pddl is not None:
        write(problem_path, problem_pddl)
    return domain_path, problem_path

def run_fast_downward(domain_pddl, problem_pddl, planner='max-astar',
                      max_time=INF, max_cost=INF, verbose=False, clean=False, temp_dir=TEMP_DIR):
    t0 = time()
    write_pddl(domain_pddl, problem_pddl, temp_dir)
    run_translate(temp_dir, verbose)
    solution = run_search(temp_dir, planner, max_time, max_cost, verbose)
    if clean:
        safe_rm_dir(temp_dir)
    print('Total runtime:', time() - t0)
    if solution is None:
        return None
    return parse_solution(solution)
