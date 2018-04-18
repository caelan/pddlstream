from __future__ import print_function

from time import time
import sys
import os
import shutil

INF = float('inf')
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

def read(filename):
    with open(filename, 'r') as f:
        return f.read()


def write(filename, string):
    with open(filename, 'w') as f:
        f.write(string)


def safe_remove(p):
    if os.path.exists(p):
        os.remove(p)


def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)


def safe_rm_file(p):
    if os.path.exists(p):
        os.remove(p)


def safe_rm_dir(d):
    if os.path.exists(d):
        shutil.rmtree(d)


##################################################

def get_fd_root():
    if ENV_VAR not in os.environ:
        raise RuntimeError('Environment variable %s is not defined.' % ENV_VAR)
    return os.environ[ENV_VAR]


def run_translate(verbose, temp_dir, use_negative=False):
    t0 = time()
    translate_path = os.path.join(get_fd_root(), FD_BIN, TRANSLATE_DIR)
    if translate_path not in sys.path:
        sys.path.append(translate_path)

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


def run_search(planner, max_time, max_cost, verbose, temp_dir):
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


def fast_downward(domain_pddl, problem_pddl, planner='max-astar',
                  max_time=INF, max_cost=INF, verbose=False, clean=False, temp_dir=TEMP_DIR):
    t0 = time()
    safe_rm_dir(temp_dir)
    ensure_dir(temp_dir)
    write(temp_dir + DOMAIN_INPUT, domain_pddl)
    write(temp_dir + PROBLEM_INPUT, problem_pddl)
    run_translate(verbose, temp_dir)
    solution = run_search(planner, max_time, max_cost, verbose, temp_dir)
    if clean:
        safe_rm_dir(temp_dir)
    print('Total runtime:', time() - t0)
    if solution is None:
        return None
    return parse_solution(solution)
