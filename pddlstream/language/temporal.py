from __future__ import print_function

#from os.path import expanduser
import os
import re
import subprocess
import time
import sys

from collections import namedtuple

from pddlstream.algorithms.downward import TEMP_DIR, DOMAIN_INPUT, PROBLEM_INPUT, parse_sequential_domain
from pddlstream.language.constants import DurativeAction
from pddlstream.utils import INF, ensure_dir, write, user_input, safe_rm_dir, read, elapsed_time, find_unique

PLANNER = 'tfd' # tfd | tflap | optic | tpshe | cerberus

# tflap: no conditional effects, no derived predicates
# optic: no negative preconditions, no conditional effects, no goal derived predicates

# TODO: previously slow instantiation was due to a missing precondition on move

# TODO: installing coin broke FD compilation so I uninstalled it
# sudo apt-get install cmake coinor-libcbc-dev coinor-libclp-dev
# sudo apt-get install coinor-libcoinutils-dev coinor-libosi-dev coinor-libcgl-dev doxygen libbz2-dev bison flex
# sudo apt-get install coinor-cbc
# sudo apt-get install apt-get -y install g++ make flex bison cmake doxygen coinor-clp coinor-libcbc-dev coinor-libclp-dev coinor-libcoinutils-dev coinor-libosi-dev coinor-libcgl-dev libbz2-dev libgsl-dev libz-dev
# sudo apt-get install g++ make flex bison cmake doxygen coinor-clp coinor-libcbc-dev coinor-libclp-dev coinor-libcoinutils-dev coinor-libosi-dev coinor-libcgl-dev libbz2-dev libgsl-dev libz-dev
# sudo apt-get remove coinor-libcbc-dev coinor-libclp-dev
# sudo apt-get remove coinor-libcoinutils-dev coinor-libosi-dev coinor-libcgl-dev

##################################################

# /home/caelan/Programs/VAL

TFD_PATH = '/home/caelan/Programs/tfd-src-0.4/downward'
MAX_TIME = 20
PLAN_FILE = 'plan'
TFD_TRANSLATE = os.path.join(TFD_PATH, 'translate/')

# TODO: the search produces unsound plans when it prints the full state-space
# TODO: still occasionally does this with the current settings

TFD_OPTIONS = {
    'a': False,   # anytime search
    't': MAX_TIME,     # success timeout
    'T': MAX_TIME,     # failure timeout
    'g': False,   # greedy search
    'l': False,    # disable lazy evaluation
    'v': True,    # disable verbose
    'y+Y': False, # CEA heuristic
    'x+X': True,  # makespan heuristic
    'G': 't',     # g-value evaluation (using m finds incorrect plans)
    'Q': 'p',     # queue
    'r': False,    # reschedule # TODO: reschedule doesn't seem to work well with conditional effects
    'O': 1,       # num ordered preferred ops
    'C': 1,       # num cheapest preferred ops
    #'E': 1000,    # num expensive preferred ops
    #'R': 1000,    # num random preferred ops,
    'e': True,    # epsilon internally
    'f': False,  # epsilon externally
    'b': True,   # reset after solution
}

# best_first_search
# makespan seems to be computed using timestep plus longest action

def format_option(pair):
    key, value = pair
    if value is True:
        return key
    if value is False:
        return None
    return '{}+{}'.format(key, value)

# Contains universal conditions: 1
# Disabling rescheduling because of universal conditions in original task!
# Doesn't look like temporal FastDownward uses non-boolean variables

# /home/caelan/Programs/VAL/validate /home/caelan/Programs/pddlstream/temp/domain.pddl /home/caelan/Programs/pddlstream/temp/problem.pddl /home/caelan/Programs/pddlstream/temp/plan

TFD_ARGS = '+'.join(sorted(filter(lambda s: s is not None, map(format_option, TFD_OPTIONS.items()))))

# Parameters just used in search (and split by +)
#TFD_COMMAND = 'plan.py n {} {} {}' # Default in plannerParameters.h
#TFD_COMMAND = 'plan.py y+Y+a+e+r+O+1+C+1+b {} {} {}' # Default in ./plan
#TFD_COMMAND = 'plan.py y+Y+e+O+1+C+1+b {} {} {}'
#TFD_COMMAND = 'plan.py +x+X+e+O+1+C+1+b+G+m+T+10+Q+p {} {} {}'
TFD_COMMAND = 'plan.py %s {} {} {}' % TFD_ARGS

# TODO: TFD sometimes returns incorrect plans
# ./VAL/validate pddlstream/temp/domain.pddl pddlstream/temp/problem.pddl pddlstream/temp/plan

# Finds a plan and then retimes it

"""
Usage: search <option characters>  (input read from stdin)
Options are:
  a - enable anytime search (otherwise finish on first plan found)
  t <timeout secs> - total timeout in seconds for anytime search (when plan found)
  T <timeout secs> - total timeout in seconds for anytime search (when no plan found)
  m <monitor file> - monitor plan, validate a given plan
  g - perform greedy search (follow heuristic)
  l - disable lazy evaluation (Lazy = use parent's f instead of child's)
  v - disable verbose printouts
  y - cyclic cg CEA heuristic
  Y - cyclic cg CEA heuristic - preferred operators
  x - cyclic cg makespan heuristic 
  X - cyclic cg makespan heuristic - preferred operators
  G [m|c|t|w] - G value evaluation, one of m - makespan, c - pathcost, t - timestamp, w [weight] - weighted / Note: One of those has to be set!
  Q [r|p|h] - queue mode, one of r - round robin, p - priority, h - hierarchical
  K - use tss known filtering (might crop search space)!
  n - no_heuristic
  r - reschedule_plans
  O [n] - prefOpsOrderedMode, with n being the number of pref ops used
  C [n] - prefOpsCheapestMode, with n being the number of pref ops used
  E [n] - prefOpsMostExpensiveMode, with n being the number of pref ops used
  e - epsilonize internally
  f - epsilonize externally
  p <plan file> - plan filename prefix
  M v - monitoring: verify timestamps
  u - do not use cachin in heuristic
"""
# b - reset_after_solution_was_found
# p - plan_name
# i - reward_only_pref_op_queue
# S - pref_ops_concurrent_mode
# R - number_pref_ops_rand_mode

# Default parameters (plan.py n {} {} {})
"""
Planner Paramters:
Anytime Search: Disabled
Timeout if plan was found: 0 seconds (no timeout)
Timeout while no plan was found: 0 seconds (no timeout)
Greedy Search: Disabled
Verbose: Enabled
Lazy Heuristic Evaluation: Enabled
Use caching in heuristic.
Cyclic CG heuristic: Disabled 	Preferred Operators: Disabled
Makespan heuristic: Disabled 	Preferred Operators: Disabled
No Heuristic: Enabled
Cg Heuristic Zero Cost Waiting Transitions: Enabled
Cg Heuristic Fire Waiting Transitions Only If Local Problems Matches State: Disabled
PrefOpsOrderedMode: Disabled with 1000 goals
PrefOpsCheapestMode: Disabled with 1000 goals
PrefOpsMostExpensiveMode: Disabled with 1000 goals
PrefOpsRandMode: Disabled with 1000 goals
PrefOpsConcurrentMode: Disabled
Reset after solution was found: Disabled
Reward only preferred operators queue: Disabled
GValues by: Timestamp
Queue management mode: Priority based
Known by logical state only filtering: Disabled
use_subgoals_to_break_makespan_ties: Disabled
Reschedule plans: Disabled
Epsilonize internally: Disabled
Epsilonize externally: Disabled
Keep original plans: Enabled
Plan name: "/home/caelan/Programs/pddlstream/temp/plan"
Plan monitor file: "" (no monitoring)
Monitoring verify timestamps: Disabled
"""

# plannerParameters.h
"""
anytime_search = false;
timeout_while_no_plan_found = 0;
timeout_if_plan_found = 0;
greedy = false;
lazy_evaluation = true;
verbose = true;
insert_let_time_pass_only_when_running_operators_not_empty = false;
cyclic_cg_heuristic = false;
cyclic_cg_preferred_operators = false;
makespan_heuristic = false;
makespan_heuristic_preferred_operators = false;
no_heuristic = false;
cg_heuristic_zero_cost_waiting_transitions = true;
cg_heuristic_fire_waiting_transitions_only_if_local_problems_matches_state = false;
use_caching_in_heuristic = true;
g_values = GTimestamp;
g_weight = 0.5;
queueManagementMode = BestFirstSearchEngine::PRIORITY_BASED;
use_known_by_logical_state_only = false;
use_subgoals_to_break_makespan_ties = false;
reschedule_plans = false;
epsilonize_internally = false;
epsilonize_externally = false;
keep_original_plans = true;
pref_ops_ordered_mode = false;
pref_ops_cheapest_mode = false;
pref_ops_most_expensive_mode = false;
pref_ops_rand_mode = false;
pref_ops_concurrent_mode = false;
number_pref_ops_ordered_mode = 1000;
number_pref_ops_cheapest_mode = 1000;
number_pref_ops_most_expensive_mode = 1000;
number_pref_ops_rand_mode = 1000;
reset_after_solution_was_found = false;
reward_only_pref_op_queue = false;
plan_name = "sas_plan";
planMonitorFileName = "";
monitoring_verify_timestamps = false;
"""

##################################################

TFLAP_PATH = '/home/caelan/Programs/tflap/src'

# Usage: tflap <domain_file> <problem_file> <output_file> [-ground] [-static] [-mutex] [-trace]
# -ground: generates the GroundedDomain.pddl and GroundedProblem.pddl files.
# -static: keeps the static data in the planning task.
# -nsas: does not make translation to SAS (finite-domain variables).
# -mutex: generates the mutex.txt file with the list of static mutex facts.
# -trace: generates the trace.txt file with the search tree.

TFLAP_COMMAND = 'tflap {} {} {}'
#TFLAP_COMMAND = 'tflap {} {} {} -trace' # Seems to repeatedly fail

##################################################

OPTIC_PATH = '/home/caelan/Programs/optic2018/src/optic/src/optic'
OPTIC_COMMAND = 'optic-clp -N {} {} | tee {}'

"""
Usage: optic/src/optic/optic-clp [OPTIONS] domainfile problemfile [planfile, if -r specified]

Options are: 

	-N	Don't optimise solution quality (ignores preferences and costs);
	-0	Abstract out timed initial literals that represent recurrent windows;
	-n<lim>	Optimise solution quality, capping cost at <lim>;

	-citation	Display citation to relevant papers;
	-b		Disable best-first search - if EHC fails, abort;
	-E		Skip EHC: go straight to best-first search;
	-e		Use standard EHC instead of steepest descent;
	-h		Disable helpful-action pruning;
	-k		Disable compression-safe action detection;
	-c		Enable the tie-breaking in RPG that favour actions that slot into the partial order earlier;
	-S		Sort initial layer facts in RPG by availability order (only use if using -c);
	-m		Disable the tie-breaking in search that favours plans with shorter makespans;
	-F		Full FF helpful actions (rather than just those in the RP applicable in the current state);
	-r		Read in a plan instead of planning;
	-T		Rather than building a partial order, build a total-order
	-v<n>		Verbose to degree n (n defaults to 1 if not specified).
	-L<n>		LP verbose to degree n (n defaults to 1 if not specified).
"""

"""
Unfortunately, at present, the planner does not fully support ADL
unless in the rules for derived predicates.  Only two aspects of
ADL can be used in action definitions:
- forall conditions, containing a simple conjunct of propositional and
  numeric facts;
- Conditional (when... ) effects, and then only with numeric conditions
  and numeric consequences on values which do not appear in the
  preconditions of actions.
"""

##################################################

# TODO: tpshe seems to be broken

"""
usage: plan.py [-h] [--generator GENERATOR] [--time TIME] [--memory MEMORY]
               [--iterated] [--no-iterated] [--plan-file PLANFILE]
               [--validate] [--no-validate]
               planner domain problem
"""

TPSHE_PATH = '/home/caelan/Programs/temporal-planning/'
#TPSHE_COMMAND = 'python {}bin/plan.py she {} {} --time {} --no-iterated'
TPSHE_COMMAND = 'bin/plan.py she {} {} --iterated'
#TPSHE_COMMAND = 'python {}bin/plan.py she {} {} --time {}'
#TPSHE_COMMAND = 'python {}bin/plan.py tempo-3 {} {} --time {}'
#TPSHE_COMMAND = 'python {}bin/plan.py stp-3 {} {} --time {}'
#temp_path = '.'
TPSHE_OUTPUT_PATH = 'tmp_sas_plan'


##################################################

CERB_PATH = '/home/caelan/Programs/cerberus'
#CERB_PATH = '/home/caelan/Programs/pddlstream/FastDownward'
#CERB_COMMAND = 'fast-downward.py {} {}'
CERB_COMMAND = 'plan.py {} {} {}'

# https://ipc2018-classical.bitbucket.io/planner-abstracts/teams_15_16.pdf

##################################################

def parse_temporal_solution(solution):
    makespan = 0.0
    plan = []
    # TODO: this regex doesn't work for @
    regex = r'(\d+.\d+):\s+\(\s*(\w+(?:\s\w+)*)\s*\)\s+\[(\d+.\d+)\]'
    for start, action, duration in re.findall(regex, solution):
        entries = action.lower().split(' ')
        action = DurativeAction(entries[0], tuple(entries[1:]), float(start), float(duration))
        plan.append(action)
        makespan = max(action.start + action.duration, makespan)
    return plan, makespan

def write_pddl(domain_pddl, problem_pddl):
    # TODO: already in downward.py
    safe_rm_dir(TEMP_DIR)  # Ensures not using old plan
    ensure_dir(TEMP_DIR)
    domain_path = TEMP_DIR + DOMAIN_INPUT
    problem_path = TEMP_DIR + PROBLEM_INPUT
    write(domain_path, domain_pddl)
    write(problem_path, problem_pddl)
    return domain_path, problem_path

def parse_plans(temp_path, plan_files):
    best_plan, best_makespan = None, INF
    for plan_file in plan_files:
        solution = read(os.path.join(temp_path, plan_file))
        plan, makespan = parse_temporal_solution(solution)
        if makespan < best_makespan:
            best_plan, best_makespan = plan, makespan
    return best_plan, best_makespan

##################################################

def get_end(action):
    return action.start + action.duration


def compute_duration(plan):
    if not plan:
        return 0
    return max(map(get_end, plan))


def retime_plan(plan, duration=1):
    if plan is None:
        return plan
    return [DurativeAction(name, args, i * duration, duration)
            for i, (name, args) in enumerate(plan)]

##################################################

TemporalDomain = namedtuple('TemporalDomain', ['name', 'requirements', 'types', 'type_dict', 'constants',
                                               'predicates', 'predicate_dict', 'functions', 'actions', 'axioms',
                                               'durative_actions', 'pddl'])

def parse_temporal_domain(domain_pddl):
    delete_pddl_imports()
    sys.path.insert(0, TFD_TRANSLATE)
    import pddl
    domain_name, requirements, constants, predicates, types, functions, actions, durative_actions, axioms = \
        pddl.tasks.parse_domain(pddl.parser.parse_nested_list(domain_pddl.splitlines()))
    sys.path.remove(TFD_TRANSLATE)
    delete_pddl_imports()
    assert not actions

    simple_from_durative = simple_from_durative_action(durative_actions)
    simple_actions = [action for triplet in simple_from_durative.values() for action in triplet]

    return TemporalDomain(domain_name, requirements, types, {ty.name for ty in types}, constants, predicates,
                          {p.name: p for p in predicates}, functions, simple_actions, axioms,
                          simple_from_durative, domain_pddl)

def parse_domain(domain_pddl):
    try:
        return parse_sequential_domain(domain_pddl)
    except AssertionError as e:
        if str(e) == ':durative-actions':
            return parse_temporal_domain(domain_pddl)
        raise e

##################################################

def delete_pddl_imports():
    deleted = {}
    for name in list(sys.modules):
        if ('pddl' in name) and ('pddlstream' not in name):
            deleted[name] = sys.modules.pop(name)
    return deleted

#def simple_action_stuff(name, parameters, condition, effects):
#    import pddl
#    parameters = [pddl.TypedObject(param.name, param.type) for param in parameters]
#    return pddl.Action(name, parameters, len(parameters), condition, effects, None)

def simple_from_durative_action(durative_actions):
    import pddl
    simple_actions = {}
    for action in durative_actions:
        parameters = [pddl.TypedObject(param.name, param.type) for param in action.parameters]
        start_condition, over_condition, end_condition = action.condition
        start_effects, end_effects = action.effects
        start_action = pddl.Action('{}-0'.format(action.name), parameters, len(action.parameters),
                                   start_condition, start_effects, None)
        over_action = pddl.Action('{}-1'.format(action.name), parameters, len(action.parameters),
                                  over_condition, [], None)
        end_action = pddl.Action('{}-2'.format(action.name), parameters, len(action.parameters),
                                 end_condition, end_effects, None)
        simple_actions[action] = (start_action, over_action, end_action)
    return simple_actions

def sequential_from_temporal(domain_path, problem_path, best_plan):
    if best_plan is None:
        return best_plan

    delete_pddl_imports()
    tfd_translate = os.path.join(TFD_PATH, 'translate/')
    sys.path.insert(0, tfd_translate)
    import pddl
    task = pddl.pddl_file.open(problem_path, domain_path)
    sys.path.remove(tfd_translate)
    delete_pddl_imports()
    #sys.modules.update(deleted)

    #print(task.function_administrator) # derived functions
    #task.dump()
    assert not task.actions
    simple_actions = simple_from_durative_action(task.durative_actions)

    over_actions = []
    state_changes = [(0, None)]
    for action in best_plan:
        durative_action = find_unique(lambda a: a.name == action.name, simple_actions)
        start_action, over_action, end_action = simple_actions[durative_action]
        start, end = action.start, get_end(action)
        state_changes.append((start, start_action))
        state_changes.append((end, end_action))
        over_actions.append(((start, end), over_action))
    state_changes = sorted(state_changes, key=lambda p: p[0])
    print(state_changes)

    sequence = []
    for i in range(1, len(state_changes)):
        # Technically should check the state change points as well
        start_t, _ = state_changes[i-1]
        end_t, end_action = state_changes[i]
        for (t1, t2), over_action in over_actions:
            if (t1 < end_t) and (start_t < t2): # Exclusive
                sequence.append(over_action)
        sequence.append(end_action)
    print(sequence)

    #import imp
    #parser_path = os.path.join(TFD_PATH, 'translate/pddl/pddl_file.py')
    #parser_module = imp.load_source('pddl', parser_path)
    return sequence

##################################################

def solve_tfd(domain_pddl, problem_pddl, max_time=INF, debug=False):
    if PLANNER == 'tfd':
        root, template = TFD_PATH, TFD_COMMAND
    elif PLANNER == 'cerberus':
        root, template = CERB_PATH, CERB_COMMAND
    elif PLANNER == 'tflap':
        root, template = TFLAP_PATH, TFLAP_COMMAND
    elif PLANNER == 'optic':
        root, template = OPTIC_PATH, OPTIC_COMMAND
    elif PLANNER == 'tpshe':
        root, template = TPSHE_PATH, TPSHE_COMMAND
    else:
        raise ValueError(PLANNER)

    start_time = time.time()
    domain_path, problem_path = write_pddl(domain_pddl, problem_pddl)
    plan_path = os.path.join(TEMP_DIR, PLAN_FILE)
    #assert not actions, "There shouldn't be any actions - just temporal actions"

    paths = [os.path.join(os.getcwd(), p) for p in (domain_path, problem_path, plan_path)]
    command = os.path.join(root, template.format(*paths))
    print(command)
    if debug:
        stdout, stderr = None, None
    else:
        stdout, stderr = open(os.devnull, 'w'), open(os.devnull, 'w')
    proc = subprocess.call(command, shell=True, cwd=root, stdout=stdout, stderr=stderr) # timeout=None (python3)
    error = proc != 0
    print('Error:', error)

    temp_path = os.path.join(os.getcwd(), TEMP_DIR)
    plan_files = sorted(f for f in os.listdir(temp_path) if f.startswith(PLAN_FILE))
    print('Plans:', plan_files)
    best_plan, best_makespan = parse_plans(temp_path, plan_files)
    if not debug:
        safe_rm_dir(TEMP_DIR)
    print('Makespan: ', best_makespan)
    print('Time:', elapsed_time(start_time))
    sequential_from_temporal(domain_path, problem_path, best_plan)

    return best_plan, best_makespan
