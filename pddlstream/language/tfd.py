from __future__ import print_function

#from os.path import expanduser
import os
import re
import subprocess

from pddlstream.algorithms.downward import TEMP_DIR, DOMAIN_INPUT, PROBLEM_INPUT
from pddlstream.language.constants import DurativeAction
from pddlstream.utils import INF, ensure_dir, write, user_input, safe_rm_dir, read

PLANNER = 'tfd' # tfd | tflap | optic

##################################################

# /home/caelan/Programs/VAL

TFD_PATH = '/home/caelan/Programs/tfd-src-0.4/downward'
MAX_TIME = 10
PLAN_FILE = 'plan'

TFD_OPTIONS = {
    'a': False,   # anytime search
    't': MAX_TIME,     # success timeout
    'T': MAX_TIME,     # failure timeout
    'g': False,   # greedy search
    'l': False,    # disable lazy evaluation
    'v': True,    # disable verbose
    'y+Y': False, # CEA heuristic
    'x+X': True,  # makespan heuristic
    'G': 't',     # g-value evaluation
    'Q': 'p',     # queue
    'r': True,    # reschedule
    'O': 1,       # num ordered preferred ops
    'C': 1,       # num cheapest preferred ops
    #'E': 1000,    # num expensive preferred ops
    #'R': 1000,    # num random preferred ops,
    'e': True,    # epsilon internally
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

# Disabling rescheduling because of universal conditions in original task!

# /home/caelan/Programs/VAL/validate /home/caelan/Programs/pddlstream/temp/domain.pddl /home/caelan/Programs/pddlstream/temp/problem.pddl /home/caelan/Programs/pddlstream/temp/plan

TFD_ARGS = '+'.join(sorted(filter(lambda s: s is not None, map(format_option, TFD_OPTIONS.items()))))

# Parameters just used in search (and split by +)
#TFD_COMMAND = 'plan.py n {} {} {}' # Default in plannerParameters.h
#TFD_COMMAND = 'plan.py y+Y+a+e+r+O+1+C+1+b {} {} {}' # Default in ./plan
#TFD_COMMAND = 'plan.py y+Y+e+O+1+C+1+b {} {} {}'
#TFD_COMMAND = 'plan.py +x+X+e+O+1+C+1+b+G+m+T+10+Q+p {} {} {}'
TFD_COMMAND = 'plan.py %s {} {} {}' % TFD_ARGS

print(TFD_COMMAND)

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

OPTIC_COMMAND = 'optic-clp -N -b {} {} {}'

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

def parse_temporal_solution(solution):
    makespan = 0.0
    plan = []
    regex = r'(\d+.\d+): \(\s*(\w+(?:\s\w+)*)\s*\) \[(\d+.\d+)\]'
    for start, action, duration in re.findall(regex, solution):
        entries = action.lower().split(' ')
        action = DurativeAction(entries[0], tuple(entries[1:]), float(start), float(duration))
        plan.append(action)
        makespan = max(action.start + action.duration, makespan)
    return plan, makespan

##################################################

def tfd(domain_pddl, problem_pddl, max_time=INF, max_cost=INF, verbose=True):
    if PLANNER == 'tfd':
        root, template = TFD_PATH, TFD_COMMAND
    elif PLANNER == 'tflap':
        root, template = TFLAP_PATH, TFLAP_COMMAND
    elif PLANNER == 'optic':
        root, template = OPTIC_PATH, OPTIC_COMMAND
    else:
        raise ValueError(PLANNER)

    safe_rm_dir(TEMP_DIR) # Ensures not using old plan
    ensure_dir(TEMP_DIR)
    domain_path = TEMP_DIR + DOMAIN_INPUT
    problem_path = TEMP_DIR + PROBLEM_INPUT
    write(domain_path, domain_pddl)
    write(problem_path, problem_pddl)

    # Contains universal conditions: 1
    # Disabling rescheduling because of universal conditions in original task!
    # Doesn't look like temporal FastDownward uses non-boolean variables

    plan_path = os.path.join(TEMP_DIR, PLAN_FILE)
    #assert not actions, "There shouldn't be any actions - just temporal actions"

    paths = [os.path.join(os.getcwd(), p) for p in (domain_path, problem_path, plan_path)]
    command = os.path.join(root, template.format(*paths))
    if verbose:
        print(command)

    stdout = None if verbose else open(os.devnull, 'w')
    #stdout = open(os.devnull, 'w')
    #stderr = open(os.devnull, 'w')
    stderr = None
    try:
        proc = subprocess.call(command.split(' '), cwd=root)
        #proc = subprocess.Popen(command.split(' '), cwd=get_tfd_root(), stdout=stdout, stderr=stderr)
        #proc.wait()
        #proc.terminate()
    except subprocess.CalledProcessError as e:
        print("Subprocess error", e.output)
        user_input("Continue?")

    temp_path = os.path.join(os.getcwd(), TEMP_DIR)
    plan_files = sorted([f for f in os.listdir(temp_path) if f.startswith(PLAN_FILE)])
    if verbose:
        print('Plans:', plan_files)

    best_plan, best_makespan =  None, INF
    for plan_file in plan_files:
        solution = read(os.path.join(temp_path, plan_file))
        plan, makespan = parse_temporal_solution(solution)
        if makespan < best_makespan:
            best_plan, best_makespan = plan, makespan
    if not verbose:
        safe_rm_dir(TEMP_DIR)
    return best_plan, best_makespan
