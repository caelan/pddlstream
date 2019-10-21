from __future__ import print_function

#from os.path import expanduser
import os
import re
import subprocess
import time
import sys

from collections import namedtuple

from pddlstream.algorithms.downward import TEMP_DIR, DOMAIN_INPUT, PROBLEM_INPUT, make_effects, \
    parse_sequential_domain, get_conjunctive_parts, write_pddl
from pddlstream.language.constants import DurativeAction
from pddlstream.utils import INF, ensure_dir, write, user_input, safe_rm_dir, read, elapsed_time, find_unique, safe_zip

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
#TFD_PATH = '/home/caelan/Programs/TemPorAl/src/src/TFD'
#TFD_PATH = '/home/caelan/Programs/TemPorAl/src/src/temporal-FD'
MAX_TIME = 20
PLAN_FILE = 'plan'
TFD_TRANSLATE = os.path.join(TFD_PATH, 'translate/') # tfd & temporal-FD
#TFD_TRANSLATE = os.path.join(TFD_PATH, 'downward/translate/') # TFD

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
    'G': 'm',     # g-value evaluation
    'Q': 'p',     # queue (r, p, h)
    'r': True,    # reschedule # TODO: reschedule doesn't seem to work well with conditional effects
    #'O': 1,       # num ordered preferred ops, TFD doesn't support
    #'C': 1,       # num cheapest preferred ops, TFD doesn't support
    #'E': 1000,    # num expensive preferred ops
    #'R': 1000,    # num random preferred ops,
    'e': True,    # epsilon internally
    'f': False,  # epsilon externally
    #'b': True,   # reset after solution, TFD doesn't support
}

# https://github.com/caelan/TemporalFastDownward/blob/020da65a39d3f44c821cc2062d1006ccb0fcd7e5/downward/search/best_first_search.cc#L376

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
# K use_known_by_logical_state_only=True

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
    # TODO: duration per action
    return [DurativeAction(name, args, i * duration, duration)
            for i, (name, args) in enumerate(plan)]

##################################################

#TemporalDomain = namedtuple('TemporalDomain', ['name', 'requirements', 'types', 'constants',
#                                               'predicates', 'functions', 'actions', 'durative_actions', 'axioms'])

SimplifiedDomain = namedtuple('SimplifiedDomain', ['name', 'requirements', 'types', 'type_dict', 'constants',
                                                   'predicates', 'predicate_dict', 'functions', 'actions', 'axioms',
                                                   'durative_actions', 'pddl'])


def parse_temporal_domain(domain_pddl):
    prefixes = ['pddl', 'normalize']
    deleted = delete_imports(prefixes)
    sys.path.insert(0, TFD_TRANSLATE)
    import pddl
    import normalize
    temporal_domain = TemporalDomain(*pddl.tasks.parse_domain(pddl.parser.parse_nested_list(domain_pddl.splitlines())))
    name, requirements, constants, predicates, types, functions, actions, durative_actions, axioms = temporal_domain
    fluents = normalize.get_fluent_predicates(temporal_domain)

    sys.path.remove(TFD_TRANSLATE)
    delete_imports(prefixes)
    sys.modules.update(deleted) # This is important otherwise classes are messed up
    import pddl
    import pddl_parser
    assert not actions

    simple_from_durative = simple_from_durative_action(durative_actions, fluents)
    simple_actions = [action for triplet in simple_from_durative.values() for action in triplet]

    requirements = pddl.Requirements([])
    types = [pddl.Type(ty.name, ty.basetype_name) for ty in types]
    pddl_parser.parsing_functions.set_supertypes(types)
    predicates = [pddl.Predicate(p.name, p.arguments) for p in predicates]
    constants = convert_parameters(constants)
    axioms = list(map(convert_axiom, axioms))

    return SimplifiedDomain(name, requirements, types, {ty.name: ty for ty in types}, constants, predicates,
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

def delete_imports(prefixes=['pddl']):
    deleted = {}
    for name in list(sys.modules):
        if not name.startswith('pddlstream') and any(name.startswith(prefix) for prefix in prefixes):
            deleted[name] = sys.modules.pop(name)
    return deleted

#def simple_action_stuff(name, parameters, condition, effects):
#    import pddl
#    parameters = [pddl.TypedObject(param.name, param.type) for param in parameters]
#    return pddl.Action(name, parameters, len(parameters), condition, effects, None)

def convert_args(args):
    return [var.name for var in args]

def convert_condition(condition):
    import pddl
    class_name = condition.__class__.__name__
    if class_name in ('Truth', 'FunctionComparison'):
        # TODO: currently ignoring numeric conditions
        return pddl.Truth()
    elif class_name == 'Atom':
        return pddl.Atom(condition.predicate, convert_args(condition.args))
    elif class_name == 'NegatedAtom':
        return pddl.NegatedAtom(condition.predicate, convert_args(condition.args))
    elif class_name == 'Conjunction':
        return pddl.conditions.Conjunction(list(map(convert_condition, condition.parts)))
    elif class_name == 'Disjunction':
        return pddl.Disjunction(list(map(convert_condition, condition.parts)))
    elif class_name == 'ExistentialCondition':
        return pddl.ExistentialCondition(convert_parameters(condition.parameters),
                                         list(map(convert_condition, condition.parts)))
    raise NotImplementedError(class_name)

def convert_effects(effects):
    import pddl
    new_effects = make_effects([('_noop',)]) # To ensure the action has at least one effect
    for effect in effects:
        class_name = effect.__class__.__name__
        if class_name == 'Effect':
            peffect_name = effect.peffect.__class__.__name__
            if peffect_name in ('Increase', 'Decrease'):
                # TODO: currently ignoring numeric conditions
                continue
            new_effects.append(pddl.Effect(convert_parameters(effect.parameters),
                                           pddl.Conjunction(list(map(convert_condition, effect.condition))).simplified(),
                                           convert_condition(effect.peffect)))
        else:
            raise NotImplementedError(class_name)
    return new_effects

def convert_axiom(axiom):
    import pddl
    parameters = convert_parameters(axiom.parameters)
    return pddl.Axiom(axiom.name, parameters, len(parameters),
                      convert_condition(axiom.condition).simplified())

def convert_parameters(parameters):
    import pddl
    return [pddl.TypedObject(param.name, param.type) for param in parameters]

SIMPLE_TEMPLATE = '{}-{}'

def simple_from_durative_action(durative_actions, fluents):
    import pddl
    simple_actions = {}
    for action in durative_actions:
        parameters = convert_parameters(action.parameters)
        conditions = list(map(convert_condition, action.condition))
        start_effects, end_effects = action.effects
        over_effects = []
        effects = list(map(convert_effects, [start_effects, over_effects, end_effects]))

        static_condition = pddl.Conjunction(list({literal for condition in conditions
                                                  for literal in get_conjunctive_parts(condition.simplified())
                                                  if literal.predicate not in fluents}))
        actions = []
        for i, (condition, effect) in enumerate(safe_zip(conditions, effects)):
            actions.append(pddl.Action(SIMPLE_TEMPLATE.format(action.name, i), parameters, len(parameters),
                                       pddl.Conjunction([static_condition, condition]).simplified(), effect, None))
            #actions[-1].dump()
        simple_actions[action] = actions
    return simple_actions

def sequential_from_temporal_plan(plan):
    if plan is None:
        return plan
    over_actions = []
    state_changes = [DurativeAction(None, [], 0, 0)]
    for durative_action in plan:
        args = durative_action.args
        start, end = durative_action.start, get_end(durative_action)
        start_action, over_action, end_action = [SIMPLE_TEMPLATE.format(durative_action.name, i) for i in range(3)]
        state_changes.append(DurativeAction(start_action, args, start, end - start))
        #state_changes.append(DurativeAction(start_action, args, start, 0))
        over_actions.append(DurativeAction(over_action, args, start, end - start))
        state_changes.append(DurativeAction(end_action, args, end, 0))
    state_changes = sorted(state_changes, key=lambda a: a.start)

    sequence = []
    for i in range(1, len(state_changes)):
        # Technically should check the state change points as well
        start_action = state_changes[i-1]
        end_action = state_changes[i]
        for over_action in over_actions:
            if (over_action.start < end_action.start) and (start_action.start < get_end(over_action)): # Exclusive
                sequence.append(over_action)
        sequence.append(end_action)
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
    # TODO: close any opened resources

    temp_path = os.path.join(os.getcwd(), TEMP_DIR)
    plan_files = sorted(f for f in os.listdir(temp_path) if f.startswith(PLAN_FILE))
    print('Plans:', plan_files)
    best_plan, best_makespan = parse_plans(temp_path, plan_files)
    #if not debug:
    #    safe_rm_dir(TEMP_DIR)
    print('Makespan: ', best_makespan)
    print('Time:', elapsed_time(start_time))

    sequential_plan = sequential_from_temporal_plan(best_plan)
    return sequential_plan, best_makespan
