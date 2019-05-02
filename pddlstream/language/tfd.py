from __future__ import print_function

#from os.path import expanduser
import os
import subprocess

from pddlstream.algorithms.downward import TEMP_DIR, write_pddl, DOMAIN_INPUT, PROBLEM_INPUT
from pddlstream.language.write_pddl import pddl_problem
from pddlstream.utils import elapsed_time, INF, read, safe_remove, ensure_dir, write, user_input, safe_rm_dir

from collections import namedtuple

DurativeAction = namedtuple('DurativeAction', ['name', 'args', 'start', 'duration'])

ENV_VAR = 'TFD_PATH'

# Parameters just used in search (and split by +)
#COMMAND = 'plan.py y+Y+a+e+r+O+1+C+1+b {} {} {}' # Default
COMMAND = 'plan.py y+Y+e+O+1+C+1+b {} {} {}'
# b => reset_after_solution_was_found = true

# TODO: try different command sto minimize makespan

PLAN_FILE = 'plan'
# plannerParameters.h

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

# Usage: tflap <domain_file> <problem_file> <output_file> [-ground] [-static] [-mutex] [-trace]
# -ground: generates the GroundedDomain.pddl and GroundedProblem.pddl files.
# -static: keeps the static data in the planning task.
# -nsas: does not make translation to SAS (finite-domain variables).
# -mutex: generates the mutex.txt file with the list of static mutex facts.
# -trace: generates the trace.txt file with the search tree.

COMMAND = 'tflap {} {} {}'
#COMMAND = 'tflap {} {} {} -trace' # Seems to repeatedly fail


# Finds a plan and then retimes it

##################################################

def has_tfd():
    return True
    #return ENV_VAR in os.environ

def get_tfd_root():
    return '/home/caelan/Programs/tflap/src'

    #return '/home/caelan/Programs/tfd-src-0.4/downward'
    #if not has_tfd():
    #    raise RuntimeError('Environment variable %s is not defined.'%ENV_VAR)
    #return os.environ[ENV_VAR]
    #return expanduser(os.environ[ENV_VAR])

# TODO: can only really move one action at a time

def tfd(domain_pddl, problem_pddl, max_time=INF, max_cost=INF, verbose=True):
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
    command = os.path.join(get_tfd_root(), COMMAND.format(*paths))
    if verbose:
        print(command)

    stdout = None if verbose else open(os.devnull, 'w')
    #stdout = open(os.devnull, 'w')
    #stderr = open(os.devnull, 'w')
    stderr = None
    try:
        proc = subprocess.call(command.split(' '), cwd=get_tfd_root())
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

    if not plan_files:
        return None, INF
    with open(os.path.join(temp_path, plan_files[-1]), 'r') as f:
        output = f.read()

    # TODO: read all the files
    plan = []
    for line in output.split('\n')[:-1]:
        entries = line[line.find('(')+1:line.find(')')].split(' ')
        action, args = entries[0], entries[1:]
        if args == ['']:
            args = []
        plan.append((action, args))
    if not verbose:
        safe_rm_dir(TEMP_DIR)
    # TODO: return timing information
    return plan, 0