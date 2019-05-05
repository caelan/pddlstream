from __future__ import print_function

import math
import os
import time

from pddlstream.algorithms.downward import TEMP_DIR, DOMAIN_INPUT, PROBLEM_INPUT, write_pddl
from pddlstream.language.write_pddl import get_problem_pddl
from pddlstream.language.tfd import solve_tfd, parse_temporal_solution, parse_plans
from pddlstream.language.conversion import obj_from_pddl_plan
from pddlstream.utils import elapsed_time, INF, read

TMP_OUTPUT_PATH = 'tmp_sas_plan'

ENV_VAR = 'TPSHE_PATH'
#COMMAND = 'python {}bin/plan.py she {} {} --time {} --no-iterated'
COMMAND = 'python {}bin/plan.py she {} {} --time {} --iterated'
#COMMAND = 'python {}bin/plan.py she {} {} --time {}'
#COMMAND = 'python {}bin/plan.py tempo-3 {} {} --time {}'
#COMMAND = 'python {}bin/plan.py stp-3 {} {} --time {}'

# TODO: tpshe seems to be broken

def solve_tpshe(domain_pddl, problem_pddl, max_time=10, verbose=False):
    domain_path, problem_path = write_pddl(domain_pddl, problem_pddl)

    tpshe_root = os.environ[ENV_VAR]
    command = COMMAND.format(tpshe_root, domain_path, problem_path, int(math.ceil(max_time)))
    t0 = time.time()
    p = os.popen(command) # NOTE - cannot pipe input easily with subprocess
    if verbose:
        print(command)
    output = p.read()
    if verbose:
        print()
        print(output)
        print('Runtime:', elapsed_time(t0))
    # if not os.path.exists(SEARCH_OUTPUT):
    #   return None
    # return read(SEARCH_OUTPUT)

    temp_path = '.'
    plan_files = sorted(f for f in os.listdir(temp_path) if f.startswith(TMP_OUTPUT_PATH))
    print('Plans:', plan_files)
    best_plan, best_makespan = parse_plans(temp_path, plan_files)
    return best_plan, best_makespan

# path = '/Users/caelan/Programs/LIS/planners/cerberus'
# './plan.py {domain} {problem} {plan}'
# ./build.py debug64
