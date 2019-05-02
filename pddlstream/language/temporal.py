from __future__ import print_function

import math
import os
import re
import time

from pddlstream.algorithms.downward import TEMP_DIR, write_pddl, DOMAIN_INPUT, PROBLEM_INPUT
from pddlstream.language.write_pddl import pddl_problem
from pddlstream.language.tfd import tfd, parse_temporal_solution
from pddlstream.language.conversion import obj_from_pddl_plan
from pddlstream.utils import elapsed_time, INF, read

TMP_OUTPUT_PATH = 'tmp_sas_plan'

ENV_VAR = 'TPSHE_PATH'
#COMMAND = 'python {}bin/plan.py she {} {} --time {} --no-iterated'
COMMAND = 'python {}bin/plan.py she {} {} --time {} --iterated'
#COMMAND = 'python {}bin/plan.py she {} {} --time {}'
#COMMAND = 'python {}bin/plan.py tempo-3 {} {} --time {}'
#COMMAND = 'python {}bin/plan.py stp-3 {} {} --time {}'

def run_tpshe(max_time, verbose):
    tpshe_root = os.environ[ENV_VAR]
    command = COMMAND.format(tpshe_root, TEMP_DIR + DOMAIN_INPUT,
                             TEMP_DIR + PROBLEM_INPUT, int(math.ceil(max_time)))
    t0 = time.time()
    p = os.popen(command) # NOTE - cannot pipe input easily with subprocess
    if verbose: print(command)
    output = p.read()
    if verbose:
        print()
        print(output)
        print('Runtime:', elapsed_time(t0))
    # if not os.path.exists(SEARCH_OUTPUT):
    #   return None
    # return read(SEARCH_OUTPUT)

    plan_files = sorted(f for f in os.listdir('.') if f.startswith(TMP_OUTPUT_PATH))
    if not plan_files:
        return None
    best_plan, best_makespan = None, INF
    for plan_file in plan_files:
        print(plan_file)
        plan, duration = parse_temporal_solution(read(plan_file))
        print(plan)
        print(plan_file, len(plan), duration)
        if duration < best_makespan:
            best_plan, best_makespan = plan, duration
    return best_plan

def solve_temporal(evaluations, goal_exp, domain_pddl):
    # path = '/Users/caelan/Programs/LIS/planners/cerberus'
    # './plan.py {domain} {problem} {plan}'
    # ./build.py debug64
    [domain_name] = re.findall(r'\(domain ([^ ]+)\)', domain_pddl)
    problem_name = domain_name
    problem_pddl = pddl_problem(domain_name, problem_name, evaluations, goal_exp, objective='total-time')
    write_pddl(domain_pddl, problem_pddl, TEMP_DIR)
    pddl_plan, cost = tfd(domain_pddl, problem_pddl)

    plan = obj_from_pddl_plan(pddl_plan)
    return plan, cost
    #solution = run_tpshe(max_time=10, verbose=True)
    #cost = INF if solution is None else 0
    #return solution, cost
