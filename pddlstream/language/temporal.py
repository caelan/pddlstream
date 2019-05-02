import math
import os
import re
import time

from pddlstream.algorithms.downward import TEMP_DIR, write_pddl
from pddlstream.language.write_pddl import pddl_problem
from pddlstream.utils import elapsed_time, INF, read

DOMAIN_PATH = 'domain.pddl'
PROBLEM_PATH = 'problem.pddl'
OUTPUT_PATH = 'sas_plan'
TMP_OUTPUT_PATH = 'tmp_sas_plan'

ENV_VAR = 'TPSHE_PATH'
#COMMAND = 'python {}bin/plan.py she {} {} --time {} --no-iterated'
COMMAND = 'python {}bin/plan.py she {} {} --time {} --iterated'
#COMMAND = 'python {}bin/plan.py she {} {} --time {}'
#COMMAND = 'python {}bin/plan.py tempo-3 {} {} --time {}'
#COMMAND = 'python {}bin/plan.py stp-3 {} {} --time {}'

def parse_tmp_solution(solution):
    total_duration = 0
    plan = []
    regex = r'(\d+.\d+): \(\s*(\w+(?:\s\w+)*)\s*\) \[(\d+.\d+)\]'
    for start_time, action, duration in re.findall(regex, solution):
        total_duration = max(float(start_time) + float(duration), total_duration)
        entries = action.lower().split(' ')
        plan.append((entries[0], tuple(entries[1:])))
    return plan, total_duration


def run_tpshe(max_time, verbose):
    tpshe_root = os.environ[ENV_VAR]
    command = COMMAND.format(tpshe_root, TEMP_DIR + DOMAIN_PATH,
                             TEMP_DIR + PROBLEM_PATH, int(math.ceil(max_time)))
    t0 = time.time()
    p = os.popen(command) # NOTE - cannot pipe input easily with subprocess
    if verbose: print(command)
    output = p.read()
    if verbose:
        print()
        print(output)
        print('Runtime:', elapsed_time(t0))
    # if not os.path.exists(OUTPUT_PATH):
    #   return None
    # return read(OUTPUT_PATH)

    plan_files = sorted(f for f in os.listdir('.') if f.startswith(TMP_OUTPUT_PATH))
    if not plan_files:
        return None
    best_plan, best_makespan = None, INF
    for plan_file in plan_files:
        print(plan_file)
        plan, duration = parse_tmp_solution(read(plan_file))
        print(plan)
        print(plan_file, len(plan), duration)
        if duration < best_makespan:
            best_plan, best_makespan = plan, duration
    return best_plan

def solve_temporal(evaluations, goal_exp, domain):
    # path = '/Users/caelan/Programs/LIS/planners/cerberus'
    # './plan.py {domain} {problem} {plan}'
    # ./build.py debug64
    [domain_name] = re.findall(r'\(domain ([^ ]+)\)', domain)
    problem_name = domain_name
    problem = pddl_problem(domain_name, problem_name, evaluations, goal_exp)
    write_pddl(domain, problem, TEMP_DIR)
    solution = run_tpshe(max_time=10, verbose=True)
    cost = INF if solution is None else 0
    return solution, cost
