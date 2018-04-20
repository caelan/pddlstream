from __future__ import print_function

import os
import shutil
import time

INF = float('inf')

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


def elapsed_time(start_time):
    return time.time() - start_time

def get_length(sequence):
    if sequence is None:
        return INF
    return len(sequence)

def print_solution(solution):
    plan, cost, evaluations = solution
    solved = plan is not None
    print()
    print('Solved: {}'.format(solved))
    print('Cost: {}'.format(cost))
    print('Length: {}'.format(get_length(plan)))
    print('Evaluations: {}'.format(len(evaluations)))
    if solved:
        for i, (action, args) in enumerate(plan):
            print('{}) {} {}'.format(i+1, action, ' '.join(map(str, args))))
