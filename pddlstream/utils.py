from __future__ import print_function

import os
import shutil
import sys
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

def clear_dir(d):
    safe_rm_dir(d)
    ensure_dir(d)

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


class Verbose(object):
    def __init__(self, verbose):
        self.verbose = verbose
    def __enter__(self):
        if not self.verbose:
            self.stdout = sys.stdout
            self.devnull = open(os.devnull, 'w')
            sys.stdout = self.devnull
        return self
    def __exit__(self, type, value, traceback):
        if not self.verbose:
            sys.stdout = self.stdout
            self.devnull.close()


class TmpCWD(object):
    def __init__(self, temp_cwd):
        self.tmp_cwd = temp_cwd
    def __enter__(self):
        self.old_cwd = os.getcwd()
        os.chdir(self.tmp_cwd)
        return self
    def __exit__(self, type, value, traceback):
        os.chdir(self.old_cwd)


def find(test, sequence):
    for item in sequence:
        if test(item):
            return item
    return None