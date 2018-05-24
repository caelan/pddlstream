from __future__ import print_function

import os
import shutil
import sys
import time
import math
import pickle

INF = float('inf')

try:
   user_input = raw_input
except NameError:
   user_input = input

def int_ceil(f):
    return int(math.ceil(f))

def read(filename):
    with open(filename, 'r') as f:
        return f.read()


def write(filename, string):
    with open(filename, 'w') as f:
        f.write(string)


def write_pickle(filename, data):
    # TODO: cannot pickle lambda or nested functions
    with open(filename, 'wb') as f:
        pickle.dump(data, f)


def read_pickle(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)


def safe_remove(p):
    if os.path.exists(p):
        os.remove(p)


def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)


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

def invert_test(test):
    return lambda *args: not test(*args)

def print_solution(solution):
    plan, cost, evaluations = solution
    solved = plan is not None
    print()
    print('Solved: {}'.format(solved))
    print('Cost: {}'.format(cost))
    print('Length: {}'.format(get_length(plan)))
    print('Evaluations: {}'.format(len(evaluations)))
    if not solved:
        return
    for i, action in enumerate(plan):
        print('{}) {}'.format(i+1, ' '.join(map(str, action))))


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

def find_unique(test, sequence):
    found, value = False, None
    for item in sequence:
        if test(item):
            if found:
                raise RuntimeError('Both elements {} and {} satisfy the test'.format(value, item))
            found, value = True, item
    if not found:
        raise RuntimeError('Unable to find an element satisfying the test')
    return value

def str_from_tuple(tup):
    return '({})'.format(', '.join(map(str, tup)))


def open_pdf(filename):
    import subprocess
    # import os
    # import webbrowser
    subprocess.Popen('open {}'.format(filename), shell=True)
    # os.system(filename)
    # webbrowser.open(filename)
    user_input('Display?')
    # safe_remove(filename)

class MockSet(object):
    def __init__(self, test=lambda item: True):
        self.test = test
    def __contains__(self, item):
        return self.test(item)

def implies(a, b):
    return not a or b

def irange(start, end=None, step=1):
    if end is None:
        end = start
        start = 0
    n = start
    while n < end:
        yield n
        n += step

def argmin(function, sequence):
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))]

def argmax(function, sequence):
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(max(scores))]


def get_file_path(file, rel_path):
    directory = os.path.dirname(os.path.abspath(file))
    return os.path.join(directory, rel_path)