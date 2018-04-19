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