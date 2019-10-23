from __future__ import print_function

import math
import os
import pickle
import shutil
import sys
import time
import random
import cProfile
import pstats

from collections import defaultdict, deque
from heapq import heappush, heappop

import numpy as np

INF = float('inf')

try:
   user_input = raw_input
except NameError:
   user_input = input

##################################################

def int_ceil(f):
    return int(math.ceil(f))


def get_python_version():
    return sys.version_info[0]


def read(filename):
    with open(filename, 'r') as f:
        return f.read()


def write(filename, string):
    with open(filename, 'w') as f:
        f.write(string)


def write_pickle(filename, data):
    # Cannot pickle lambda or nested functions
    with open(filename, 'wb') as f:
        pickle.dump(data, f)


def read_pickle(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)


def safe_remove(p):
    if os.path.exists(p):
        os.remove(p)


def mkdir(d):
    if not os.path.exists(d):
        os.makedirs(d)


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


def get_file_path(file, rel_path):
    directory = os.path.dirname(os.path.abspath(file))
    return os.path.join(directory, rel_path)


def open_pdf(filename):
    import subprocess
    # import os
    # import webbrowser
    subprocess.Popen('open {}'.format(filename), shell=True)
    # os.system(filename)
    # webbrowser.open(filename)
    user_input('Display?')
    # safe_remove(filename)
    # TODO: close output

##################################################

def elapsed_time(start_time):
    return time.time() - start_time


def safe_zip(sequence1, sequence2):
    assert len(sequence1) == len(sequence2)
    return zip(sequence1, sequence2)


def get_mapping(sequence1, sequence2):
    return dict(safe_zip(sequence1, sequence2))


def apply_mapping(sequence, mapping):
    return tuple(mapping.get(e, e) for e in sequence)


#def safe_apply_mapping(sequence, mapping)
#    return tuple(mapping[e] for e in sequence)


def negate_test(test):
    return lambda *args, **kwargs: not test(*args, **kwargs)


def flatten(iterable_of_iterables):
    return (item for iterables in iterable_of_iterables for item in iterables)


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


def argmin(fn, iterable):
    return min(iterable, key=fn)


def argmax(fn, iterable):
    return max(iterable, key=fn)


def invert_dict(d):
    return {v: k for k, v in d.items()}


def randomize(iterable):
    sequence = list(iterable)
    random.shuffle(sequence)
    return sequence

##################################################

BYTES_PER_KILOBYTE = math.pow(2, 10)
BYTES_PER_GIGABYTE = math.pow(2, 30)
KILOBYTES_PER_GIGABYTE = BYTES_PER_GIGABYTE / BYTES_PER_KILOBYTE

def get_peak_memory_in_kb():
    # TODO: use psutil instead
    import psutil
    # https://pypi.org/project/psutil/
    # https://psutil.readthedocs.io/en/latest/
    #rss: aka "Resident Set Size", this is the non-swapped physical memory a process has used. (bytes)
    #vms: aka "Virtual Memory Size", this is the total amount of virtual memory used by the process. (bytes)
    #shared: (Linux) memory that could be potentially shared with other processes.
    #text (Linux, BSD): aka TRS (text resident set) the amount of memory devoted to executable code.
    #data (Linux, BSD): aka DRS (data resident set) the amount of physical memory devoted to other than executable code.
    #lib (Linux): the memory used by shared libraries.
    #dirty (Linux): the number of dirty pages.
    #pfaults (macOS): number of page faults.
    #pageins (macOS): number of actual pageins.
    process = psutil.Process(os.getpid())
    #process.pid()
    #process.ppid()
    pmem = process.memory_info() # this seems to actually get the current memory!
    memory_in_kb = pmem.vms / BYTES_PER_KILOBYTE
    return memory_in_kb
    #print(process.memory_full_info())
    #print(process.memory_percent())
    # process.rlimit(psutil.RLIMIT_NOFILE)  # set resource limits (Linux only)
    #print(psutil.virtual_memory())
    #print(psutil.swap_memory())
    #print(psutil.pids())
    #try:
    #    # This will only work on Linux systems.
    #    with open("/proc/self/status") as status_file:
    #        for line in status_file:
    #            parts = line.split()
    #            if parts[0] == "VmPeak:":
    #                return float(parts[1])
    #except IOError:
    #    pass
    #return 0.

def check_memory(max_memory):
    if max_memory == INF:
        return True
    peak_memory = get_peak_memory_in_kb()
    #print('Peak memory: {} | Max memory: {}'.format(peak_memory, max_memory))
    if peak_memory <= max_memory:
        return True
    print('Peak memory of {} KB exceeds memory limit of {} KB'.format(
        int(peak_memory), int(max_memory)))
    return False

##################################################

class Saver(object):
    # TODO: contextlib
    def save(self):
        raise NotImplementedError()
    def restore(self):
        raise NotImplementedError()
    def __enter__(self):
        # TODO: move the saving to enter?
        self.save()
        return self
    def __exit__(self, type, value, traceback):
        self.restore()


class Profiler(Saver):
    fields = ['tottime', 'cumtime']
    def __init__(self, field='tottime', num=10):
        assert field in self.fields
        self.field = field
        self.num = num
        self.pr = cProfile.Profile()
    def save(self):
        self.pr.enable()
        return self.pr
    def restore(self):
        self.pr.disable()
        stats = pstats.Stats(self.pr).sort_stats(self.field)
        stats.print_stats(self.num)
        return stats


class Verbose(Saver): # TODO: use DisableOutput
    def __init__(self, verbose=False):
        self.verbose = verbose
    def save(self):
        if self.verbose:
            return
        self.stdout = sys.stdout
        self.devnull = open(os.devnull, 'w')
        sys.stdout = self.devnull
        #self.stderr = sys.stderr
        #self.devnull = open(os.devnull, 'w')
        #sys.stderr = self.stderr
    def restore(self):
        if self.verbose:
            return
        sys.stdout = self.stdout
        self.devnull.close()
        #sys.stderr = self.stderr
        #self.devnull.close()


class TmpCWD(Saver):
    def __init__(self, temp_cwd):
        self.tmp_cwd = temp_cwd
    def save(self):
        self.old_cwd = os.getcwd()
        os.chdir(self.tmp_cwd)
    def restore(self):
        os.chdir(self.old_cwd)

##################################################

class MockSet(object):
    def __init__(self, test=lambda item: True):
        self.test = test
    def __contains__(self, item):
        return self.test(item)


class HeapElement(object):
    def __init__(self, key, value):
        self.key = key
        self.value = value
    def __lt__(self, other):
        return self.key < other.key
    def __iter__(self):
        return iter([self.key, self.value])
    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__, self.key, self.value)

##################################################

def sorted_str_from_list(obj):
    return '[{}]'.format(', '.join(sorted(str_from_object(item) for item in obj)))

def str_from_object(obj):  # str_object
    if type(obj) in [list]: #, np.ndarray):
        return '[{}]'.format(', '.join(str_from_object(item) for item in obj))
    if type(obj) == tuple:
        return '({})'.format(', '.join(str_from_object(item) for item in obj))
    #if isinstance(obj, dict):
    if type(obj) in [dict, defaultdict]:
        return '{{{}}}'.format(', '.join('{}: {}'.format(str_from_object(key), str_from_object(obj[key])) \
                                  for key in sorted(obj.keys(), key=lambda k: str_from_object(k))))
    if type(obj) in [set, frozenset]:
        return '{{{}}}'.format(', '.join(sorted(str_from_object(item) for item in obj)))
    #if type(obj) in (float, np.float64):
    #    obj = round(obj, 3)
    #    if obj == 0: obj = 0  # NOTE - catches -0.0 bug
    #    return '%.3f' % obj
    #if isinstance(obj, types.FunctionType):
    #    return obj.__name__
    return str(obj)
    #return repr(obj)

##################################################

def incoming_from_edges(edges):
    incoming_vertices = defaultdict(set)
    for v1, v2 in edges:
        incoming_vertices[v2].add(v1)
    return incoming_vertices

def outgoing_from_edges(edges):
    outgoing_vertices = defaultdict(set)
    for v1, v2 in edges:
        outgoing_vertices[v1].add(v2)
    return outgoing_vertices

def neighbors_from_orders(orders):
    return incoming_from_edges(orders), \
           outgoing_from_edges(orders)

def adjacent_from_edges(edges):
    undirected_edges = defaultdict(set)
    for v1, v2 in edges:
        undirected_edges[v1].add(v2)
        undirected_edges[v2].add(v1)
    return undirected_edges

def topological_sort(vertices, orders, priority_fn=lambda v: 0):
    # Can also do a DFS version
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)
    ordering = []
    queue = []
    for v in vertices:
        if not incoming_edges[v]:
            heappush(queue, HeapElement(priority_fn(v), v))
    while queue:
        v1 = heappop(queue).value
        ordering.append(v1)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1)
            if not incoming_edges[v2]:
                heappush(queue, HeapElement(priority_fn(v2), v2))
    return ordering

def grow_component(sources, edges, disabled=set()):
    processed = set(disabled)
    cluster = []
    queue = deque()
    def add_cluster(v):
        if v not in processed:
            processed.add(v)
            cluster.append(v)
            queue.append(v)

    for v0 in sources:
        add_cluster(v0)
    while queue:
        # TODO: add clusters here to ensure proper BFS
        v1 = queue.popleft()
        for v2 in edges[v1]:
            add_cluster(v2)
    return cluster

def breadth_first_search(source, edges, **kwargs):
    return grow_component([source], edges, **kwargs)

def get_connected_components(vertices, edges):
    undirected_edges = adjacent_from_edges(edges)
    clusters = []
    processed = set()
    for v0 in vertices:
        cluster = grow_component({v0}, undirected_edges, processed)
        processed.update(cluster)
        if cluster:
            clusters.append([v for v in vertices if v in cluster])
    return clusters

##################################################

def is_hashable(value):
    #return isinstance(value, Hashable) # TODO: issue with hashable and numpy 2.7.6
    try:
        hash(value)
    except TypeError:
        return False
    return True


def hash_or_id(value):
    #return id(value)
    try:
        hash(value)
        return value
    except TypeError:
        return id(value)


def is_64bits():
    #return sys.maxsize > 2**32
    import platform
    bit, _ = platform.architecture()
    return bit == '64bit'


def inclusive_range(start, stop, step=1):
    sequence = list(np.arange(start, stop, step))
    if sequence and (sequence[-1] == stop):
        sequence.append(stop)
    return sequence
