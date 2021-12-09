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
import io

from collections import defaultdict, deque, Counter, namedtuple
from itertools import count
from heapq import heappush, heappop

import numpy as np

INF = float('inf')
SEPARATOR = '\n' + 80*'-'  + '\n'

try:
   user_input = raw_input
except NameError:
   user_input = input

inf_generator = count

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
    # Can sometimes read pickle3 from python2 by calling twice
    with open(filename, 'rb') as f:
        try:
            return pickle.load(f)
        except UnicodeDecodeError as e:
            return pickle.load(f, encoding='latin1')


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


def safe_apply_mapping(sequence, mapping):
    # TODO: flip arguments order
    return tuple(mapping[e] for e in sequence)


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
    # TODO: combine with my other infinite generator
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
        if self.num is None:
            return None
        stream = None
        #stream = io.StringIO()
        stats = pstats.Stats(self.pr, stream=stream).sort_stats(self.field) # TODO: print multiple
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

class Comparable(object):
    def __lt__(self, other):
        raise NotImplementedError()
    def __eq__(self, other):
        return not (self < other) and not (other < self)
    def __ne__(self, other):
        return (self < other) or (other < self)
    def __gt__(self, other):
        return other < self
    def __ge__(self, other):
        return not self < other
    def __le__(self, other):
        return not other < self

class MockSet(object):
    def __init__(self, test=lambda item: True):
        self.test = test
    def __contains__(self, item):
        return self.test(item)

class Score(Comparable): # tuple
    def __init__(self, *args):
        # TODO: convert to float
        #super(Score, self).__init__(args)
        self.values = tuple(args)
    def check_other(self, other):
        return isinstance(other, Score) and (len(self.values) == len(other.values))
    def __lt__(self, other):
        assert self.check_other(other)
        return self.values < other.values
    def __iter__(self):
        return iter(self.values)
    def __neg__(self):
        return self.__class__(*(type(value).__neg__(value) for value in self.values))
    def __add__(self, other):
        return self.__class__(*(self.values + other.values))
    def __repr__(self):
        return '{}{}'.format(self.__class__.__name__, self.values)

class HeapElement(Comparable):
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

def sorted_str_from_list(obj, **kwargs):
    return '[{}]'.format(', '.join(sorted(str_from_object(item, **kwargs) for item in obj)))

def str_from_object(obj, ndigits=None):  # str_object
    if type(obj) in [list]: #, np.ndarray):
        return '[{}]'.format(', '.join(str_from_object(item, ndigits) for item in obj))
    if type(obj) == tuple:
        return '({})'.format(', '.join(str_from_object(item, ndigits) for item in obj))
    #if isinstance(obj, dict):
    if type(obj) in [dict, defaultdict, Counter]:
        return '{{{}}}'.format(', '.join('{}: {}'.format(str_from_object(key, ndigits), str_from_object(obj[key], ndigits)) \
                                  for key in sorted(obj.keys(), key=lambda k: str_from_object(k, ndigits))))
    if type(obj) in [set, frozenset]:
        return '{{{}}}'.format(', '.join(sorted(str_from_object(item, ndigits) for item in obj)))
    if (ndigits is not None) and (type(obj) in [float, np.float64]):
        obj = round(obj, ndigits=ndigits)
        if obj == 0.:
            obj = 0.  # NOTE - catches -0.0 bug
        return '{0:.{1}f}'.format(obj, ndigits)
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

##################################################

def filter_orders(vertices, orders):
    # TODO: rename to filter edges?
    return [order for order in orders if all(v in vertices for v in order)]

def is_valid_topological_sort(vertices, orders, solution):
    orders = filter_orders(vertices, orders)
    if Counter(vertices) != Counter(solution):
        return False
    index_from_vertex = {v: i for i, v in enumerate(solution)}
    for v1, v2 in orders:
        if index_from_vertex[v1] >= index_from_vertex[v2]:
            return False
    return True

def dfs_topological_sort(vertices, orders, priority_fn=lambda v: 0):
    # TODO: DFS for all topological sorts
    orders = filter_orders(vertices, orders)
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)

    def dfs(history, visited):
        reverse_ordering = []
        v1 = history[-1]
        if v1 in visited:
            return reverse_ordering
        visited.add(v1)
        for v2 in sorted(outgoing_edges[v1], key=priority_fn, reverse=True):
            if v2 in history:
                return None # Contains a cycle
            result = dfs(history + [v2], visited)
            if result is None:
                return None
            reverse_ordering.extend(result)
        reverse_ordering.append(v1)
        return reverse_ordering

    visited = set()
    reverse_order = []
    for v0 in sorted(vertices, key=priority_fn, reverse=True):
        if not incoming_edges[v0]:
            result = dfs([v0], visited)
            if result is None:
                return None
            reverse_order.extend(result)

    ordering = reverse_order[::-1]
    assert(is_valid_topological_sort(vertices, orders, ordering))
    return ordering

def topological_sort(vertices, orders, priority_fn=lambda v: 0):
    orders = filter_orders(vertices, orders)
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)
    ordering = []
    queue = []
    for v in vertices:
        if not incoming_edges[v]:
            heappush(queue, HeapElement(priority_fn(v), v))
    while queue:
        priority, v1 = heappop(queue) # Lowest to highest
        ordering.append(v1)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1)
            if not incoming_edges[v2]:
                heappush(queue, HeapElement(priority_fn(v2), v2))
    if len(ordering) != len(vertices):
        return None
    assert is_valid_topological_sort(vertices, orders, ordering)
    return ordering

def layer_sort(vertices, orders): # priority_fn=lambda v: 0
    # TODO: more efficient hypergraph/layer distance (h_max)
    orders = filter_orders(vertices, orders)
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)
    visited = {}
    queue = []
    for v in vertices:
        if not incoming_edges[v]:
            visited[v] = 0
            heappush(queue, HeapElement(visited[v], v))
    while queue:
        g, v1 = heappop(queue)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1) # TODO: non-uniform cost function for max
            if not incoming_edges[v2] and (v2 not in visited):
                visited[v2] = g + 1
                heappush(queue, HeapElement(visited[v2], v2))
    return visited

def is_acyclic(vertices, orders):
    return topological_sort(vertices, orders) is not None

def sample_topological_sort(vertices, orders):
    # https://stackoverflow.com/questions/38551057/random-topological-sorting-with-uniform-distribution-in-near-linear-time
    # https://www.geeksforgeeks.org/all-topological-sorts-of-a-directed-acyclic-graph/
    priorities = {v: random.random() for v in vertices}
    return topological_sort(vertices, orders, priority_fn=priorities.get)

def transitive_closure(vertices, orders):
    # Warshall's algorithm
    orders = filter_orders(vertices, orders)
    closure = set(orders)
    for k in vertices:
        for i in vertices:
            for j in vertices:
                if ((i, j) not in closure) and ((i, k) in closure) and ((k, j) in closure):
                    closure.add((i, j))
    return closure

##################################################

def grow_component(sources, edges, disabled=set()):
    processed = set(disabled)
    cluster = []
    queue = deque()

    def add_cluster(v):
        if v in processed:
            return
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

def get_ancestors(source, edges):
    return set(breadth_first_search(source, incoming_from_edges(edges))) - {source}

def get_descendants(source, edges):
    return set(breadth_first_search(source, outgoing_from_edges(edges))) - {source}

def get_connected_components(vertices, edges):
    edges = filter_orders(vertices, edges)
    undirected_edges = adjacent_from_edges(edges)
    clusters = []
    processed = set()
    for v0 in vertices:
        if v0 in processed:
            continue
        cluster = grow_component({v0}, undirected_edges, processed)
        processed.update(cluster)
        if cluster:
            clusters.append([v for v in vertices if v in cluster])
    return clusters

##################################################

SearchNode = namedtuple('Node', ['g', 'parent'])

def dijkstra(sources, edges, op=sum): # sum | max
    if not isinstance(edges, dict):
        edges = {edge: 1 for edge in edges}
    _, outgoing_edges = neighbors_from_orders(edges)
    visited = {}
    queue = []
    for v0 in sources:
        visited[v0] = SearchNode(g=0, parent=None)
        queue.append(HeapElement(visited[v0].g, v0))

    while queue:
        current_g, current_v = heappop(queue)
        if visited[current_v].g < current_g:
            continue
        for next_v in outgoing_edges[current_v]:
            next_g = op([current_g, edges[(current_v, next_v)]])
            if (next_v not in visited) or (next_g < visited[next_v].g):
                visited[next_v] = SearchNode(next_g, current_v)
                heappush(queue, HeapElement(next_g, next_v))
    return visited

##################################################

def is_hashable(value):
    #return isinstance(value, Hashable) # TODO: issue with hashable and numpy 2.7.6
    try:
        hash(value)
    except TypeError:
        return False
    return True


# def hash_or_id(value):
#     if is_hashable(value):
#         return hash(value)
#     return id(value)


def value_or_id(value):
    if is_hashable(value):
        return value
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


def read_pddl(this_file, pddl_filename):
    directory = os.path.dirname(os.path.abspath(this_file))
    return read(os.path.join(directory, pddl_filename))


def lowercase(*strings):
    return [string.lower() for string in strings]


def str_eq(s1, s2, ignore_case=True):
    if ignore_case:
        s1 = s1.lower()
        s2 = s2.lower()
    return s1 == s2


def clip(value, lower=-INF, upper=+INF):
    return min(max(lower, value), upper)