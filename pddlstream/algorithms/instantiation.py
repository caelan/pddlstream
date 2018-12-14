from collections import deque, defaultdict, Sized
from heapq import heappush, heappop
from itertools import product

from pddlstream.language.conversion import is_atom, head_from_fact
from pddlstream.language.constants import get_prefix, get_args
from pddlstream.language.object import Object
from pddlstream.utils import safe_zip, HeapElement


def get_mapping(atoms1, atoms2):
    mapping = {}
    for a1, a2 in safe_zip(atoms1, atoms2):
        assert get_prefix(a1) == get_prefix(a2)
        for arg1, arg2 in safe_zip(a1.args, a2.args):
            if mapping.get(arg1, arg2) == arg2:
                mapping[arg1] = arg2
            else:
                return None
    return mapping

##################################################

class Instantiator(Sized): # Dynamic Stream Instantiator
    def __init__(self, evaluations, streams):
        # TODO: priority queue based on effort. Useful for both incremental and focused
        # One difference is that focused considers path while incremental is just immediate
        self.streams = streams
        #self.streams_from_atom = defaultdict(list)
        self.stream_instances = {}
        self.stream_queue = deque()
        #self.stream_queue = []
        # TODO: rename atom to head in most places
        self.atoms = {}
        self.atoms_from_domain = defaultdict(list)
        for stream in self.streams:
            if not stream.inputs: # TODO: need to do with with domain...
                self._add_instance(stream, tuple())
        for atom in evaluations:
            self.add_atom(atom)

    def __len__(self):
        return len(self.stream_queue)

    def push(self, stream_instance):
        # TODO: update self.stream_instances
        self.stream_queue.append(stream_instance)

    def head(self):
        return self.stream_queue[0]

    def pop(self):
        return self.stream_queue.popleft()

    def poppush(self): # replace
        self.stream_queue.rotate(-1)

    def _add_instance(self, stream, input_objects, effort=0):
        stream_instance = stream.get_instance(input_objects)
        if stream_instance in self.stream_instances:
            return False
        self.stream_instances[stream_instance] = effort
        self.push(stream_instance)
        return True

    def _add_combinations(self, stream, atoms):
        domain = list(map(head_from_fact, stream.domain))
        for combo in product(*atoms):
            mapping = get_mapping(domain, combo)
            if mapping is None:
                continue
            input_objects = tuple(mapping[p] for p in stream.inputs)
            self._add_instance(stream, input_objects)

    def _add_new_instances(self, new_atom):
        for i, stream in enumerate(self.streams):
            for j, domain_fact in enumerate(stream.domain):
                if new_atom.function != get_prefix(domain_fact):
                    continue
                if any(isinstance(b, Object) and (a != b) for a, b in
                       safe_zip(new_atom.args, get_args(domain_fact))):
                    continue
                self.atoms_from_domain[(i, j)].append(new_atom)
                atoms = [self.atoms_from_domain[(i, k)] if j != k else [new_atom]
                          for k in range(len(stream.domain))]
                self._add_combinations(stream, atoms)

    def add_atom(self, atom, effort=0):
        if not is_atom(atom):
            return False
        head = atom.head
        if head in self.atoms:
            return False
        self.atoms[head] = effort
        # TODO: doing this in a way that will eventually allow constants
        self._add_new_instances(head)
        return True

    #def __next__(self):
    #    pass
    #
    #def __iter__(self):
    #    while self.stream_queue:
    #        stream_instance = self.stream_queue.popleft()
    #        yield stream_instance # Remove from set?
