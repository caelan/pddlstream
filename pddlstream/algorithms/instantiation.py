from collections import deque, defaultdict
from itertools import product

from pddlstream.language.conversion import is_atom, head_from_fact
from pddlstream.language.constants import get_prefix, get_args
from pddlstream.language.object import Object


def get_mapping(atoms1, atoms2, initial={}):
    assert len(atoms1) == len(atoms2)
    mapping = initial.copy()
    for a1, a2 in zip(atoms1, atoms2):
        assert(get_prefix(a1) == get_prefix(a2))
        #for arg1, arg2 in zip(get_args(a1), a2.args): # TODO: this is because eval vs predicate
        for arg1, arg2 in zip(a1.args, a2.args):
            if mapping.get(arg1, arg2) == arg2:
                mapping[arg1] = arg2
            else:
                return None
    return mapping


class Instantiator(object): # Dynamic Stream Instantiator
    def __init__(self, evaluations, streams):
        # TODO: filter eager
        #self.streams_from_atom = defaultdict(list)
        self.streams = streams
        self.stream_instances = set()
        self.stream_queue = deque()
        self.atoms = set()
        self.atoms_from_domain = defaultdict(list)
        for stream in self.streams:
            if not stream.inputs:
                self._add_instance(stream, tuple())
        for atom in evaluations:
            self.add_atom(atom)

    #def __next__(self):
    #    pass
    #
    #def __iter__(self):
    #    while self.stream_queue:
    #        stream_instance = self.stream_queue.popleft()
    #        # TODO: remove from set?
    #        yield stream_instance

    def _add_instance(self, stream, input_objects):
        stream_instance = stream.get_instance(input_objects)
        if stream_instance in self.stream_instances:
            return False
        self.stream_instances.add(stream_instance)
        self.stream_queue.append(stream_instance)
        return True

    def add_atom(self, atom):
        if not is_atom(atom):
            return False
        head = atom.head
        if head in self.atoms:
            return False
        self.atoms.add(head)
        # TODO: doing this in a way that will eventually allow constants

        for i, stream in enumerate(self.streams):
            for j, domain_atom in enumerate(stream.domain):
                if get_prefix(head) != get_prefix(domain_atom):
                    continue
                if len(head.args) != len(get_args(domain_atom)):
                    raise ValueError(head, domain_atom)
                if any(isinstance(b, Object) and (a != b) for (a, b) in
                       zip(head.args, get_args(domain_atom))):
                    continue
                self.atoms_from_domain[(i, j)].append(head)
                values = [self.atoms_from_domain[(i, k)] if j != k else [head]
                          for k in range(len(stream.domain))]
                domain = list(map(head_from_fact, stream.domain))
                #domain = stream.domain
                for combo in product(*values):
                    mapping = get_mapping(domain, combo)
                    if mapping is None:
                        continue
                    input_objects = tuple(mapping[p] for p in stream.inputs)
                    self._add_instance(stream, input_objects)
        return True

