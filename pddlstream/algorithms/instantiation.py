from collections import defaultdict, namedtuple, Sized
from heapq import heappush, heappop
from itertools import product

from pddlstream.language.conversion import is_atom, head_from_fact
from pddlstream.language.function import Function, FunctionInstance
from pddlstream.language.external import compute_external_effort, compute_instance_effort
from pddlstream.language.object import Object
from pddlstream.utils import safe_zip, HeapElement

# TODO: maybe store effort as well unit effort here
Priority = namedtuple('Priority', ['effort', 'num']) # num ensures FIFO

def get_mapping(atoms1, atoms2):
    mapping = {}
    for a1, a2 in safe_zip(atoms1, atoms2):
        assert a1.function == a2.function
        for arg1, arg2 in safe_zip(a1.args, a2.args):
            if mapping.get(arg1, arg2) == arg2:
                mapping[arg1] = arg2
            else:
                return None
    return mapping

##################################################

class Instantiator(Sized): # Dynamic Instantiator
    def __init__(self, evaluations, streams,
                 unit_efforts=True, combine_fn=max, max_effort=None):
        # TODO: switch to deque when unit efforts?
        # Focused considers path effort while incremental is just immediate effort
        self.unit_efforts = unit_efforts
        self.combine_fn = combine_fn # max | sum
        self.max_effort = max_effort # exclusive
        self.streams = streams
        #self.streams_from_atom = defaultdict(list)
        self.effort_from_instance = {}
        self.stream_queue = []
        self.function_queue = []
        self.num_pushes = 0 # shared between the queues
        # TODO: rename atom to head in most places
        self.effort_from_atom = {}
        self.atoms_from_domain = defaultdict(list)
        for stream in self.streams:
            if not stream.domain:
                self._add_instance(stream, tuple(), 0)
        for atom in evaluations:
            self.add_atom(atom, 0)

    #########################

    def __len__(self):
        return len(self.stream_queue) + len(self.function_queue)

    def prune_effort(self, external, effort):
        if self.max_effort is None:
            return False # Allows infinite effort
        return (not isinstance(external, Function)) and (self.max_effort <= effort)

    def push(self, instance, effort):
        # TODO: update self.effort_from_instance with new effort?
        # TODO: flush stale priorities?
        queue = self.function_queue if isinstance(instance, FunctionInstance) else self.stream_queue
        priority = Priority(effort, self.num_pushes)
        heappush(queue, HeapElement(priority, instance))
        self.num_pushes += 1

    def pop_stream(self):
        priority, instance = heappop(self.stream_queue)
        return instance, priority.effort

    def pop_function(self):
        priority, instance = heappop(self.function_queue)
        return instance, priority.effort

    def pop(self):
        if self.function_queue:
            return self.pop_function()
        return self.pop_stream()

    def min_effort(self):
        priority, _ = self.stream_queue[0]
        return priority.effort

    #########################

    def _add_instance(self, stream, input_objects, domain_effort):
        instance = stream.get_instance(input_objects)
        assert instance not in self.effort_from_instance
        effort = domain_effort + compute_instance_effort(
            instance, unit_efforts=self.unit_efforts)
        #if instance in self.effort_from_instance:
        #    assert self.effort_from_instance[instance] <= effort
        #    return False
        if self.prune_effort(instance.external, effort):
            return False
        self.effort_from_instance[instance] = effort
        self.push(instance, effort)
        return True

    def _add_combinations(self, stream, atoms):
        domain = list(map(head_from_fact, stream.domain))
        for combo in product(*atoms):
            mapping = get_mapping(domain, combo)
            if mapping is None:
                continue
            domain_effort = self.combine_fn([0]+[self.effort_from_atom[a] for a in combo])
            input_objects = tuple(mapping[p] for p in stream.inputs)
            self._add_instance(stream, input_objects, domain_effort)

    def _add_new_instances(self, new_atom):
        for s_idx, stream in enumerate(self.streams):
            effort_bound = self.effort_from_atom[new_atom] + compute_external_effort(
                stream, unit_efforts=self.unit_efforts)
            if self.prune_effort(stream, effort_bound):
                continue
            for d_idx, domain_fact in enumerate(stream.domain):
                domain_atom = head_from_fact(domain_fact)
                if new_atom.function != domain_atom.function:
                    continue
                if any(isinstance(b, Object) and (a != b) for a, b in
                       safe_zip(new_atom.args, domain_atom.args)):
                    continue # TODO: handle domain constants nicely
                self.atoms_from_domain[s_idx, d_idx].append(new_atom)
                atoms = [self.atoms_from_domain[s_idx, d2_idx] if d_idx != d2_idx else [new_atom]
                          for d2_idx in range(len(stream.domain))]
                self._add_combinations(stream, atoms)

    def add_atom(self, atom, effort):
        if not is_atom(atom):
            return False
        head = atom.head
        if head in self.effort_from_atom:
            assert self.effort_from_atom[head] <= effort
            return False
        self.effort_from_atom[head] = effort
        self._add_new_instances(head)
        return True
