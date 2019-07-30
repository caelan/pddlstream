from collections import defaultdict, namedtuple, Sized
from heapq import heappush, heappop
from itertools import product

from pddlstream.algorithms.common import COMPLEXITY_OP
from pddlstream.algorithms.relation import compute_order, Relation, solve_satisfaction
from pddlstream.language.constants import is_parameter
from pddlstream.language.conversion import is_atom, head_from_fact
from pddlstream.utils import safe_zip, HeapElement

USE_RELATION = True

# TODO: maybe store unit complexity here as well as a tiebreaker
Priority = namedtuple('Priority', ['complexity', 'num']) # num ensures FIFO

def is_instance(atom, schema):
    return (atom.function == schema.function) and \
            all(is_parameter(b) or (a == b)
                for a, b in safe_zip(atom.args, schema.args))

def test_mapping(atoms1, atoms2):
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

# http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.43.7049&rep=rep1&type=pdf

class Instantiator(Sized): # Dynamic Instantiator
    def __init__(self, streams, evaluations={}):
        # TODO: lazily instantiate upon demand
        self.streams = streams
        #self.streams_from_atom = defaultdict(list)
        self.queue = []
        self.num_pushes = 0 # shared between the queues
        # TODO: rename atom to head in most places
        self.complexity_from_atom = {}
        self.atoms_from_domain = defaultdict(list)
        for stream in self.streams:
            if not stream.domain:
                assert not stream.inputs
                self.push_instance(stream.get_instance([]))
        for atom, node in evaluations.items():
            self.add_atom(atom, node.complexity)
        # TODO: revisit deque and add functions to front

    #########################

    def __len__(self):
        return len(self.queue)

    def compute_complexity(self, instance):
        domain_complexity = COMPLEXITY_OP([self.complexity_from_atom[head_from_fact(f)]
                                           for f in instance.get_domain()] + [0])
        return domain_complexity + instance.external.get_complexity(instance.num_calls)

    def push_instance(self, instance):
        # TODO: flush stale priorities?
        complexity = self.compute_complexity(instance)
        priority = Priority(complexity, self.num_pushes)
        heappush(self.queue, HeapElement(priority, instance))
        self.num_pushes += 1

    def pop_stream(self):
        priority, instance = heappop(self.queue)
        return instance

    def min_complexity(self):
        priority, _ = self.queue[0]
        return priority.complexity

    #########################

    def _add_combinations(self, stream, atoms):
        if not all(atoms):
            return
        domain = list(map(head_from_fact, stream.domain))
        # Most constrained variable/atom to least constrained
        for combo in product(*atoms):
            mapping = test_mapping(domain, combo)
            if mapping is not None:
                input_objects = tuple(mapping[p] for p in stream.inputs)
                self.push_instance(stream.get_instance(input_objects))

    def _add_combinations_relation(self, stream, atoms):
        if not all(atoms):
            return
        # TODO: might be a bug here?
        domain = list(map(head_from_fact, stream.domain))
        # TODO: compute this first?
        relations = [Relation(filter(is_parameter, domain[index].args),
                              [tuple(a for a, b in safe_zip(atom.args, domain[index].args)
                                     if is_parameter(b)) for atom in atoms[index]])
                     for index in compute_order(domain, atoms)]
        solution = solve_satisfaction(relations)
        for element in solution.body:
            mapping = solution.get_mapping(element)
            input_objects = tuple(mapping[p] for p in stream.inputs)
            self.push_instance(stream.get_instance(input_objects))

    def _add_new_instances(self, new_atom):
        for s_idx, stream in enumerate(self.streams):
            for d_idx, domain_fact in enumerate(stream.domain):
                domain_atom = head_from_fact(domain_fact)
                if is_instance(new_atom, domain_atom):
                    # TODO: handle domain constants more intelligently
                    self.atoms_from_domain[s_idx, d_idx].append(new_atom)
                    atoms = [self.atoms_from_domain[s_idx, d2_idx] if d_idx != d2_idx else [new_atom]
                              for d2_idx in range(len(stream.domain))]
                    if USE_RELATION:
                        self._add_combinations_relation(stream, atoms)
                    else:
                        self._add_combinations(stream, atoms)

    def add_atom(self, atom, complexity):
        if not is_atom(atom):
            return False
        head = atom.head
        if head in self.complexity_from_atom:
            assert self.complexity_from_atom[head] <= complexity
            return False
        self.complexity_from_atom[head] = complexity
        self._add_new_instances(head)
        return True
