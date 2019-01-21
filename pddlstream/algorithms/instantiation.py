from collections import defaultdict, namedtuple, Sized, deque
from heapq import heappush, heappop
from itertools import product

from pddlstream.algorithms.common import COMPLEXITY_OP, compute_call_complexity
from pddlstream.algorithms.relation import compute_order, Relation, solve_satisfaction
from pddlstream.language.constants import is_parameter
from pddlstream.language.conversion import is_atom, head_from_fact
from pddlstream.language.function import Function, FunctionInstance
from pddlstream.utils import safe_zip, HeapElement

# TODO: maybe store effort as well unit effort here
Priority = namedtuple('Priority', ['effort', 'num']) # num ensures FIFO

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

def pop_queue(queue):
    if isinstance(queue, deque):
        return queue.popleft()
    else:
        return heappop(queue)

##################################################

# http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.43.7049&rep=rep1&type=pdf

class Instantiator(Sized): # Dynamic Instantiator
    def __init__(self, evaluations, streams,
                 unit_efforts=True, max_effort=None, use_deque=False):
        # TODO: lazily instantiate upon demand
        self.unit_complexities = unit_efforts
        self.max_complexity = max_effort # exclusive
        self.use_heap = use_deque
        self.streams = streams
        #self.streams_from_atom = defaultdict(list)
        self.stream_queue = []
        self.function_queue = []
        self.num_pushes = 0 # shared between the queues
        # TODO: rename atom to head in most places
        self.effort_from_atom = {}
        self.atoms_from_domain = defaultdict(list)
        for stream in self.streams:
            if not stream.domain:
                self._add_instance(stream, {})
        for atom, node in evaluations.items():
            self.add_atom(atom, node.complexity)

    #########################

    def __len__(self):
        return len(self.stream_queue) + len(self.function_queue)

    def prune_effort(self, external, effort):
        if self.max_complexity is None:
            return False # Allows infinite effort
        return (not isinstance(external, Function)) and (self.max_complexity <= effort)

    def push(self, instance):
        # TODO: flush stale priorities?
        complexity = self._compute_complexity(instance)
        priority = Priority(complexity, self.num_pushes)
        queue = self.function_queue if isinstance(instance, FunctionInstance) else self.stream_queue
        if isinstance(queue, deque):
            queue.append(HeapElement(priority, instance))
        else:
            heappush(queue, HeapElement(priority, instance))
        self.num_pushes += 1

    def pop_stream(self):
        priority, instance = pop_queue(self.stream_queue)
        return instance, priority.effort

    def pop_function(self):
        priority, instance = pop_queue(self.function_queue)
        return instance, priority.effort

    def pop(self):
        if self.function_queue:
            return self.pop_function()
        return self.pop_stream()

    def min_effort(self):
        priority, _ = self.stream_queue[0]
        return priority.effort

    #########################

    def _compute_complexity(self, instance):
        domain_complexity = COMPLEXITY_OP([self.effort_from_atom[head_from_fact(f)]
                                       for f in instance.get_domain()] + [0])
        return domain_complexity + compute_call_complexity(instance.num_calls)

    def _add_instance(self, stream, mapping):
        input_objects = tuple(mapping[p] for p in stream.inputs)
        instance = stream.get_instance(input_objects)
        #domain_effort = COMPLEXITY_OP([self.effort_from_atom[head_from_fact(f)]
        #                               for f in instance.get_domain()] + [0])
        #effort = domain_effort + compute_instance_effort(
        #    instance, unit_efforts=self.unit_complexities)
        #if self.prune_effort(instance.external, effort):
        #    return False
        self.push(instance)
        return True

    def _add_combinations(self, stream, atoms):
        if not all(atoms):
            return
        domain = list(map(head_from_fact, stream.domain))
        # Most constrained variable/atom to least constrained
        for combo in product(*atoms):
            mapping = test_mapping(domain, combo)
            if mapping is not None:
                self._add_instance(stream, mapping)

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
            self._add_instance(stream, solution.get_mapping(element))

    def _add_new_instances(self, new_atom):
        for s_idx, stream in enumerate(self.streams):
            #effort_bound = self.effort_from_atom[new_atom] + compute_external_effort(
            #    stream, unit_efforts=self.unit_complexities)
            #if self.prune_effort(stream, effort_bound):
            #    continue
            for d_idx, domain_fact in enumerate(stream.domain):
                domain_atom = head_from_fact(domain_fact)
                if is_instance(new_atom, domain_atom):
                    # TODO: handle domain constants more intelligently
                    self.atoms_from_domain[s_idx, d_idx].append(new_atom)
                    atoms = [self.atoms_from_domain[s_idx, d2_idx] if d_idx != d2_idx else [new_atom]
                              for d2_idx in range(len(stream.domain))]
                    self._add_combinations(stream, atoms)
                    #self._add_combinations_relation(stream, atoms)

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
