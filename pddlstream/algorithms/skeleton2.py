from __future__ import print_function

import time
from collections import namedtuple, Sized
from heapq import heappush, heappop

from pddlstream.algorithms.common import is_instance_ready, compute_complexity
from pddlstream.algorithms.disabled import process_instance
from pddlstream.algorithms.skeleton import update_bindings, update_cost
from pddlstream.language.constants import is_plan, INFEASIBLE
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult
from pddlstream.utils import elapsed_time, HeapElement, apply_mapping

PRUNE_BINDINGS = True

# TODO: prioritize bindings using effort
# TODO: use complexity rather than attempts for ordering
# TODO: FIFO
Priority = namedtuple('Priority', ['attempts', 'remaining'])

def compute_affected(stream_plan, index):
    affected_indices = []
    if len(stream_plan) <= index:
        return affected_indices
    result = stream_plan[index]
    output_objects = set(result.output_objects) if isinstance(result, StreamResult) else set()
    if not output_objects:
        return affected_indices
    for index2 in range(index + 1, len(stream_plan)):
        result2 = stream_plan[index2]
        if set(result2.instance.input_objects) & output_objects:
            affected_indices.append(index2)
    return affected_indices

class Skeleton(object):
    def __init__(self, queue, stream_plan, action_plan, cost):
        self.index = len(queue.skeletons)
        queue.skeletons.append(self)
        self.queue = queue
        self.stream_plan = stream_plan
        self.action_plan = action_plan
        self.cost = cost
        self.best_binding = None
        self.root = Binding(self, self.cost)
        self.affected_indices = [compute_affected(self.stream_plan, index)
                                 for index in range(len(self.stream_plan))]
        # TODO: compute this all at once via hashing
    def bind_action_plan(self, mapping):
        return [(name, apply_mapping(args, mapping)) for name, args in self.action_plan]

##################################################

class Binding(object):
    def __init__(self, skeleton, cost, mapping={}, index=0):
        self.skeleton = skeleton
        self.cost = cost
        self.mapping = mapping
        self.index = index
        self.children = []
        if self.index < len(skeleton.stream_plan):
            self.result = skeleton.stream_plan[self.index].remap_inputs(self.mapping)
        else:
            self.result = None
        self.attempts = 0
        self.calls = 0
        if (self.skeleton.best_binding is None) or (self.skeleton.best_binding.index < self.index):
            self.skeleton.best_binding = self
    def up_to_date(self):
        if self.result is None:
            return True
        if PRUNE_BINDINGS:
            return self.result.instance.num_calls <= self.attempts
        else:
            return self.calls == self.result.instance.num_calls
    def compute_complexity(self):
        if self.result is None:
            return 0
        return compute_complexity(self.skeleton.queue.evaluations, self.result.get_domain()) + \
               self.result.external.get_complexity(self.attempts) # attempts, calls
    def do_evaluate_helper(self, indices):
        # TODO: update this online for speed purposes
        if (self.index in indices) and (not self.children or type(self.result) == FunctionResult): # not self.attempts
            return True
        if not indices or (max(indices) < self.index):
            return False
        return all(binding.do_evaluate_helper(indices) for binding in self.children) # TODO: all or any?
    def do_evaluate(self):
        return not self.children or self.do_evaluate_helper(self.skeleton.affected_indices[self.index])
    def get_element(self):
        remaining = len(self.skeleton.stream_plan) - self.index
        priority = Priority(self.attempts, remaining)
        return HeapElement(priority, self)

##################################################

class SkeletonQueue(Sized):
    def __init__(self, store, domain, disable=True):
        self.store = store
        self.evaluations = store.evaluations
        self.domain = domain
        self.skeletons = []
        self.queue = []
        self.binding_from_key = {}
        self.bindings_from_instance = {}
        self.enabled_bindings = set()
        self.disable = disable

    def is_active(self):
        return self.queue and (not self.store.is_terminated())

    def new_skeleton(self, stream_plan, action_plan, cost):
        skeleton = Skeleton(self, stream_plan, action_plan, cost)
        heappush(self.queue, skeleton.root.get_element())

    def _process_binding(self, binding):
        is_new = False
        if self.store.best_cost <= binding.cost:
            return False, is_new
        if binding.result is None:
            action_plan = binding.skeleton.bind_action_plan(binding.mapping)
            self.store.add_plan(action_plan, binding.cost)
            return False, is_new
        binding.attempts += 1
        if PRUNE_BINDINGS and not binding.do_evaluate():
            # TODO: causes redudant plan skeletons to be identified (along with complexity using attempts instead of calls)
            return True, is_new
        instance = binding.result.instance
        if not is_instance_ready(self.evaluations, instance):
            raise RuntimeError(instance)
        if binding.up_to_date():
            is_new = process_instance(self.store, self.domain, instance, disable=self.disable)
        for new_result in instance.get_results(start=binding.calls):
            if new_result.is_successful():
                new_mapping = update_bindings(binding.mapping, binding.result, new_result)
                new_cost = update_cost(binding.cost, binding.result, new_result)
                new_binding = Binding(binding.skeleton, new_cost, new_mapping, binding.index + 1)
                binding.children.append(new_binding)
                heappush(self.queue, new_binding.get_element())
        binding.calls = instance.num_calls
        binding.attempts = max(binding.attempts, binding.calls)
        readd = not instance.enumerated
        return readd, is_new

    def _process_root(self):
        _, binding = heappop(self.queue)
        readd, is_new = self._process_binding(binding)
        if readd:
            heappush(self.queue, binding.get_element())
        return is_new

    def greedily_process(self, max_attempts=0):
        while self.is_active():
            key, _ = self.queue[0]
            if max_attempts < key.attempts:
                break
            self._process_root()

    def process_until_new(self):
        # TODO: process the entire queue once instead
        is_new = False
        while self.is_active() and (not is_new):
            is_new |= self._process_root()
            self.greedily_process()
        return is_new

    def process_complexity(self, complexity_limit):
        # TODO: could copy the queue and filter instances that exceed complexity_limit
        #self.greedily_process(max_attempts=complexity_limit) # This isn't quite the complexity limit
        disabled_bindings = []
        while self.is_active():
            _, binding = heappop(self.queue)
            if binding.compute_complexity() <= complexity_limit: # not binding.up_to_date() or
                readd, _ = self._process_binding(binding)
                if readd:
                    heappush(self.queue, binding.get_element())
            else:
                disabled_bindings.append(binding)
            self.greedily_process()
        for binding in disabled_bindings:
            heappush(self.queue, binding.get_element())

    def timed_process(self, max_time):
        start_time = time.time()
        while self.is_active() and (elapsed_time(start_time) <= max_time):
            self._process_root()
            self.greedily_process()

    def process(self, stream_plan, action_plan, cost, complexity_limit, max_time=0):
        # TODO: detect infeasibility when an intermediate stream fails
        start_time = time.time()
        if is_plan(stream_plan):
            self.new_skeleton(stream_plan, action_plan, cost)
            self.greedily_process()
        elif stream_plan is INFEASIBLE:
            self.process_until_new()
        #self.process_complexity(complexity_limit)
        self.timed_process(max_time - elapsed_time(start_time))
        # TODO: accelerate the best bindings
        #self.accelerate_best_bindings()

    def __len__(self):
        return len(self.queue)