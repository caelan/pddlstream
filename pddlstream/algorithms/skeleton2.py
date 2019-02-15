from __future__ import print_function

import time
from operator import itemgetter
from collections import namedtuple, Sized
from heapq import heappush, heappop, heapreplace

from pddlstream.algorithms.common import is_instance_ready, EvaluationNode
from pddlstream.algorithms.disabled import process_instance
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult, StreamResult
from pddlstream.language.constants import is_plan, INFEASIBLE
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.utils import elapsed_time, HeapElement, safe_zip, apply_mapping, str_from_object, get_mapping

from pddlstream.algorithms.skeleton import Skeleton, update_bindings, update_cost

Priority = namedtuple('Priority', ['attempted', 'attempts', 'remaining'])

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
    def get_element(self):
        attempted = (self.attempts != 0)
        remaining = len(self.skeleton.stream_plan) - self.index
        priority = Priority(attempted, self.attempts, remaining)
        return HeapElement(priority, self)

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
        #self.new_binding(skeleton.root)
        binding = Binding(skeleton, cost)
        heappush(self.queue, binding.get_element())
        # TODO: handle case when the plan is empty

    def _process_root(self):
        is_new = False
        _, binding = heappop(self.queue)
        if self.store.best_cost <= binding.cost:
            return is_new
        if binding.result is None:
            action_plan = binding.skeleton.bind_action_plan(binding.mapping)
            self.store.add_plan(action_plan, binding.cost)
            return is_new
        instance = binding.result.instance
        if not is_instance_ready(self.evaluations, instance):
            raise RuntimeError(instance)
        if binding.attempts == instance.num_calls:
            is_new = process_instance(self.store, self.domain, instance, disable=self.disable)
        for new_result in instance.get_results(start=binding.attempts):
            if new_result.is_successful():
                new_mapping = update_bindings(binding.mapping, binding.result, new_result)
                new_cost = update_cost(binding.cost, binding.result, new_result)
                new_binding = Binding(binding.skeleton, new_cost, new_mapping, binding.index + 1)
                binding.children.append(new_binding)
                heappush(self.queue, new_binding.get_element())
        binding.attempts = instance.num_calls
        if isinstance(instance, StreamResult) and not instance.external.outputs and instance.successes:
            # Set of possible output is exhausted
            # More generally, keep track of results and prevent repeat bindings
            return is_new
        if not instance.enumerated:
            heappush(self.queue, binding.get_element())
        return is_new

    def greedily_process(self):
        while self.is_active():
            key, _ = self.queue[0]
            if key.attempted:
                break
            self._process_root()

    def timed_process(self, max_time):
        start_time = time.time()
        while self.is_active() and (elapsed_time(start_time) <= max_time):
            self._process_root()
            self.greedily_process()

    def process(self, stream_plan, action_plan, cost, complexity_limit, max_time=0):
        start_time = time.time()
        if is_plan(stream_plan):
            self.new_skeleton(stream_plan, action_plan, cost)
            self.greedily_process()
        elif stream_plan is INFEASIBLE:
            # TODO: use complexity_limit
            self.process_until_new()
        self.timed_process(max_time - elapsed_time(start_time))
        # TODO: accelerate the best bindings
        #self.accelerate_best_bindings()

    def __len__(self):
        return len(self.queue)