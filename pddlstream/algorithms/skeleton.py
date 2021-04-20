from __future__ import print_function

import time
from collections import namedtuple, Sized
from heapq import heappush, heappop

from pddlstream.algorithms.common import is_instance_ready, compute_complexity, stream_plan_complexity, add_certified
from pddlstream.algorithms.disabled import process_instance, update_bindings, update_cost, bind_action_plan
from pddlstream.algorithms.reorder import get_output_objects
from pddlstream.language.constants import is_plan, INFEASIBLE
from pddlstream.language.function import FunctionResult
from pddlstream.utils import elapsed_time, HeapElement, apply_mapping, INF

PRUNE_BINDINGS = True

# TODO: automatically set the opt level to be zero for any streams that are bound?

Priority = namedtuple('Priority', ['not_best', 'complexity', 'attempts', 'remaining', 'cost']) # TODO: FIFO
Affected = namedtuple('Affected', ['indices', 'has_cost'])

def compute_affected_downstream(stream_plan, index):
    # TODO: affected upstream
    affected_indices = [index]
    if len(stream_plan) <= index: # TODO: len(stream_plan) - 1?
        return Affected(affected_indices, has_cost=False)
    # TODO: if the cost is pruned, then add everything that contributes, not just the last function
    result = stream_plan[index]
    has_cost = (type(result) is FunctionResult)
    output_objects = set(get_output_objects(result))
    if not output_objects: # TODO: should I do conditions instead?
        return Affected(affected_indices, has_cost)
    for index2 in range(index + 1, len(stream_plan)):
        result2 = stream_plan[index2]
        if output_objects & result2.instance.get_all_input_objects():
            output_objects.update(get_output_objects(result2))
            affected_indices.append(index2)
            has_cost |= (type(result2) is FunctionResult)
    return Affected(affected_indices, has_cost)

def compute_affected_component(stream_plan, index):
    pass

##################################################

class Skeleton(object):
    def __init__(self, queue, stream_plan, action_plan, cost):
        # TODO: estimate statistics per stream_instance online and use to reorder the skeleton
        self.queue = queue
        self.index = len(self.queue.skeletons)
        self.stream_plan = stream_plan
        self.action_plan = action_plan
        self.cost = cost
        self.best_binding = None
        self.root = Binding(self, self.cost, history=[], mapping={}, index=0, parent=None, parent_result=None)
        self.affected_indices = [compute_affected_downstream(self.stream_plan, index)
                                 for index in range(len(self.stream_plan))]
        # TODO: compute this all at once via hashing
    def update_best(self, binding):
        if (self.best_binding is None) or (self.best_binding.index < binding.index) or \
                ((self.best_binding.index == binding.index) and (binding.cost < self.best_binding.cost)):
            self.best_binding = binding
            #print('Skeleton {} | Progress: {} | New best: {}'.format(
            #    self.index, self.best_binding.index, self.best_binding))
            return True
        return False
    def bind_stream_result(self, index, mapping):
        return self.stream_plan[index].remap_inputs(mapping) # Has optimistic output objects
    def bind_action_plan(self, mapping):
        return bind_action_plan(self.action_plan, mapping)

##################################################

class Binding(object):
    def __init__(self, skeleton, cost, history, mapping, index, parent, parent_result):
    #def __init__(self, skeleton, cost=0., history=[], mapping={}, index=0, parent=None):
        self.skeleton = skeleton
        self.cost = cost
        self.history = history
        self.mapping = mapping
        self.index = index
        self.parent = parent
        if self.parent is not None:
            self.parent.children.append(self)
        self.parent_result = parent_result

        self.children = []
        self._result = False
        self.attempts = 0 # The number of times _process_binding has been called
        self.calls = 0 # The index for result_history
        self.complexity = None
        self.max_history = max(self.history) if self.history else 0
        #self.parent_complexity = parent_complexity
        self.skeleton.update_best(self)
    @property
    def result(self):
        if self._result is False:
            self._result = None
            if self.index <= len(self.skeleton.stream_plan) - 1:
                self._result = self.skeleton.bind_stream_result(self.index, self.mapping)
        return self._result
    @property
    def is_fully_bound(self):
        #return self.index >= len(self.skeleton.stream_plan)
        return self.result is None
    #@property
    #def complexity(self):
    #    # This does a plan linearization version of complexity
    #    return self.parent_complexity + self.calls
    def is_best(self):
        return self.skeleton.best_binding is self
    def is_dominated(self):
        return self.skeleton.queue.store.best_cost <= self.cost
    def up_to_date(self):
        if self.is_fully_bound:
            return True
        #if PRUNE_BINDINGS:
        #    return self.result.instance.num_calls <= self.attempts
        #else:
        return self.calls == self.result.instance.num_calls
    def compute_complexity(self):
        if self.is_fully_bound:
            return 0
        # TODO: intelligently compute/cache this
        if self.complexity is None:
            full_history = self.history + [self.calls] # TODO: include the full history or just the relevant history
            self.complexity = stream_plan_complexity(self.skeleton.queue.evaluations,
                                                     self.skeleton.stream_plan, full_history)
        #raw_input('Complexity: {} {}'.format(self.result, complexity))
        return self.complexity
        #return compute_complexity(self.skeleton.queue.evaluations, self.result.get_domain()) + \
        #       self.result.external.get_complexity(self.attempts) # attempts, calls
    def check_complexity(self, complexity_limit):
        if (complexity_limit < self.max_history) or (complexity_limit < self.calls):
            return False
        return self.compute_complexity() <= complexity_limit
    def do_evaluate_helper(self, affected):
        # TODO: update this online for speed purposes
        # TODO: store the result
        if self.is_dominated():
            return affected.has_cost
        if not self.children: # or type(self.result) == FunctionResult): # not self.attempts
            return self.index in affected.indices
        # TODO: only prune functions here if the reset of the plan is feasible
        #if not indices or (max(indices) < self.index):
        #    return False
        # TODO: discard bindings that have been pruned by their cost
        return any(binding.do_evaluate_helper(affected) for binding in self.children) # TODO: any or all
    def do_evaluate(self):
        return self.do_evaluate_helper(self.skeleton.affected_indices[self.index])
    def get_priority(self):
        # TODO: use effort instead
        # TODO: instead of remaining, use the index in the queue to reprocess earlier ones
        not_best = not self.is_best()
        #priority = self.attempts
        #priority = self.compute_complexity()
        priority = self.compute_complexity() + (self.attempts - self.calls) # TODO: check this
        remaining = len(self.skeleton.stream_plan) - self.index
        return Priority(not_best, priority, self.attempts, remaining, self.cost)
    def post_order(self):
        for child in self.children:
            for binding in child.post_order():
                yield binding
        yield self
    def get_ancestors(self):
        if self.parent is not None:
            for ancestor in self.parent.get_ancestors():
                yield ancestor
        yield self
    def recover_bound_results(self):
        return [binding.parent_result for binding in list(self.get_ancestors())[1:]]
    def update_bindings(self):
        new_bindings = []
        instance = self.result.instance
        for call_idx in range(self.calls, instance.num_calls):
            for new_result in instance.results_history[call_idx]: # TODO: don't readd if successful already
                if new_result.is_successful():
                    new_bindings.append(Binding(
                        skeleton=self.skeleton,
                        cost=update_cost(self.cost, self.result, new_result),
                        history=self.history + [call_idx],
                        mapping=update_bindings(self.mapping, self.result, new_result),
                        index=self.index + 1, # TODO: history instead of results_history
                        parent=self,
                        parent_result=new_result))
        self.calls = instance.num_calls
        self.attempts = max(self.attempts, self.calls)
        self.complexity = None # Forces re-computation
        return new_bindings
    def __repr__(self):
        return '{}(skeleton={}, {})'.format(self.__class__.__name__, self.skeleton.index, self.result)

##################################################

STANDBY = None

class SkeletonQueue(Sized):
    def __init__(self, store, domain, disable=True):
        self.store = store
        self.domain = domain
        self.skeletons = []
        self.queue = []
        self.disable = disable

    @property
    def evaluations(self):
        return self.store.evaluations

    def __len__(self):
        return len(self.queue)

    def is_active(self):
        return self.queue and (not self.store.is_terminated())

    def push_binding(self, binding):
        # TODO: add to a separate queue if not active
        priority = binding.get_priority()
        element = HeapElement(priority, binding)
        heappush(self.queue, element)

    def pop_binding(self):
        priority, binding = heappop(self.queue)
        #return binding
        return priority, binding

    def peak_binding(self):
        if not self.queue:
            return None
        priority, binding = self.queue[0]
        return priority, binding

    def new_skeleton(self, stream_plan, action_plan, cost):
        skeleton = Skeleton(self, stream_plan, action_plan, cost)
        self.skeletons.append(skeleton)
        self.push_binding(skeleton.root)
        return skeleton

    #########################

    def _process_binding(self, binding):
        readd = is_new = False
        if binding.is_dominated():
            return readd, is_new
        if binding.is_fully_bound:
            action_plan = binding.skeleton.bind_action_plan(binding.mapping)
            self.store.add_plan(action_plan, binding.cost)
            is_new = True
            return readd, is_new
        binding.attempts += 1
        instance = binding.result.instance
        if PRUNE_BINDINGS and not binding.do_evaluate():
            # TODO: causes redundant plan skeletons to be identified (along with complexity using attempts instead of calls)
            # Do I need to re-enable this stream in case another skeleton needs it?
            # TODO: should I perform this when deciding to sample something new instead?
            #if instance.enumerated:
            #    return False, is_new
            return STANDBY, is_new
        #if not is_instance_ready(self.evaluations, instance):
        #    raise RuntimeError(instance)
        if binding.up_to_date():
            new_results, _ = process_instance(self.store, self.domain, instance, disable=self.disable)
            is_new = bool(new_results)
        for new_binding in binding.update_bindings():
            self.push_binding(new_binding)
        readd = not instance.enumerated
        return readd, is_new

    def _process_root(self):
        _, binding = self.pop_binding()
        readd, is_new = self._process_binding(binding)
        if readd is not False:
            self.push_binding(binding)
        return is_new

    #########################

    def greedily_process(self, max_attempts=1, only_best=True):
        while self.is_active():
            # TODO: break if the complexity limit is exceeded
            priority, binding = self.peak_binding()
            #print(binding.is_best() == not priority.not_best) # TODO: update if stale
            if (only_best and priority.not_best) or (priority.attempts >= max_attempts):
                break
            self._process_root()

    def process_until_new(self, complexity_limit, print_frequency=1.):
        # TODO: process the entire queue once instead
        print('Sampling until new output values')
        is_new = False
        attempts = 0
        last_time = time.time()
        standby = []
        while self.is_active() and (not is_new):
            attempts += 1
            _, binding = self.pop_binding()
            readd, is_new = self._process_binding(binding)
            if readd is True:
                self.push_binding(binding)
            elif readd is STANDBY:
                standby.append(binding)
            #is_new |= self._process_root()
            self.greedily_process(complexity_limit)
            if print_frequency <= elapsed_time(last_time):
                print('Queue: {} | Attempts: {} | Time: {:.3f}'.format(
                    len(self.queue), attempts, elapsed_time(last_time)))
                last_time = time.time()
        for binding in standby:
            self.push_binding(binding)
        return is_new

    def process_complexity(self, complexity_limit):
        raise NotImplementedError()
        # TODO: could copy the queue and filter instances that exceed complexity_limit
        #self.greedily_process(max_attempts=complexity_limit) # This isn't quite the complexity limit
        disabled_bindings = []
        while self.is_active():
            _, binding = self.pop_binding()
            if binding.check_complexity(complexity_limit): # not binding.up_to_date() or
                readd, _ = self._process_binding(binding)
                if readd is True:
                    self.push_binding(binding)
                    continue
                self.greedily_process()
            disabled_bindings.append(binding)
        for binding in disabled_bindings:
            self.push_binding(binding)
        # TODO: increment the complexity level even more if nothing below in the queue

    def timed_process(self, complexity_limit, max_time):
        start_time = time.time()
        iterations = num_new = 0
        while self.is_active() and (elapsed_time(start_time) <= max_time):
            iterations += 1
            num_new += self._process_root()
            self.greedily_process(complexity_limit)
        print('Iterations: {} | New: {} | Time: {:.3f}'.format(iterations, num_new, elapsed_time(start_time)))

    #########################

    def accelerate_best_bindings(self, **kwargs):
        # TODO: more generally reason about streams on several skeletons
        # TODO: reset the complexity values for old streams
        for skeleton in self.skeletons:
            for result in skeleton.best_binding.recover_bound_results():
                # TODO: just accelerate the facts within the plan preimage
                result.call_index = 0 # Pretends the fact was first
                add_certified(self.evaluations, result, **kwargs) # TODO: should special have a complexity of INF?

    def process(self, stream_plan, action_plan, cost, complexity_limit, max_time=0, accelerate=False):
        start_time = time.time()
        if is_plan(stream_plan):
            self.new_skeleton(stream_plan, action_plan, cost)
            self.greedily_process()
        elif (stream_plan is INFEASIBLE) and not self.process_until_new(complexity_limit):
            # Move this after process_complexity
            return False
        #if not is_plan(stream_plan):
        #    print('Complexity:', complexity_limit) # TODO: revisit this
        #    self.process_complexity(complexity_limit)
        remaining_time = max_time - elapsed_time(start_time)
        print('Remaining sampling time: {:.3f} seconds'.format(remaining_time))
        self.timed_process(complexity_limit, remaining_time)
        if accelerate:
           self.accelerate_best_bindings()
        return True
