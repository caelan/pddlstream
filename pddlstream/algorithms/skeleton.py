from __future__ import print_function

import time
from operator import itemgetter
from collections import namedtuple, Sized
from heapq import heappush, heappop, heapreplace

from pddlstream.algorithms.common import is_instance_ready, EvaluationNode
from pddlstream.algorithms.disabled import process_instance
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult
from pddlstream.language.constants import is_plan, INFEASIBLE
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.utils import elapsed_time, HeapElement, safe_zip, apply_mapping, str_from_object


# The motivation for immediately instantiating is to avoid unnecessary sampling
# Consider a stream result DAG A -> C, B -> C
# If A is already successfully sampled, don't want to resample A until B is sampled

def puncture(sequence, index):
    return sequence[:index] + sequence[index+1:]

def update_bindings(bindings, opt_result, result):
    new_bindings = bindings.copy()
    if isinstance(result, StreamResult):
        for opt, obj in safe_zip(opt_result.output_objects, result.output_objects):
            assert (opt not in new_bindings)  # TODO: return failure if conflicting bindings
            new_bindings[opt] = obj
    return new_bindings

def update_cost(cost, opt_result, result):
    new_cost = cost
    if type(result) is FunctionResult:
        new_cost += (result.value - opt_result.value)
    return new_cost

##################################################

class Skeleton(object):
    def __init__(self, queue, stream_plan, action_plan, cost):
        self.queue = queue
        self.stream_plan = stream_plan
        self.action_plan = action_plan
        self.cost = cost
        self.root = Binding(self.queue, self,
                            stream_indices=range(len(stream_plan)),
                            stream_attempts=[0]*len(stream_plan),
                            bound_results={},
                            bindings={},
                            cost=cost)
        self.best_binding = self.root
    def bind_stream_plan(self, mapping, indices=None):
        if indices is None:
            indices = range(len(self.stream_plan))
        return [self.stream_plan[index].remap_inputs(mapping) for index in indices]
    def bind_action_plan(self, mapping):
        return [(name, apply_mapping(args, mapping)) for name, args in self.action_plan]

##################################################

Priority = namedtuple('Priority', ['attempted', 'effort'])

class Binding(object):
    # TODO: maintain a tree instead. Propagate the best subtree upwards
    def __init__(self, queue, skeleton, stream_indices, stream_attempts,
                 bound_results, bindings, cost):
        assert len(stream_indices) == len(stream_attempts)
        self.queue = queue
        self.skeleton = skeleton
        self.stream_indices = list(stream_indices)
        self.stream_attempts = list(stream_attempts)
        self.bound_results = bound_results
        self.bindings = bindings
        self.cost = cost
        self.children = []
        self.enumerated = False # What if result enumerated with zero calls?
        self._stream_plan = None
        # Maybe just reset the indices for anything that isn't applicable
        # n+1 sample represented
        # TODO: store partial orders
        # TODO: store applied results
        # TODO: the problem is that I'm not actually doing all combinations because I'm passing attempted
    @property
    def stream_plan(self):
        if self._stream_plan is None:
            self._stream_plan = self.skeleton.bind_stream_plan(self.bindings, self.stream_indices)
        return self._stream_plan
    @property
    def action_plan(self):
        return self.skeleton.bind_action_plan(self.bindings)
    def is_dominated(self):
        # TODO: what should I do if the cost=inf (from incremental/exhaustive)
        return self.queue.store.has_solution() and (self.queue.store.best_cost <= self.cost)
    def is_enabled(self):
        return not (self.enumerated or self.is_dominated())
    def get_priority(self):
        # Infinite cost if skeleton is exhausted
        # Attempted is equivalent to whether any stream result is disabled
        num_attempts = sum(self.stream_attempts)
        attempted = num_attempts != 0
        # TODO: lexicographic tiebreaking using plan cost and other skeleton properties
        return Priority(attempted, (num_attempts, len(self.stream_attempts)))
    def get_element(self):
        return HeapElement(self.get_priority(), self)
    def get_key(self):
        # Each stream result is unique (affects hashing)
        return self.skeleton, tuple(self.stream_indices), frozenset(self.bindings.items())
    def _instantiate(self, index, new_result):
        if not new_result.is_successful():
            return None # TODO: check if satisfies target certified
        opt_result = self.stream_plan[index]
        bound_results = self.bound_results.copy()
        bound_results[self.stream_indices[index]] = new_result
        binding = Binding(self.queue, self.skeleton,
                          puncture(self.stream_indices, index),
                          puncture(self.stream_attempts, index),
                          bound_results,
                          update_bindings(self.bindings, opt_result, new_result),
                          update_cost(self.cost, opt_result, new_result))
        if len(binding.stream_indices) < len(self.skeleton.best_binding.stream_indices):
            self.skeleton.best_binding = binding
        self.children.append(binding)
        self.queue.new_binding(binding)
        return binding
    def update_instances(self):
        updated = False
        for index, (opt_result, attempt) in enumerate(safe_zip(self.stream_plan, self.stream_attempts)):
            if self.enumerated:
                return updated
            if opt_result.instance.num_calls != attempt:
                updated = True
                for new_result in opt_result.instance.get_results(start=attempt):
                    self._instantiate(index, new_result)
                self.stream_attempts[index] = opt_result.instance.num_calls
                self.enumerated |= opt_result.instance.enumerated
        return updated
    def __repr__(self):
        #return '{}({})'.format(self.__class__.__name__, str_from_object(self.stream_plan))
        return '{}({})'.format(self.__class__.__name__, str_from_object(self.action_plan))

##################################################

class SkeletonQueue(Sized):
    # TODO: handle this in a partially ordered way
    # TODO: alternatively store just preimage and reachieve
    # TODO: make an "action" for returning to the search (if it is the best decision)
    # TODO: could just maintain a list of active instances and sample/propagate
    # TODO: store bindings in a factored form that only combines when needed

    # TODO: update bindings given outcomes of eager streams
    # TODO: immediately evaluate eager streams in the queue

    def __init__(self, store, goal_expression, domain):
        self.store = store
        self.evaluations = store.evaluations
        #self.goal_expression = goal_expression
        self.domain = domain
        self.skeletons = []
        self.queue = []
        self.binding_from_key = {}
        self.bindings_from_instance = {}
        self.enabled_bindings = set()

    ####################

    def _flush_stale(self):
        while self.queue:
            queue_priority, binding = self.queue[0]
            current_priority = binding.get_priority()
            if queue_priority == current_priority:
                return
            heapreplace(self.queue, binding.get_element())

    ####################

    #def _reenable_stream_plan(self, stream_plan):
    #    # TODO: only disable if not used elsewhere
    #    # TODO: could just hash instances
    #    # TODO: do I actually need to reenable? Yes it ensures that
    #    # TODO: check if the index is the only one being sampled
    #    # for result in stream_plan:
    #    #    result.instance.disabled = False
    #    stream_plan[0].instance.enable(self.evaluations, self.domain)
    #    # TODO: move functions as far forward as possible to prune these plans
    #    # TODO: make function evaluations low success as soon as finite cost

    # Maybe the reason repeat skeletons are happening is that the currently active thing is disabled
    # But another one on the plan isn't
    # Could scan the whole queue each time a solution is found

    def update_enabled(self, binding):
        if not binding.is_enabled() and (binding in self.enabled_bindings):
            self.disable_binding(binding)

    def is_enabled(self, binding):
        self.update_enabled(binding)
        return binding in self.enabled_bindings

    def enable_binding(self, binding):
        assert binding not in self.enabled_bindings
        self.enabled_bindings.add(binding)
        for result in binding.stream_plan:
            instance = result.instance
            if instance not in self.bindings_from_instance:
                self.bindings_from_instance[instance] = set()
            self.bindings_from_instance[instance].add(binding)

    def disable_binding(self, binding):
        assert binding in self.enabled_bindings
        self.enabled_bindings.remove(binding)
        for result in binding.stream_plan:
            instance = result.instance
            self.bindings_from_instance[instance].remove(binding)
            if not self.bindings_from_instance[instance]:
                del self.bindings_from_instance[instance]

    ####################

    def new_binding(self, binding):
        key = binding.get_key()
        if key in self.binding_from_key:
            print('Binding already visited!') # Could happen if binding is the same
            #return
        self.binding_from_key[key] = binding
        if not binding.is_enabled():
            return
        if not binding.stream_indices:
            # if is_solution(self.domain, self.evaluations, bound_plan, self.goal_expression):
            self.store.add_plan(binding.action_plan, binding.cost)
            # TODO: could update active for all items in a queue fashion
            return
        binding.update_instances()
        if binding.is_enabled():
            self.enable_binding(binding)
            heappush(self.queue, binding.get_element())

    def new_skeleton(self, stream_plan, action_plan, cost):
        skeleton = Skeleton(self, stream_plan, action_plan, cost)
        self.skeletons.append(skeleton)
        self.new_binding(skeleton.root)

    ####################

    def _generate_results(self, instance):
        # assert(instance.opt_index == 0)
        if not is_instance_ready(self.evaluations, instance):
            raise RuntimeError(instance)
        is_new = process_instance(self.store, self.domain, instance)
        for i, binding in enumerate(list(self.bindings_from_instance[instance])):
            #print(i, binding)
            # Maybe this list grows but not all the things are accounted for
            if self.is_enabled(binding):
                binding.update_instances()
                self.update_enabled(binding)
        #print()
        return is_new

    def _process_root(self):
        is_new = False
        self._flush_stale()
        _, binding = heappop(self.queue)
        if not self.is_enabled(binding):
            return is_new
        assert not binding.update_instances() #self.update_enabled(binding)
        is_new = self._generate_results(binding.stream_plan[0].instance)
        # _decompose_synthesizer_skeleton(queue, skeleton, stream_index)
        if self.is_enabled(binding):
            heappush(self.queue, binding.get_element())
        return is_new

    ####################

    def is_active(self):
        return self.queue and (not self.store.is_terminated())

    def greedily_process(self):
        while self.is_active():
            self._flush_stale()
            key, _ = self.queue[0]
            if key.attempted:
                break
            self._process_root()

    def process_until_new(self):
        # TODO: process the entire queue once instead
        is_new = False
        while self.is_active() and (not is_new):
            is_new |= self._process_root()
            self.greedily_process()
        return is_new

    def timed_process(self, max_time):
        start_time = time.time()
        while self.is_active() and (elapsed_time(start_time) <= max_time):
            self._process_root()
            self.greedily_process()
            # TODO: print cost updates when progress with a new skeleton

    def accelerate_best_bindings(self):
        # TODO: reset the values for old streams
        for skeleton in self.skeletons:
            for _, result in sorted(skeleton.best_binding.bound_results.items(), key=itemgetter(0)):
                # TODO: just accelerate the facts within the plan preimage
                result.call_index = 0 # Pretends the fact was first
                new_complexity = result.compute_complexity(self.evaluations)
                for fact in result.get_certified():
                    evaluation = evaluation_from_fact(fact)
                    if new_complexity < self.evaluations[evaluation].complexity:
                        self.evaluations[evaluation] = EvaluationNode(new_complexity, result)

    def process(self, stream_plan, action_plan, cost, complexity_limit, max_time=0):
        # TODO: manually add stream_plans for synthesizers/optimizers
        start_time = time.time()
        if is_plan(stream_plan):
            #print([result for result in stream_plan if result.optimistic])
            #raw_input('New skeleton')
            self.new_skeleton(stream_plan, action_plan, cost)
            self.greedily_process()
        elif stream_plan is INFEASIBLE:
            # TODO: use complexity_limit
            self.process_until_new()
        self.timed_process(max_time - elapsed_time(start_time))
        self.accelerate_best_bindings()

        # Only currently blocking streams with after called
        # Can always process streams with a certain complexity
        # Temporarily pop off the queue and then re-add
        # Domination occurs when no downstream skeleton that
        # Is it worth even doing the dynamic instantiation?
        # IF some set fails where the output is an input
        # Scale input

    def __len__(self):
        return len(self.queue)

##################################################

# from pddlstream.language.synthesizer import SynthStreamResult
# def _decompose_synthesizer_skeleton(queue, skeleton, index):
#     stream_plan, plan_attempts, bindings, plan_index, cost = skeleton
#     opt_result = stream_plan[index]
#     if (plan_attempts[index] == 0) and isinstance(opt_result, SynthStreamResult):
#         # TODO: only decompose if failure?
#         decomposition = opt_result.decompose()
#         new_stream_plan = stream_plan[:index] + decomposition + stream_plan[index+1:]
#         new_plan_attempts = plan_attempts[:index] + [0]*len(decomposition) + plan_attempts[index+1:]
#         queue.new_binding(new_stream_plan, new_plan_attempts, bindings, plan_index, cost)

##################################################

# TODO: want to minimize number of new sequences as they induce overhead
# TODO: estimate how many times a stream needs to be queried (acceleration)
#
# def compute_sampling_cost(stream_plan, stats_fn=get_stream_stats):
#     # TODO: we are in a POMDP. If not the case, then the geometric cost policy is optimal
#     if stream_plan is None:
#         return INF
#     expected_cost = 0
#     for result in reversed(stream_plan):
#         p_success, overhead = stats_fn(result)
#         expected_cost += geometric_cost(overhead, p_success)
#     return expected_cost
#     # TODO: mix between geometric likelihood and learned distribution
#     # Sum tail distribution for number of future
#     # Distribution on the number of future attempts until successful
#     # Average the tail probability mass
#
# def compute_belief(attempts, p_obs):
#     return pow(p_obs, attempts)
#
# def compute_success_score(plan_attempts, p_obs=.9):
#     beliefs = [compute_belief(attempts, p_obs) for attempts in plan_attempts]
#     prior = 1.
#     for belief in beliefs:
#         prior *= belief
#     return -prior
#
# def compute_geometric_score(plan_attempts, overhead=1, p_obs=.9):
#     # TODO: model the decrease in belief upon each failure
#     # TODO: what if stream terminates? Assign high cost
#     expected_cost = 0
#     for attempts in plan_attempts:
#         p_success = compute_belief(attempts, p_obs)
#         expected_cost += geometric_cost(overhead, p_success)
#     return expected_cost

##################################################

# from pddlstream.algorithms.downward import task_from_domain_problem, get_problem, get_action_instances, \
#    get_goal_instance, plan_preimage, is_valid_plan, substitute_derived, is_applicable, apply_action
# from pddlstream.algorithms.reorder import replace_derived
# from pddlstream.algorithms.scheduling.recover_axioms import extract_axiom_plan

# def is_solution(domain, evaluations, action_plan, goal_expression):
#     task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs=True))
#     action_instances = get_action_instances(task, action_plan) + [get_goal_instance(task.goal)]
#     #original_init = task.init
#     task.init = set(task.init)
#     for instance in action_instances:
#         axiom_plan = extract_axiom_plan(task, instance, negative_from_name={}, static_state=task.init)
#         if axiom_plan is None:
#             return False
#         #substitute_derived(axiom_plan, instance)
#         #if not is_applicable(task.init, instance):
#         #    return False
#         apply_action(task.init, instance)
#     return True
#     #replace_derived(task, set(), plan_instances)
#     #preimage = plan_preimage(plan_instances, [])
#     #return is_valid_plan(original_init, action_instances) #, task.goal)
