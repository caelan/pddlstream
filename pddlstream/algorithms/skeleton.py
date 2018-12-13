import time
from collections import namedtuple, Sized, defaultdict
from heapq import heappush, heappop, heapreplace

from pddlstream.algorithms.algorithm import add_certified, add_facts
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult, StreamInstance
from pddlstream.utils import elapsed_time, HeapElement, INF, safe_zip, apply_mapping, str_from_object

# The motivation for immediately instantiating is to avoid unnecessary sampling
# Consider a stream result DAG A -> C, B -> C
# If A is already successfully sampled, don't want to resample A until B is sampled

def remove_blocked(evaluations, instance, new_results):
    if new_results and isinstance(instance, StreamInstance):
        evaluations.pop(evaluation_from_fact(instance.get_blocked_fact()), None)

##################################################

def _reenable_stream_plan(queue, stream_plan):
    # TODO: only disable if not used elsewhere
    # TODO: could just hash instances
    # TODO: do I actually need to reenable? Yes it ensures that
    # TODO: check if the index is the only one being sampled
    # for result in stream_plan:
    #    result.instance.disabled = False
    stream_plan[0].instance.enable(queue.evaluations, queue.domain)
    # TODO: move functions as far forward as possible to prune these plans
    # TODO: make function evaluations low success as soon as finite cost

##################################################

def history_slice(instance, start=0):
    new_results = []
    for call_index in range(start, instance.num_calls):
        new_results.extend(instance.results_history[call_index])
    return new_results

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

class Skeleton(object):
    def __init__(self, stream_plan, action_plan, cost):
        self.stream_plan = stream_plan
        self.action_plan = action_plan
        self.cost = cost
        #self.root
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
    def __init__(self, queue, skeleton, stream_indices, stream_attempts, bindings, cost):
        self.queue = queue
        self.skeleton = skeleton
        self.stream_indices = stream_indices
        self.stream_attempts = stream_attempts
        self.bindings = bindings
        self.cost = cost
        self._enumerated = False
        self._stream_plan = None
    @property
    def stream_plan(self):
        if self._stream_plan is None:
            self._stream_plan = self.skeleton.bind_stream_plan(self.bindings, self.stream_indices)
        return self._stream_plan
    def is_enumerated(self):
        if self._enumerated:
            return True
        self._enumerated = any(result.instance.enumerated and (result.instance.num_calls == attempt)
                               for result, attempt in zip(self.stream_plan, self.stream_attempts))
        return self._enumerated
    def get_priority(self):
        # Infinite cost if skeleton is exhausted
        # Attempted is equivalent to whether any stream result is disabled
        attempted = sum(self.stream_attempts) != 0
        effort = compute_effort(self.stream_attempts)
        # TODO: lexicographic tiebreaking using plan cost
        return Priority(attempted, effort)
    #def get_key(self):
    #    return self.skeleton, tuple(self.stream_indices), frozenset(self.bindings.items())
    def _instantiate(self, index, new_result):
        if not new_result.is_successful():
            return # TODO: check if satisfies target certified
        opt_result = self.stream_plan[index]
        self.queue._new_binding(self.skeleton,
                                puncture(self.stream_indices, index),
                                puncture(self.stream_attempts, index),
                                update_bindings(self.bindings, opt_result, new_result),
                                update_cost(self.cost, opt_result, new_result))
    def update_instances(self):
        updated = False
        if self.is_enumerated():
            # TODO: delete binding from _instantiate_binding
            return updated
        for index, (opt_result, attempt) in enumerate(safe_zip(self.stream_plan, self.stream_attempts)):
            if opt_result.instance.num_calls == attempt:
                continue
            for new_result in history_slice(opt_result.instance, start=attempt):
                self._instantiate(index, new_result)
            self.stream_attempts[index] = opt_result.instance.num_calls
            updated = True
            # Would be nice to just check enumerated here
        return updated
    def __iter__(self):
        return iter((self.skeleton, self.stream_plan, self.stream_indices, self.stream_attempts,
                     self.bindings, self.cost))

##################################################

class SkeletonQueue(Sized):
    # TODO: handle this in a partially ordered way
    # TODO: alternatively store just preimage and reachieve
    # TODO: make an "action" for returning to the search (if it is the best decision)
    # TODO: could just maintain a list of active instances and sample/propagate
    # TODO: store bindings in a factored form that only combines when needed
    # TODO: update bindings given outcomes of eager streams
    # TODO: immediately evaluate eager streams in the queue

    def __init__(self, store, evaluations, goal_expression, domain):
        self.store = store
        self.evaluations = evaluations
        #self.goal_expression = goal_expression
        self.domain = domain
        self.skeletons = []
        self.queue = []
        self.binding_from_key = {}
        self.bindings_from_instance = defaultdict(set)

    ####################

    def flush(self):
        while self.queue:
            queue_priority, binding = self.queue[0]
            current_priority = binding.get_priority()
            if queue_priority == current_priority:
                return
            heapreplace(self.queue, HeapElement(current_priority, binding))

    def push(self, binding):
        if binding.is_enumerated():
            return
        heappush(self.queue, HeapElement(binding.get_priority(), binding))

    def pop(self):
        self.flush()
        _, binding = heappop(self.queue)
        return binding

    ####################

    def _new_binding(self, skeleton, stream_indices, stream_attempts, mapping, cost):
        assert len(stream_indices) == len(stream_attempts)
        # Each stream result is unique (affects hashing)
        key = (tuple(stream_indices), frozenset(mapping.items()), skeleton)
        if key in self.binding_from_key:
            print('Binding already visited!')
            return
        if (self.store.best_cost < INF) and (self.store.best_cost <= cost):
            # TODO: what should I do if the cost=inf (from incremental/exhaustive)
            # _reenable_stream_plan(queue, stream_plan)
            return
        if not stream_indices:
            # if is_solution(self.domain, self.evaluations, bound_plan, self.goal_expression):
            self.store.add_plan(skeleton.bind_action_plan(mapping), cost)
            return
        binding = Binding(self, skeleton, stream_indices, stream_attempts, mapping, cost)
        self.binding_from_key[key] = binding
        binding.update_instances()
        if binding.is_enumerated():
            return
        self.push(binding)
        for result in binding.stream_plan:
            self.bindings_from_instance[result.instance].add(binding)

    def new_skeleton(self, stream_plan, action_plan, cost):
        skeleton = Skeleton(stream_plan, action_plan, cost)
        self.skeletons.append(skeleton)
        stream_indices = list(range(len(stream_plan)))
        stream_attempts = [0]*len(stream_plan)
        mapping = {}
        self._new_binding(skeleton, stream_indices, stream_attempts, mapping, cost)

    ####################

    def _generate_results(self, instance, accelerate=1):
        # assert(instance.opt_index == 0)
        if not all(evaluation_from_fact(f) in self.evaluations for f in instance.get_domain()):
            raise RuntimeError(instance)
        if instance.enumerated:
            return False
        new_results, new_facts = instance.next_results(accelerate=accelerate, verbose=self.store.verbose)
        instance.disable(self.evaluations, self.domain)  # Disable only if actively sampled
        remove_blocked(self.evaluations, instance, new_results)
        for result in new_results:
            add_certified(self.evaluations, result)
        add_facts(self.evaluations, new_facts, result=None)  # TODO: record the instance
        for binding in self.bindings_from_instance[instance]:
            binding.update_instances()
        return bool(new_results) | bool(new_facts)

    def _process_root(self):
        is_new = False
        binding = self.pop()
        if not binding.update_instances():
            is_new = self._generate_results(binding.stream_plan[0].instance)
        # _decompose_synthesizer_skeleton(queue, skeleton, stream_index)
        if not binding.is_enumerated():
            self.push(binding)
        return is_new

    ####################

    def is_active(self):
        return self.queue and (not self.store.is_terminated())

    def greedily_process(self):
        while self.is_active():
            self.flush()
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

    def process(self, stream_plan, action_plan, cost, max_time=0):
        start_time = time.time()
        if stream_plan is None:
            if not self.process_until_new():
                return False
        else:
            #print([result for result in stream_plan if result.optimistic])
            #raw_input('New skeleton')
            self.new_skeleton(stream_plan, action_plan, cost)
            self.greedily_process()
        self.timed_process(max_time - elapsed_time(start_time))
        return True

    def __len__(self):
        return len(self.queue)

##################################################

# TODO: want to minimize number of new sequences as they induce overhead
# TODO: estimate how many times a stream needs to be queried (acceleration)

def compute_effort(stream_attempts):
    attempts = sum(stream_attempts)
    return attempts, len(stream_attempts)

##################################################

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
