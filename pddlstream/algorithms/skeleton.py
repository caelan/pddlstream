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

def is_enumerated(binding):
    return any(result.instance.enumerated and (result.instance.num_calls == attempt)
               for result, attempt in zip(binding.stream_plan, binding.stream_attempts))

def remove_blocked(evaluations, instance, new_results):
    if new_results and isinstance(instance, StreamInstance):
        evaluations.pop(evaluation_from_fact(instance.get_blocked_fact()), None)

##################################################

def _bind_solution(queue, bindings, skeleton_index, cost):
    plan_skeleton = queue.skeleton_plans[skeleton_index].action_plan
    bound_plan = [(name, apply_mapping(args, bindings)) for name, args in plan_skeleton]
    # if is_solution(queue.domain, queue.evaluations, bound_plan, queue.goal_expression):
    queue.store.add_plan(bound_plan, cost)

def _reenable_stream_plan(queue, stream_plan):
    # TODO: only disable if not used elsewhere
    # TODO: could just hash instances
    # TODO: move functions as far forward as possible to prune these plans
    # TODO: do I actually need to reenable? Yes it ensures that
    # TODO: check if the index is the only one being sampled
    # for result in stream_plan:
    #    result.instance.disabled = False
    stream_plan[0].instance.enable(queue.evaluations, queue.domain)

##################################################

def _generate_new_results(queue, instance, accelerate=1):
    # assert(instance.opt_index == 0)
    if not all(evaluation_from_fact(f) in queue.evaluations for f in instance.get_domain()):
        raise RuntimeError(instance)
    if instance.enumerated:
        return False
    new_results, new_facts = instance.next_results(accelerate=accelerate, verbose=queue.store.verbose)
    instance.disable(queue.evaluations, queue.domain)  # Disable only if actively sampled
    remove_blocked(queue.evaluations, instance, new_results)
    for result in new_results:
        add_certified(queue.evaluations, result)
    add_facts(queue.evaluations, new_facts, result=None) # TODO: record the instance
    for skeleton, _ in queue.bindings_from_instance[instance]:
        _instantiate_binding(queue, skeleton)
    return bool(new_results) | bool(new_facts)

##################################################

def _update_bindings(bindings, opt_result, result):
    new_bindings = bindings.copy()
    if isinstance(result, StreamResult):
        for opt, obj in safe_zip(opt_result.output_objects, result.output_objects):
            assert (opt not in new_bindings)  # TODO: return failure if conflicting bindings
            new_bindings[opt] = obj
    return new_bindings

def _update_cost(cost, opt_result, result):
    new_cost = cost
    if type(result) is FunctionResult:
        new_cost += (result.value - opt_result.value)
    return new_cost

def puncture(sequence, index):
    return sequence[:index] + sequence[index+1:]

def _instantiate_binding(queue, binding):
    updated = False
    if is_enumerated(binding):
        return updated
    stream_plan, stream_indices, stream_attempts, bindings, skeleton_index, cost = binding
    for stream_index, (opt_result, attempt) in enumerate(safe_zip(stream_plan, stream_attempts)):
        if opt_result.instance.num_calls == attempt:
            continue
        new_results = []
        for call_index in range(attempt, opt_result.instance.num_calls):
            new_results.extend(opt_result.instance.results_history[call_index])
        for result in new_results:
            if result.is_successful():
                # TODO: check if satisfies target certified
                queue.new_binding(skeleton_index,
                                  puncture(stream_indices, stream_index),
                                  puncture(stream_attempts, stream_index),
                                  _update_bindings(bindings, opt_result, result),
                                  _update_cost(cost, opt_result, result))
        stream_attempts[stream_index] = opt_result.instance.num_calls
        updated = True
        # Would be nice to just check enumerated here
    return updated

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
#

##################################################

Skeleton = namedtuple('SkeletonPlan', ['stream_plan', 'action_plan', 'cost'])
Binding = namedtuple('Binding', ['stream_plan', 'stream_indices', 'stream_attempts',
                                 'bindings', 'skeleton_index', 'cost'])
BindingPriority = namedtuple('BindingPriority', ['attempted', 'effort'])

class SkeletonQueue(Sized):
    # TODO: handle this in a partially ordered way
    # TODO: alternatively store just preimage and reachieve
    # TODO: make an "action" for returning to the search (if it is the best decision)
    # TODO: could just maintain a list of active instances and sample/propagate
    # TODO: store bindings in a factored form that only combines when needed
    # TODO: update bindings given outcomes of eager streams
    # TODO: immediately evaluate eager streams in the queue
    # TODO: maintain a tree instead. Propagate the best subtree upwards

    def __init__(self, store, evaluations, goal_expression, domain):
        self.store = store
        self.evaluations = evaluations
        self.goal_expression = goal_expression
        self.domain = domain
        self.queue = []
        self.skeleton_plans = []
        self.binding_set = {}
        self.bindings_from_instance = defaultdict(list)

    def flush(self):
        while self.queue:
            old_priority, binding = self.queue[0]
            new_priority = compute_priority(binding)
            if old_priority == new_priority:
                return
            heapreplace(self.queue, HeapElement(new_priority, binding))

    def push(self, binding):
        if is_enumerated(binding):
            return
        priority = compute_priority(binding)
        heappush(self.queue, HeapElement(priority, binding))

    #def peek(self):
    #    self.flush()
    #    _, skeleton = self.queue[0]
    #    return skeleton

    def pop(self):
        self.flush()
        _, binding = heappop(self.queue)
        return binding

    def new_binding(self, skeleton_index, stream_indices, stream_attempts, mapping, cost):
        assert len(stream_indices) == len(stream_attempts)
        # Each stream result is unique (affects hashing)
        key = (tuple(stream_indices), frozenset(mapping.items()), skeleton_index)
        if key in self.binding_set:
            print('Binding already visited!')
            return
        if (self.store.best_cost < INF) and (self.store.best_cost <= cost):
            # TODO: what should I do if the cost=inf (from incremental/exhaustive)
            # _reenable_stream_plan(queue, stream_plan)
            return
        if not stream_indices:
            _bind_solution(self, mapping, skeleton_index, cost)
            return
        skeleton = self.skeleton_plans[skeleton_index]
        stream_plan = [skeleton.stream_plan[index].remap_inputs(mapping) for index in stream_indices]
        binding = Binding(stream_plan, stream_indices, stream_attempts, mapping, skeleton_index, cost)
        self.binding_set[key] = binding
        _instantiate_binding(self, binding)
        if is_enumerated(binding):
            return
        self.push(binding)
        for stream_index in range(len(stream_plan)):
            instance = stream_plan[stream_index].instance
            self.bindings_from_instance[instance].append((binding, stream_index))

    def new_skeleton(self, stream_plan, action_plan, cost):
        skeleton_index = len(self.skeleton_plans)
        self.skeleton_plans.append(Skeleton(stream_plan, action_plan, cost))
        stream_indices = list(range(len(stream_plan)))
        stream_attempts = [0]*len(stream_plan)
        mapping = {}
        self.new_binding(skeleton_index, stream_indices, stream_attempts, mapping, cost)

    ####################

    def is_active(self):
        return self.queue and (not self.store.is_terminated())

    def process_root(self):
        is_new = False
        binding = self.pop()
        if not _instantiate_binding(self, binding):
            opt_instance = binding.stream_plan[0].instance
            is_new = _generate_new_results(self, opt_instance)
        # _decompose_synthesizer_skeleton(queue, skeleton, stream_index)
        if not is_enumerated(binding):
            self.push(binding)
        return is_new

    def greedily_process(self):
        while self.is_active():
            self.flush()
            key, _ = self.queue[0]
            if key.attempted:
                break
            self.process_root()

    def process_until_new(self):
        # TODO: process the entire queue once instead
        is_new = False
        while self.is_active() and (not is_new):
            is_new |= self.process_root()
            self.greedily_process()

    def timed_process(self, max_time):
        start_time = time.time()
        while self.is_active() and (elapsed_time(start_time) <= max_time):
            self.process_root()
            self.greedily_process()
            # TODO: print cost updates when progress with a new skeleton

    ####################

    def process(self, stream_plan, action_plan, cost, max_time=INF):
        start_time = time.time()
        if stream_plan is None:
            if not self:
                return False
            self.process_until_new()
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

def compute_priority(binding):
    # Infinite cost if skeleton is exhausted
    # Attempted is equivalent to whether any stream result is disabled
    attempted = sum(binding.stream_attempts) != 0
    effort = compute_effort(binding.stream_attempts)
    # TODO: lexicographic tiebreak using plan cost
    return BindingPriority(attempted, effort)

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
