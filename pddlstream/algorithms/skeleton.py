import time
from collections import namedtuple, Sized
from heapq import heappush, heappop

from pddlstream.algorithms.algorithm import add_certified, add_facts
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult, StreamInstance
from pddlstream.language.synthesizer import SynthStreamResult
from pddlstream.utils import elapsed_time, HeapElement, INF, safe_zip


def _bind_solution(queue, bindings, skeleton_index, cost):
    action_plan = [(name, tuple(bindings.get(o, o) for o in args))
                   for name, args in queue.skeleton_plans[skeleton_index].action_plan]
    # if is_solution(queue.domain, queue.evaluations, action_plan, queue.goal_expression):
    queue.store.add_plan(action_plan, cost)

def _reenable_stream_plan(queue, stream_plan):
    # TODO: what should I do if the cost=inf (from incremental/exhaustive)
    # for result in stream_plan:
    #    result.instance.disabled = False
    stream_plan[0].instance.enable(queue.evaluations, queue.domain)
    # TODO: only disable if not used elsewhere
    # TODO: could just hash instances

##################################################

def _add_existing_results(stream_plan, plan_attempts, results):
    for stream_index, (result, attempt) in enumerate(safe_zip(stream_plan, plan_attempts)):
        if result.instance.num_calls != attempt:
            for call_index in range(attempt, result.instance.num_calls):
                results.extend(result.instance.results_history[call_index])
            return stream_index
    return None

def _query_new_results(queue, instance, results, accelerate=1):
    # assert(instance.opt_index == 0)
    if not all(evaluation_from_fact(f) in queue.evaluations for f in instance.get_domain()):
        raise RuntimeError(instance)
    new_results, new_facts = instance.next_results(accelerate=accelerate, verbose=queue.store.verbose)
    instance.disable(queue.evaluations, queue.domain)  # Disable only if actively sampled
    if new_results and isinstance(instance, StreamInstance):
        queue.evaluations.pop(evaluation_from_fact(instance.get_blocked_fact()), None)
    results.extend(new_results)
    is_wild = bool(add_facts(queue.evaluations, new_facts, result=None))  # TODO: use instance
    return is_wild

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

def _bind_new_skeletons(queue, skeleton, index, results):
    stream_plan, plan_attempts, bindings, skeleton_index, cost = skeleton
    opt_result = stream_plan[index] # TODO: could do several at once but no real point
    for result in results:
        add_certified(queue.evaluations, result)
        #if (type(result) is PredicateResult) and (opt_result.value != result.value):
        if not result.is_successful():
            continue # TODO: check if satisfies target certified
        new_stream_plan =  stream_plan[:index] + stream_plan[index+1:]
        new_plan_attempts = plan_attempts[:index] + plan_attempts[index+1:]
        new_bindings = _update_bindings(bindings, opt_result, result)
        new_cost = _update_cost(cost, opt_result, result)
        queue.add_binding(new_stream_plan, new_plan_attempts, new_bindings, skeleton_index, new_cost)

def _decompose_synthesizer_skeleton(queue, skeleton, index):
    stream_plan, plan_attempts, bindings, plan_index, cost = skeleton
    opt_result = stream_plan[index]
    if (plan_attempts[index] == 0) and isinstance(opt_result, SynthStreamResult):
        # TODO: only decompose if failure?
        decomposition = opt_result.decompose()
        new_stream_plan = stream_plan[:index] + decomposition + stream_plan[index+1:]
        new_plan_attempts = plan_attempts[:index] + [0]*len(decomposition) + plan_attempts[index+1:]
        queue.add_binding(new_stream_plan, new_plan_attempts, bindings, plan_index, cost)

def process_skeleton(queue, skeleton):
    # TODO: hash combinations to prevent repeats
    stream_plan, plan_attempts, bindings, skeleton_index, cost = skeleton
    is_new, is_wild = False, False
    if not stream_plan:
        _bind_solution(queue, bindings, skeleton_index, cost)
        return is_new
    if (queue.store.best_cost < INF) and (queue.store.best_cost <= cost):
        _reenable_stream_plan(queue, stream_plan)
        return is_new

    new_results = []
    stream_index = _add_existing_results(stream_plan, plan_attempts, new_results)
    if stream_index is None:
        stream_index = 0
        opt_instance = stream_plan[stream_index].instance
        is_wild |= _query_new_results(queue, opt_instance, new_results)
        is_new |= bool(new_results) # | is_wild
    opt_result = stream_plan[stream_index]
    plan_attempts[stream_index] = opt_result.instance.num_calls

    _bind_new_skeletons(queue, skeleton, stream_index, new_results)
    _decompose_synthesizer_skeleton(queue, skeleton, stream_index)
    if not opt_result.instance.enumerated:
        queue.add_binding(*skeleton)
    return is_new

##################################################

BindingKey = namedtuple('BindingKey', ['attempted', 'effort'])
Binding = namedtuple('Binding', ['stream_plan', 'plan_attempts', 'bindings', 'plan_index', 'cost'])
Skeleton = namedtuple('SkeletonPlan', ['stream_plan', 'action_plan', 'cost'])

class SkeletonQueue(Sized):
    # TODO: handle this in a partially ordered way
    # TODO: alternatively store just preimage and reachieve
    # TODO: hash existing plan skeletons to prevent repeats
    def __init__(self, store, evaluations, goal_expression, domain):
        self.store = store
        self.evaluations = evaluations
        self.goal_expression = goal_expression
        self.domain = domain
        self.queue = []
        self.skeleton_plans = []
        # TODO: include eager streams in the queue?
        # TODO: make an "action" for returning to the search (if it is the best decision)

    def add_binding(self, stream_plan, plan_attempts, bindings, plan_index, cost):
        stream_plan = [result.remap_inputs(bindings) for result in stream_plan]
        attempted = sum(plan_attempts) != 0 # Bias towards unused
        effort = compute_effort(plan_attempts)
        key = BindingKey(attempted, effort)
        skeleton = Binding(stream_plan, plan_attempts, bindings, plan_index, cost)
        heappush(self.queue, HeapElement(key, skeleton))

    def new_skeleton(self, stream_plan, action_plan, cost):
        # TODO: iteratively recompute full plan skeletons
        skeleton_index = len(self.skeleton_plans)
        self.skeleton_plans.append(Skeleton(stream_plan, action_plan, cost))
        plan_attempts = [0]*len(stream_plan)
        self.add_binding(stream_plan, plan_attempts, {}, skeleton_index, cost)
        self.greedily_process()

    ####################

    def is_active(self):
        return self.queue and (not self.store.is_terminated())

    def greedily_process(self):
        # TODO: search until new disabled or new evaluation?
        while self.is_active():
            key, _ = self.queue[0]
            if key.attempted:
                break
            _, skeleton = heappop(self.queue)
            process_skeleton(self, skeleton)

    def process_until_new(self):
        is_new = False
        while self.is_active() and (not is_new):
            _, skeleton = heappop(self.queue)
            is_new |= process_skeleton(self, skeleton)
            self.greedily_process()

    def timed_process(self, max_time):
        start_time = time.time()
        while self.is_active() and (elapsed_time(start_time) <= max_time):
            _, skeleton = heappop(self.queue)
            process_skeleton(self, skeleton)
            self.greedily_process()

    ####################

    def process(self, stream_plan, action_plan, cost, max_time=INF):
        start_time = time.time()
        if stream_plan is None:
            if not self:
                return False
            self.process_until_new()
        elif not stream_plan:
            self.store.add_plan(action_plan, cost)
        else:
            self.new_skeleton(stream_plan, action_plan, cost)
        self.timed_process(max_time - elapsed_time(start_time))
        return True

    def __len__(self):
        return len(self.queue)

##################################################

# TODO: want to minimize number of new sequences as they induce overhead
# TODO: estimate how many times a stream needs to be queried (acceleration)

def compute_effort(plan_attempts):
    attempts = sum(plan_attempts)
    return attempts, len(plan_attempts)

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
