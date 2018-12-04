import time
from collections import namedtuple, Sized
from heapq import heappush, heappop

from pddlstream.algorithms.algorithm import add_certified, add_facts
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult, StreamInstance
from pddlstream.language.synthesizer import SynthStreamResult
from pddlstream.utils import elapsed_time, HeapElement, INF
#from pddlstream.algorithms.downward import task_from_domain_problem, get_problem, get_action_instances, \
#    get_goal_instance, plan_preimage, is_valid_plan, substitute_derived, is_applicable, apply_action
#from pddlstream.algorithms.reorder import replace_derived
#from pddlstream.algorithms.scheduling.recover_axioms import extract_axiom_plan

# TODO: handle this in a partially ordered way
# TODO: alternatively store just preimage and reachieve

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

##################################################

def process_skeleton(skeleton, queue, accelerate=1):
    # TODO: hash combinations to prevent repeats
    stream_plan, plan_attempts, bindings, plan_index, cost = skeleton
    new_values = False
    is_wild = False
    if not stream_plan:
        action_plan = [(name, tuple(bindings.get(o, o) for o in args))
                      for name, args in queue.skeleton_plans[plan_index].action_plan]
        #if is_solution(queue.domain, queue.evaluations, action_plan, queue.goal_expression):
        queue.store.add_plan(action_plan, cost)
        return new_values

    if (queue.store.best_cost < INF) and (queue.store.best_cost <= cost):
        # TODO: what should I do if the cost=inf (from incremental/exhaustive)
        #for result in stream_plan:
        #    result.instance.disabled = False
        stream_plan[0].instance.enable(queue.evaluations, queue.domain)
        # TODO: only disable if not used elsewhere
        # TODO: could just hash instances
        return new_values

    assert(len(stream_plan) == len(plan_attempts))
    results = []
    index = None
    for i, (result, attempt) in enumerate(zip(stream_plan, plan_attempts)):
        if result.instance.num_calls != attempt:
            for j in range(attempt, result.instance.num_calls):
                results.extend(result.instance.results_history[j])
            index = i
            break

    if index is None:
        index = 0
        instance = stream_plan[index].instance
        #assert(instance.opt_index == 0)
        if not all(evaluation_from_fact(f) in queue.evaluations for f in instance.get_domain()):
            raise RuntimeError(instance)
        new_results, new_facts = instance.next_results(accelerate=accelerate, verbose=queue.store.verbose)
        instance.disable(queue.evaluations, queue.domain) # Disable only if actively sampled
        if new_results and isinstance(instance, StreamInstance):
            queue.evaluations.pop(evaluation_from_fact(instance.get_blocked_fact()), None)
        results.extend(new_results)
        new_values |= bool(results)
        is_wild |= bool(add_facts(queue.evaluations, new_facts, result=None)) # TODO: use instance
        #new_values |= is_wild

    opt_result = stream_plan[index] # TODO: could do several at once but no real point
    for result in results:
        add_certified(queue.evaluations, result)
        #if (type(result) is PredicateResult) and (opt_result.value != result.value):
        if not result.is_successful():
            continue # TODO: check if satisfies target certified
        new_bindings = bindings.copy()
        new_stream_plan =  stream_plan[:index] + stream_plan[index+1:]
        new_plan_attempts = plan_attempts[:index] + plan_attempts[index+1:]
        if isinstance(result, StreamResult):
            for opt, obj in zip(opt_result.output_objects, result.output_objects):
                assert(opt not in new_bindings) # TODO: return failure if conflicting bindings
                new_bindings[opt] = obj
        new_cost = cost
        if type(result) is FunctionResult:
            new_cost += (result.value - opt_result.value)
        queue.add_skeleton(new_stream_plan, new_plan_attempts, new_bindings, plan_index, new_cost)

    if (plan_attempts[index] == 0) and isinstance(opt_result, SynthStreamResult): # TODO: only add if failure?
        decomposition = opt_result.decompose()
        new_stream_plan = stream_plan[:index] + decomposition + stream_plan[index+1:]
        new_plan_attempts = plan_attempts[:index] + [0]*len(decomposition) + plan_attempts[index+1:]
        queue.add_skeleton(new_stream_plan, new_plan_attempts, bindings, plan_index, cost)
    if not opt_result.instance.enumerated:
        plan_attempts[index] = opt_result.instance.num_calls
        queue.add_skeleton(*skeleton)
    return new_values

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

SkeletonKey = namedtuple('SkeletonKey', ['attempted', 'effort'])
Skeleton = namedtuple('Skeleton', ['stream_plan', 'plan_attempts',
                                   'bindings', 'plan_index', 'cost'])

SkeletonPlan = namedtuple('SkeletonPlan', ['stream_plan', 'action_plan', 'cost'])

class SkeletonQueue(Sized):
    # TODO: hash existing plan skeletons to prevent the same
    def __init__(self, store, evaluations, goal_expression, domain):
        self.store = store
        self.evaluations = evaluations
        self.goal_expression = goal_expression
        self.domain = domain
        self.queue = []
        self.skeleton_plans = []
        # TODO: include eager streams in the queue?
        # TODO: make an "action" for returning to the search (if it is the best decision)

    def add_skeleton(self, stream_plan, plan_attempts, bindings, plan_index, cost):
        stream_plan = [result.remap_inputs(bindings) for result in stream_plan]
        attempted = sum(plan_attempts) != 0 # Bias towards unused
        effort = compute_effort(plan_attempts)
        key = SkeletonKey(attempted, effort)
        skeleton = Skeleton(stream_plan, plan_attempts, bindings, plan_index, cost)
        heappush(self.queue, HeapElement(key, skeleton))

    def new_skeleton(self, stream_plan, action_plan, cost):
        # TODO: iteratively recompute full plan skeletons
        plan_index = len(self.skeleton_plans)
        self.skeleton_plans.append(SkeletonPlan(stream_plan, action_plan, cost))
        plan_attempts = [0]*len(stream_plan)
        self.add_skeleton(stream_plan, plan_attempts, {}, plan_index, cost)
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
            process_skeleton(skeleton, self)

    def process_until_success(self):
        success = False
        while self.is_active() and (not success):
            _, skeleton = heappop(self.queue)
            success |= process_skeleton(skeleton, self)
            # TODO: break if successful?
            self.greedily_process()

    def timed_process(self, max_time):
        start_time = time.time()
        while self.is_active() and (elapsed_time(start_time) <= max_time):
            _, skeleton = heappop(self.queue)
            process_skeleton(skeleton, self)
            self.greedily_process()

    def __len__(self):
        return len(self.queue)

##################################################

def process_instance(evaluations, instance, verbose=True):
    success = False
    if instance.enumerated:
        return success
    new_results, new_facts = instance.next_results(verbose=verbose)
    for result in new_results:
        success |= bool(add_certified(evaluations, result))
    bool(add_facts(evaluations, new_facts, result=None))
    return success

def process_stream_plan(evaluations, domain, stream_plan, disabled, max_failures=INF, **kwargs):
    # TODO: had old implementation of these
    # TODO: could do version that allows bindings and is able to return
    # effort_weight is None # Keep in a queue
    failures = 0
    for result in stream_plan:
        if max_failures < failures:
            break
        instance = result.instance
        assert not instance.enumerated
        if all(evaluation_from_fact(f) in evaluations for f in instance.get_domain()):
            failures += not process_instance(evaluations, instance, **kwargs)
            instance.disable(evaluations, domain)
            if not instance.enumerated:
                disabled.add(instance)
    # TODO: indicate whether should resolve w/o disabled
    return not failures

def process_skeleton_queue(store, queue, stream_plan, action_plan, cost, max_sample_time):
    start_time = time.time()
    if stream_plan is None:
        if not queue:
            return False
        queue.process_until_success()
    elif not stream_plan:
        store.add_plan(action_plan, cost)
    else:
        queue.new_skeleton(stream_plan, action_plan, cost)
    queue.timed_process(max_sample_time - elapsed_time(start_time))
    return True