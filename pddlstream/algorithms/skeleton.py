import time
from collections import defaultdict, namedtuple, Sized
from heapq import heappush, heappop
from itertools import product

from pddlstream.algorithms.algorithm import add_certified, add_facts
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.reorder import get_stream_stats
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.function import FunctionResult, PredicateResult
from pddlstream.language.statistics import geometric_cost
from pddlstream.language.stream import StreamResult, StreamInstance
from pddlstream.language.wild_stream import WildInstance
from pddlstream.language.synthesizer import SynthStreamResult
from pddlstream.utils import elapsed_time, HeapElement, INF

def get_stream_plan_index(stream_plan):
    if not stream_plan:
        return 0
    return max(r.opt_index for r in stream_plan)

def is_double_bound(stream_instance, double_bindings):
    if double_bindings is None:
        return True
    bindings = [double_bindings[o] for o in stream_instance.input_objects if o in double_bindings]
    return len(set(bindings)) != len(bindings)

def optimistic_process_streams(evaluations, streams, double_bindings=None):
    instantiator = Instantiator(evaluations, streams)
    stream_results = []
    while instantiator.stream_queue:
        stream_instance = instantiator.stream_queue.popleft()
        if not is_double_bound(stream_instance, double_bindings):
            continue
        for stream_result in stream_instance.next_optimistic():
            for fact in stream_result.get_certified():
                instantiator.add_atom(evaluation_from_fact(fact))
            stream_results.append(stream_result) # TODO: don't readd if all repeated facts?
    return stream_results

##################################################

def optimistic_stream_grounding(stream_instance, bindings, evaluations, opt_evaluations, immediate=False):
    # TODO: combination for domain predicates
    evaluation_set = set(evaluations)
    opt_instances = []
    input_objects = [bindings.get(i, [i]) for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = dict(zip(stream_instance.input_objects, combo))
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping))) # TODO: could just instantiate first
        if domain <= opt_evaluations:
            instance = stream_instance.external.get_instance(combo)
            if (instance.opt_index != 0) and (not immediate or (domain <= evaluation_set)):
                instance.opt_index -= 1
            opt_instances.append(instance)
    return opt_instances

def optimistic_process_stream_plan(evaluations, stream_plan):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    evaluations = set(evaluations)
    opt_evaluations = set(evaluations)
    opt_bindings = defaultdict(list)
    opt_results = []
    for opt_result in stream_plan:
        # TODO: could just do first step
        for instance in optimistic_stream_grounding(opt_result.instance, opt_bindings,
                                                    evaluations, opt_evaluations):
            results = instance.next_optimistic()
            opt_evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            opt_results += results
            for result in results:
                if isinstance(result, StreamResult): # Could not add if same value
                    for opt, obj in zip(opt_result.output_objects, result.output_objects):
                        opt_bindings[opt].append(obj)
    return opt_results, opt_bindings

##################################################

# TODO: can either entirely replace arguments on plan or just pass bindings
# TODO: handle this in a partially ordered way
# TODO: alternatively store just preimage and reachieve

def instantiate_plan(bindings, stream_plan, evaluations, domain):
    if not stream_plan:
        return []
    new_stream_plan = [result.remap_inputs(bindings) for result in stream_plan]
    new_stream_plan[0].instance.disable(evaluations, domain)
    return new_stream_plan

def process_stream_plan(skeleton, queue, accelerate=1):
    # TODO: hash combinations to prevent repeats
    stream_plan, plan_attempts, bindings, plan_index, cost = skeleton
    new_values = False
    if not stream_plan:
        action_plan = queue.skeleton_plans[plan_index].action_plan
        bound_plan = [(name, tuple(bindings.get(o, o) for o in args)) for name, args in action_plan]
        queue.store.add_plan(bound_plan, cost)
        return new_values
    if (queue.store.best_cost < INF) and (queue.store.best_cost <= cost):
        # TODO: what should I do if the cost=inf (from incremental/exhaustive)
        #for result in stream_plan:
        #    result.instance.disabled = False
        stream_plan[0].instance.disabled = False
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
        assert (not any(evaluation_from_fact(f) not in queue.evaluations for f in instance.get_domain()))
        new_results = instance.next_results(accelerate=accelerate, verbose=queue.store.verbose)
        if new_results and isinstance(instance, StreamInstance):
            queue.evaluations.pop(evaluation_from_fact(instance.get_blocked_fact()), None)
        results.extend(new_results)
        new_values |= (len(results) != 0)
        if isinstance(instance, WildInstance):
            add_facts(queue.evaluations, instance.next_facts(verbose=queue.store.verbose), result=None) # TODO: use instance

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

def compute_effort(plan_attempts):
    attempts = sum(plan_attempts)
    return attempts, len(plan_attempts)

# TODO: want to minimize number of new sequences as they induce overhead

def compute_sampling_cost(stream_plan, stats_fn=get_stream_stats):
    # TODO: we are in a POMDP. If not the case, then the geometric cost policy is optimal
    if stream_plan is None:
        return INF
    expected_cost = 0
    for result in reversed(stream_plan):
        p_success, overhead = stats_fn(result)
        expected_cost += geometric_cost(overhead, p_success)
    return expected_cost
    # TODO: mix between geometric likelihood and learned distribution
    # Sum tail distribution for number of future
    # Distribution on the number of future attempts until successful
    # Average the tail probability mass

def compute_belief(attempts, p_obs):
    return pow(p_obs, attempts)

def compute_success_score(plan_attempts, p_obs=.9):
    beliefs = [compute_belief(attempts, p_obs) for attempts in plan_attempts]
    prior = 1.
    for belief in beliefs:
        prior *= belief
    return -prior

def compute_geometric_score(plan_attempts, overhead=1, p_obs=.9):
    # TODO: model the decrease in belief upon each failure
    # TODO: what if stream terminates? Assign high cost
    expected_cost = 0
    for attempts in plan_attempts:
        p_success = compute_belief(attempts, p_obs)
        expected_cost += geometric_cost(overhead, p_success)
    return expected_cost

##################################################

SkeletonKey = namedtuple('SkeletonKey', ['attempted', 'effort'])
Skeleton = namedtuple('Skeleton', ['stream_plan', 'plan_attempts',
                                   'bindings', 'plan_index', 'cost'])

SkeletonPlan = namedtuple('SkeletonPlan', ['stream_plan', 'action_plan', 'cost'])

class SkeletonQueue(Sized):
    def __init__(self, store, evaluations, domain):
        self.store = store
        self.evaluations = evaluations
        self.domain = domain
        self.queue = []
        self.skeleton_plans = []
        # TODO: include eager streams in the queue?
        # TODO: make an "action" for returning to the search (if it is the best decision)

    def add_skeleton(self, stream_plan, plan_attempts, bindings, plan_index, cost):
        stream_plan = instantiate_plan(bindings, stream_plan, self.evaluations, self.domain)
        attempted = sum(plan_attempts) != 0 # Bias towards unused
        effort = compute_effort(plan_attempts)
        key = SkeletonKey(attempted, effort)
        skeleton = Skeleton(stream_plan, plan_attempts, bindings, plan_index, cost)
        heappush(self.queue, HeapElement(key, skeleton))

    def new_skeleton(self, stream_plan, action_plan, cost):
        plan_index = len(self.skeleton_plans)
        self.skeleton_plans.append(SkeletonPlan(stream_plan, action_plan, cost))
        plan_attempts = [0]*len(stream_plan)
        self.add_skeleton(stream_plan, plan_attempts, {}, plan_index, cost)

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
            process_stream_plan(skeleton, self)

    def process_until_success(self):
        success = False
        while self.is_active() and (not success):
            _, skeleton = heappop(self.queue)
            success |= process_stream_plan(skeleton, self)
            # TODO: break if successful?
            self.greedily_process()

    def timed_process(self, max_time):
        start_time = time.time()
        while self.is_active() and (max_time <= elapsed_time(start_time)):
            _, skeleton = heappop(self.queue)
            process_stream_plan(skeleton, self)
            self.greedily_process()

    def __len__(self):
        return len(self.queue)