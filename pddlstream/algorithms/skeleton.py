import time
from collections import defaultdict, namedtuple, Sized
from heapq import heappush, heappop
from itertools import product

from pddlstream.algorithms.algorithm import add_certified
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.reorder import get_stream_stats
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.function import FunctionResult, PredicateResult
from pddlstream.language.statistics import geometric_cost
from pddlstream.language.stream import StreamResult
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
        for instance in optimistic_stream_grounding(opt_result.instance, opt_bindings, evaluations, opt_evaluations):
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
# TODO: no point not doing all at once if unique
# TODO: alternatively store just preimage and reachieve

def instantiate_first(bindings, stream_plan):
    if not stream_plan:
        return None
    opt_result = stream_plan[0] # TODO: could do several at once but no real point
    input_objects = [bindings.get(i, i) for i in opt_result.instance.input_objects]
    instance = opt_result.instance.external.get_instance(input_objects)
    instance.disabled = True
    return instance

def instantiate_plan(bindings, stream_plan):
    new_stream_plan = []
    for result in stream_plan:
        input_objects = [bindings.get(i, i) for i in result.instance.input_objects]
        new_instance = result.instance.external.get_instance(input_objects)
        new_instance.disabled = True
        if isinstance(result, StreamResult):
            new_result = result.__class__(new_instance, result.output_objects, result.opt_index)
        elif isinstance(result, FunctionResult):
            new_result = result.__class__(new_instance, result.value, result.opt_index)
        else:
            raise ValueError(result)
            #new_result = result.__class__(new_instance, result.opt_index)
        new_stream_plan.append(new_result)
    return new_stream_plan

def process_stream_plan(skeleton, queue, accelerate=1):
    instance, num_processed, bindings, stream_plan, plan_index, cost = skeleton
    if not stream_plan:
        action_plan = queue.skeleton_plans[plan_index].action_plan
        bound_plan = [(name, tuple(bindings.get(o, o) for o in args)) for name, args in action_plan]
        queue.store.add_plan(bound_plan, cost)
        return
    if queue.store.best_cost <= cost:
        instance.disabled = False # TODO: only disable if not used elsewhere
        # TODO: could just hash instances
        return
    index = 0
    opt_result = stream_plan[index] # TODO: could do several at once but no real point
    assert(not any(evaluation_from_fact(f) not in queue.evaluations for f in instance.get_domain()))
    # TODO: hash combinations to prevent repeats

    results = []
    for i in range(num_processed, len(instance.results_history)):
        results.extend(instance.results_history[i])
    #accelerate = 25 # TODO: can compute this policy using the initial prior
    # TODO: need to apply this to plan skeletons
    results.extend(instance.next_results(accelerate=accelerate, verbose=queue.store.verbose))
    for result in results:
        add_certified(queue.evaluations, result)
        if (type(result) is PredicateResult) and (opt_result.value != result.value):
            continue # TODO: check if satisfies target certified
        new_bindings = bindings.copy()
        if isinstance(result, StreamResult):
            for opt, obj in zip(opt_result.output_objects, result.output_objects):
                assert(opt not in new_bindings) # TODO: return failure if conflicting bindings
                new_bindings[opt] = obj
        new_stream_plan = stream_plan[:index] + stream_plan[index+1:]
        new_cost = cost
        if type(result) is FunctionResult:
            new_cost += (result.value - opt_result.value)
        queue.add_skeleton(new_bindings, new_stream_plan, plan_index, new_cost)

    if (instance.num_calls == 1) and isinstance(opt_result, SynthStreamResult): # TODO: only add if failure?
        new_stream_plan = stream_plan[:index] + opt_result.decompose() + stream_plan[index+1:]
        queue.add_skeleton(bindings, new_stream_plan, plan_index, cost)
    if not instance.enumerated:
        queue.increment_skeleton(skeleton)

##################################################

def compute_sampling_cost(stream_plan, stats_fn=get_stream_stats):
    # TODO: we are in a POMDP. If not the case, then the geometric cost policy is optimal
    if stream_plan is None:
        return INF
    expected_cost = 0
    for result in reversed(stream_plan):
        p_success, overhead = stats_fn(result)
        expected_cost += geometric_cost(overhead, p_success)
    return expected_cost

def score_stream_plan(stream_plan):
    attempts = 0
    #for result in stream_plan:
    for result in stream_plan[:1]:
        attempts += len(result.instance.results_history)
    return SkeletonKey(attempts, len(stream_plan))


# TODO: identify streams that are bottlenecks (i.e. need to sample certainly)
# TODO: estimate sequence-level events (or gauge success on downstream events)
# TODO: don't sample from things that are satisfied downstream
# TODO: want to minimize number of new sequences as they induce overhead

SkeletonKey = namedtuple('SkeletonKey', ['attempts', 'length'])
Skeleton = namedtuple('Skeleton', ['instance', 'num_processed', 'bindings',
                                   'stream_plan', 'plan_index', 'cost'])

SkeletonPlan = namedtuple('Skeleton', ['stream_plan', 'action_plan', 'cost'])

ResultPair = namedtuple('ResultPair', ['result', 'num_calls'])
# TODO: I could just put the num pairs in the skeleton itself
# TODO: Need to start from zero

class SkeletonQueue(Sized):
    def __init__(self, store, evaluations):
        self.store = store
        self.evaluations = evaluations
        self.queue = []
        self.skeleton_plans = []

        # The bindings are only used when actually instantiating things
        # TODO: maybe automatically spawn all combinations of things resulting from a sampled value
        # No matter the actual underlying values, these are the same
        # TODO: the eager instantiating idea would handle the row of things
        # Choose the best in the set of possible options recursively

    def add_skeleton(self, bindings, stream_plan, plan_index, cost):
        stream_plan = instantiate_plan(bindings, stream_plan)
        key = SkeletonKey(0, len(stream_plan))
        instance = instantiate_first(bindings, stream_plan)
        skeleton = Skeleton(instance, 0, bindings, stream_plan, plan_index, cost)
        #skeleton = Skeleton(instance, len(instance.results_history), bindings, stream_plan, plan_index, cost)
        heappush(self.queue, HeapElement(key, skeleton))
        # TODO: could instantiate all combinations as soon as they started
        # TODO: could also make the first iteration check for exising values free
        # TODO: maybe I should be able to choose values from easiest to hardest (dynamic rescheduling of things)
        # Yeah, this way I could take advantage of new combinations easily
        #print(len(stream_plan), compute_sampling_cost(stream_plan))
        #print(stream_plan)

    def new_skeleton(self, stream_plan, action_plan, cost):
        #stream_index = 0
        plan_index = len(self.skeleton_plans)
        self.skeleton_plans.append(SkeletonPlan(stream_plan, action_plan, cost))
        #self.activated_indices.append(0)
        self.add_skeleton({}, stream_plan, plan_index, cost)

    def increment_skeleton(self, skeleton):
        # TODO: compute expected sampling effort required
        instance, num_processed, bindings, stream_plan, action_plan, cost = skeleton
        score = score_stream_plan(stream_plan)
        new_skeleton = Skeleton(instance, len(instance.results_history),
                               bindings, stream_plan, action_plan, cost)
        heappush(self.queue, HeapElement(score, new_skeleton))

    def greedily_process(self, max_time):
        # TODO: search until new disabled or new evaluation?
        start_time = time.time()
        while self.queue and (not self.store.is_terminated()):
            key, skeleton = self.queue[0]
            if (key.attempts != 0) and (max_time <= elapsed_time(start_time)):
                break
            heappop(self.queue)
            process_stream_plan(skeleton, self)

    def fairly_process(self):
        # TODO: max queue attempts?
        # TODO: use greedily process queue with some boost parameter to focus sampling
        old_queue = list(self.queue)
        self.queue[:] = []
        for _, skeleton in old_queue:
            if self.store.is_terminated():
                break
            #print('Attempts: {} | Length: {}'.format(key.attempts, key.length))
            process_stream_plan(skeleton, self)
            if self.store.is_terminated():
                break
            self.greedily_process(0)

    def __len__(self):
        return len(self.queue)