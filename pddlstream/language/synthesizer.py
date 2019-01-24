from collections import deque, Counter

from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.language.constants import is_plan
from pddlstream.language.conversion import substitute_expression
from pddlstream.language.function import FunctionResult
from pddlstream.language.optimizer import get_cluster_values
from pddlstream.language.statistics import Performance
from pddlstream.language.stream import Stream, StreamInstance, StreamResult, StreamInfo
from pddlstream.utils import neighbors_from_orders


def decompose_stream_plan(stream_plan):
    if not is_plan(stream_plan):
        return stream_plan
    new_stream_plan = []
    for result in stream_plan:
        if isinstance(result, SynthStreamResult):
            new_stream_plan.extend(result.decompose())
        else:
            new_stream_plan.append(result)
    return new_stream_plan

class SynthStreamResult(StreamResult):
    def get_functions(self):
        return substitute_expression(self.instance.external.functions, self.get_mapping())
    def decompose(self):
        mapping = self.get_mapping()
        results = []
        for i, stream in enumerate(self.instance.external.streams):
            macro_from_micro = self.instance.external.macro_from_micro[i]
            input_objects = tuple(mapping[macro_from_micro[inp]] for inp in stream.inputs)
            instance = stream.get_instance(input_objects)
            output_objects = tuple(mapping[macro_from_micro[out]] for out in stream.outputs)
            results.append(StreamResult(instance, output_objects))
        return results

class SynthStreamInstance(StreamInstance):
    pass
    #def decompose(self):
    #    return self.streams

class SynthStream(Stream):
    # TODO: wild stream optimizer
    _Instance = SynthStreamInstance
    _Result = SynthStreamResult
    def __init__(self, synthesizer, inputs, domain, outputs, certified, functions,
                 streams, macro_from_micro):
        def gen_fn(*input_values): # TODO: take in guess values for inputs?
            assert (len(inputs) == len(input_values))
            mapping = dict(zip(inputs, input_values))
            targets = substitute_expression(certified | functions, mapping)
            return synthesizer.gen_fn(outputs, targets) # TODO: could also return a map
        #info = None # TODO: stream info
        info = StreamInfo() # TODO: use StreamSynthesizer?
        super(SynthStream, self).__init__(synthesizer.name, gen_fn, inputs, domain, outputs, certified, info)
        self.synthesizer = synthesizer
        self.streams = streams
        self.functions = tuple(functions)
        self.macro_from_micro = macro_from_micro
    def update_statistics(self, overhead, success):
        self.synthesizer.update_statistics(overhead, success)
    def get_p_success(self):
        return self.synthesizer.get_p_success()
    def get_overhead(self):
        return self.synthesizer.get_overhead()
    #def decompose(self):
    #    return self.streams

##################################################


class StreamSynthesizer(Performance): # JointStream | Stream Combiner
    def __init__(self, name, streams, gen_fn, post_only=False):
        super(StreamSynthesizer, self).__init__(name, StreamInfo())
        self.name = name
        self.streams = {s.lower(): m for s, m in streams.items()}
        self.gen_fn = gen_fn
        self.macro_results = {}
        self.post_only = post_only
    #def get_instances(self):
    #    raise NotImplementedError()
    def get_synth_stream(self, stream_plan):
        key = frozenset(stream_plan)
        if key in self.macro_results:
            return self.macro_results[key]
        streams = list(filter(lambda r: isinstance(r, StreamResult), stream_plan))
        if len(streams) < 1: # No point if only one...
            return None
        inputs, domain, outputs, certified, functions, macro_from_micro, \
            input_objects, output_objects, fluent_facts = get_cluster_values(stream_plan)
        if fluent_facts:
            raise NotImplementedError()
        mega_stream = SynthStream(self, inputs, domain,
                                  outputs, certified, functions,
                                  streams, macro_from_micro)
        mega_instance = mega_stream.get_instance(input_objects)
        self.macro_results[key] = SynthStreamResult(mega_instance, output_objects)
        return self.macro_results[key]
    def __repr__(self):
        return '{}{}'.format(self.name, self.streams)

# TODO: worthwhile noting that the focused algorithm does not search over all plan skeletons directly...

##################################################

# TODO: factor this into algorithms

# TODO: 
# 1) Iteratively resolve for the next stream plan to apply rather than do in sequence
# 2) Apply to a constraint network specifcation
# 3) Satisfy a constraint network were free variables aren't given by streams
# 4) Allow algorithms to return not feasible to combine rather than impose minimums
# 5) Make a method (rather than a spec) that traverses the constraint graph and prunes weak links/constraints that can't be planned 
# 6) Post process all feasible skeletons at once
# 7) Planning and execution view of the algorithm
# 8) Algorithm that does the matching of streams to variables
# 9) Add implied facts (e.g. types) to the constraint network as preconditons


def expand_cluster(synthesizer, v, neighbors, processed):
    cluster = {v}
    queue = deque([v])
    while queue:
        v1 = queue.popleft()
        for v2 in neighbors[v1]:
            if (v2 not in processed) and (v2.instance.external.name in synthesizer.streams):
                cluster.add(v2)
                queue.append(v2)
                processed.add(v2)
    return cluster

def get_synthetic_stream_plan(stream_plan, synthesizers):
    # TODO: fix this implementation of this to be as follows:
    # 1) Prune graph not related
    # 2) Cluster
    # 3) Try combinations of replacing on stream plan
    if not is_plan(stream_plan) or (not synthesizers):
        return stream_plan
    orders = get_partial_orders(stream_plan)
    for order in list(orders):
        orders.add(order[::-1])
    neighbors, _ = neighbors_from_orders(orders)
    # TODO: what if many possibilities?
    # TODO: cluster first and then plan using the macro and regular streams

    processed = set()
    new_stream_plan = []
    for result in stream_plan: # Processing in order is important
        if result in processed:
            continue
        processed.add(result)
        # TODO: assert that it has at least one thing in it
        for synthesizer in synthesizers:
            # TODO: something could be an input and output of a cut...
            if result.instance.external.name not in synthesizer.streams:
                continue
            # TODO: need to ensure all are covered I think?
            # TODO: don't do if no streams within
            cluster = expand_cluster(synthesizer, result, neighbors, processed)
            counts = Counter(r.instance.external.name for r in cluster)
            if not all(n <= counts[name] for name, n in synthesizer.streams.items()):
                continue
            ordered_cluster = [r for r in stream_plan if r in cluster]
            synthesizer_result = synthesizer.get_synth_stream(ordered_cluster)
            if synthesizer_result is None:
                continue
            new_stream_plan.append(synthesizer_result)
            new_stream_plan += filter(lambda s: isinstance(s, FunctionResult), ordered_cluster)
            break
        else:
            new_stream_plan.append(result)
    return new_stream_plan

##################################################

"""
def get_synthetic_stream_plan2(stream_plan, synthesizers):
    # TODO: pass subgoals along the plan in directly
    # TODO: could just do this on the objects themselves to start
    free_parameters = set()
    for result in stream_plan:
        if isinstance(result, StreamResult):
            free_parameters.update(result.output_objects)
    print(free_parameters)

    # TODO: greedy method first
    new_plan = []
    facts = set()
    while True:
        candidates = []
        for result in stream_plan:
            if result.instance.get_domain() <= facts:
                candidates.append(result)
        selection = candidates[-1]
        new_plan.append(selection)
    print(new_plan)
    print(stream_plan)
    print(synthesizers)
    raise NotImplementedError()
"""