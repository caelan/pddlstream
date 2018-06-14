from collections import deque, Counter

from pddlstream.algorithms.reorder import get_partial_orders

from pddlstream.algorithms.algorithm import neighbors_from_orders
from pddlstream.conversion import substitute_expression, Minimize
from pddlstream.language.function import PredicateResult, FunctionResult
from pddlstream.language.stream import Stream, StreamInstance, StreamResult, StreamInfo
from pddlstream.statistics import Performance


class SynthStreamResult(StreamResult):
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
    _Instance = SynthStreamInstance
    _Result = SynthStreamResult
    def __init__(self, synthesizer, gen_fn, inputs, domain, outputs, certified, streams, macro_from_micro):
        info = None # TODO: stream info
        super(SynthStream, self).__init__(synthesizer.name, gen_fn, inputs, domain, outputs, certified, info)
        self.synthesizer = synthesizer
        self.streams = streams
        self.macro_from_micro = macro_from_micro
    def update_statistics(self, overhead, success):
        self.synthesizer.update_statistics(overhead, success)
    def get_p_success(self):
        return self.synthesizer.get_p_success()
    def get_overhead(self):
        return self.synthesizer.get_overhead()
    def get_effort(self):
        return self.synthesizer.get_effort()
    #def decompose(self):
    #    return self.streams

##################################################

class StreamSynthesizer(Performance): # JointStream | Stream Combiner
    macro_results = {}
    def __init__(self, name, streams, gen_fn):
        super(StreamSynthesizer, self).__init__(name, StreamInfo())
        self.name = name
        self.streams = {s.lower(): m for s, m in streams.items()}
        self.gen_fn = gen_fn
    def get_gen_fn(self, inputs, outputs, certified):
        # TODO: take in guess values for inputs?
        def new_gen_fn(*input_values):
            assert (len(inputs) == len(input_values))
            mapping = dict(zip(inputs, input_values))
            targets = substitute_expression(certified, mapping)
            return self.gen_fn(outputs, targets)  # TODO: could also return a map
        return new_gen_fn
    def get_synth_stream(self, stream_plan):
        key = frozenset(stream_plan)
        if key in self.macro_results:
            return self.macro_results[key]
        param_from_obj = {}
        macro_from_micro = []
        inputs, domain, outputs, certified = [], set(), [], set()
        input_objects, output_objects = [], []
        functions = set()
        streams = []
        for result in stream_plan:
            local_mapping = {}
            stream = result.instance.external
            for inp, input_object in zip(stream.inputs, result.instance.input_objects):
                # TODO: only do optimistic parameters?
                # if isinstance()
                if input_object not in param_from_obj:
                    param_from_obj[input_object] = '?i{}'.format(len(inputs))
                    inputs.append(param_from_obj[input_object])
                    input_objects.append(input_object)
                local_mapping[inp] = param_from_obj[input_object]
            domain.update(set(substitute_expression(stream.domain, local_mapping)) - certified)

            if isinstance(result, PredicateResult):
                # functions.append(Equal(stream.head, result.value))
                mapping = {inp: param_from_obj[inp] for inp in result.instance.input_objects}
                functions.update(substitute_expression(result.get_certified(), mapping))
            elif isinstance(result, FunctionResult):
                functions.add(substitute_expression(Minimize(stream.head), local_mapping))
            else:
                for out, output_object in zip(stream.outputs, result.output_objects):
                    if output_object not in param_from_obj:
                        param_from_obj[output_object] = '?o{}'.format(len(outputs))
                        outputs.append(param_from_obj[output_object])
                        output_objects.append(output_object)
                    local_mapping[out] = param_from_obj[output_object]
                certified.update(substitute_expression(stream.certified, local_mapping))
                streams.append(stream)
                macro_from_micro.append(local_mapping)

        gen_fn = self.get_gen_fn(inputs, outputs, certified | functions)
        mega_stream = SynthStream(self, gen_fn,
                                  inputs=tuple(inputs), domain=domain,
                                  outputs=tuple(outputs), certified=certified,
                                  streams=streams, macro_from_micro=macro_from_micro)
        mega_instance = mega_stream.get_instance(input_objects)
        self.macro_results[key] = SynthStreamResult(mega_instance, output_objects)
        return self.macro_results[key]
    def __repr__(self):
        return '{}{}'.format(self.name, self.streams)

# TODO: worthwhile noting that the focused algorithm does not search over all plan skeletons directly...

##################################################

def get_synthetic_stream_plan(stream_plan, synthesizers):
    if (stream_plan is None) or (not synthesizers):
        return stream_plan
    orders = get_partial_orders(stream_plan)
    for order in list(orders):
        orders.add(order[::-1])
    neighbors, _ = neighbors_from_orders(orders)
    # TODO: what if many possibilities?
    # TODO: cluster first and then plan using the macro and regular streams

    processed = set()
    new_stream_plan = []
    for v in stream_plan: # Processing in order is important
        if v in processed:
            continue
        processed.add(v)
        # TODO: assert that it has at least one thing in it

        for synthesizer in synthesizers:
            # TODO: something could be an input and output of a cut...
            if v.instance.external.name not in synthesizer.streams:
                continue
            # TODO: need to ensure all are covered I think?
            # TODO: don't do if no streams within

            cluster = {v}
            queue = deque([v])
            while queue:
                v1 = queue.popleft()
                for v2 in neighbors[v1]:
                    if (v2 not in processed) and (v2.instance.external.name in synthesizer.streams):
                        cluster.add(v2)
                        queue.append(v2)
                        processed.add(v2)
            counts = Counter(r.instance.external.name for r in cluster)
            if not all(n <= counts[name] for name, n in synthesizer.streams.items()):
                continue
            ordered_cluster = [r for r in stream_plan if r in cluster]
            new_stream_plan.append(synthesizer.get_synth_stream(ordered_cluster))
            new_stream_plan += filter(lambda s: isinstance(s, FunctionResult), ordered_cluster)
            break
        else:
            new_stream_plan.append(v)
    return new_stream_plan