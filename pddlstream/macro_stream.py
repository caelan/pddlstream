from collections import deque, Counter

from pddlstream.conversion import substitute_expression, Minimize
from pddlstream.function import PredicateResult, FunctionResult
from pddlstream.reorder import get_partial_orders, neighbors_from_orders, topological_sort
from pddlstream.stream import Stream, StreamInstance, StreamResult, StreamInfo

# TODO: locally optimize a feasible plan skeleton (can be done with sampling as well)
# Extract backpointers to what achieved each used fact. Seed with current values

class MacroResult(StreamResult):
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

class MacroInstance(StreamInstance):
    pass
    #def decompose(self):
    #    return self.streams

class Macro(Stream):
    _Instance = MacroInstance
    _Result = MacroResult
    def __init__(self, name, gen_fn, inputs, domain, outputs, certified, streams, macro_from_micro):
        super(Macro, self).__init__(name, gen_fn, inputs, domain, outputs, certified)
        self.streams = streams
        self.macro_from_micro = macro_from_micro
    #def decompose(self):
    #    return self.streams
    # TODO: decomposition ability for each of these...

class StreamSynthesizer(object): # JointStream | Stream Combiner
    # TODO: load from a file as well
    macro_results = {}
    def __init__(self, name, streams, gen_fn):
        self.name = name
        self.streams = streams # Names of streams. Include counts in the future
        self.gen_fn = gen_fn
    def get_gen_fn(self, inputs, outputs, certified):
        # TODO: take in guess values for inputs? (i.e.
        def new_gen_fn(*input_values):
            assert (len(inputs) == len(input_values))
            mapping = dict(zip(inputs, input_values))
            targets = substitute_expression(certified, mapping)
            return self.gen_fn(outputs, targets)  # TODO: could also return a map
        return new_gen_fn
    def get_macro_result(self, cluster):
        key = frozenset(cluster)
        if key in self.macro_results:
            return self.macro_results[key]
        param_from_obj = {}
        macro_from_micro = []
        inputs, domain, outputs, certified = [], set(), [], set()
        input_objects, output_objects = [], []
        functions = set()
        streams = []
        for result in cluster:  # TODO: branch on sequences here
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
        mega_stream = Macro(self.name, gen_fn,
                             inputs=tuple(inputs), domain=domain,
                             outputs=tuple(outputs), certified=certified,
                             streams=streams, macro_from_micro=macro_from_micro)
        # mega_stream.info = StreamInfo() # TODO: stream info
        mega_instance = mega_stream.get_instance(input_objects)
        self.macro_results[key] = MacroResult(mega_instance, output_objects)
        return self.macro_results[key]

def get_macro_stream_plan(stream_plan, dynamic_streams):
    if (stream_plan is None) or not dynamic_streams:
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

        for dynamic in dynamic_streams:
            # TODO: something could be an input and output of a cut...
            if v.instance.external.name not in dynamic.streams:
                continue
            # TODO: need to ensure all are covered I think?
            # TODO: don't do if no streams within

            cluster = {v}
            queue = deque([v])
            while queue:
                v1 = queue.popleft()
                for v2 in neighbors[v1]:
                    if (v2 not in processed) and (v2.instance.external.name in dynamic.streams):
                        cluster.add(v2)
                        queue.append(v2)
                        processed.add(v2)
            counts = Counter(r.instance.external.name for r in cluster)
            if not all(n <= counts[name] for name, n in dynamic.streams.items()):
                continue
            ordered_cluster = [r for r in stream_plan if r in cluster]
            new_stream_plan.append(dynamic.get_macro_result(ordered_cluster))
            new_stream_plan += filter(lambda s: isinstance(s, FunctionResult), ordered_cluster)
            break
        else:
            new_stream_plan.append(v)
    return new_stream_plan