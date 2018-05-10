from collections import deque, Counter

from pddlstream.conversion import substitute_expression, Minimize
from pddlstream.function import PredicateResult, FunctionResult
from pddlstream.reorder import get_partial_orders, neighbors_from_orders, topological_sort
from pddlstream.stream import Stream, StreamResult, StreamInfo

# TODO: locally optimize a feasible plan skeleton (can be done with sampling as well)
# Extract backpointers to what achieved each used fact. Seed with current values

class DynamicStream(object): # JointStream | Stream Combiner
    # TODO: load from a file as well
    macro_results = {}
    def __init__(self, name, streams, gen_fn):
        self.name = name
        self.streams = streams # Names of streams. Include counts in the future
        self.gen_fn = gen_fn

class MacroResult(StreamResult):
    def __init__(self, instance, output_objects, child_results):
        super(MacroResult, self).__init__(instance, output_objects)
        self.child_results = child_results

def get_gen_fn(inputs, outputs, certified, gen_fn):
    # TODO: take in guess values for inputs? (i.e.
    def new_gen_fn(*input_values):
        assert(len(inputs) == len(input_values))
        mapping = dict(zip(inputs, input_values))
        targets = substitute_expression(certified, mapping)
        return gen_fn(outputs, targets)  # TODO: could also return a map
    return new_gen_fn

def get_macro_stream_result(dynamic, cluster):
    # TODO: cache the cluster and return it if possible
    param_from_obj = {}
    inputs = []
    domain = set()
    outputs = []
    certified = set()
    input_objects = []
    output_objects = []
    functions = set()
    # TODO: always have the option of not making a mega stream
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

    gen_fn = get_gen_fn(inputs, outputs, certified | functions, dynamic.gen_fn)
    mega_stream = Stream(dynamic.name, gen_fn,
                              inputs=tuple(inputs), domain=domain,
                              outputs=tuple(outputs), certified=certified)
    #mega_stream.info = StreamInfo() # TODO: stream info
    mega_instance = mega_stream.get_instance(input_objects)
    return MacroResult(mega_instance, output_objects, cluster)

def get_macro_stream_plan(stream_plan, dynamic_streams):
    if stream_plan is None:
        return None
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
            new_stream_plan.append(get_macro_stream_result(dynamic, ordered_cluster))
            new_stream_plan += filter(lambda s: isinstance(s, FunctionResult), ordered_cluster)
            break
        else:
            new_stream_plan.append(v)
    return new_stream_plan