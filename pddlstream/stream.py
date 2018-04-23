from pddlstream.conversion import list_from_conjunction, objects_from_values, substitute_expression
from pddlstream.fast_downward import parse_lisp
from pddlstream.object import OptimisticObject
from pddlstream.utils import str_from_tuple


def from_list_gen_fn(list_gen_fn):
    return list_gen_fn


def from_gen_fn(gen_fn):
    return lambda *args: ([output_values] for output_values in gen_fn(*args))


def from_list_fn(list_fn):
    return lambda *args: iter([list_fn(*args)])


def from_fn(fn):
    def list_fn(*args):
        outputs = fn(*args)
        if outputs is None:
            return []
        return [outputs]
    return from_list_fn(list_fn)


def from_test(test):
    return from_fn(lambda *args: tuple() if test(*args) else None)

def from_rule():
    return True

##################################################


class Stream(object):
    def __init__(self, name, gen_fn, inputs, domain, outputs, certified):
        self.name = name
        self.gen_fn = gen_fn
        self.inputs = tuple(inputs)
        self.domain = tuple(domain)
        self.outputs = tuple(outputs)
        self.certified = tuple(certified)
        self.instances = {}
    def get_instance(self, input_values):
        input_values = tuple(input_values)
        if input_values not in self.instances:
            self.instances[input_values] = StreamInstance(self, input_values)
        return self.instances[input_values]
    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)


class StreamInstance(object):
    def __init__(self, stream, input_values):
        self.stream = stream
        self.input_values = tuple(input_values)
        self._generator = None
        self.enumerated = False
        self.disabled = False
    def get_mapping(self):
        return dict(zip(self.stream.inputs, self.input_values))
    def get_domain(self):
        return substitute_expression(self.stream.domain, self.get_mapping())
    def next_outputs(self):
        assert not self.enumerated
        if self._generator is None:
            self._generator = self.stream.gen_fn(*(iv.value for iv in self.input_values))
        try:
            return list(map(objects_from_values, next(self._generator)))
        except StopIteration:
            self.enumerated = True
        return []
    def next_optimistic(self):
        if self.enumerated or self.disabled:
            return []
        opt_values = tuple(OptimisticObject.from_inputs(self, i) for i in
                           range(len(self.stream.outputs)))
        return [opt_values]
    def __repr__(self):
        return '{}:{}->{}'.format(self.stream.name, self.input_values, self.stream.outputs)

class StreamResult(object):
    def __init__(self, stream_instance, output_values):
        self.stream_instance = stream_instance
        self.output_values = output_values
    def get_mapping(self):
        return dict(list(zip(self.stream_instance.stream.inputs, self.stream_instance.input_values)) +
                    list(zip(self.stream_instance.stream.outputs, self.output_values)))
    def get_certified(self):
        return substitute_expression(self.stream_instance.stream.certified,
                                     self.get_mapping())
    def __repr__(self):
        return '{}:{}->{}'.format(self.stream_instance.stream.name,
                                  str_from_tuple(self.stream_instance.input_values),
                                  str_from_tuple(self.output_values))

##################################################

STREAM_ATTRIBUTES = [':stream', ':inputs', ':domain', ':outputs', ':certified']

def parse_stream_pddl(stream_pddl, stream_map):
    streams = []
    if stream_pddl is None:
        return streams
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, stream_name = next(stream_iter)
    assert('stream' == pddl_type)

    for stream in stream_iter:
        if stream[0] == ':stream':
            attributes = [stream[i] for i in range(0, len(stream), 2)]
            assert(STREAM_ATTRIBUTES == attributes)
            name, inputs, domain, outputs, certified = [stream[i] for i in range(1, len(stream), 2)]
            if name not in stream_map:
                raise ValueError('Undefined stream conditional generator: {}'.format(name))
            streams.append(Stream(name, stream_map[name],
                         tuple(inputs), list_from_conjunction(domain),
                         tuple(outputs), list_from_conjunction(certified)))
        elif stream[0] == ':rule':
            # TODO: implement rules
            # TODO: add eager stream if multiple conditions otherwise apply and add to stream effects
            pass
        else:
            raise ValueError(stream[0])
    return streams

# class Generator(object):
#     # TODO: could also do one that doesn't have state
#     # TODO: could make a function that returns a generator as well
#     def __init__(self, *inputs):
#         self.inputs = tuple(inputs)
#         self.calls = 0
#         self.enumerated = False
#     #def __iter__(self):
#     #def __call__(self, *args, **kwargs):
#     def generate(self, context=None):
#         # TODO: could replace with current values for things
#         raise NotImplementedError()
#         #raise StopIteration()
#     # TODO: count calls and max_calls?
# if self.stream.max_calls <= self.calls:
#    self.enumerated = True

# TODO: could even parse a stream like an action to some degree
# TODO: constant map?
# TODO: should each be a list or a string
