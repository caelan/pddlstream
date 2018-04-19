from collections import namedtuple

from pddlstream.conversion import list_from_conjunction
from pddlstream.fast_downward import parse_lisp


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

# TODO: denote rule by just treu
STREAM_ATTRIBUTES = [':stream', ':inputs', ':domain', ':outputs', ':certified']
Stream = namedtuple('Stream', ['name', 'gen_fn', 'inputs', 'domain', 'outputs', 'certified'])


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
        self.calls = 0
    def mapping(self, output_values=None):
        pairs = zip(self.stream.inputs, self.input_values)
        if output_values is not None:
            assert(len(self.stream.outputs) == len(output_values))
            pairs += zip(self.stream.outputs, output_values)
        return dict(pairs)
    def next_outputs(self):
        assert not self.enumerated
        if self._generator is None:
            self._generator = self.stream.gen_fn(*(iv.value for iv in self.input_values))
        self.calls += 1
        #if self.stream.max_calls <= self.calls:
        #    self.enumerated = True
        try:
            return next(self._generator)
        except StopIteration:
            self.enumerated = True
        return []
    def __repr__(self):
        return '{}:{}->{}'.format(self.stream.name, self.input_values, self.stream.outputs)


def parse_stream(stream_pddl, stream_map):
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, stream_name = next(stream_iter)
    assert('stream' == pddl_type)

    streams = []
    for stream in stream_iter:
        attributes = [stream[i] for i in range(0, len(stream), 2)]
        assert(STREAM_ATTRIBUTES == attributes)
        name, inputs, domain, outputs, certified = [stream[i] for i in range(1, len(stream), 2)]
        if name not in stream_map:
            raise ValueError('Undefined stream conditional generator: {}'.format(name))
        streams.append(Stream(name, stream_map[name],
                     tuple(inputs), list_from_conjunction(domain),
                     tuple(outputs), list_from_conjunction(certified)))
    return streams