from pddlstream.conversion import list_from_conjunction, objects_from_values, opt_from_values, \
    substitute_expression, is_head, get_prefix, get_args, Equal, values_from_objects
from pddlstream.fast_downward import parse_lisp
from pddlstream.object import Object, OptimisticObject
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

def opt_obj_from_value(value):
    if Object.has_value(value):
        return Object.from_value(value)
    return OptimisticObject.from_opt(value)
    # TODO: better way of doing this?
    #return OptimisticObject._obj_from_inputs.get(value, Object.from_value(value))

def get_empty_fn(stream):
    # TODO: also use None to designate this
    return lambda *input_values: []

def get_shared_fn(stream):
    def fn(*input_values):
        output_values = tuple((stream, i) for i in range(len(stream.outputs)))
        return [output_values]
    return fn

def create_partial_fn():
    # TODO: indices or names
    def get_partial_fn(stream):
        def fn(*input_values):
            output_values = tuple((stream, i) for i in range(len(stream.outputs)))
            return [output_values]
        return fn
    return get_partial_fn

def get_unique_fn(stream):
    def fn(*input_values):
        input_objects = map(opt_obj_from_value, input_values)
        stream_instance = stream.get_instance(input_objects)
        output_values = tuple((stream_instance, i) for i in range(len(stream.outputs)))
        return [output_values]
    return fn

##################################################

class StreamResult(object):
    def __init__(self, stream_instance, output_objects):
        self.stream_instance = stream_instance
        self.output_objects = output_objects
    def get_mapping(self):
        return dict(list(zip(self.stream_instance.stream.inputs, self.stream_instance.input_objects)) +
                    list(zip(self.stream_instance.stream.outputs, self.output_objects)))
    def get_certified(self):
        return substitute_expression(self.stream_instance.stream.certified,
                                     self.get_mapping())
    def __repr__(self):
        return '{}:{}->{}'.format(self.stream_instance.stream.name,
                                  str_from_tuple(self.stream_instance.input_objects),
                                  str_from_tuple(self.output_objects))

class StreamInstance(object):
    def __init__(self, stream, input_values):
        self.stream = stream
        self.input_objects = tuple(input_values)
        self._generator = None
        self.enumerated = False
        self.disabled = False
    def get_input_values(self):
        return values_from_objects(self.input_objects)
    def get_mapping(self):
        return dict(zip(self.stream.inputs, self.input_objects))
    def get_domain(self):
        return substitute_expression(self.stream.domain, self.get_mapping())
    def next_outputs(self):
        assert not self.enumerated
        if self._generator is None:
            self._generator = self.stream.gen_fn(*self.get_input_values())
        try:
            return list(map(objects_from_values, next(self._generator)))
        except StopIteration:
            self.enumerated = True
        return []
    def next_optimistic(self):
        if self.enumerated or self.disabled:
            return []
        # TODO: (potentially infinite) sequence of optimistic objects
        output_values_list = self.stream.opt_fn(*self.get_input_values())
        # TODO: how do I distinguish between real not real verifications of things?
        #output_values_list = [tuple((self, i) for i in range(len(self.stream.outputs)))]
        #output_objects = tuple(map(OptimisticObject.from_opt, output_values))
        return list(map(opt_from_values, output_values_list))
    def __repr__(self):
        return '{}:{}->{}'.format(self.stream.name, self.input_objects, self.stream.outputs)
    def dump_output_list(self, output_list):
        print('{}:{}->[{}]'.format(self.stream.name, str_from_tuple(self.get_input_values()),
                                   ', '.join(map(str_from_tuple, map(values_from_objects, output_list)))))

class Stream(object):
    _InstanceClass = StreamInstance
    def __init__(self, name, gen_fn, inputs, domain, outputs, certified):
        # Each stream could certify a stream-specific fact as well
        self.name = name
        self.gen_fn = gen_fn
        self.inputs = tuple(inputs)
        self.domain = tuple(domain)
        self.outputs = tuple(outputs)
        self.certified = tuple(certified)
        #self.opt_fn = get_unique_fn(self)
        self.opt_fn = get_shared_fn(self)
        self.instances = {}
    def get_instance(self, input_values):
        input_values = tuple(input_values)
        if input_values not in self.instances:
            self.instances[input_values] = self._InstanceClass(self, input_values)
        return self.instances[input_values]
    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)

##################################################

# Key distinction is that this value isn't treated as an object

#class Head(StreamInstance):
class FunctionInstance(StreamInstance):
    def next_outputs(self):
        assert not self.enumerated
        self.enumerated = True
        # TODO: check output and assert only one?
        value = self.stream.gen_fn(*self.get_input_values())
        #return [value]
        return [(value,)]
    def dump_output_list(self, output_list):
        #[value] = output_list
        [(value,)] = output_list
        print('{}:{}->{}'.format(self.stream.name, str_from_tuple(self.get_input_values()), value))

# TODO: have this extend Stream?
#class Function(Stream):
class Function(Stream):
    _InstanceClass = FunctionInstance
    def __init__(self, head, fn, domain):
        name = get_prefix(head)
        inputs = get_args(head)
        outputs = ('?x',)
        certified = [Equal(head, '?x')]
        super(Function, self).__init__(name, fn, inputs, domain, outputs, certified)
        #self.fn = fn
    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)

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
            pass
            # TODO: implement rules
            # TODO: add eager stream if multiple conditions otherwise apply and add to stream effects
        elif stream[0] == ':function':
            assert(len(stream) == 3)
            head = tuple(stream[1])
            assert(is_head(head))
            #inputs = get_args(head)
            domain = list_from_conjunction(stream[2])
            name = get_prefix(head)
            if name not in stream_map:
                raise ValueError('Undefined external function: {}'.format(name))
            streams.append(Function(head, stream_map[name], domain))
            print(streams[-1])
            #streams.append(Stream(name, stream_map[name], tuple(inputs), domain, tuple(),
            #                      Equal(head, 1)))
            # TODO: this must be eager in the case of incremental
        elif stream[0] == ':predicate': # Cannot just use args if want a bound
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
