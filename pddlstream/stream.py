from pddlstream.conversion import list_from_conjunction, objects_from_values, opt_from_values, \
    substitute_expression, is_head, get_prefix, get_args, Equal, Not, values_from_objects, obj_from_value_expression
from pddlstream.fast_downward import parse_lisp
from pddlstream.object import Object, OptimisticObject
from pddlstream.utils import str_from_tuple, INF


def from_list_gen_fn(list_gen_fn):
    return list_gen_fn
    #return lambda *args: ListGenerator(list_gen_fn(*args))


def from_gen_fn(gen_fn):
    #return lambda *args: ([output_values] for output_values in gen_fn(*args))
    list_gen_fn = lambda *args: ([output_values] for output_values in gen_fn(*args))
    return from_list_gen_fn(list_gen_fn)


def from_list_fn(list_fn):
    #return lambda *args: iter([list_fn(*args)])
    return lambda *args: ListGenerator(iter([list_fn(*args)]), max_calls=1)


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
        self.output_history = []
    def get_input_values(self):
        return values_from_objects(self.input_objects)
    def get_mapping(self):
        return dict(zip(self.stream.inputs, self.input_objects))
    def get_domain(self):
        return substitute_expression(self.stream.domain, self.get_mapping())
    def next_outputs(self, verbose=False, context=None):
        assert not self.enumerated
        if self._generator is None:
            self._generator = self.stream.gen_fn(*self.get_input_values())
        if isinstance(self._generator, ValueGenerator):
            new_values = self._generator.generate(context=context)
            self.enumerated = self._generator.enumerated
        else:
            try:
                new_values = next(self._generator)
            except StopIteration:
                new_values = []
                self.enumerated = True
        new_objects = list(map(objects_from_values, new_values))
        if verbose:
            self.dump_output_list(new_objects)
        self.output_history.append(new_objects)
        return new_objects
    def next_certified(self, **kwargs):
        if self._generator is None:
            self._generator = self.stream.gen_fn(*self.get_input_values())
        new_certified = []
        if isinstance(self._generator, FactGenerator):
            new_certified += list(map(obj_from_value_expression, self._generator.generate(context=None)))
            self.enumerated = self._generator.enumerated
        else:
            for output_objects in self.next_outputs(**kwargs):
                mapping = dict(list(zip(self.stream.inputs, self.input_objects)) +
                               list(zip(self.stream.outputs, output_objects)))
                new_certified += substitute_expression(self.stream.certified, mapping)
        return new_certified
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
        self.opt_fn = get_unique_fn(self)
        #self.opt_fn = get_shared_fn(self)
        self.instances = {}
        self.prioritized = False
    def get_instance(self, input_values):
        input_values = tuple(input_values)
        if input_values not in self.instances:
            self.instances[input_values] = self._InstanceClass(self, input_values)
        return self.instances[input_values]
    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)

##################################################

class FunctionInstance(StreamInstance): # Head(StreamInstance):
    _opt_value = 0
    def next_outputs(self, verbose=False, context=None):
        assert not self.enumerated
        self.enumerated = True
        # TODO: check output and assert only one?
        self.value = self.stream.gen_fn(*self.get_input_values())
        #return [value]
        return [(self.value,)]
    def next_certified(self, **kwargs):
        self.enumerated = True
        self.value = self.stream.gen_fn(*self.get_input_values())
        expression = substitute_expression(self.stream.head, self.get_mapping())
        return [Equal(expression, self.value)]
    def next_optimistic(self):
        if self.enumerated or self.disabled:
            return []
        return [(self._opt_value,)]
    def dump_output_list(self, output_list):
        #[value] = output_list
        [(value,)] = output_list
        print('{}:{}->{}'.format(self.stream.name, str_from_tuple(self.get_input_values()), value))

class Function(Stream):
    """
    The key difference from a stream is that the outputs aren't treated as objects
    We also do not make the closed world assumption
    """
    _InstanceClass = FunctionInstance
    def __init__(self, head, fn, domain):
        self.head = head
        name = get_prefix(head)
        inputs = get_args(head)
        outputs = ('?r',)
        certified = [Equal(head, '?r')]
        super(Function, self).__init__(name, fn, inputs, domain, outputs, certified)
    def __repr__(self):
        return '{}:{}->R'.format(self.name, self.inputs)

##################################################

class PredicateInstance(StreamInstance):
    _opt_value = True
    def next_outputs(self, verbose=False, context=None):
        assert not self.enumerated
        self.enumerated = True
        self.value = self.stream.gen_fn(*self.get_input_values())
        assert(self.value in (True, False))
        return [(self.value,)]
    def next_certified(self, **kwargs):
        self.enumerated = True
        self.value = self.stream.gen_fn(*self.get_input_values())
        assert(self.value in (True, False))
        expression = substitute_expression(self.stream.literal, self.get_mapping())
        if self.value:
            return [expression]
        return [Not(expression)] # TODO: simplify if inverted
    def next_optimistic(self):
        if self.enumerated or self.disabled:
            return []
        return [(self._opt_value,)]
    def dump_output_list(self, output_list):
        [(value,)] = output_list
        print('{}:{}->{}'.format(self.stream.name, str_from_tuple(self.get_input_values()), value))

class Predicate(Stream):
    _InstanceClass = PredicateInstance
    def __init__(self, literal, fn, domain):
        self.literal = literal
        name = get_prefix(literal)
        inputs = get_args(literal)
        super(Predicate, self).__init__(name, fn, inputs, domain, [], [literal])
    def __repr__(self):
        return '{}:{}->{T,F}'.format(self.name, self.inputs)

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
        # TODO: wild stream?
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
            #streams.append(Stream(name, stream_map[name], tuple(inputs), domain, tuple(),
            #                      Equal(head, 1)))
            # TODO: this must be eager in the case of incremental
        elif stream[0] == ':predicate': # Cannot just use args if want a bound
            assert(len(stream) == 3)
            head = tuple(stream[1])
            assert(is_head(head))
            name = get_prefix(head)
            if name not in stream_map:
                raise ValueError('Undefined external function: {}'.format(name))
            streams.append(Predicate(head, stream_map[name], list_from_conjunction(stream[2])))
        else:
            raise ValueError(stream[0])
    return streams

##################################################

class ValueGenerator(object):
    # TODO: I could also include this in a
    #def __init__(self, *inputs):
    #    self.inputs = tuple(inputs)
    def __init__(self):
        self.enumerated = False
    #def __iter__(self):
    #def __call__(self, *args, **kwargs):
    def generate(self, context=None):
        raise NotImplementedError()

class ListGenerator(ValueGenerator):
    # TODO: could also pass gen_fn(*inputs)
    # TODO: can test whether is instance for context
    def __init__(self, generator, max_calls=INF):
        super(ListGenerator, self).__init__()
        #super(ListGeneratorFn, self).__init__(*inputs)
        #self.generator = gen_fn(*inputs)
        self.generator = generator
        self.max_calls = max_calls
        self.calls = 0
        self.enumerated = (self.calls < self.max_calls)
    def generate(self, context=None):
        self.calls += 1
        if self.max_calls <= self.calls:
            self.enumerated = True
        try:
            return next(self.generator)
        except StopIteration:
            self.enumerated = True
        return []

##################################################

class FactGenerator(object):
    """
    Previously called a wild stream
    """
    def __init__(self):
        self.enumerated = False
    def generate(self, context=None):
        raise NotImplementedError()
