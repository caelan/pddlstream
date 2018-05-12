from collections import Counter, defaultdict
from itertools import count

from pddlstream.conversion import list_from_conjunction, objects_from_values, opt_from_values, \
    substitute_expression, opt_obj_from_value, get_args, is_parameter
from pddlstream.fast_downward import parse_lisp
from pddlstream.function import Result, Instance, External, ExternalInfo, geometric_cost, parse_function, \
    parse_predicate, DEFAULT_OVERHEAD, DEFAULT_P_SUCCESS, DEBUG
from pddlstream.utils import str_from_tuple, INF
import time

class BoundedGenerator(object):
    def __init__(self, generator, max_calls=INF):
        self.generator = generator
        self.max_calls = max_calls
        self.calls = 0
    @property
    def enumerated(self):
        return self.max_calls <= self.calls
    def next(self):
        if self.enumerated:
            raise StopIteration()
        self.calls += 1
        return next(self.generator)

##################################################

def from_list_gen_fn(list_gen_fn):
    return list_gen_fn
    #return lambda *args: ListGenerator(list_gen_fn(*args))


def from_gen_fn(gen_fn):
    #return lambda *args: ([output_values] for output_values in gen_fn(*args))
    list_gen_fn = lambda *args: ([output_values] for output_values in gen_fn(*args))
    return from_list_gen_fn(list_gen_fn)

##################################################

def from_list_fn(list_fn):
    #return lambda *args: iter([list_fn(*args)])
    return lambda *args: BoundedGenerator(iter([list_fn(*args)]), max_calls=1)


def from_fn(fn):
    def list_fn(*args):
        outputs = fn(*args)
        return [] if outputs is None else [outputs]
    return from_list_fn(list_fn)


def from_test(test):
    return from_fn(lambda *args: tuple() if test(*args) else None)

def from_rule():
    return from_test(lambda *args: True)

def empty_gen():
    return lambda *args: iter([])

##################################################

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

class DebugValue(object): # TODO: could just do an object
    _output_counts = defaultdict(count)
    def __init__(self, stream, input_values, output_parameter):
        self.stream = stream
        self.input_values = input_values
        self.output_parameter = output_parameter
        self.index = next(self._output_counts[output_parameter])
    def __repr__(self):
        return '${}{}'.format(self.output_parameter[1:], self.index)

##################################################

# TODO: debug stream functions

class StreamInfo(ExternalInfo):
    # TODO: make bound, effort, etc meta-parameters of the algorithms or part of the problem?
    def __init__(self, eager=False, bound_fn=get_unique_fn,
                 p_success=DEFAULT_P_SUCCESS, overhead=DEFAULT_OVERHEAD):
        # TODO: could change frequency/priority for the incremental algorithm
        self.eager = eager
        self.bound_fn = bound_fn
        self.p_success = p_success
        self.overhead = overhead
        self.effort = geometric_cost(self.overhead, self.p_success)
        #self.order = 0
        # TODO: context?

##################################################

class StreamResult(Result):
    def __init__(self, instance, output_objects):
        super(StreamResult, self).__init__(instance)
        self.output_objects = output_objects
    def get_mapping(self):
        return dict(list(zip(self.instance.external.inputs, self.instance.input_objects)) +
                    list(zip(self.instance.external.outputs, self.output_objects)))
    def get_certified(self):
        return substitute_expression(self.instance.external.certified, self.get_mapping())
    def get_tuple(self):
        name = self.instance.external.name
        return name, self.instance.input_objects, self.output_objects
    def __repr__(self):
        return '{}:{}->{}'.format(self.instance.external.name,
                                  str_from_tuple(self.instance.input_objects),
                                  str_from_tuple(self.output_objects))

class StreamInstance(Instance):
    def __init__(self, stream, input_objects):
        super(StreamInstance, self).__init__(stream, input_objects)
        self._generator = None
    def _next_outputs(self):
        if self._generator is None:
            self._generator = self.external.gen_fn(*self.get_input_values())
        new_values = []
        try:
            new_values = next(self._generator)
        except StopIteration:
            self.enumerated = True
        if isinstance(self._generator, BoundedGenerator):
            self.enumerated = self._generator.enumerated
        return new_values
    def next_results(self, verbose=False):
        start_time = time.time()
        assert not self.enumerated
        new_values = self._next_outputs()
        if verbose:
            print('{}:{}->[{}]'.format(self.external.name, str_from_tuple(self.get_input_values()),
                                       ', '.join(map(str_from_tuple, new_values))))
        results = [self.external._Result(self, objects_from_values(output_values)) for output_values in new_values]
        self.update_statistics(start_time, results)
        return results
    def next_optimistic(self):
        if self.enumerated or self.disabled:
            return []
        # TODO: (potentially infinite) sequence of optimistic objects
        # TODO: how do I distinguish between real not real verifications of things?
        new_optimistic = self.external.opt_fn(*self.get_input_values())
        return [self.external._Result(self, opt_from_values(output_opt)) for output_opt in new_optimistic]
    def __repr__(self):
        return '{}:{}->{}'.format(self.external.name, self.input_objects, self.external.outputs)

class Stream(External):
    _Instance = StreamInstance
    _Result = StreamResult
    def __init__(self, name, gen_fn, inputs, domain, outputs, certified):
        for p, c in Counter(outputs).items():
            if c != 1:
                raise ValueError('Output [{}] for stream [{}] is not unique'.format(p, name))
        for p in set(inputs) & set(outputs):
            raise ValueError('Parameter [{}] for stream [{}] is both an input and output'.format(p, name))
        for p in {a for i in certified for a in get_args(i) if is_parameter(a)} - set(inputs + outputs):
            raise ValueError('Parameter [{}] for stream [{}] is not included within outputs'.format(p, name))
        super(Stream, self).__init__(name, inputs, domain)
        # Each stream could certify a stream-specific fact as well
        if gen_fn == DEBUG:
            #gen_fn = from_fn(lambda *args: tuple(object() for _ in self.outputs))
            gen_fn = from_fn(lambda *args: tuple(DebugValue(name, args, o) for o in self.outputs))
        self.gen_fn = gen_fn
        self.outputs = tuple(outputs)
        self.certified = tuple(certified)
        self.opt_fn = get_shared_fn(self) # get_unique_fn | get_shared_fn
        self.info = StreamInfo()
    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)

##################################################

# TODO: WildStream, FactStream

# class WildResult(object):
#     def __init__(self, stream_instance, output_objects):
#         self.stream_instance = stream_instance
#         #mapping = dict(list(zip(self.stream_instance.stream.inputs, self.stream_instance.input_objects)) +
#         #            list(zip(self.stream_instance.stream.outputs, output_objects)))
#         #self.certified = substitute_expression(self.stream_instance.stream.certified, mapping)
#     def get_certified(self):
#         #return self.certified
#     def __repr__(self):
#         #return '{}:{}->{}'.format(self.stream_instance.stream.name,
#         #                          str_from_tuple(self.stream_instance.input_objects),
#         #                          list(self.certified))
#
# def next_certified(self, **kwargs):
#     if self._generator is None:
#         self._generator = self.stream.gen_fn(*self.get_input_values())
#     new_certified = []
#     if isinstance(self._generator, FactGenerator):
#         new_certified += list(map(obj_from_value_expression, self._generator.generate(context=None)))
#         self.enumerated = self._generator.enumerated
#     else:
#         for output_objects in self.next_outputs(**kwargs):
#             mapping = dict(list(zip(self.stream.inputs, self.input_objects)) +
#                            list(zip(self.stream.outputs, output_objects)))
#             new_certified += substitute_expression(self.stream.certified, mapping)
#     return new_certified

##################################################

STREAM_ATTRIBUTES = [':stream', ':inputs', ':domain', ':outputs', ':certified']

def parse_stream(lisp_list, stream_map):
    attributes = [lisp_list[i] for i in range(0, len(lisp_list), 2)]
    assert (STREAM_ATTRIBUTES == attributes)
    values = [lisp_list[i] for i in range(1, len(lisp_list), 2)]
    name, inputs, domain, outputs, certified = values
    if stream_map == DEBUG:
        gen_fn = DEBUG
    else:
        if name not in stream_map:
            raise ValueError('Undefined stream conditional generator: {}'.format(name))
        gen_fn = stream_map[name]
    return Stream(name, gen_fn, tuple(inputs), list_from_conjunction(domain),
                  tuple(outputs), list_from_conjunction(certified))


def parse_stream_pddl(stream_pddl, stream_map):
    streams = []
    if stream_pddl is None:
        return streams
    if all(isinstance(e, External) for e in stream_pddl):
        return stream_pddl
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, stream_name = next(stream_iter)
    assert('stream' == pddl_type)
    #debug = (stream_map == DEBUG)

    for lisp_list in stream_iter:
        name = lisp_list[0]
        if name == ':stream':
            external = parse_stream(lisp_list, stream_map)
        elif name == ':wild':
            raise NotImplementedError(name)
        elif name == ':rule':
            continue
            # TODO: implement rules
            # TODO: add eager stream if multiple conditions otherwise apply and add to stream effects
        elif name == ':function':
            external = parse_function(lisp_list, stream_map)
        elif name == ':predicate': # Cannot just use args if want a bound
            external = parse_predicate(lisp_list, stream_map)
        else:
            raise ValueError(name)
        if any(e.name == external.name for e in streams):
            raise ValueError('Stream [{}] is not unique'.format(external.name))
        streams.append(external)
    return streams
