import time
from collections import Counter, defaultdict, namedtuple, Sequence
from itertools import count

from pddlstream.language.conversion import list_from_conjunction, dnf_from_positive_formula, \
    substitute_expression, get_args, is_parameter, get_formula_operators, AND, OR
from pddlstream.language.external import ExternalInfo, Result, Instance, External, DEBUG, get_procedure_fn, parse_lisp_list
from pddlstream.language.generator import get_next, from_fn
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.utils import str_from_tuple

VEBOSE_FAILURES = True

def get_empty_fn():
    return lambda *input_values: None

def get_constant_fn(constant):
    return lambda *input_values: constant

def get_identity_fn(indices):
    return lambda *input_values: tuple(input_values[i] for i in indices)

##################################################

#UniqueOpt = namedtuple('UniqueOpt', ['instance', 'output_index'])
SharedOpt = namedtuple('SharedOpt', ['external', 'output_index'])

def get_shared_gen_fn(stream): # TODO: identify bound
    def gen_fn(*input_values):
        output_values = tuple(SharedOpt(stream, i) for i in range(len(stream.outputs)))
        yield [output_values]
    return gen_fn

# def create_partial_fn():
#     # TODO: indices or names
#     raise NotImplementedError()
#     #return get_partial_fn

def get_constant_gen_fn(stream, constant):
    def gen_fn(*input_values):
        assert(len(stream.inputs) == len(input_values))
        output_values = tuple(constant for _ in range(len(stream.outputs)))
        yield [output_values]
    return gen_fn

# def get_unique_fn(stream):
#     # TODO: this should take into account the output number...
#     def fn(*input_values):
#         #input_objects = map(opt_obj_from_value, input_values)
#         #stream_instance = stream.get_instance(input_objects)
#         #output_values = tuple(UniqueOpt(stream_instance, i) for i in range(len(stream.outputs)))
#         output_values = tuple(object() for _ in range(len(stream.outputs)))
#         return [output_values]
#     return fn

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

class StreamInfo(ExternalInfo):
    def __init__(self, opt_gen_fn=None, eager=False,
                 p_success=None, overhead=None):
        # TODO: could change frequency/priority for the incremental algorithm
        super(StreamInfo, self).__init__(eager, p_success, overhead)
        self.opt_gen_fn = opt_gen_fn
        #self.order = 0

class StreamResult(Result):
    def __init__(self, instance, output_objects, opt_index=None):
        super(StreamResult, self).__init__(instance, opt_index)
        self.output_objects = tuple(output_objects)
        self.mapping = dict(zip(self.instance.external.outputs, self.output_objects))
        self.mapping.update(instance.mapping)
        self.certified = substitute_expression(self.instance.external.certified, self.get_mapping())
    def get_mapping(self):
        return self.mapping
    def get_certified(self):
        return self.certified
    def get_tuple(self):
        name = self.instance.external.name
        return name, self.instance.input_objects, self.output_objects
    def remap_inputs(self, bindings):
        input_objects = [bindings.get(i, i) for i in self.instance.input_objects]
        new_instance = self.instance.external.get_instance(input_objects)
        return self.__class__(new_instance, self.output_objects, self.opt_index)
    def is_successful(self):
        return True
    def __repr__(self):
        return '{}:{}->{}'.format(self.instance.external.name,
                                  str_from_tuple(self.instance.input_objects),
                                  str_from_tuple(self.output_objects))

class StreamInstance(Instance):
    def __init__(self, stream, input_objects):
        super(StreamInstance, self).__init__(stream, input_objects)
        self._generator = None
        self.opt_index = stream.num_opt_fns
    def _check_output_values(self, new_values):
        if not isinstance(new_values, Sequence):
            raise ValueError('An output list for stream [{}] is not a sequence: {}'.format(self.external.name, new_values))
        for output_values in new_values:
            if not isinstance(output_values, Sequence):
                raise ValueError('An output tuple for stream [{}] is not a sequence: {}'.format(
                    self.external.name, output_values))
            if len(output_values) != len(self.external.outputs):
                raise ValueError('An output tuple for stream [{}] has length {} instead of {}: {}'.format(
                    self.external.name, len(output_values), len(self.external.outputs), output_values))
    def _next_outputs(self):
        if self._generator is None:
            input_values = self.get_input_values()
            try:
                self._generator = self.external.gen_fn(*input_values)
            except TypeError as err:
                print('Stream [{}] expects {} inputs'.format(self.external.name, len(input_values)))
                raise err
        new_values, self.enumerated = get_next(self._generator)
        return new_values
    def next_results(self, accelerate=1, verbose=False):
        all_new_values = []
        all_results = []
        start_calls = self.num_calls
        for attempt in range(accelerate):
            if all_results or self.enumerated:
                break
            start_time = time.time()
            new_values = self._next_outputs()
            self._check_output_values(new_values)
            results = [self.external._Result(self, tuple(map(Object.from_value, ov))) for ov in new_values]
            all_new_values.extend(new_values)
            all_results.extend(results)
            self.update_statistics(start_time, results)
        if verbose and (VEBOSE_FAILURES or all_new_values):
            print('{}-{}) {}:{}->[{}]'.format(start_calls, self.num_calls, self.external.name,
                                           str_from_tuple(self.get_input_values()),
                                       ', '.join(map(str_from_tuple, all_new_values))))
        return all_results
    def next_optimistic(self):
        # TODO: compute this just once and store
        if self.enumerated or self.disabled:
            return []
        # TODO: (potentially infinite) sequence of optimistic objects
        # TODO: how do I distinguish between real not real verifications of things?
        new_values = []
        for output_list in self.external.opt_gen_fn(*self.get_input_values()):
            new_values.extend(output_list)
        self._check_output_values(new_values)
        results = []
        for i, output_values in enumerate(new_values):
            output_objects = []
            for j, value in enumerate(output_values):
                #unique = object()
                unique = (self, i, j)
                param = unique if (self.opt_index == 0) else value
                output_objects.append(OptimisticObject.from_opt(value, param))
            results.append(self.external._Result(self, output_objects, opt_index=self.opt_index))
        return results
    def __repr__(self):
        return '{}:{}->{}'.format(self.external.name, self.input_objects, self.external.outputs)

class Stream(External):
    _Instance = StreamInstance
    _Result = StreamResult
    def __init__(self, name, gen_fn, inputs, domain, outputs, certified, info, fluents=[]):
        if info is None:
            info = StreamInfo(p_success=None, overhead=None)
        for p, c in Counter(outputs).items():
            if c != 1:
                raise ValueError('Output [{}] for stream [{}] is not unique'.format(p, name))
        for p in set(inputs) & set(outputs):
            raise ValueError('Parameter [{}] for stream [{}] is both an input and output'.format(p, name))
        parameters = {a for i in certified for a in get_args(i) if is_parameter(a)}
        for p in (parameters - set(inputs + outputs)):
            raise ValueError('Parameter [{}] for stream [{}] is not included within outputs'.format(p, name))
        super(Stream, self).__init__(name, info, inputs, domain)
        self.fluents = fluents
        #assert not self.fluents
        # Each stream could certify a stream-specific fact as well
        if gen_fn == DEBUG:
            #gen_fn = from_fn(lambda *args: tuple(object() for _ in self.outputs))
            gen_fn = from_fn(lambda *args: tuple(DebugValue(name, args, o) for o in self.outputs))
        self.gen_fn = gen_fn
        self.outputs = tuple(outputs)
        self.certified = tuple(certified)
        self.constants.update(a for i in certified for a in get_args(i) if not is_parameter(a))

        # TODO: generalize to a hierarchical sequence
        always_unique = False
        if always_unique:
            self.num_opt_fns = 0
            #self.opt_list_fn = get_unique_fn(self)
            self.opt_gen_fn = get_constant_gen_fn(self, None)
            raise NotImplementedError()
        else:
            self.num_opt_fns = 1
            self.opt_gen_fn = get_shared_gen_fn(self) if (self.info.opt_gen_fn is None) else self.info.opt_gen_fn
        #self.bound_list_fn = None
        #self.opt_fns = [get_unique_fn(self), get_shared_fn(self)] # get_unique_fn | get_shared_fn
        #self.opt_fns = [get_unique_fn(self)] # get_unique_fn | get_shared_fn
    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)

##################################################

def parse_stream(lisp_list, stream_map, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {':stream', ':inputs', ':domain', ':fluents', ':outputs', ':certified'}
    name = value_from_attribute[':stream']
    domain = value_from_attribute.get(':domain', None)
    # TODO: dnf_from_positive_formula(value_from_attribute.get(':domain', []))
    if not (get_formula_operators(domain) <= {AND}):
        # TODO: allow positive DNF
        raise ValueError('Stream [{}] domain must be a conjunction'.format(name))
    certified = value_from_attribute.get(':certified', None)
    if not (get_formula_operators(certified) <= {AND}):
        raise ValueError('Stream [{}] certified must be a conjunction'.format(name))
    return Stream(name, get_procedure_fn(stream_map, name),
                  value_from_attribute.get(':inputs', []),
                  list_from_conjunction(domain),
                  value_from_attribute.get(':outputs', []),
                  list_from_conjunction(certified),
                  stream_info.get(name, None),
                  value_from_attribute.get(':fluents', []))  # TODO: None
