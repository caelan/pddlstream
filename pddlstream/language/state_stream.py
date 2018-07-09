import time
from collections import Counter, defaultdict, namedtuple, Sequence
from itertools import count

from pddlstream.language.conversion import list_from_conjunction, substitute_expression, get_prefix, get_args, is_parameter
from pddlstream.language.external import ExternalInfo, Result, Instance, External, DEBUG
from pddlstream.language.stream import DebugValue
from pddlstream.language.generator import get_next, from_fn
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.utils import str_from_tuple

try:
    from inspect import getfullargspec as get_arg_spec
except ImportError:
    from inspect import getargspec as get_arg_spec

# TODO: no need to optimistic
# TODO: should I extend function or stream or predicate?


class StateStreamInfo(ExternalInfo):
    def __init__(self, eager=False, p_success=None, overhead=None):
        # TODO: could change frequency/priority for the incremental algorithm
        super(StateStreamInfo, self).__init__(eager, p_success, overhead)

class StateStreamResult(Result):
    def __init__(self, instance, output_objects):
        #opt_index = None
        opt_index = 0
        super(StateStreamResult, self).__init__(instance, opt_index)
        assert(len(output_objects) == len(instance.external.outputs))
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
        print(bindings)
        input_objects = [bindings.get(i, i) for i in self.instance.input_objects]
        new_instance = self.instance.external.get_instance(input_objects)
        return self.__class__(new_instance, self.output_objects, self.opt_index)
    def __repr__(self):
        return '{}:{}->{}'.format(self.instance.external.name,
                                  str_from_tuple(self.instance.input_objects),
                                  str_from_tuple(self.output_objects))

class StateStreamInstance(Instance):
    def __init__(self, stream, input_objects, fluent_facts):
        super(StateStreamInstance, self).__init__(stream, input_objects)
        self._generator = None
        #self.opt_index = 0 # TODO:
        self.fluent_facts = fluent_facts
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
            self._generator = self.external.gen_fn(*self.get_input_values())
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
        if verbose and all_new_values:
            print('{}-{}) {}:{}->[{}]'.format(start_calls, self.num_calls, self.external.name,
                                           str_from_tuple(self.get_input_values()),
                                       ', '.join(map(str_from_tuple, all_new_values))))
        return all_results
    def __repr__(self):
        return '{}:{}->{}'.format(self.external.name, self.input_objects, self.external.outputs)

class StateStream(External):
    _Instance = StateStreamInstance
    _Result = StateStreamResult
    def __init__(self, name, gen_fn, inputs, domain, fluents,
                 outputs, certified, info):
        # Outputs should not be inputs to other streams
        # Outputs should not be used in two actions
        # Can achieve several conditions in the same action, but no point
        # For now, condition shouldn't be achievable by other means
        if info is None:
            info = StateStreamInfo(p_success=None, overhead=None)
        for p, c in Counter(outputs).items():
            if c != 1:
                raise ValueError('Output [{}] for stream [{}] is not unique'.format(p, name))
        for p in set(inputs) & set(outputs):
            raise ValueError('Parameter [{}] for stream [{}] is both an input and output'.format(p, name))
        parameters = {a for i in certified for a in get_args(i) if is_parameter(a)}
        for p in (parameters - set(inputs + outputs)):
            raise ValueError('Parameter [{}] for stream [{}] is not included within outputs'.format(p, name))
        super(StateStream, self).__init__(name, info, inputs, domain)

        #self.axiom_thing = 'f-{}'.format(p) # TODO: finish this
        if gen_fn == DEBUG:
            gen_fn = from_fn(lambda *args: tuple(DebugValue(name, args, o) for o in self.outputs))
        self.gen_fn = gen_fn
        self.outputs = tuple(outputs)
        assert(not self.outputs)
        self.certified = tuple(certified)
        assert(len(self.certified) == 1)
        self.constants.update(a for i in certified for a in get_args(i) if not is_parameter(a))

        # TODO: could also have a function to partition into tuples to be applied
        self.fluents = fluents # TODO: alternatively could make this a function determining whether to include
        self.negated_predicates = {get_prefix(f): '~{}'.format(get_prefix(f)) for f in self.certified}
        self.axioms = []

    def get_instance(self, input_objects, fluent_facts):
        key = (tuple(input_objects), frozenset(fluent_facts))
        if key not in self.instances:
            self.instances[key] = self._Instance(self, input_objects, fluent_facts)
        return self.instances[key]

    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)

##################################################

STREAM_ATTRIBUTES = [':state-stream', ':inputs', ':domain', ':fluents', ':outputs', ':certified']

def get_gen_fn(stream_map, name):
    if stream_map == DEBUG:
        return DEBUG
    if name not in stream_map:
        raise ValueError('Undefined stream conditional generator: {}'.format(name))
    return stream_map[name]

def parse_state_stream(lisp_list, stream_map, stream_info):
    attributes = [lisp_list[i] for i in range(0, len(lisp_list), 2)]
    values = [lisp_list[i] for i in range(1, len(lisp_list), 2)]
    value_from_attribute = dict(zip(attributes, values))
    name = value_from_attribute[':state-stream']
    return StateStream(name, get_gen_fn(stream_map, name),
                       value_from_attribute.get(':inputs', []),
                       list_from_conjunction(value_from_attribute.get(':domain', [])),
                       value_from_attribute.get(':fluents', []), # TODO: None
                       value_from_attribute.get(':outputs', []),
                       list_from_conjunction(value_from_attribute.get(':certified', [])),
                       stream_info.get(name, None))
