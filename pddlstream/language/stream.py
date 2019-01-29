import time
from collections import Counter, defaultdict, namedtuple, Sequence
from itertools import count

from pddlstream.algorithms.common import INTERNAL_EVALUATION, add_fact
from pddlstream.algorithms.downward import make_axiom
from pddlstream.language.constants import AND, get_prefix, get_args, is_parameter, Fact, concatenate
from pddlstream.language.conversion import list_from_conjunction, substitute_expression, \
    get_formula_operators, values_from_objects, obj_from_value_expression
from pddlstream.language.external import ExternalInfo, Result, Instance, External, DEBUG, get_procedure_fn, \
    parse_lisp_list
from pddlstream.language.generator import get_next, from_fn
from pddlstream.language.object import Object, OptimisticObject, UniqueOptValue
from pddlstream.utils import str_from_object, get_mapping, irange, apply_mapping

VERBOSE_FAILURES = True
DEFAULT_UNIQUE = False
NEGATIVE_BLOCKED = True
NEGATIVE_SUFFIX = '-negative'

# TODO: could also make only wild facts and automatically identify output tuples satisfying certified
# TODO: default effort cost of streams with more inputs to be higher (but negated are free)
# TODO: automatically convert to test streams on inputs

def get_empty_fn():
    return lambda *input_values: None

def get_constant_fn(constant):
    return lambda *input_values: constant

def get_identity_fn(indices):
    return lambda *input_values: tuple(input_values[i] for i in indices)

##################################################

# TODO: make wild the default output
WildOutput = namedtuple('WildOutput', ['values', 'facts'])

class OptValue(namedtuple('OptValue', ['stream', 'inputs', 'input_objects', 'output'])):
    @property
    def values(self):
        return values_from_objects(self.input_objects)

class PartialInputs(object):
    def __init__(self, inputs='', unique=DEFAULT_UNIQUE): #, num=1):
        self.inputs = tuple(inputs.split())
        self.unique = unique
        #self.num = num
    def get_opt_gen_fn(self, stream_instance):
        stream = stream_instance.external
        inputs = stream.inputs if self.unique else self.inputs
        assert set(inputs) <= set(stream.inputs)
        # TODO: ensure no scoping errors with inputs
        def gen_fn(*input_values):
            input_objects = stream_instance.input_objects
            mapping = get_mapping(stream.inputs, input_objects)
            selected_objects = tuple(mapping[inp] for inp in inputs)
            #for _ in irange(self.num):
            for _ in irange(stream_instance.num_optimistic):
                yield [tuple(OptValue(stream.name, inputs, selected_objects, out)
                             for out in stream.outputs)]
        return gen_fn
    def __repr__(self):
        return repr(self.__dict__)

def get_constant_gen_fn(stream, constant):
    def gen_fn(*input_values):
        assert (len(stream.inputs) == len(input_values))
        yield [tuple(constant for _ in range(len(stream.outputs)))]
    return gen_fn

##################################################

# def get_unique_fn(stream):
#     # TODO: this should take into account the output number...
#     def fn(*input_values):
#         #input_objects = map(opt_obj_from_value, input_values)
#         #stream_instance = stream.get_instance(input_objects)
#         #output_values = tuple(UniqueOpt(stream_instance, i) for i in range(len(stream.outputs)))
#         output_values = tuple(object() for _ in range(len(stream.outputs)))
#         return [output_values]
#     return fn

def get_debug_gen_fn(stream):
    return from_fn(lambda *args: tuple(DebugValue(stream.name, args, o) for o in stream.outputs))

class DebugValue(object): # TODO: could just do an object
    _output_counts = defaultdict(count)
    _prefix = '@' # $ | @
    def __init__(self, stream, input_values, output_parameter):
        self.stream = stream
        self.input_values = input_values
        self.output_parameter = output_parameter
        self.index = next(self._output_counts[output_parameter])
    def __repr__(self):
        # Can also just return first letter of the prefix
        return '{}{}{}'.format(self._prefix, self.output_parameter[1:], self.index)

##################################################

class StreamInfo(ExternalInfo):
    def __init__(self, opt_gen_fn=PartialInputs(), eager=False,
                 p_success=None, overhead=None, negate=False, effort_fn=None, simultaneous=False):
        # TODO: could change frequency/priority for the incremental algorithm
        super(StreamInfo, self).__init__(eager, p_success, overhead, effort_fn)
        self.opt_gen_fn = opt_gen_fn # TODO: call this an abstraction instead
        self.negate = negate
        self.simultaneous = simultaneous
        #self.order = 0
    def __repr__(self):
        return repr(self.__dict__)

##################################################

class StreamResult(Result):
    def __init__(self, instance, output_objects, opt_index=None,
                 call_index=None, list_index=None, optimistic=True):
        super(StreamResult, self).__init__(instance, opt_index, call_index, optimistic)
        self.output_objects = tuple(output_objects)
        assert len(self.output_objects) == len(self.external.outputs)
        self.list_index = list_index
        self._mapping = None
        self._certified = None
        self._stream_fact = None
    @property
    def mapping(self):
        if self._mapping is None:
            self._mapping = get_mapping(self.external.outputs, self.output_objects)
            self._mapping.update(self.instance.get_mapping())
        return self._mapping
    def get_mapping(self):
        return self.mapping
    @property
    def stream_fact(self):
        if self._stream_fact is None:
            self._stream_fact = substitute_expression(self.external.stream_fact, self.mapping)
        return self._stream_fact
    @property
    def certified(self):
        if self._certified is None:
            self._certified = substitute_expression(self.external.certified, self.get_mapping())
        return self._certified
    def get_certified(self):
        return self.certified
    def get_tuple(self):
        return self.external.name, self.instance.input_objects, self.output_objects
    def remap_inputs(self, bindings):
        # TODO: speed this procedure up
        #if not any(o in bindings for o in self.instance.get_objects()):
        #    return self
        input_objects = apply_mapping(self.instance.input_objects, bindings)
        fluent_facts = [Fact(get_prefix(f), apply_mapping(get_args(f), bindings))
                        for f in self.instance.fluent_facts]
        new_instance = self.external.get_instance(input_objects, fluent_facts=fluent_facts)
        new_instance.opt_index = self.instance.opt_index
        return self.__class__(new_instance, self.output_objects, self.opt_index,
                              self.call_index, self.list_index, self.optimistic)
    def is_successful(self):
        return True
    def __repr__(self):
        return '{}:{}->{}'.format(self.external.name,
                                  str_from_object(self.instance.input_objects),
                                  str_from_object(self.output_objects))

class StreamInstance(Instance):
    _Result = StreamResult
    def __init__(self, stream, input_objects, fluent_facts):
        super(StreamInstance, self).__init__(stream, input_objects)
        self._generator = None
        self.opt_index = stream.num_opt_fns
        self.fluent_facts = frozenset(fluent_facts)
        opt_gen_fn = self.external.info.opt_gen_fn
        self.opt_gen_fn = opt_gen_fn.get_opt_gen_fn(self) \
            if isinstance(opt_gen_fn, PartialInputs) else opt_gen_fn
        self.num_optimistic = 1
        self._axiom_predicate = None
        self._disabled_axiom = None

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

    def _check_wild_facts(self, new_facts):
        if not isinstance(new_facts, Sequence):
            raise ValueError('Output wild facts for wild stream [{}] is not a sequence: {}'.format(
                self.external.name, new_facts))

    def get_result(self, object_objects, opt_index=None, list_index=None, optimistic=True):
        # TODO: ideally would increment a flag per stream for each failure
        call_index = self.num_calls
        #call_index = self.successes # Only counts iterations that return results for complexity
        return self._Result(self, tuple(object_objects), opt_index=opt_index,
                            call_index=call_index, list_index=list_index, optimistic=optimistic)

    def use_unique(self):
        return self.opt_index == 0

    def get_objects(self): # TODO: lazily compute
        return set(self.input_objects) | {o for f in self.fluent_facts for o in get_args(f)}

    def get_fluent_values(self):
        return [Fact(get_prefix(f), values_from_objects(get_args(f))) for f in self.fluent_facts]

    def _create_generator(self):
        if self._generator is not None:
            return
        input_values = self.get_input_values()
        if self.external.is_fluent(): # self.fluent_facts
            self._generator = self.external.gen_fn(*input_values, fluents=self.get_fluent_values())
        else:
            self._generator = self.external.gen_fn(*input_values)

    def _next_outputs(self):
        self._create_generator()
        output, self.enumerated = get_next(self._generator, default=None)
        if output is None:
            output = []
        if not self.external.is_wild and not isinstance(output, WildOutput):
            return WildOutput(output, [])
        if len(output) != 2:
            raise RuntimeError('Wild stream [{}] does not generate pairs of output values and wild facts'.format(
                self.external.name))
        return output

    def next_results(self, accelerate=1, verbose=False):
        # TODO: prune repeated values
        all_new_values = []
        all_new_facts = []
        all_results = []
        start_calls = self.num_calls
        for attempt in range(accelerate):
            if all_results or self.enumerated:
                break
            start_time = time.time()
            new_values, new_facts = self._next_outputs()
            self._check_output_values(new_values)
            self._check_wild_facts(new_facts)
            new_results = [self.get_result(map(Object.from_value, output_values),
                                           list_index=list_index, optimistic=False)
                           for list_index, output_values in enumerate(new_values)]
            all_new_values.extend(new_values)
            all_new_facts.extend(new_facts)
            all_results.extend(new_results)
            self.update_statistics(start_time, new_results)
        if verbose:
            end_calls = self.num_calls - 1
            call_range = start_calls if start_calls == end_calls else \
                '{}-{}'.format(start_calls, end_calls)
            if VERBOSE_FAILURES or all_new_values:
                print('{}) {}:{}->{}'.format(call_range, self.external.name,
                                             str_from_object(self.get_input_values()),
                                             str_from_object(all_new_values)))
            if all_new_facts:
                # TODO: format all_new_facts
                print('{}) {}:{}->{}'.format(call_range, self.external.name,
                                             str_from_object(self.get_input_values()), all_new_facts))
        return all_results, list(map(obj_from_value_expression, all_new_facts))

    def next_optimistic(self):
        # TODO: compute this just once and store
        if self.enumerated or self.disabled:
            return []
        # TODO: (potentially infinite) sequence of optimistic objects
        # TODO: how do I distinguish between real and not real verifications of things?
        # TODO: reuse these?
        self.opt_results = []
        output_set = set()
        for output_list in self.opt_gen_fn(*self.get_input_values()):
            self._check_output_values(output_list)
            for i, output_values in enumerate(output_list):
                output_objects = []
                for output_index, value in enumerate(output_values):
                    # TODO: maybe record history of values here?
                    unique = UniqueOptValue(self, len(self.opt_results), output_index) # object()
                    param = unique if self.use_unique() else value
                    output_objects.append(OptimisticObject.from_opt(value, param))
                output_objects = tuple(output_objects)
                if output_objects not in output_set:
                    output_set.add(output_objects) # No point returning the exact thing here...
                    self.opt_results.append(self._Result(self, output_objects, opt_index=self.opt_index,
                                                         call_index=len(self.opt_results), list_index=0))
        return self.opt_results

    def get_blocked_fact(self):
        if self.external.is_fluent():
            assert self._axiom_predicate is not None
            return Fact(self._axiom_predicate, self.input_objects)
        return Fact(self.external.blocked_predicate, self.input_objects)

    def disable(self, evaluations, domain):
        #assert not self.disabled
        super(StreamInstance, self).disable(evaluations, domain)
        if not self.external.is_fluent(): # self.fluent_facts:
            if self.external.is_negated() and not self.successes:
                add_fact(evaluations, self.get_blocked_fact(), result=INTERNAL_EVALUATION)
            return

        if self._axiom_predicate is not None:
            return
        index = len(self.external.disabled_instances)
        self.external.disabled_instances.append(self)
        self._axiom_predicate = '_ax{}-{}'.format(self.external.blocked_predicate, index)
        add_fact(evaluations, self.get_blocked_fact(), result=INTERNAL_EVALUATION)
        # TODO: allow reporting back which components lead to failure

        static_fact = Fact(self._axiom_predicate, self.external.inputs)
        preconditions = [static_fact] + list(self.fluent_facts)
        derived_fact = Fact(self.external.blocked_predicate, self.external.inputs)
        self._disabled_axiom = make_axiom(
            parameters=self.external.inputs,
            preconditions=preconditions,
            derived=derived_fact)
        domain.axioms.append(self._disabled_axiom)

    def enable(self, evaluations, domain):
        super(StreamInstance, self).enable(evaluations, domain)
        if self._axiom_predicate is not None: # TODO: re-enable?
            raise NotImplementedError(self)

    def __repr__(self):
        return '{}:{}->{}'.format(self.external.name, self.input_objects, self.external.outputs)

##################################################

class Stream(External):
    _Instance = StreamInstance
    def __init__(self, name, gen_fn, inputs, domain, outputs, certified, info, fluents=[], is_wild=False):
        super(Stream, self).__init__(name, info, inputs, domain)
        self.outputs = tuple(outputs)
        self.certified = tuple(certified)
        self.constants.update(a for i in certified for a in get_args(i) if not is_parameter(a))

        for p, c in Counter(self.outputs).items():
            if not is_parameter(p):
                raise ValueError('Output [{}] for stream [{}] is not a parameter'.format(p, name))
            if c != 1:
                raise ValueError('Output [{}] for stream [{}] is not unique'.format(p, name))
        for p in set(self.inputs) & set(self.outputs):
            raise ValueError('Parameter [{}] for stream [{}] is both an input and output'.format(p, name))
        certified_parameters = {a for i in certified for a in get_args(i) if is_parameter(a)}
        for p in (certified_parameters - set(self.inputs + self.outputs)):
            raise ValueError('Parameter [{}] for stream [{}] is not included within outputs'.format(p, name))
        for p in (set(self.outputs) - certified_parameters):
            print('Warning! Output [{}] for stream [{}] is not covered by a certified condition'.format(p, name))

        # TODO: automatically switch to unique if only used once
        self.gen_fn = get_debug_gen_fn(self) if gen_fn == DEBUG else gen_fn
        assert callable(self.gen_fn)
        self.num_opt_fns = 1 if self.outputs else 0 # Always unique if no outputs
        if isinstance(self.info.opt_gen_fn, PartialInputs) and self.info.opt_gen_fn.unique:
            self.num_opt_fns = 0
        #self.bound_list_fn = None # TODO: generalize to a hierarchical sequence
        #self.opt_fns = [get_unique_fn(self), get_shared_fn(self)] # get_unique_fn | get_shared_fn

        self.fluents = [] if gen_fn == DEBUG else fluents
        if NEGATIVE_BLOCKED:
            self.blocked_predicate = '~{}{}'.format(self.name, NEGATIVE_SUFFIX) # Args are self.inputs
        else:
            self.blocked_predicate = '~{}'.format(self.name)
        self.disabled_instances = []
        self.is_wild = is_wild
        self.stream_fact = Fact('_{}'.format(name), concatenate(inputs, outputs)) # TODO: just add to certified?

        if self.is_negated():
            if self.outputs:
                raise ValueError('Negated streams cannot have outputs: {}'.format(self.outputs))
            #assert len(self.certified) == 1 # TODO: is it okay to have more than one fact?
            for certified in self.certified:
                if not (set(self.inputs) <= set(get_args(certified))):
                    raise ValueError('Negated streams must have certified facts including all input parameters')

    def is_fluent(self):
        return self.fluents

    def is_negated(self):
        return self.info.negate

    def get_instance(self, input_objects, fluent_facts=frozenset()):
        assert all(isinstance(obj, Object) or isinstance(obj, OptimisticObject) for obj in input_objects)
        key = (tuple(input_objects), frozenset(fluent_facts))
        if key not in self.instances:
            self.instances[key] = self._Instance(self, input_objects, fluent_facts)
        return self.instances[key]

    # TODO: method that converts a stream into a test stream

    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)

##################################################

def parse_stream(lisp_list, stream_map, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {':stream', ':wild-stream', ':inputs',
                                         ':domain', ':fluents', ':outputs', ':certified'}
    is_wild = (':wild-stream' in value_from_attribute)
    name = value_from_attribute[':wild-stream'] if is_wild else value_from_attribute[':stream']
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
                  stream_info.get(name, StreamInfo()),
                  fluents=value_from_attribute.get(':fluents', []),
                  is_wild=is_wild)
