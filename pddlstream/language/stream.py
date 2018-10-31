import time
from collections import Counter, defaultdict, namedtuple, Sequence
from itertools import count

from pddlstream.algorithms.downward import make_preconditions, make_parameters
from pddlstream.language.constants import AND, get_prefix, get_args, is_parameter
from pddlstream.language.conversion import list_from_conjunction, remap_objects, \
    substitute_expression, get_formula_operators, evaluation_from_fact, values_from_objects, obj_from_value_expression
from pddlstream.language.external import ExternalInfo, Result, Instance, External, DEBUG, get_procedure_fn, \
    parse_lisp_list
from pddlstream.language.generator import get_next, from_fn
from pddlstream.language.object import Object, OptimisticObject, UniqueOptValue
from pddlstream.utils import str_from_object, get_mapping, irange

VERBOSE_FAILURES = True
INTERNAL = False
DEFAULT_UNIQUE = False
NEGATIVE_BLOCKED = True

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

OptValue = namedtuple('OptValue', ['stream', 'inputs', 'values', 'output'])

class PartialInputs(object):
    def __init__(self, inputs='', unique=False): #, num=1):
        self.inputs = tuple(inputs.split())
        self.unique = unique
        #self.num = num
    def get_opt_gen_fn(self, stream):
        inputs = stream.inputs if self.unique else self.inputs
        assert set(inputs) <= set(stream.inputs)
        # TODO: ensure no scoping error with inputs
        def gen_fn(*input_values):
            input_objects = tuple(map(Object.from_value, input_values))
            instance = stream.get_instance(input_objects)
            mapping = get_mapping(stream.inputs, input_objects)
            values = tuple(mapping[inp] for inp in inputs)
            assert(len(inputs) == len(values))
            #for _ in irange(self.num):
            for _ in irange(instance.num_optimistic):
                yield [tuple(OptValue(stream.name, inputs, values, out)
                             for out in stream.outputs)]
        return gen_fn

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
    return from_fn(lambda *args: tuple(DebugValue(stream.name, args, o)
                                       for o in stream.outputs))

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
    def __init__(self, opt_gen_fn=PartialInputs(unique=DEFAULT_UNIQUE), eager=False,
                 p_success=None, overhead=None, negate=False, effort_fn=None, simultaneous=False):
        # TODO: could change frequency/priority for the incremental algorithm
        super(StreamInfo, self).__init__(eager, p_success, overhead, effort_fn)
        self.opt_gen_fn = opt_gen_fn
        self.negate = negate
        self.simultaneous = simultaneous
        #self.order = 0

class StreamResult(Result):
    def __init__(self, instance, output_objects, opt_index=None, call_index=None, list_index=None):
        super(StreamResult, self).__init__(instance, opt_index)
        self.output_objects = tuple(output_objects)
        self.mapping = get_mapping(self.external.outputs, self.output_objects)
        self.mapping.update(instance.mapping)
        self.certified = substitute_expression(self.external.certified, self.get_mapping())
        self.call_index = call_index
        self.list_index = list_index
    def get_mapping(self):
        return self.mapping
    def get_certified(self):
        return self.certified
    def get_tuple(self):
        return self.external.name, self.instance.input_objects, self.output_objects
    def remap_inputs(self, bindings):
        input_objects = remap_objects(self.instance.input_objects, bindings)
        fluent_facts = [(get_prefix(f),) + remap_objects(get_args(f), bindings)
                        for f in self.instance.fluent_facts]
        new_instance = self.external.get_instance(input_objects, fluent_facts=fluent_facts)
        new_instance.opt_index = self.instance.opt_index
        return self.__class__(new_instance, self.output_objects, self.opt_index)
    def is_successful(self):
        return True
    def __repr__(self):
        return '{}:{}->{}'.format(self.external.name,
                                  str_from_object(self.instance.input_objects),
                                  str_from_object(self.output_objects))

class StreamInstance(Instance):
    def __init__(self, stream, input_objects, fluent_facts):
        super(StreamInstance, self).__init__(stream, input_objects)
        self._generator = None
        self.opt_index = stream.num_opt_fns
        self.fluent_facts = frozenset(fluent_facts)
        self.axiom_predicate = None
        self.disabled_axiom = None
        self.num_optimistic = 1

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

    def get_result(self, object_objects, opt_index=None, list_index=None):
        return self.external._Result(self, tuple(object_objects), opt_index=opt_index,
                                     call_index=self.num_calls, list_index=list_index)

    def use_unique(self):
        return self.opt_index == 0

    def get_fluent_values(self):
        return [(get_prefix(f),) + values_from_objects(get_args(f)) for f in self.fluent_facts]

    def _create_generator(self):
        if self._generator is None:
            input_values = self.get_input_values()
            #try:
            if self.external.is_fluent(): # self.fluent_facts
                self._generator = self.external.gen_fn(*input_values, fluents=self.get_fluent_values())
            else:
                self._generator = self.external.gen_fn(*input_values)
            #except TypeError as err:
            #    print('Stream [{}] expects {} inputs'.format(self.external.name, len(input_values)))
            #    raise err

    def _next_outputs(self):
        self._create_generator()
        output, self.enumerated = get_next(self._generator, default=None)
        if output is None:
            return [], []
        if not self.external.is_wild:
            return output, []
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
            new_results = [self.get_result(map(Object.from_value, output_values), list_index=list_index)
                           for list_index, output_values in enumerate(new_values)]
            all_new_values.extend(new_values)
            all_new_facts.extend(new_facts)
            all_results.extend(new_results)
            self.update_statistics(start_time, new_results)
        if verbose and (VERBOSE_FAILURES or all_new_values):
            print('{}-{}) {}:{}->{}'.format(start_calls, self.num_calls, self.external.name,
                                            str_from_object(self.get_input_values()),
                                            str_from_object(all_new_values)))
        if verbose and all_new_facts:
            # TODO: format all_new_facts
            print('{}-{}) {}:{}->{}'.format(start_calls, self.num_calls, self.external.name,
                                            str_from_object(self.get_input_values()), all_new_facts))
        return all_results, list(map(obj_from_value_expression, all_new_facts))

    def next_optimistic(self):
        # TODO: compute this just once and store
        if self.enumerated or self.disabled:
            return []
        # TODO: (potentially infinite) sequence of optimistic objects
        # TODO: how do I distinguish between real and not real verifications of things?
        # TODO: resue these?
        self.opt_results = []
        output_set = set()
        for output_list in self.external.opt_gen_fn(*self.get_input_values()):
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
                    self.opt_results.append(self.external._Result(self, output_objects, opt_index=self.opt_index,
                                                                  call_index=len(self.opt_results), list_index=0))
        return self.opt_results

    def get_blocked_fact(self):
        if self.external.is_fluent():
            assert self.axiom_predicate is not None
            return (self.axiom_predicate,) + self.input_objects
        return (self.external.blocked_predicate,) + self.input_objects

    def disable(self, evaluations, domain):
        #assert not self.disabled
        super(StreamInstance, self).disable(evaluations, domain)
        if not self.external.is_fluent(): # self.fluent_facts:
            if self.external.is_negated() and not self.successes:
                evaluations[evaluation_from_fact(self.get_blocked_fact())] = INTERNAL
            return

        if self.axiom_predicate is not None:
            return
        index = len(self.external.disabled_instances)
        self.external.disabled_instances.append(self)
        self.axiom_predicate = '_ax{}-{}'.format(self.external.blocked_predicate, index)
        evaluations[evaluation_from_fact(self.get_blocked_fact())] = INTERNAL
        # TODO: allow reporting back which components lead to failure

        import pddl
        static_fact = (self.axiom_predicate,) + self.external.inputs
        preconditions = [static_fact] + list(self.fluent_facts)
        self.disabled_axiom = pddl.Axiom(name=self.external.blocked_predicate,
                                         parameters=make_parameters(self.external.inputs),
                                         num_external_parameters=len(self.external.inputs),
                                         condition=make_preconditions(preconditions))
        domain.axioms.append(self.disabled_axiom)

    def enable(self, evaluations, domain):
        super(StreamInstance, self).enable(evaluations, domain)
        if self.axiom_predicate is not None: # TODO: re-enable?
            raise NotImplementedError(self)

    def __repr__(self):
        return '{}:{}->{}'.format(self.external.name, self.input_objects, self.external.outputs)

class Stream(External):
    _Instance = StreamInstance
    _Result = StreamResult
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
        self.num_opt_fns = 1 if self.outputs else 0 # Always unique if no outputs
        if isinstance(self.info.opt_gen_fn, PartialInputs):
            if self.info.opt_gen_fn.unique:
                self.num_opt_fns = 0
            self.opt_gen_fn = self.info.opt_gen_fn.get_opt_gen_fn(self)
        else:
            self.opt_gen_fn = self.info.opt_gen_fn
        #self.bound_list_fn = None # TODO: generalize to a hierarchical sequence
        #self.opt_fns = [get_unique_fn(self), get_shared_fn(self)] # get_unique_fn | get_shared_fn

        self.fluents = [] if gen_fn == DEBUG else fluents
        if NEGATIVE_BLOCKED:
            self.blocked_predicate = '~{}-negative'.format(self.name) # Args are self.inputs
        else:
            self.blocked_predicate = '~{}'.format(self.name)
        self.disabled_instances = []
        self.is_wild = is_wild

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
        key = (tuple(input_objects), frozenset(fluent_facts))
        if key not in self.instances:
            self.instances[key] = self._Instance(self, input_objects, fluent_facts)
        return self.instances[key]

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
