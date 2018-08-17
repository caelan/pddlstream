import time
from collections import Counter, defaultdict, namedtuple, Sequence
from itertools import count

from pddlstream.language.conversion import list_from_conjunction, dnf_from_positive_formula, remap_objects, \
    substitute_expression, get_args, is_parameter, get_formula_operators, AND, OR, get_prefix, \
    evaluation_from_fact, values_from_objects, obj_from_value_expression
from pddlstream.language.external import ExternalInfo, Result, Instance, External, DEBUG, get_procedure_fn, parse_lisp_list
from pddlstream.language.generator import get_next, from_fn
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.algorithms.downward import OBJECT, fd_from_fact
from pddlstream.utils import str_from_tuple

VEBOSE_FAILURES = True
INTERNAL = False

# TODO: could also make only wild facts and automatically identify output tuples satisfying certified

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
                 p_success=None, overhead=None, negate=False):
        # TODO: could change frequency/priority for the incremental algorithm
        super(StreamInfo, self).__init__(eager, p_success, overhead)
        self.opt_gen_fn = opt_gen_fn
        self.negate = negate
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
        input_objects = remap_objects(self.instance.input_objects, bindings)
        fluent_facts = [(get_prefix(f),) + remap_objects(get_args(f), bindings)
                        for f in self.instance.fluent_facts]
        new_instance = self.instance.external.get_instance(input_objects, fluent_facts=fluent_facts)
        return self.__class__(new_instance, self.output_objects, self.opt_index)
    def is_successful(self):
        return True
    def __repr__(self):
        return '{}:{}->{}'.format(self.instance.external.name,
                                  str_from_tuple(self.instance.input_objects),
                                  str_from_tuple(self.output_objects))

class StreamInstance(Instance):
    def __init__(self, stream, input_objects, fluent_facts):
        super(StreamInstance, self).__init__(stream, input_objects)
        self._generator = None
        self.opt_index = stream.num_opt_fns
        self.fluent_facts = frozenset(fluent_facts)
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

    def get_fluent_values(self):
        return [(get_prefix(f),) + values_from_objects(get_args(f)) for f in self.fluent_facts]

    def _create_generator(self):
        if self._generator is None:
            input_values = self.get_input_values()
            try:
                if self.external.is_fluent(): # self.fluent_facts
                    self._generator = self.external.gen_fn(*input_values, fluents=self.get_fluent_values())
                else:
                    self._generator = self.external.gen_fn(*input_values)
            except TypeError as err:
                print('Stream [{}] expects {} inputs'.format(self.external.name, len(input_values)))
                raise err

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
            all_new_values.extend(new_values)
            all_new_facts.extend(new_facts)
            new_results = [self.external._Result(self, tuple(map(Object.from_value, ov))) for ov in new_values]
            all_results.extend(new_results)
            self.update_statistics(start_time, new_results)
        if verbose and (VEBOSE_FAILURES or all_new_values):
            print('{}-{}) {}:{}->[{}]'.format(start_calls, self.num_calls, self.external.name,
                                           str_from_tuple(self.get_input_values()),
                                       ', '.join(map(str_from_tuple, all_new_values))))
        if verbose and all_new_facts:
            print('{}-{}) {}:{}->{}'.format(start_calls, self.num_calls, self.external.name,
                                            str_from_tuple(self.get_input_values()), all_new_facts))
        return all_results, list(map(obj_from_value_expression, all_new_facts))
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
    def get_blocked_fact(self):
        return (self.external.blocked_predicate,) + self.input_objects

    def disable(self, evaluations, domain):
        #assert not self.disabled
        super(StreamInstance, self).disable(evaluations, domain)
        if not self.external.is_fluent(): # self.fluent_facts:
            if self.external.is_negated():
                evaluations[evaluation_from_fact(self.get_blocked_fact())] = INTERNAL
            return

        # TODO: re-enable
        import pddl
        index = len(self.external.disabled_instances)
        self.external.disabled_instances.append(self)

        negated_name = self.external.blocked_predicate
        static_name = '_ax{}-{}'.format(negated_name, index)
        static_eval = evaluation_from_fact((static_name,) + self.input_objects)
        evaluations[static_eval] = INTERNAL

        parameters = tuple(pddl.TypedObject(p, OBJECT) for p in self.external.inputs)
        static_atom = fd_from_fact((static_name,) + self.external.inputs)
        precondition = pddl.Conjunction([static_atom] + list(map(fd_from_fact, self.fluent_facts)))
        domain.axioms.append(pddl.Axiom(name=negated_name, parameters=parameters,
                                        num_external_parameters=len(self.external.inputs),
                                        condition=precondition))

    def __repr__(self):
        return '{}:{}->{}'.format(self.external.name, self.input_objects, self.external.outputs)

class Stream(External):
    _Instance = StreamInstance
    _Result = StreamResult
    def __init__(self, name, gen_fn, inputs, domain, outputs, certified, info, fluents=[], is_wild=False):
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
            self.num_opt_fns = 1 if self.outputs else 0 # Always unique if no outputs
            self.opt_gen_fn = get_shared_gen_fn(self) if (self.info.opt_gen_fn is None) else self.info.opt_gen_fn
        #self.bound_list_fn = None
        #self.opt_fns = [get_unique_fn(self), get_shared_fn(self)] # get_unique_fn | get_shared_fn
        #self.opt_fns = [get_unique_fn(self)] # get_unique_fn | get_shared_fn

        self.fluents = fluents
        self.blocked_predicate = '~{}'.format(self.name) # Args are self.inputs
        self.disabled_instances = []
        self.is_wild = is_wild

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
    assert set(value_from_attribute) <= {':stream', ':wild-stream', ':inputs', ':domain', ':fluents', ':outputs', ':certified'}
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
                  stream_info.get(name, None),
                  fluents=value_from_attribute.get(':fluents', []),
                  is_wild=is_wild)
