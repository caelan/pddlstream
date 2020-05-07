from collections import Counter

from pddlstream.algorithms.common import compute_complexity
from pddlstream.language.constants import get_args, is_parameter, get_prefix
from pddlstream.language.conversion import values_from_objects, substitute_fact
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.language.statistics import Performance, PerformanceInfo, DEFAULT_SEARCH_OVERHEAD
from pddlstream.utils import elapsed_time, get_mapping, flatten

DEBUG = 'debug'

never_defer = lambda *args, **kwargs: False
defer_unique = lambda result, *args, **kwargs: result.is_refined()
defer_shared = lambda *args, **kwargs: True

def select_inputs(instance, inputs):
    external = instance.external
    assert set(inputs) <= set(external.inputs)
    mapping = get_mapping(external.inputs, instance.input_objects)
    return tuple(mapping[inp] for inp in inputs)

def get_defer_any_unbound(unique=False):
    def defer_any_unbound(result, bound_objects=set(), *args, **kwargs):
        # The set bound_objects may contain shared objects in which case replanning is required
        if unique and not defer_unique(result):
            return False
        return not all(isinstance(obj, Object) or (obj in bound_objects) for obj in result.input_objects)
    return defer_any_unbound

def get_defer_all_unbound(inputs='', unique=False): # TODO: shortcut for all inputs
    inputs = tuple(inputs.split())
    # Empty implies defer_shared

    def defer_all_unbound(result, bound_objects=set(), *args, **kwargs):
        if unique and not defer_unique(result):
            return False
        return not any(isinstance(obj, Object) or (obj in bound_objects)
                       for obj in select_inputs(result.instance, inputs))
    return defer_all_unbound

def get_domain_predicates(streams):
    return {get_prefix(a) for s in streams for a in s.domain}

##################################################

class ExternalInfo(PerformanceInfo):
    def __init__(self, eager=False, p_success=None, overhead=None, effort=None, defer_fn=never_defer):
        super(ExternalInfo, self).__init__(p_success, overhead, effort)
        # TODO: enable eager=True for inexpensive test streams by default
        # TODO: make any info just a dict
        self.eager = eager
        self.defer_fn = defer_fn
        #self.complexity_fn = complexity_fn

##################################################

class Result(object):
    def __init__(self, instance, opt_index, call_index, optimistic):
        self.instance = instance
        self.opt_index = opt_index
        self.call_index = call_index
        self.optimistic = optimistic

    @property
    def external(self):
        return self.instance.external

    @property
    def info(self):
        return self.external.info

    @property
    def name(self):
        return self.external.name

    @property
    def input_objects(self):
        return self.instance.input_objects

    def is_refined(self):
        return self.opt_index == 0 # TODO: base on output objects instead?

    def is_deferrable(self, *args, **kwargs):
        return self.info.defer_fn(self, *args, **kwargs)

    def get_domain(self):
        return self.instance.get_domain()

    def get_certified(self):
        raise NotImplementedError()

    def get_components(self):
        return [self]

    def get_unsatisfiable(self):
        return [self.get_components()]

    def get_action(self):
        raise NotImplementedError()

    def remap_inputs(self, bindings):
        raise NotImplementedError()

    def is_successful(self):
        raise NotImplementedError()

    def compute_complexity(self, evaluations):
        # Should be constant
        return compute_complexity(evaluations, self.get_domain()) + \
               self.external.get_complexity(self.call_index)

    def get_effort(self, **kwargs):
        if not self.optimistic:
            return 0  # Unit efforts?
        if self.external.is_negated():
            return 0
        # TODO: this should be the min of all instances
        return self.instance.get_effort(**kwargs)

##################################################

class Instance(object):
    _Result = None
    def __init__(self, external, input_objects):
        self.external = external
        self.input_objects = tuple(input_objects)
        self.disabled = False # TODO: perform disabled using complexity
        self.history = [] # TODO: facts history
        self.results_history = []
        self.opt_results = []
        self._mapping = None
        self._domain = None
        self.reset()

    @property
    def info(self):
        return self.external.info

    @property
    def mapping(self):
        if self._mapping is None:
            self._mapping = get_mapping(self.external.inputs, self.input_objects)
            for constant in self.external.constants:
                self._mapping[constant] = Object.from_name(constant)
        return self._mapping

    def get_mapping(self):
        return self.mapping

    @property
    def domain(self):
        if self._domain is None:
            #self._domain = substitute_expression(self.external.domain, self.get_mapping())
            self._domain = tuple(substitute_fact(atom, self.get_mapping())
                                 for atom in self.external.domain)
        return self._domain

    def get_domain(self):
        return self.domain

    def get_objects(self):
        return set(self.input_objects)

    def get_input_values(self):
        return values_from_objects(self.input_objects)

    #def is_first_call(self): # TODO: use in streams
    #    return self.online_calls == 0
    #
    #def has_previous_success(self):
    #    return self.online_success != 0

    def reset(self):
        #self.enable(evaluations={}, domain=None)
        self.disabled = False
        self.opt_index = self.external.num_opt_fns
        self.num_calls = 0
        self.enumerated = False
        self.successful = False

    def next_results(self, verbose=False):
        raise NotImplementedError()

    def all_results(self, **kwargs):
        while not self.enumerated:
            self.next_results(**kwargs)
        return self.get_results()

    def get_results(self, start=0):
        results = []
        for index in range(start, self.num_calls):
            results.extend(self.results_history[index])
        return results

    def compute_complexity(self, evaluations):
        # Will change as self.num_calls increases
        return compute_complexity(evaluations, self.get_domain()) + \
               self.external.get_complexity(self.num_calls)

    def get_effort(self, search_overhead=DEFAULT_SEARCH_OVERHEAD):
        # TODO: handle case where resampled several times before the next search (search every ith time)
        replan_effort = self.opt_index * search_overhead  # By linearity of expectation
        effort_fn = self.external.info.effort
        if callable(effort_fn):
            return replan_effort + effort_fn(*self.get_input_values())
        return replan_effort + self.external.get_effort(search_overhead=search_overhead)

    def update_statistics(self, start_time, results):
        overhead = elapsed_time(start_time)
        successes = len([r.is_successful() for r in results])
        self.external.update_statistics(overhead, bool(successes))
        self.results_history.append(results)
        #self.successes += successes

    def disable(self, evaluations, domain):
        self.disabled = True

    def enable(self, evaluations, domain):
        self.disabled = False

##################################################

class External(Performance):
    _Instance = None
    def __init__(self, name, info, inputs, domain):
        super(External, self).__init__(name, info)
        self.inputs = tuple(inputs)
        self.domain = tuple(domain)
        for p, c in Counter(self.inputs).items():
            if not is_parameter(p):
                # AssertionError: Expected item to be a variable: q2 in (?q1 q2)
                raise ValueError('Input [{}] for stream [{}] is not a parameter'.format(p, name))
            if c != 1:
                raise ValueError('Input [{}] for stream [{}] is not unique'.format(p, name))
        parameters = {a for i in self.domain for a in get_args(i) if is_parameter(a)}
        for p in (parameters - set(self.inputs)):
            raise ValueError('Parameter [{}] for stream [{}] is not included within inputs'.format(p, name))
        for p in (set(self.inputs) - parameters):
            print('Warning! Input [{}] for stream [{}] is not covered by a domain condition'.format(p, name))
        self.constants = {a for i in self.domain for a in get_args(i) if not is_parameter(a)}
        self.instances = {}
        self.num_opt_fns = 0
    def reset(self, *args, **kwargs):
        for instance in self.instances.values():
            instance.reset(*args, **kwargs)
    def is_fluent(self):
        raise NotImplementedError()
    def is_negated(self):
        raise NotImplementedError()
    def is_special(self):
        return False
    def get_complexity(self, num_calls):
        raise NotImplementedError()
    def get_instance(self, input_objects):
        input_objects = tuple(input_objects)
        assert len(input_objects) == len(self.inputs)
        if input_objects not in self.instances:
            self.instances[input_objects] = self._Instance(self, input_objects)
        return self.instances[input_objects]

##################################################

def get_procedure_fn(stream_map, name):
    if stream_map == DEBUG:
        return DEBUG
    if name not in stream_map:
        raise ValueError('Undefined external procedure: {}'.format(name))
    return stream_map[name]

def is_attribute(attribute):
    return isinstance(attribute, str) and attribute.startswith(':')

def parse_lisp_list(lisp_list):
    attributes = [lisp_list[i] for i in range(0, len(lisp_list), 2)]
    for attribute in attributes:
        if not is_attribute(attribute):
            raise ValueError('Expected an attribute but got: {}'.format(attribute))
    values = [lisp_list[i] for i in range(1, len(lisp_list), 2)]
    if len(lisp_list) % 2 != 0:
        raise ValueError('No value specified for attribute [{}]'.format(lisp_list[-1]))
    return get_mapping(attributes, values)
