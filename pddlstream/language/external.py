from collections import Counter

from pddlstream.algorithms.common import compute_complexity
from pddlstream.language.constants import get_args, is_parameter, get_prefix, Fact
from pddlstream.language.conversion import values_from_objects, substitute_fact, obj_from_value_expression
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.language.statistics import Performance, PerformanceInfo, DEFAULT_SEARCH_OVERHEAD, Stats
from pddlstream.utils import elapsed_time, get_mapping, flatten, INF, safe_apply_mapping, Score, INF

DEBUG = 'debug'
SHARED_DEBUG = 'shared_debug'
DEBUG_MODES = [DEBUG, SHARED_DEBUG]

never_defer = lambda *args, **kwargs: False
defer_unique = lambda result, *args, **kwargs: result.is_refined()
defer_shared = lambda *args, **kwargs: True

def select_inputs(instance, inputs):
    external = instance.external
    assert set(inputs) <= set(external.inputs)
    mapping = get_mapping(external.inputs, instance.input_objects)
    return safe_apply_mapping(inputs, mapping)

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

def convert_constants(fact):
    # TODO: take the constant map as an input
    # TODO: throw an error if undefined
    return Fact(get_prefix(fact), [p if is_parameter(p) else Object.from_name(p) for p in get_args(fact)])

##################################################

class ExternalInfo(PerformanceInfo):
    def __init__(self, eager=False, eager_skeleton=False, defer_fn=never_defer, **kwargs):
        super(ExternalInfo, self).__init__(**kwargs)
        # TODO: enable eager=True for inexpensive test streams by default
        # TODO: change p_success and overhead if it's a function or test stream
        self.eager = eager
        self.eager_skeleton = eager_skeleton # TODO: apply in binding and adaptive
        # TODO: automatically set tests and costs to be eager
        self.defer_fn = defer_fn # Old syntax was defer=True
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
    @property
    def domain(self):
        return self.instance.domain
    def is_refined(self):
        # TODO: result.opt_index is None
        return self.opt_index == 0 # TODO: base on output objects instead
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
    def compute_complexity(self, evaluations, **kwargs):
        # Should be constant
        return compute_complexity(evaluations, self.get_domain(), **kwargs) + \
               self.external.get_complexity(self.call_index)
    def get_effort(self, **kwargs):
        if not self.optimistic:
            return 0  # Unit efforts?
        if self.external.is_negated:
            return 0
        # TODO: this should be the min of all instances
        return self.instance.get_effort(**kwargs)
    def success_heuristic(self): # High is likely to succeed
        # self.external.is_function
        num_free = sum(isinstance(obj, OptimisticObject) for obj in self.input_objects)
        return Score(num_free, -len(self.external.inputs)) # TODO: treat objects in the same domain as a unit
    def overhead_heuristic(self): # Low is cheap
        return self.external.overhead_heuristic()
    def stats_heuristic(self): # Low is cheap and unlikely to succeed
        #return self.overhead_heuristic() + self.success_heuristic()
        return Score(self.overhead_heuristic(), self.success_heuristic())
        #return Stats(self.overhead_heuristic(), self.success_heuristic())
    def effort_heuristic(self): # Low is cheap and likely to succeed
        return Score(self.overhead_heuristic(), -self.success_heuristic())

##################################################

class Instance(object):
    _Result = None
    def __init__(self, external, input_objects):
        self.external = external
        self.input_objects = tuple(input_objects)
        self.disabled = False # TODO: perform disabled using complexity
        self.history = [] # TODO: facts history
        self.results_history = []
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
            #for constant in self.external.constants: # TODO: no longer needed
            #    self._mapping[constant] = Object.from_name(constant)
        return self._mapping
    @property
    def domain(self):
        if self._domain is None:
            #self._domain = substitute_expression(self.external.domain, self.mapping)
            self._domain = tuple(substitute_fact(atom, self.mapping)
                                 for atom in self.external.domain)
        return self._domain
    def get_iteration(self):
        return INF if self.enumerated else self.num_calls
    def get_domain(self):
        return self.domain
    def get_all_input_objects(self):
        return set(self.input_objects)
    def get_input_values(self):
        return values_from_objects(self.input_objects)
    #def is_first_call(self): # TODO: use in streams
    #    return self.online_calls == 0
    #def has_previous_success(self):
    #    return self.online_success != 0
    def reset(self):
        #self.enable(evaluations={}, domain=None)
        self.disabled = False
        self.opt_index = self.external.num_opt_fns
        self.num_calls = 0
        self.enumerated = False
        self.successful = False
    def is_refined(self):
        return self.opt_index == 0
    def refine(self):
        # TODO: could instead create a new Instance per opt_index
        if not self.is_refined():
            self.opt_index -= 1
        return self.opt_index

    def next_results(self, verbose=False):
        raise NotImplementedError()

    def first_results(self, num=1, **kwargs):
        results = []
        index = 0
        while len(results) < num:
            while index >= len(self.results_history):
                if self.enumerated:
                    return results
                self.next_results(**kwargs)
            results.extend(self.results_history[index])
            index += 1
        return results

    def all_results(self, **kwargs):
        return self.first_results(num=INF, **kwargs)

    def get_results(self, start=0):
        results = []
        for index in range(start, self.num_calls):
            results.extend(self.results_history[index])
        return results

    def compute_complexity(self, evaluations, **kwargs):
        # Will change as self.num_calls increases
        #num_calls = INF if self.enumerated else self.num_calls
        return compute_complexity(evaluations, self.get_domain(), **kwargs) + \
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
        successes = sum(r.is_successful() for r in results)
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
        self.domain = tuple(map(convert_constants, domain))
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
    def reset(self, *args, **kwargs):
        for instance in self.instances.values():
            instance.reset(*args, **kwargs)
    # TODO: naming convention for statics and fluents
    @property
    def has_outputs(self):
        raise NotImplementedError()
    @property
    def is_fluent(self):
        raise NotImplementedError()
    @property
    def is_negated(self):
        raise NotImplementedError()
    @property
    def is_special(self):
        return self.is_fluent or self.is_negated
    @property
    def is_function(self):
        raise NotImplementedError()
    @property
    def is_cost(self):
        return False
    @property
    def zero_complexity(self):
        return self.is_special or not self.has_outputs
    def get_complexity(self, num_calls=0):
        if self.zero_complexity:
            return 0
        return num_calls + 1
    def get_instance(self, input_objects):
        input_objects = tuple(input_objects)
        assert len(input_objects) == len(self.inputs)
        if input_objects not in self.instances:
            self.instances[input_objects] = self._Instance(self, input_objects)
        return self.instances[input_objects]
    def overhead_heuristic(self): # Low is little overhead
        # TODO: infer other properties from use in the context of a stream plan
        # TODO: use num_certified (only those that are in another stream) instead of num_outputs?
        #num_inputs = len(self.inputs)
        #num_domain = len(self.domain)
        return Score(self.is_fluent, not self.is_function, self.has_outputs, len(self.inputs)) # structural/relational overhead
        #overhead = 1e0*num_inputs + 1e1*num_outputs + 1e2*bool(num_fluents)
        #return overhead

##################################################

def get_procedure_fn(stream_map, name):
    if not isinstance(stream_map, dict): # DEBUG_MODES
        return stream_map
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
