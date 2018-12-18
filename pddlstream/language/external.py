from collections import Counter

from pddlstream.language.conversion import substitute_expression, values_from_objects
from pddlstream.language.constants import get_args, is_parameter, is_plan
from pddlstream.language.object import Object
from pddlstream.language.statistics import geometric_cost, Performance, PerformanceInfo
from pddlstream.utils import elapsed_time, get_mapping, INF

DEBUG = 'debug'

class ExternalInfo(PerformanceInfo):
    def __init__(self, eager, p_success, overhead, effort_fn):
        super(ExternalInfo, self).__init__(p_success, overhead)
        # TODO: enable eager=True for inexpensive test streams by default
        # TODO: make any info just a dict
        self.eager = eager
        self.effort_fn = effort_fn

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

    def get_domain(self):
        return self.instance.get_domain()

    def get_certified(self):
        raise NotImplementedError()

    def get_tuple(self):
        raise NotImplementedError()

    def remap_inputs(self, bindings):
        raise NotImplementedError()

    def is_successful(self):
        raise NotImplementedError()

##################################################

class Instance(object):
    _Result = None
    def __init__(self, external, input_objects):
        self.external = external
        self.input_objects = tuple(input_objects)
        self.enumerated = False
        self.disabled = False
        self.opt_index = 0
        self.results_history = []
        self.num_calls = len(self.results_history)
        self.successes = 0
        self.opt_results = []
        self._mapping = None
        self._domain = None

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
            self._domain = substitute_expression(self.external.domain, self.get_mapping())
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

    def next_results(self, accelerate=1, verbose=False):
        raise NotImplementedError()

    def get_results(self, start=0):
        results = []
        for index in range(start, self.num_calls):
            results.extend(self.results_history[index])
        return results

    def update_statistics(self, start_time, results):
        overhead = elapsed_time(start_time)
        successes = len([r.is_successful() for r in results])
        self.external.update_statistics(overhead, bool(successes))
        self.results_history.append(results)
        self.num_calls = len(self.results_history)
        self.successes += successes

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

    def get_instance(self, input_objects):
        input_objects = tuple(input_objects)
        if input_objects not in self.instances:
            self.instances[input_objects] = self._Instance(self, input_objects)
        return self.instances[input_objects]

##################################################

DEFAULT_SEARCH_OVERHEAD = 1e-2
# Can also include the overhead to process skeletons

def get_unit_effort(effort):
    if (effort == 0) or (effort == INF):
        return effort
    return 1

def compute_external_effort(external, unit_efforts=False, search_overhead=DEFAULT_SEARCH_OVERHEAD):
    effort_fn = external.info.effort_fn
    if effort_fn is None:
        p_success = external.get_p_success()
        effort = geometric_cost(external.get_overhead(), p_success) + \
                 (1-p_success)*geometric_cost(search_overhead, p_success)
    elif callable(effort_fn):
        effort = 0 # This really is a bound on the effort
    else:
        effort = float(effort_fn)
    return get_unit_effort(effort) if unit_efforts else effort

def compute_instance_effort(instance, unit_efforts=False, search_overhead=DEFAULT_SEARCH_OVERHEAD):
    # TODO: handle case where resampled several times before the next search (search every ith time)
    effort = instance.opt_index * search_overhead # By linearity of expectation
    effort_fn = instance.external.info.effort_fn
    if effort_fn is None:
        effort += compute_external_effort(
            instance.external, unit_efforts=False, search_overhead=search_overhead)
    elif callable(effort_fn):
        effort += effort_fn(*instance.get_input_values())
    else:
        effort += float(effort_fn)
    return get_unit_effort(effort) if unit_efforts else effort

def compute_result_effort(result, **kwargs):
    if not result.optimistic:
        return 0 # Unit efforts?
    return compute_instance_effort(result.instance, **kwargs)

def compute_plan_effort(stream_plan, **kwargs):
    if not is_plan(stream_plan):
        return INF
    return sum((compute_result_effort(result, **kwargs) for result in stream_plan), 0)

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
