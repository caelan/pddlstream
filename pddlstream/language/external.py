from collections import Counter

from pddlstream.language.conversion import substitute_expression, values_from_objects
from pddlstream.language.constants import get_args, is_parameter
from pddlstream.language.object import Object
from pddlstream.language.statistics import geometric_cost, Performance
from pddlstream.utils import elapsed_time, get_mapping, INF

DEBUG = 'debug'

class ExternalInfo(object):
    def __init__(self, eager, p_success, overhead, effort_fn):
        # TODO: enable eager=True for inexpensive test streams by default
        self.eager = eager
        self.p_success = p_success
        self.overhead = overhead
        self.effort_fn = effort_fn
        # TODO: allow specification of p_success & overhead as effort

##################################################

class Result(object):
    def __init__(self, instance, opt_index):
        self.instance = instance
        self.opt_index = opt_index

    @property
    def external(self):
        return self.instance.external

    def get_domain(self):
        return self.instance.get_domain()

    def get_certified(self):
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
        self.mapping = get_mapping(self.external.inputs, self.input_objects)
        for constant in self.external.constants:
            self.mapping[constant] = Object.from_name(constant)
        self.domain = substitute_expression(self.external.domain, self.get_mapping())
        self.successes = 0
        self.opt_results = []

    def update_statistics(self, start_time, results):
        overhead = elapsed_time(start_time)
        successes = len([r.is_successful() for r in results])
        self.external.update_statistics(overhead, bool(successes))
        self.results_history.append(results)
        self.successes += successes

    #def get_result(self):
    #    raise NotImplementedError()

    @property
    def num_calls(self):
        return len(self.results_history)

    # def get_belief(self):
    #     #return 1.
    #     #prior = self.external.prior
    #     prior = 1. - 1e-2
    #     n = self.num_calls
    #     p_obs_given_state = self.external.get_p_success()
    #     p_state = prior
    #     for i in range(n):
    #         p_nobs_and_state = (1-p_obs_given_state)*p_state
    #         p_nobs_and_nstate = (1-p_state)
    #         p_nobs = p_nobs_and_state + p_nobs_and_nstate
    #         p_state = p_nobs_and_state/p_nobs
    #     return p_state

    # def update_belief(self, success):
    #     # Belief that remaining sequence is non-empty
    #     # Belief only degrades in this case
    #     nonempty = 0.9
    #     p_success_nonempty = 0.5
    #     if success:
    #         p_success = p_success_nonempty*nonempty
    #     else:
    #         p_success = (1-p_success_nonempty)*nonempty + (1-nonempty)

    #def get_p_success(self):
        #p_success_belief = self.external.get_p_success()
        #belief = self.get_belief()
        #return p_success_belief*belief
        # TODO: use the external as a prior
        # TODO: Bayesian estimation of likelihood that has result
        # Model hidden state of whether has values or if will produce values?
        # TODO: direct estimation of different buckets in which it will finish
        # TODO: we have samples from the CDF or something

    #def get_p_success(self):
    #    return self.external.get_p_success()
    #
    #def get_overhead(self):
    #    return self.external.get_overhead()

    def get_effort(self):
        if self.external.info.effort_fn is not None:
            return self.external.info.effort_fn(*self.get_input_values())
        return geometric_cost(self.external.get_overhead(), self.external.get_p_success())

    def get_input_values(self):
        return values_from_objects(self.input_objects)

    def get_mapping(self):
        return self.mapping

    def get_domain(self):
        return self.domain

    #def is_first_call(self): # TODO: use in streams
    #    return self.online_calls == 0
    #
    #def has_previous_success(self):
    #    return self.online_success != 0

    def next_results(self, accelerate=1, verbose=False):
        raise NotImplementedError()

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
            if c != 1:
                raise ValueError('Input [{}] for stream [{}] is not unique'.format(p, name))
        parameters = {a for i in self.domain for a in get_args(i) if is_parameter(a)}
        for p in (parameters - set(self.inputs)):
            raise ValueError('Parameter [{}] for stream [{}] is not included within inputs'.format(p, name))
        self.constants = {a for i in self.domain for a in get_args(i) if not is_parameter(a)}
        self.instances = {}

    def get_instance(self, input_objects):
        input_objects = tuple(input_objects)
        if input_objects not in self.instances:
            self.instances[input_objects] = self._Instance(self, input_objects)
        return self.instances[input_objects]

##################################################

def get_plan_effort(stream_plan):
    if stream_plan is None:
        return INF
    if not stream_plan:
        return 0
    return sum(result.instance.get_effort() for result in stream_plan)

def get_procedure_fn(stream_map, name):
    if stream_map == DEBUG:
        return DEBUG
    if name not in stream_map:
        raise ValueError('Undefined external procedure: {}'.format(name))
    return stream_map[name]

def parse_lisp_list(lisp_list):
    assert(len(lisp_list) % 2 == 0)
    attributes = [lisp_list[i] for i in range(0, len(lisp_list), 2)]
    values = [lisp_list[i] for i in range(1, len(lisp_list), 2)]
    return get_mapping(attributes, values)

#def is_property(s):
#    return s.startswith(':')