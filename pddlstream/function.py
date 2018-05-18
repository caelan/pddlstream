from collections import Counter

from pddlstream.conversion import values_from_objects, substitute_expression, get_prefix, get_args, Equal, Not, is_head, \
    list_from_conjunction, is_parameter, str_from_head
from pddlstream.utils import str_from_tuple, INF
import time

DEBUG = 'debug'

class ExternalInfo(object):
    def __init__(self, eager, p_success, overhead):
        self.eager = eager
        self.p_success = p_success
        self.overhead = overhead

def geometric_cost(cost, p):
    if p == 0:
        return INF
    return cost/p

class FunctionInfo(ExternalInfo):
    def __init__(self, opt_fn=None, eager=False, p_success=None, overhead=None):
        super(FunctionInfo, self).__init__(eager, p_success, overhead)
        self.opt_fn = opt_fn
        #self.order = 0

##################################################

class Performance(object):
    def __init__(self, name, info):
        self.name = name.lower()
        self.info = info
        self.total_calls = 0
        self.total_overhead = 0
        self.total_successes = 0

    def update_statistics(self, overhead, success):
        self.total_calls += 1
        self.total_overhead += overhead
        self.total_successes += success

    def _estimate_p_success(self, reg_p_success=1, reg_calls=1):
        calls = self.total_calls + reg_calls
        if calls == 0:
            return reg_p_success
        # TODO: use prior from info instead?
        return float(self.total_successes + reg_p_success*reg_calls) / calls

    def _estimate_overhead(self, reg_overhead=0, reg_calls=1):
        calls = self.total_calls + reg_calls
        if calls == 0:
            return reg_overhead
        # TODO: use prior from info instead?
        return float(self.total_overhead + reg_overhead*reg_calls) / calls

    def get_p_success(self):
        if self.info.p_success is None:
            return self._estimate_p_success()
        return self.info.p_success

    def get_overhead(self):
        if self.info.overhead is None:
            return self._estimate_overhead()
        return self.info.overhead

    def get_effort(self):
        return geometric_cost(self.get_overhead(), self.get_p_success())

##################################################

class Result(object):
    def __init__(self, instance, opt_index):
        self.instance = instance
        self.opt_index = opt_index

    def get_certified(self):
        raise NotImplementedError()


class Instance(object):
    def __init__(self, external, input_objects):
        self.external = external
        self.input_objects = tuple(input_objects)
        self.enumerated = False
        self.disabled = False
        self.opt_index = 0
        self.total_calls = 0
        self.total_overhead = 0
        self.total_successes = 0
        self.results_history = []
        self.mapping = dict(zip(self.external.inputs, self.input_objects))
        self.domain = substitute_expression(self.external.domain, self.get_mapping())

    def update_statistics(self, start_time, results):
        overhead = time.time() - start_time
        success = len(results) != 0
        self.total_calls += 1
        self.total_overhead += overhead
        self.total_successes += success
        self.external.update_statistics(overhead, success)
        self.results_history.append(results)
        # TODO: update the global as well

    def get_p_success(self):
        return self.external.get_p_success()
        # TODO: use the external as a prior
        # TODO: Bayesian estimation of likelihood that has result
        # Model hidden state of whether has values or if will produce values?

    def get_overhead(self):
        return self.external.get_overhead()

    def get_effort(self):
        return geometric_cost(self.get_overhead(), self.get_p_success())

    def get_input_values(self):
        return values_from_objects(self.input_objects)

    def get_mapping(self):
        return self.mapping

    def get_domain(self):
        return self.domain

    def next_results(self, verbose=False):
        raise NotImplementedError()

class External(Performance):
    _Instance = None
    def __init__(self, name, info, inputs, domain):
        super(External, self).__init__(name, info)
        for p, c in Counter(inputs).items():
            if c != 1:
                raise ValueError('Input [{}] for stream [{}] is not unique'.format(p, name))
        for p in {a for i in domain for a in get_args(i) if is_parameter(a)} - set(inputs):
            raise ValueError('Parameter [{}] for stream [{}] is not included within inputs'.format(p, name))
        self.inputs = tuple(inputs)
        self.domain = tuple(domain)
        self.instances = {}

    def get_instance(self, input_objects):
        input_objects = tuple(input_objects)
        if input_objects not in self.instances:
            self.instances[input_objects] = self._Instance(self, input_objects)
        return self.instances[input_objects]


##################################################

class FunctionResult(Result):
    def __init__(self, instance, value, opt_index=None):
        super(FunctionResult, self).__init__(instance, opt_index)
        self.instance = instance
        self.value = value

    def get_certified(self):
        return [Equal(self.instance.get_head(), self.value)]

    def get_tuple(self):
        name = self.instance.external.name
        return name, self.instance.input_objects, self.value

    def __repr__(self):
        return '{}={}'.format(str_from_head(self.instance.get_head()), self.value)


class FunctionInstance(Instance):  # Head(Instance):
    #_opt_value = 0

    def get_head(self):
        return substitute_expression(self.external.head, self.get_mapping())

    def next_results(self, stream_plan=None, verbose=False):
        start_time = time.time()
        assert not self.enumerated
        self.enumerated = True
        input_values = self.get_input_values()
        try:
            value = self.external.fn(*input_values)
        except TypeError:
            raise TypeError('Function [{}] expects {} inputs'.format(self.external.name, len(input_values)))
        self.value = self.external._codomain(value)
        # TODO: cast the inputs and test whether still equal?
        #if not (type(self.value) is self.external._codomain):
        #if not isinstance(self.value, self.external._codomain):
        if self.value != value:
            raise ValueError('Function [{}] produced a nonintegral value [{}]. '
                             'FastDownward only supports integral costs. '
                             'To "use" real costs, scale each cost by a large factor, '
                             'capturing the most significant bits.'.format(self.external.name, self.value))
        if self.value < 0:
            raise ValueError('Function [{}] produced a negative value [{}]'.format(self.external.name, self.value))
        if verbose:
            print('{}{}={}'.format(get_prefix(self.external.head),
                                   str_from_tuple(self.get_input_values()), self.value))
        results = [self.external._Result(self, self.value)]
        if isinstance(self, PredicateInstance) and (self.value != self.external.opt_fn(*input_values)):
            self.update_statistics(start_time, []) # TODO: do this more automatically
        else:
            self.update_statistics(start_time, results)
        return results

    def next_optimistic(self):
        if self.enumerated or self.disabled:
            return []
        opt_value = self.external.opt_fn(*self.get_input_values())
        return [self.external._Result(self, opt_value, opt_index=self.opt_index)]

    def __repr__(self):
        # return '{}:{}->{}'.format(self.instance.external.name, self.instance.inputs, self.value)
        return '{}=?{}'.format(str_from_head(self.get_head()), self.external._codomain.__name__)


class Function(External):
    """
    An external nonnegative function F(i1, ..., ik) -> 0 <= int
    External functions differ from streams in that their output isn't an object
    """
    _Instance = FunctionInstance
    _Result = FunctionResult
    _codomain = int
    _default_p_success = 1
    _default_overhead = None
    def __init__(self, head, fn, domain, info):
        if info is None:
            info = FunctionInfo(p_success=self._default_p_success, overhead=self._default_overhead)
        super(Function, self).__init__(get_prefix(head), info, get_args(head), domain)
        self.head = head
        opt_fn = lambda *args: self._codomain()
        if fn == DEBUG:
            fn = opt_fn
        self.fn = fn
        self.opt_fn = opt_fn if (self.info.opt_fn is None) else self.info.opt_fn
    def __repr__(self):
        return '{}=?{}'.format(str_from_head(self.head), self._codomain.__name__)


##################################################

class PredicateResult(FunctionResult):
    def get_certified(self):
        expression = self.instance.get_head()
        if self.value:
            return [expression]
        return [Not(expression)]


class PredicateInstance(FunctionInstance):
    #_opt_value = True # TODO: make this False to be consistent with Function?
    #_opt_value = Predicate._codomain()
    #_opt_value = False
    pass


class Predicate(Function):
    """
    An external predicate P(i1, ..., ik) -> {False, True}
    External predicates do not make the closed world assumption
    """
    _Instance = PredicateInstance
    _Result = PredicateResult
    _codomain = bool
    _default_p_success = None
    _default_overhead = None
    #def is_negative(self):
    #    return self._Instance._opt_value is False
    def __init__(self, *args):
        super(Predicate, self).__init__(*args)
        assert(self.info.opt_fn is None)

##################################################

def parse_common(lisp_list, stream_map, stream_info):
    assert (2 <= len(lisp_list) <= 3)
    head = tuple(lisp_list[1])
    assert (is_head(head))
    name = get_prefix(head)
    if stream_map == DEBUG:
        fn = DEBUG
    else:
        if name not in stream_map:
            raise ValueError('Undefined external function: {}'.format(name))
        fn = stream_map[name]
    domain = []
    if len(lisp_list) == 3:
        domain = list_from_conjunction(lisp_list[2])
    info = stream_info.get(name, None)
    return head, fn, domain, info

def parse_function(*args):
    return Function(*parse_common(*args))

def parse_predicate(*args):
    return Predicate(*parse_common(*args))
