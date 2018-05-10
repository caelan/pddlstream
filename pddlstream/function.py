from collections import Counter

from pddlstream.conversion import values_from_objects, substitute_expression, get_prefix, get_args, Equal, Not, is_head, \
    list_from_conjunction, is_parameter, str_from_head
from pddlstream.utils import str_from_tuple, INF
import time

DEFAULT_P_SUCCESS = 0.75
DEFAULT_OVERHEAD = 1 # TODO: estimate search overhead


class ExternalInfo(object):
    pass


def geometric_cost(cost, p):
    if p == 0:
        return INF
    return cost/p

class FunctionInfo(ExternalInfo):
    def __init__(self, eager=False, bound_fn=0, p_success=DEFAULT_P_SUCCESS, overhead=DEFAULT_OVERHEAD):
        self.eager = eager
        self.bound_fn = bound_fn
        self.p_success = p_success
        self.overhead = overhead
        self.effort = geometric_cost(self.overhead, self.p_success)
        #self.order = 0

##################################################


class Result(object):
    def __init__(self, instance):
        self.instance = instance

    def get_certified(self):
        raise NotImplementedError()


class Instance(object):
    def __init__(self, external, input_objects):
        self.external = external
        self.input_objects = tuple(input_objects)
        self.enumerated = False
        self.disabled = False
        self.total_calls = 0
        self.total_overhead = 0
        self.total_successes = 0
        # self.results_history = []

    def update_statistics(self, start_time, results):
        overhead = time.time() - start_time
        success = len(results) != 0
        self.total_calls += 1
        self.total_overhead += overhead
        self.total_successes += success
        self.external.update_statistics(overhead, success)
        # TODO: update the global as well

    def estimate_p_success(self, reg_p_success=1, reg_calls=0):
        raise NotImplementedError()
        # TODO: use the external as a prior
        # TODO: Bayesian estimation of likelihood that has result

    def estimate_overhead(self, reg_overhead=0, reg_calls=0):
        raise NotImplementedError()

    def get_input_values(self):
        return values_from_objects(self.input_objects)

    def get_mapping(self):
        return dict(zip(self.external.inputs, self.input_objects))

    def get_domain(self):
        return substitute_expression(self.external.domain, self.get_mapping())

    def next_results(self, stream_plan=None, verbose=False):
        raise NotImplementedError()


class External(object):
    _Instance = None
    def __init__(self, name, inputs, domain):
        for p, c in Counter(inputs).items():
            if c != 1:
                raise ValueError('Input [{}] for stream [{}] is not unique'.format(p, name))
        for p in {a for i in domain for a in get_args(i) if is_parameter(a)} - set(inputs):
            raise ValueError('Parameter [{}] for stream [{}] is not included within inputs'.format(p, name))
        self.name = name.lower()
        self.inputs = tuple(inputs)
        self.domain = tuple(domain)
        self.instances = {}
        self.total_calls = 0
        self.total_overhead = 0
        self.total_successes = 0
        # TODO: write the statistics to a file using the stream filename?

    def update_statistics(self, overhead, success):
        self.total_calls += 1
        self.total_overhead += overhead
        self.total_successes += success

    def estimate_p_success(self, reg_p_success=1, reg_calls=0):
        calls = self.total_calls + reg_calls
        if calls == 0:
            return None
        # TODO: use prior from info instead?
        return float(self.total_successes + reg_p_success*reg_calls) / calls

    def estimate_overhead(self, reg_overhead=0, reg_calls=0):
        calls = self.total_calls + reg_calls
        if calls == 0:
            return None
        # TODO: use prior from info instead?
        return float(self.total_overhead + reg_overhead*reg_calls) / calls

    def get_instance(self, input_objects):
        input_objects = tuple(input_objects)
        if input_objects not in self.instances:
            self.instances[input_objects] = self._Instance(self, input_objects)
        return self.instances[input_objects]


##################################################

class FunctionResult(Result):
    def __init__(self, instance, value):
        super(FunctionResult, self).__init__(instance)
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
    _opt_value = 0

    def get_head(self):
        return substitute_expression(self.external.head, self.get_mapping())

    def next_results(self, stream_plan=None, verbose=False):
        start_time = time.time()
        assert not self.enumerated
        self.enumerated = True
        value = self.external.fn(*self.get_input_values())
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
        # TODO: include as failure if a predicate evaluation is unsuccessful
        self.update_statistics(start_time, results)
        return results

    def next_optimistic(self):
        if self.enumerated or self.disabled:
            return []
        return [self.external._Result(self, self._opt_value)]

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
    _default_overhead = DEFAULT_OVERHEAD
    def __init__(self, head, fn, domain):
        super(Function, self).__init__(get_prefix(head), get_args(head), domain)
        self.head = head
        self.fn = fn
        self.info = FunctionInfo(p_success=self._default_p_success, overhead=self._default_overhead)
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
    _opt_value = False
    pass


class Predicate(Function):
    """
    An external predicate P(i1, ..., ik) -> {False, True}
    External predicates do not make the closed world assumption
    """
    _Instance = PredicateInstance
    _Result = PredicateResult
    _codomain = bool
    _default_p_success = 0.5
    _default_overhead = DEFAULT_OVERHEAD
    def is_negative(self):
        return self._Instance._opt_value is False

##################################################


def parse_function(lisp_list, stream_map):
    assert (len(lisp_list) == 3)
    head = tuple(lisp_list[1])
    assert (is_head(head))
    # inputs = get_args(head)
    domain = list_from_conjunction(lisp_list[2])
    name = get_prefix(head)
    if name not in stream_map:
        raise ValueError('Undefined external function: {}'.format(name))
    return Function(head, stream_map[name], domain)

def parse_predicate(lisp_list, stream_map):
    assert (len(lisp_list) == 3)
    head = tuple(lisp_list[1])
    assert (is_head(head))
    name = get_prefix(head)
    if name not in stream_map:
        raise ValueError('Undefined external function: {}'.format(name))
    return Predicate(head, stream_map[name], list_from_conjunction(lisp_list[2]))