from collections import Counter

from pddlstream.conversion import values_from_objects, substitute_expression, get_prefix, get_args, Equal, Not, is_head, \
    list_from_conjunction, is_parameter, str_from_head
from pddlstream.utils import str_from_tuple, INF

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
        # self.results_history = []

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

    def get_instance(self, input_values):
        input_values = tuple(input_values)
        if input_values not in self.instances:
            self.instances[input_values] = self._Instance(self, input_values)
        return self.instances[input_values]


##################################################

class FunctionResult(Result):
    def __init__(self, instance, value):
        super(FunctionResult, self).__init__(instance)
        self.instance = instance
        self.value = value

    def get_certified(self):
        return [Equal(self.instance.get_head(), self.value)]

    def __repr__(self):
        return '{}={}'.format(str_from_head(self.instance.get_head()), self.value)


class FunctionInstance(Instance):  # Head(Instance):
    _opt_value = 0

    def get_head(self):
        return substitute_expression(self.external.head, self.get_mapping())

    def next_results(self, stream_plan=None, verbose=False):
        assert not self.enumerated
        self.enumerated = True
        self.value = self.external.fn(*self.get_input_values())
        #if not (type(self.value) is self.external._codomain):
        #if not isinstance(self.value, self.external._codomain):
        #    raise ValueError('Function [{}] produced a nonintegral value [{}]. '
        #                     'FastDownward only supports integral costs. '
        #                     'To "use" real costs, scale each cost by a large factor, '
        #                     'capturing the most significant bits.'.format(self.external.name, self.value))
        if self.value < 0:
            raise ValueError('Function [{}] produced a negative value [{}]'.format(self.external.name, self.value))
        if verbose:
            print('{}{}={}'.format(get_prefix(self.external.head),
                                   str_from_tuple(self.get_input_values()), self.value))
        return [self.external._Result(self, self.value)]

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