from pddlstream.conversion import values_from_objects, substitute_expression, get_prefix, get_args, Equal, Not
from pddlstream.utils import str_from_tuple


def str_from_head(head):
    return '{}{}'.format(get_prefix(head), str_from_tuple(get_args(head)))


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

    def next_results(self):
        raise NotImplementedError()


class External(object):
    _Instance = None

    def __init__(self, name, inputs, domain):
        self.name = name.lower()
        self.inputs = tuple(inputs)
        self.domain = tuple(domain)
        self.instances = {}
        self.prioritized = False

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

    def next_results(self, verbose=False, context=None):
        assert not self.enumerated
        self.enumerated = True
        self.value = self.external.fn(*self.get_input_values())
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
    The key difference from a stream is that the outputs aren't treated as objects
    """
    _Instance = FunctionInstance
    _Result = FunctionResult
    _codomain = int

    def __init__(self, head, fn, domain):
        super(Function, self).__init__(get_prefix(head), get_args(head), domain)
        self.head = head
        self.fn = fn

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
    _opt_value = True
    # assert(self.value in (True, False))
    pass


class Predicate(Function):
    """
    We do not make the closed world assumption
    """
    _Instance = PredicateInstance
    _Result = PredicateResult
    _codomain = bool
