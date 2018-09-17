import time

from pddlstream.language.conversion import substitute_expression, list_from_conjunction, str_from_head
from pddlstream.language.constants import Not, Equal, get_prefix, get_args, is_head
from pddlstream.language.external import ExternalInfo, Result, Instance, External, DEBUG, get_procedure_fn
from pddlstream.utils import str_from_object

# https://stackoverflow.com/questions/847936/how-can-i-find-the-number-of-arguments-of-a-python-function
#try:
#    from inspect import getfullargspec as get_arg_spec
#except ImportError:
#    from inspect import getargspec as get_arg_spec

#from inspect import getargspec as get_arg_spec
#from inspect import signature

class FunctionInfo(ExternalInfo):
    def __init__(self, opt_fn=None, eager=False, p_success=None, overhead=None):
        super(FunctionInfo, self).__init__(eager, p_success, overhead)
        self.opt_fn = opt_fn
        #self.order = 0

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

    def remap_inputs(self, bindings):
        # TODO: move this to the instance class?
        input_objects = [bindings.get(i, i) for i in self.instance.input_objects]
        new_instance = self.instance.external.get_instance(input_objects)
        new_instance.opt_index = self.instance.opt_index
        return self.__class__(new_instance, self.value, self.opt_index)

    def is_successful(self):
        return True

    def __repr__(self):
        return '{}={}'.format(str_from_head(self.instance.get_head()), self.value)


class FunctionInstance(Instance):  # Head(Instance):
    #_opt_value = 0

    def get_head(self):
        return substitute_expression(self.external.head, self.get_mapping())

    def next_results(self, accelerate=1, verbose=False):
        start_time = time.time()
        assert not self.enumerated
        self.enumerated = True
        input_values = self.get_input_values()
        try:
            value = self.external.fn(*input_values)
        except TypeError as err:
            print('Function [{}] expects {} inputs'.format(self.external.name, len(input_values)))
            raise err
        self.value = self.external._codomain(value)
        # TODO: cast the inputs and test whether still equal?
        #if not (type(self.value) is self.external._codomain):
        #if not isinstance(self.value, self.external._codomain):
        #if self.value != value:
        #    raise ValueError('Function [{}] produced a nonintegral value [{}]. '
        #                     'FastDownward only supports integral costs. '
        #                     'To "use" real costs, scale each cost by a large factor, '
        #                     'capturing the most significant bits.'.format(self.external.name, self.value))
        if self.value < 0:
            raise ValueError('Function [{}] produced a negative value [{}]'.format(self.external.name, self.value))
        if verbose:
            print('0) {}{}={}'.format(get_prefix(self.external.head),
                                      str_from_object(self.get_input_values()), self.value))
        results = [self.external._Result(self, self.value)]
        #if isinstance(self, PredicateInstance) and (self.value != self.external.opt_fn(*input_values)):
        #    self.update_statistics(start_time, [])
        self.update_statistics(start_time, results)
        new_facts = []
        return results, new_facts

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
    #_codomain = int
    _codomain = float
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
        #arg_spec = get_arg_spec(self.fn)
        #if len(self.inputs) != len(arg_spec.args):
        #    raise TypeError('Function [{}] expects inputs {} but its procedure has inputs {}'.format(
        #        self.name, list(self.inputs), arg_spec.args))
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

    def is_successful(self):
        opt_value = self.instance.external.opt_fn(*self.instance.get_input_values())
        return self.value == opt_value


class PredicateInstance(FunctionInstance):
    #_opt_value = True # TODO: make this False to be consistent with Function?
    #_opt_value = Predicate._codomain()
    #_opt_value = False
    pass
    #def was_successful(self, results):
    #    #self.external.opt_fn(*input_values)
    #    return any(r.value for r in results)


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
    fn = get_procedure_fn(stream_map, name)
    domain = []
    if len(lisp_list) == 3:
        domain = list_from_conjunction(lisp_list[2])
    info = stream_info.get(name, None)
    return head, fn, domain, info

def parse_function(*args):
    return Function(*parse_common(*args))

def parse_predicate(*args):
    return Predicate(*parse_common(*args))
