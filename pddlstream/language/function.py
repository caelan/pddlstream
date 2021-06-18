import time

from pddlstream.language.conversion import substitute_expression, list_from_conjunction, str_from_head
from pddlstream.language.constants import Not, Equal, get_prefix, get_args, is_head, FunctionAction
from pddlstream.language.external import ExternalInfo, Result, Instance, External, DEBUG_MODES, get_procedure_fn
from pddlstream.utils import str_from_object, apply_mapping

# https://stackoverflow.com/questions/847936/how-can-i-find-the-number-of-arguments-of-a-python-function
#try:
#    from inspect import getfullargspec as get_arg_spec
#except ImportError:
#    from inspect import getargspec as get_arg_spec

#from inspect import getargspec as get_arg_spec
#from inspect import signature

##################################################

def add_opt_function(name, base_fn, stream_map, stream_info, constant=0., coefficient=1., **external_kwargs):
    stream_fn = lambda *args, **kwargs: constant + coefficient*base_fn(*args, **kwargs)
    stream_map[name] = stream_fn
    opt_fn = lambda *args, **kwargs: constant
    info = FunctionInfo(opt_fn=opt_fn, **external_kwargs)
    stream_info[name] = info
    return stream_map, stream_info

##################################################

class FunctionInfo(ExternalInfo):
    _default_eager = True
    def __init__(self, opt_fn=None, eager=_default_eager, verbose=True, **kwargs): # Setting eager=True as a heuristic
        super(FunctionInfo, self).__init__(eager=eager, **kwargs)
        self.opt_fn = opt_fn
        self.verbose = verbose # TODO: move to ExternalInfo
        #self.order = 0

class FunctionResult(Result):
    def __init__(self, instance, value, optimistic=True):
        super(FunctionResult, self).__init__(instance, opt_index=0, call_index=0, optimistic=optimistic)
        self.instance = instance
        self.value = value
        self._certified = None
        # TODO: could add empty output_objects tuple
    @property
    def certified(self):
        if self._certified is None:
            self._certified = [Equal(self.instance.head, self.value)]
        return self._certified
    def get_certified(self):
        return self.certified
    def get_action(self):
        return FunctionAction(self.name, self.input_objects)
    def remap_inputs(self, bindings):
        #if not any(o in bindings for o in self.instance.get_all_input_objects()):
        #    return self
        input_objects = apply_mapping(self.instance.input_objects, bindings)
        new_instance = self.external.get_instance(input_objects)
        return self.__class__(new_instance, self.value, self.optimistic)
    def is_successful(self):
        return True
    def __repr__(self):
        #from pddlstream.algorithms.downward import get_cost_scale
        #value = math.log(self.value) # TODO: number of digits to display
        return '{}={:.3f}'.format(str_from_head(self.instance.head), self.value)

class FunctionInstance(Instance):
    _Result = FunctionResult
    #_opt_value = 0
    def __init__(self, external, input_objects):
        super(FunctionInstance, self).__init__(external, input_objects)
        self._head = None
    @property
    def head(self):
        if self._head is None:
            self._head = substitute_expression(self.external.head, self.mapping)
        return self._head
    @property
    def value(self):
        assert len(self.history) == 1
        return self.history[0]
    def _compute_output(self):
        self.enumerated = True
        self.num_calls += 1
        if self.history:
            return self.value
        input_values = self.get_input_values()
        value = self.external.fn(*input_values)
        # TODO: cast the inputs and test whether still equal?
        # if not (type(self.value) is self.external._codomain):
        # if not isinstance(self.value, self.external.codomain):
        if value < 0:
            raise ValueError('Function [{}] produced a negative value [{}]'.format(self.external.name, value))
        self.history.append(self.external.codomain(value))
        return self.value
    def next_results(self, verbose=False):
        assert not self.enumerated
        start_time = time.time()
        start_history = len(self.history)
        value = self._compute_output()
        new_results = [self._Result(self, value, optimistic=False)]
        new_facts = []

        if (value is not False) and verbose:
            # TODO: str(new_results[-1])
            print('iter={}, outs={}) {}{}={:.3f}'.format(
                self.get_iteration(), len(new_results), get_prefix(self.external.head),
                str_from_object(self.get_input_values()), value))
        if start_history <= len(self.history) - 1:
            self.update_statistics(start_time, new_results)
        self.successful |= any(r.is_successful() for r in new_results)
        return new_results, new_facts
    def next_optimistic(self):
        if self.enumerated or self.disabled:
            return []
        # TODO: cache this value
        opt_value = self.external.opt_fn(*self.get_input_values())
        self.opt_results = [self._Result(self, opt_value, optimistic=True)]
        return self.opt_results
    def __repr__(self):
        return '{}=?{}'.format(str_from_head(self.head), self.external.codomain.__name__)

class Function(External):
    """
    An external nonnegative function F(i1, ..., ik) -> 0 <= int
    External functions differ from streams in that their output isn't an object
    """
    codomain = float # int | float
    _Instance = FunctionInstance
    #_default_p_success = 0.99 # 0.99 | 1  # Might be pruned using cost threshold
    def __init__(self, head, fn, domain, info):
        # TODO: function values that act as preconditions (cost must be below threshold)
        if info is None:
            # TODO: move the defaults to FunctionInfo in the event that an optimistic fn is specified
            info = FunctionInfo() #p_success=self._default_p_success)
        super(Function, self).__init__(get_prefix(head), info, get_args(head), domain)
        self.head = head
        opt_fn = lambda *args: self.codomain()
        self.fn = opt_fn if (fn in DEBUG_MODES) else fn
        #arg_spec = get_arg_spec(self.fn)
        #if len(self.inputs) != len(arg_spec.args):
        #    raise TypeError('Function [{}] expects inputs {} but its procedure has inputs {}'.format(
        #        self.name, list(self.inputs), arg_spec.args))
        self.opt_fn = opt_fn if (self.info.opt_fn is None) else self.info.opt_fn
        self.num_opt_fns = 0 # TODO: support multiple opt_fns
    @property
    def function(self):
        return get_prefix(self.head)
    @property
    def has_outputs(self):
        return False
    @property
    def is_fluent(self):
        return False
    @property
    def is_negated(self):
        return False
    @property
    def is_function(self):
        return True
    @property
    def is_cost(self):
        return True
    def __repr__(self):
        return '{}=?{}'.format(str_from_head(self.head), self.codomain.__name__)

##################################################

class PredicateInfo(FunctionInfo):
    _default_eager = False

class PredicateResult(FunctionResult):
    def get_certified(self):
        # TODO: cache these results
        expression = self.instance.head
        return [expression if self.value else Not(expression)]
    def is_successful(self):
        opt_value = self.external.opt_fn(*self.instance.get_input_values())
        return self.value == opt_value

class PredicateInstance(FunctionInstance):
    _Result = PredicateResult
    #_opt_value = True # True | False | Predicate._codomain()
    #def was_successful(self, results):
    #    #self.external.opt_fn(*input_values)
    #    return any(r.value for r in results)

class Predicate(Function):
    """
    An external predicate P(i1, ..., ik) -> {False, True}
    External predicates do not make the closed world assumption
    """
    _Instance = PredicateInstance
    codomain = bool
    #def is_negative(self):
    #    return self._Instance._opt_value is False
    def __init__(self, head, fn, domain, info):
        if info is None:
            info = PredicateInfo()
        super(Predicate, self).__init__(head, fn, domain, info)
        assert(self.info.opt_fn is None)
        self.blocked_predicate = self.name
    @property
    def predicate(self):
        return self.function
    @property
    def is_negated(self):
        return True
    @property
    def is_cost(self):
        return False

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
