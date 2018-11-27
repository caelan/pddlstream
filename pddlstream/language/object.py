from collections import namedtuple
from itertools import count
from pddlstream.language.constants import get_parameter_name
from pddlstream.utils import str_from_object, is_hashable

USE_HASH = True
USE_OBJ_STR = True
USE_OPT_STR = True

class Object(object):
    _prefix = 'v'
    _obj_from_id = {}
    _obj_from_value = {}
    _obj_from_name = {}
    def __init__(self, value, stream_instance=None, name=None):
        self.value = value
        self.index = len(Object._obj_from_name)
        if name is None:
            name = '{}{}'.format(self._prefix, self.index)
        self.pddl = name
        self.stream_instance = stream_instance # TODO: store first created stream instance
        Object._obj_from_id[id(self.value)] = self
        Object._obj_from_name[self.pddl] = self
        if is_hashable(value):
            Object._obj_from_value[self.value] = self
    @staticmethod
    def from_id(value):
        if id(value) not in Object._obj_from_id:
            return Object(value)
        return Object._obj_from_id[id(value)]
    @staticmethod
    def has_value(value):
        if USE_HASH and not is_hashable(value):
            return id(value) in Object._obj_from_id
        return value in Object._obj_from_value
    @staticmethod
    def from_value(value):
        if USE_HASH and not is_hashable(value):
            return Object.from_id(value)
        if value not in Object._obj_from_value:
            return Object(value)
        return Object._obj_from_value[value]
    @staticmethod
    def from_name(name):
        return Object._obj_from_name[name]
    def __lt__(self, other): # For heapq on python3
        return self.index < other.index
    def __repr__(self):
        if USE_OBJ_STR:
            return str_from_object(self.value) # str
        return self.pddl

# TODO: just one object class or have Optimistic extend Object
# TODO: make a parameter class that has access to some underlying value

UniqueOptValue = namedtuple('UniqueOpt', ['instance', 'sequence_index', 'output_index'])

class OptimisticObject(object):
    _prefix = '#o' # $ % #
    _obj_from_inputs = {}
    _obj_from_name = {}
    _count_from_prefix = {}
    def __init__(self, value, param):
        # TODO: store first created instance
        self.value = value
        self.param = param
        self.index = len(OptimisticObject._obj_from_inputs)
        self.pddl = '{}{}'.format(self._prefix, self.index)
        OptimisticObject._obj_from_inputs[(value, param)] = self
        OptimisticObject._obj_from_name[self.pddl] = self
        self.repr_name = self.pddl
        if USE_OPT_STR and isinstance(self.param, UniqueOptValue):
            # TODO: instead just endow UniqueOptValue with a string function
            parameter = self.param.instance.external.outputs[self.param.output_index]
            prefix = get_parameter_name(parameter)[:1]
            var_index = next(self._count_from_prefix.setdefault(prefix, count()))
            self.repr_name = '#{}{}'.format(prefix, var_index) #self.index)
    @staticmethod
    def from_opt(value, param):
        # TODO: make param have a default value?
        key = (value, param)
        if key not in OptimisticObject._obj_from_inputs:
            return OptimisticObject(value, param)
        return OptimisticObject._obj_from_inputs[key]
    @staticmethod
    def from_name(name):
        return OptimisticObject._obj_from_name[name]
    def __lt__(self, other): # For heapq on python3
        return self.index < other.index
    def __repr__(self):
        return self.repr_name
        #return repr(self.repr_name) # Prints in quotations
