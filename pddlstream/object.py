from collections import Hashable, namedtuple

USE_HASH = True

class Object(object):
    _prefix = 'o'
    _obj_from_id = {}
    _obj_from_value = {}
    #_obj_from_index = []
    _obj_from_name = {}
    def __init__(self, value, stream_instance=None, name=None):
        # TODO: unique vs hash
        self.value = value
        self.index = len(Object._obj_from_name)
        if name is None:
            name = '{}{}'.format(self._prefix, self.index)
        self.name = name
        self.stream_instance = stream_instance # TODO: store first created stream instance
        Object._obj_from_id[id(self.value)] = self
        Object._obj_from_name[self.name] = self
        if isinstance(value, Hashable):
            Object._obj_from_value[self.value] = self
    @property
    def pddl(self):
        #return self._template.format(self.n)
        #return '{}{}'.format(self._prefix, self.index)
        return self.name
    @staticmethod
    def from_id(value):
        if id(value) not in Object._obj_from_id:
            return Object(value)
        return Object._obj_from_id[id(value)]
    @staticmethod
    def has_value(value):
        if USE_HASH and not isinstance(value, Hashable):
            return id(value) in Object._obj_from_id
        return value in Object._obj_from_value
    @staticmethod
    def from_value(value):
        if USE_HASH and not isinstance(value, Hashable):
            return Object.from_id(value)
        if value not in Object._obj_from_value:
            return Object(value)
        return Object._obj_from_value[value]
    #@staticmethod
    #def from_index(index):
    #    return Object._obj_from_index[index]
    @staticmethod
    def from_name(name):
        #index = int(name.split(Object._prefix)[1]) # TODO: match regex or strip prefix
        #return Object.from_index(index)
        return Object._obj_from_name[name]
    def __lt__(self, other): # For heapq on python3
        return self.index < other.index
    def __repr__(self):
        #return repr(self.value)
        return self.pddl

# TODO: just one object class or have Optimistic extend Object
# TODO: make a parameter class that has access to some underlying value

#class SharedObject(object):
#    def __init__(self, stream, output_index):
#        self.stream = stream
#        self.output_index = output_index
#
#class PartialObject(object):
#    pass
#
#class UniqueObject(namedtuple('UO', ['stream_instance', 'output_index'])):
#    @property
#    def parameter(self):
#        return self.stream_instance.stream.outputs[self.output_index]
#    def __repr__(self):
#        return '#{}{}'.format(self.parameter[1:], self.output_index)

class OptimisticObject(object):
    _prefix = '#' # $ % #
    _obj_from_inputs = {}
    _obj_from_name= {}
    def __init__(self, value):
        # TODO: store first created instance
        self.value = value
        #stream_instance, output_index = value
        #self.stream_instance = stream_instance
        #self.output_index = output_index
        self.index = len(OptimisticObject._obj_from_inputs)
        #self.name = '{}{}{}'.format(self._prefix, self.parameter[1:], self.index)
        self.name = '{}{}'.format(self._prefix, self.index)
        OptimisticObject._obj_from_inputs[value] = self
        OptimisticObject._obj_from_name[self.name] = self
    #@property
    #def parameter(self):
    #    return self.stream_instance.stream.outputs[self.output_index]
    @staticmethod
    def from_opt(opt):
        if opt not in OptimisticObject._obj_from_inputs:
            return OptimisticObject(opt)
        return OptimisticObject._obj_from_inputs[opt]
    @staticmethod
    def from_name(name):
        return OptimisticObject._obj_from_name[name]
    @property
    def pddl(self):
        return self.name
    def __lt__(self, other): # For heapq on python3
        return self.index < other.index
    def __repr__(self):
        #return repr(self.value)
        return self.pddl
