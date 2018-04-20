import collections

class Object(object):
    _use_hash = True
    _prefix = 'o'
    _obj_from_id = {}
    _obj_from_value = {}
    #_obj_from_index = []
    _obj_from_name = {}
    def __init__(self, value):
        # TODO: unique vs hash
        self.value = value
        self.index = len(Object._obj_from_name)
        self.name = '{}{}'.format(self._prefix, self.index)
        Object._obj_from_id[id(self.value)] = self
        Object._obj_from_name[self.name] = self
        if isinstance(value, collections.Hashable):
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
    def from_value(value):
        if Object._use_hash and not isinstance(value, collections.Hashable):
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
    def __repr__(self):
        #return repr(self.value)
        return self.pddl

class OptimisticObject(object):
    _prefix = '$'
    _obj_from_inputs = {}
    def __init__(self, stream_instance, output_index):
        self.stream_instance = stream_instance
        self.output_index = output_index
        self.index = len(OptimisticObject._obj_from_inputs)
        self.name = '{}{}{}'.format(self._prefix, self.parameter[1:], self.index)
    @property
    def parameter(self):
        return self.stream_instance.stream.outputs[self.output_index]
    @staticmethod
    def from_inputs(*inputs):
        if inputs not in OptimisticObject._obj_from_inputs:
            OptimisticObject._obj_from_inputs[inputs] = OptimisticObject(*inputs)
        return OptimisticObject._obj_from_inputs[inputs]
    @property
    def pddl(self):
        return self.name
    def __repr__(self):
        #return repr(self.value)
        return self.pddl
