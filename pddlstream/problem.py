import collections

class Object(object):
    _prefix = 'o'
    _obj_from_id = {}
    _obj_from_value = {}
    #_obj_from_index = []
    _obj_from_name = {}
    def __init__(self, value):
        # TODO: unique vs hash
        self.value = value
        # TODO: could use id(self)
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
        if not isinstance(value, collections.Hashable):
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
