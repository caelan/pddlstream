from utils import read

class Problem(object):
    def __init__(self, domain_pddl=None, domain_path=None,
                 streams=[], constant_map={}):
        # TODO: allow reading and extending a problem file as well
        # TODO: objective
        assert((domain_pddl is None) != (domain_path is None))
        if domain_path is not None:
            domain_pddl = read(domain_path)
        self.domain_pddl = domain_pddl
        self.streams = streams
        self.constant_map = constant_map
    def pddl(self):
        return ''


class Object(object):
    _template = 'x{}'
    num = 0
    def __init__(self, value):
        # TODO: unique vs hash
        self.value = value
        self.n = self.__class__.num # TODO: could use id(self)
        self.__class__.num += 1
    @property
    def pddl(self):
        return self._template.format(self.n)
    def __repr__(self):
        return repr(self.value)
