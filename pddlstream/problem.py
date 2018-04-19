from pddlstream.utils import read
from pddlstream.fast_downward import run_fast_downward, translate_task, write_pddl
import re
import collections

class PDDLProblem(object):
    def __init__(self, domain_pddl, problem_pddl, object_map={}):
        self.domain_pddl = domain_pddl
        self.problem_pddl = problem_pddl
        self.object_map = object_map
        # TODO: could map constants to PDDL replacement
        # TODO: need to parse all constants as such
    def convert_plan(self, s_plan):
        if s_plan is None:
            return None
        return [(action, map(self.value_from_name, args)) for action, args in s_plan]
    def value_from_name(self, s):
        regex = r'{(\w+)}'
        match = re.match(regex, s) # search | findall
        if match is None:
            return s
        name = match.groups()[0]
        #return self.object_map[name] # TODO: fail if doesn't exist?
        return self.object_map.get(name, name)

def solve_pddl_problem(pddl_problem, **kwargs):
    s_plan = run_fast_downward(pddl_problem.domain_pddl, pddl_problem.problem_pddl, **kwargs)
    return pddl_problem.convert_plan(s_plan)

##################################################

class PDDLStreamProblem(object):
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

##################################################

class Object(object):
    #_template = 'o{}'
    _prefix = 'o'
    # TODO: maintain dictionary here for looking up values. Test if underlying thing has a hash
    _obj_from_value = {}
    #_obj_from_index = []
    _obj_from_name = {}
    def __init__(self, value):
        # TODO: unique vs hash
        self.value = value
        # TODO: could use id(self)
        self.index = len(Object._obj_from_value)
        self.name = '{}{}'.format(self._prefix, self.index)
        Object._obj_from_value[self.value] = self
        Object._obj_from_name[self.name] = self
    @property
    def pddl(self):
        #return self._template.format(self.n)
        #return '{}{}'.format(self._prefix, self.index)
        return self.name
    @staticmethod
    def from_value(value):
        #if isinstance(0, collections.Hashable):
        #   pass
        # TODO: just has based on its id in memory?
        if value not in Object._obj_from_value: # TODO: return here instead of updating?
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

##################################################

class Stream(object):
    # TODO: could even parse a stream like an action to some degree
    # TODO: constant map?
    def __init__(self, inp, domain, fn, out, certifed, name=None):
        # TODO: should each be a list or a string
        pass
