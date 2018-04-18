from utils import read
from fast_downward import run_fast_downward, translate_task, write_pddl
import re

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

class Object(object):
    _template = 'x{}'
    num = 0
    # TODO: maintain dictionary here for looking up values. Test if underlying thing has a hash
    # TODO: PDDL name to object
    # TODO: hash to object
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
