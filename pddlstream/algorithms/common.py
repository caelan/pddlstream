import time
from collections import namedtuple, OrderedDict

from pddlstream.language.constants import is_plan
from pddlstream.language.conversion import evaluation_from_fact, obj_from_value_expression, revert_solution
from pddlstream.utils import INF, elapsed_time

# Complexity is a way to characterize the number of external evaluations required for a solution
# Most algorithms regularize to prefer lower complexity solutions
# Also consider depth, level, priority, layer
# Effort incorporates success rate while complexity doesn't
# Complexity could incorporate how likely something is to help with a task in general though
# Effort relates to future expected time while complexity refers to past time

COMPLEXITY_OP = max # max | sum
INIT_EVALUATION = None
INTERNAL_EVALUATION = False

EvaluationNode = namedtuple('EvaluationNode', ['complexity', 'result'])
Solution = namedtuple('Solution', ['plan', 'cost'])

class SolutionStore(object):
    def __init__(self, evaluations, max_time, success_cost, verbose):
        # TODO: store a map from head to value?
        # TODO: include other problem information here?
        self.evaluations = evaluations
        #self.initial_evaluations = copy.copy(evaluations)
        self.start_time = time.time()
        self.max_time = max_time
        #self.cost_fn = get_length if unit_costs else None
        self.success_cost = success_cost # Inclusive
        self.verbose = verbose
        self.best_plan = None
        self.best_cost = INF
        #self.best_cost = self.cost_fn(self.best_plan)
        self.solutions = []
    def add_plan(self, plan, cost):
        # TODO: double-check that plan is a solution
        if not is_plan(plan) or (self.best_cost <= cost):
            return
        solution = Solution(plan, cost)
        self.best_plan, self.best_cost = solution
        self.solutions.append(solution)
    def has_solution(self):
        return is_plan(self.best_plan)
    def is_solved(self):
        return self.has_solution() and (self.best_cost <= self.success_cost)
    def elapsed_time(self):
        return elapsed_time(self.start_time)
    def is_timeout(self):
        return self.max_time <= self.elapsed_time()
    def is_terminated(self):
        return self.is_solved() or self.is_timeout()
    #def __repr__(self):
    #    raise NotImplementedError()
    def extract_solution(self):
        return revert_solution(self.best_plan, self.best_cost, self.evaluations)

##################################################

def add_fact(evaluations, fact, result=INIT_EVALUATION, complexity=0):
    evaluation = evaluation_from_fact(fact)
    if evaluation in evaluations:
        return False
    evaluations[evaluation] = EvaluationNode(complexity, result)
    return True


def add_facts(evaluations, facts, **kwargs):
    new_evaluations = []
    for fact in facts:
        if add_fact(evaluations, fact, **kwargs):
            new_evaluations.append(evaluation_from_fact(fact))
    return new_evaluations


def add_certified(evaluations, result):
    complexity = result.compute_complexity(evaluations)
    return add_facts(evaluations, result.get_certified(), result=result, complexity=complexity)


def evaluations_from_init(init):
    evaluations = OrderedDict()
    for raw_fact in init:
        fact = obj_from_value_expression(raw_fact)
        add_fact(evaluations, fact, result=INIT_EVALUATION, complexity=0)
    return evaluations


def compute_call_complexity(num_calls):
    return num_calls + 1


def compute_complexity(evaluations, facts):
    if not facts:
        return 0
    return COMPLEXITY_OP(evaluations[evaluation_from_fact(fact)].complexity for fact in facts)


def is_instance_ready(evaluations, instance):
    return all(evaluation_from_fact(f) in evaluations
               for f in instance.get_domain())
