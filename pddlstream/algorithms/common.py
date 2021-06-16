import time
from collections import namedtuple, OrderedDict

from pddlstream.language.constants import is_plan, get_length, FAILED #, INFEASIBLE, SUCCEEDED
from pddlstream.language.conversion import evaluation_from_fact, obj_from_value_expression, revert_solution
from pddlstream.utils import INF, elapsed_time, check_memory

# Complexity is a way to characterize the number of external evaluations required for a solution
# Most algorithms regularize to prefer lower complexity solutions
# Also consider depth, level, priority, layer
# Effort incorporates success rate while complexity doesn't
# Complexity could incorporate how likely something is to help with a task in general though
# Effort relates to future expected time while complexity refers to past time

COMPLEXITY_OP = max # max | sum
INIT_EVALUATION = None
INTERNAL_EVALUATION = False
UNKNOWN_EVALUATION = 'unknown'

EvaluationNode = namedtuple('EvaluationNode', ['complexity', 'result'])
Solution = namedtuple('Solution', ['plan', 'cost', 'time'])

SOLUTIONS = [] # TODO: remove global variable

class SolutionStore(object):
    def __init__(self, evaluations, max_time, success_cost, verbose, max_memory=INF):
        # TODO: store a map from head to value?
        # TODO: include other problem information here?
        # TODO: determine when the plan converges
        self.evaluations = evaluations
        #self.initial_evaluations = copy.copy(evaluations)
        self.start_time = time.time()
        self.max_time = max_time
        self.max_memory = max_memory
        self.success_cost = success_cost # Inclusive
        self.verbose = verbose
        #self.best_cost = self.cost_fn(self.best_plan)
        self.solutions = []
        self.sample_time = 0.
    @property
    def search_time(self):
        return self.elapsed_time() - self.sample_time
    @property
    def best_plan(self):
        # TODO: return INFEASIBLE if can prove no solution
        return self.solutions[-1].plan if self.solutions else FAILED
    @property
    def best_cost(self):
        return self.solutions[-1].cost if self.solutions else INF
    def add_plan(self, plan, cost):
        # TODO: double-check that plan is a solution
        if is_plan(plan) and (cost < self.best_cost):
            self.solutions.append(Solution(plan, cost, elapsed_time(self.start_time)))
    def has_solution(self):
        return is_plan(self.best_plan)
    def is_solved(self):
        return self.has_solution() and (self.best_cost <= self.success_cost)
    def elapsed_time(self):
        return elapsed_time(self.start_time)
    def is_timeout(self):
        return (self.max_time <= self.elapsed_time()) or not check_memory(self.max_memory)
    def is_terminated(self):
        return self.is_solved() or self.is_timeout()
    #def __repr__(self):
    #    raise NotImplementedError()
    def extract_solution(self):
        SOLUTIONS[:] = self.solutions
        return revert_solution(self.best_plan, self.best_cost, self.evaluations)
    def export_summary(self): # TODO: log, etc...
        # TODO: SOLUTIONS
        #status = SUCCEEDED if self.is_solved() else FAILED # TODO: INFEASIBLE, OPTIMAL
        return {
            'solved': self.is_solved(),
            #'solved': self.has_solution(),
            'solutions': len(self.solutions),
            'cost': self.best_cost,
            'length': get_length(self.best_plan),
            'evaluations': len(self.evaluations),
            'search_time': self.search_time,
            'sample_time': self.sample_time,
            'run_time': self.elapsed_time(),
            'timeout': self.is_timeout(),
            #'status': status,
        }

##################################################

def add_fact(evaluations, fact, result=INIT_EVALUATION, complexity=0):
    evaluation = evaluation_from_fact(fact)
    if (evaluation not in evaluations) or (complexity < evaluations[evaluation].complexity):
        evaluations[evaluation] = EvaluationNode(complexity, result)
        return True
    return False


def add_facts(evaluations, facts, **kwargs):
    new_evaluations = []
    for fact in facts:
        if add_fact(evaluations, fact, **kwargs):
            new_evaluations.append(evaluation_from_fact(fact))
    return new_evaluations


def add_certified(evaluations, result, **kwargs):
    complexity = result.compute_complexity(evaluations, **kwargs)
    return add_facts(evaluations, result.get_certified(), result=result, complexity=complexity)


def evaluations_from_init(init):
    evaluations = OrderedDict()
    for raw_fact in init:
        fact = obj_from_value_expression(raw_fact)
        add_fact(evaluations, fact, result=INIT_EVALUATION, complexity=0)
    return evaluations


def combine_complexities(complexities, complexity_op=COMPLEXITY_OP):
    return complexity_op([0] + list(complexities))


def compute_complexity(evaluations, facts, complexity_op=COMPLEXITY_OP):
    if not facts:
        return 0
    return complexity_op(evaluations[evaluation_from_fact(fact)].complexity for fact in facts)

##################################################

def optimistic_complexity(evaluations, optimistic_facts, fact):
    if fact in optimistic_facts: # Matters due to reachieving
        return optimistic_facts[fact]
    evaluation = evaluation_from_fact(fact)
    #if evaluation in evaluations:
    return evaluations[evaluation].complexity
    #return optimistic_facts[fact]


def stream_plan_preimage(stream_plan):
    # Easy because monotonic
    preimage = set()
    achieved = set()
    for stream in stream_plan:
        preimage.update(set(stream.get_domain()) - achieved)
        achieved.update(stream.get_certified())
    return preimage


def stream_plan_complexity(evaluations, stream_plan, stream_calls, complexity_op=COMPLEXITY_OP):
    if not is_plan(stream_plan):
        return INF
    # TODO: difference between a result having a particular complexity and the next result having something
    #optimistic_facts = {}
    optimistic_facts = {fact: evaluations[evaluation_from_fact(fact)].complexity
                        for fact in stream_plan_preimage(stream_plan)}
    result_complexities = []
    #complexity = 0
    for i, result in enumerate(stream_plan):
        # if result.external.get_complexity(num_calls=INF) == 0: # TODO: skip if true
        result_complexity = complexity_op([0] + [optimistic_facts[fact]
                                                 #optimistic_complexity(evaluations, optimistic_facts, fact)
                                                 for fact in result.get_domain()])
        # if stream_calls is None:
        #     num_calls = result.instance.num_calls
        # else:
        num_calls = stream_calls[i]
        result_complexity += result.external.get_complexity(num_calls)
        result_complexities.append(result_complexity)
        #complexity = complexity_op(complexity, result_complexity)
        for fact in result.get_certified():
            if fact not in optimistic_facts:
                optimistic_facts[fact] = result_complexity
    complexity = complexity_op([0] + result_complexities)
    return complexity


def is_instance_ready(evaluations, instance):
    return all(evaluation_from_fact(f) in evaluations
               for f in instance.get_domain())
