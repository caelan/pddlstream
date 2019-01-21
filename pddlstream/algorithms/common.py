from collections import namedtuple, OrderedDict

from pddlstream.language.conversion import evaluation_from_fact, obj_from_value_expression

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
