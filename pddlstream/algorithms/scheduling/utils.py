from pddlstream.algorithms.downward import add_predicate, make_predicate, get_literals, fact_from_fd, conditions_hold, \
    apply_action, get_derived_predicates
from pddlstream.language.constants import And, Not
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.utils import apply_mapping


def partition_results(evaluations, results, apply_now):
    applied_results = []
    deferred_results = []
    opt_evaluations = set(evaluations)
    for result in results:
        assert(not result.instance.disabled)
        assert(not result.instance.enumerated)
        domain = set(map(evaluation_from_fact, result.instance.get_domain()))
        if isinstance(result, FunctionResult) or (apply_now(result) and (domain <= opt_evaluations)):
            applied_results.append(result)
            opt_evaluations.update(map(evaluation_from_fact, result.get_certified()))
        else:
            deferred_results.append(result)
    return applied_results, deferred_results

def partition_external_plan(external_plan):
    function_plan = list(filter(lambda r: isinstance(r, FunctionResult), external_plan))
    stream_plan = list(filter(lambda r: r not in function_plan, external_plan))
    return stream_plan, function_plan

def add_unsatisfiable_to_goal(domain, goal_expression):
    import pddl
    from pddlstream.language.optimizer import UNSATISFIABLE
    add_predicate(domain, make_predicate(UNSATISFIABLE, []))
    negated_atom = pddl.NegatedAtom(UNSATISFIABLE, tuple())
    for action in domain.actions:
        if negated_atom not in action.precondition.parts:
            action.precondition = pddl.Conjunction([action.precondition, negated_atom]).simplified()
    return And(goal_expression, Not((UNSATISFIABLE,)))


def get_instance_facts(instance, node_from_atom):
    # TODO: ignores conditional effect conditions
    facts = []
    for precondition in get_literals(instance.action.precondition):
        if precondition.negated:
            continue
        args = apply_mapping(precondition.args, instance.var_mapping)
        literal = precondition.__class__(precondition.predicate, args)
        fact = fact_from_fd(literal)
        if fact in node_from_atom:
            facts.append(fact)
    return facts
