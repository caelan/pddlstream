from pddlstream.algorithms.downward import add_predicate, make_predicate, get_literals, fact_from_fd, conditions_hold, \
    apply_action
from pddlstream.algorithms.scheduling.recover_axioms import get_derived_predicates
from pddlstream.language.constants import And, Not
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams
from pddlstream.utils import INF, apply_mapping


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

def evaluations_from_stream_plan(evaluations, stream_results, max_effort=INF):
    opt_evaluations = set(evaluations)
    for result in stream_results:
        assert(not result.instance.disabled)
        assert(not result.instance.enumerated)
        domain = set(map(evaluation_from_fact, result.instance.get_domain()))
        assert(domain <= opt_evaluations)
        opt_evaluations.update(map(evaluation_from_fact, result.get_certified()))
    node_from_atom = get_achieving_streams(evaluations, stream_results)
    result_from_evaluation = {evaluation_from_fact(f): n.result for f, n in node_from_atom.items()
                              if (max_effort is not None) and (n.effort < max_effort)}
    return result_from_evaluation

def partition_external_plan(external_plan):
    function_plan = list(filter(lambda r: isinstance(r, FunctionResult), external_plan))
    stream_plan = list(filter(lambda r: r not in function_plan, external_plan))
    return stream_plan, function_plan

def add_unsatisfiable_to_goal(domain, goal_expression):
    import pddl
    from pddlstream.language.optimizer import UNSATISFIABLE, BLOCK_ADDITIONS
    add_predicate(domain, make_predicate(UNSATISFIABLE, []))
    if not BLOCK_ADDITIONS:
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

def simplify_conditional_effects(opt_task, action_instances, negative_from_name={}):
    # TODO: extract out the minimum set of conditional effects that are actually required
    # TODO: handle more general case where can choose to achieve particular conditional effects
    # will likely require planning with streams
    axioms_from_name = get_derived_predicates(opt_task.axioms)
    state = set(opt_task.init)
    for action_instance in action_instances:
        for effects in [action_instance.add_effects, action_instance.del_effects]:
            for i, (conditions, effect) in reversed(list(enumerate(effects))):
                if any(c.predicate in axioms_from_name for c in conditions):
                    raise NotImplementedError('Conditional effects cannot currently involve derived predicates')
                neg_conditions = [literal for literal in conditions
                                  if literal.predicate in negative_from_name]
                pos_conditions = [literal for literal in conditions
                                  if literal not in neg_conditions]
                #if conditions_hold(real_state, opt_conditions):
                if conditions_hold(state, pos_conditions):
                    # Holds in optimistic state
                    # Assuming that must achieve all possible conditional effects
                    if neg_conditions:
                        # Assuming that negative conditions should not be achieved
                        if len(neg_conditions) != 1:
                            raise NotImplementedError()
                        action_instance.precondition.extend(
                            l.negate() for l in neg_conditions)
                        effects.pop(i)
                    else:
                        action_instance.precondition.extend(pos_conditions)
                        effects[i] = ([], effect)
                #elif not conditions_hold(opt_state, opt_conditions):
                else:
                    # Does not hold in optimistic state
                    effects.pop(i)
        apply_action(state, action_instance)
