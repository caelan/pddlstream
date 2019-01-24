from pddlstream.algorithms.downward import make_action, make_predicate, make_parameters, make_domain, \
    add_predicate
from pddlstream.language.constants import Head, And, Not
from pddlstream.language.conversion import pddl_from_object, obj_from_pddl, substitute_expression, is_parameter
from pddlstream.language.effort import compute_result_effort
from pddlstream.language.function import FunctionResult
from pddlstream.language.optimizer import UNSATISFIABLE, is_optimizer_result, BLOCK_ADDITIONS
from pddlstream.language.stream import StreamResult
from pddlstream.utils import INF

BOUND_PREDICATE = '_bound'


def enforce_single_binding(result, preconditions, effects):
    binding_facts = [(BOUND_PREDICATE, pddl_from_object(out)) for out in result.output_objects]
    preconditions.extend(Not(fact) for fact in binding_facts)
    effects.extend(fact for fact in binding_facts)

def get_stream_actions(results, unique_binding=False, effort_scale=1, **kwargs):
    result_from_name = {}
    stream_actions = []
    for i, result in enumerate(results):
        #if not isinstance(stream_result, StreamResult):
        if type(result) == FunctionResult:
            continue
        effort = compute_result_effort(result, **kwargs)
        if effort == INF:
            continue
        # TODO: state constraints
        # TODO: selectively negate axioms
        result_name = '{}-{}'.format(result.external.name, i)
        #result_name = '{}_{}_{}'.format(result.external.name, # No spaces & parens
        #                        ','.join(map(pddl_from_object, result.instance.input_objects)),
        #                        ','.join(map(pddl_from_object, result.output_objects)))
        assert result_name not in result_from_name
        result_from_name[result_name] = result

        preconditions = list(result.instance.get_domain())
        effects = list(result.get_certified())
        if unique_binding:
            enforce_single_binding(result, preconditions, effects)
        if is_optimizer_result(result): # These effects don't seem to be pruned
            effects.append(substitute_expression(result.external.stream_fact, result.get_mapping()))
        stream_actions.append(make_action(result_name, [], preconditions, effects, effort_scale * effort))
    return stream_actions, result_from_name

def add_stream_actions(domain, results, **kwargs):
    stream_actions, result_from_name = get_stream_actions(results, **kwargs)
    output_objects = []
    for result in result_from_name.values():
        if isinstance(result, StreamResult):
            output_objects.extend(map(pddl_from_object, result.output_objects))
    new_constants = list(make_parameters(set(output_objects) | set(domain.constants)))
    # to_untyped_strips, free_variables
    new_domain = make_domain(constants=new_constants, predicates=domain.predicates,
                             actions=domain.actions[:] + stream_actions, axioms=domain.axioms)
    #new_domain = copy.copy(domain)
    return new_domain, result_from_name

##################################################

def extract_function_results(results_from_head, action, pddl_args):
    import pddl
    if action.cost is None:
        return None
    expression = action.cost.expression
    if not isinstance(expression, pddl.PrimitiveNumericExpression):
        return None
    var_mapping = {p.name: a for p, a in zip(action.parameters, pddl_args)}
    obj_args = tuple(obj_from_pddl(var_mapping[p] if is_parameter(p) else p)
                     for p in expression.args)
    head = Head(expression.symbol, obj_args)
    [(_, result)] = results_from_head[head]
    if result is None:
        return None
    return result

def add_unsatisfiable_to_goal(domain, goal_expression):
    import pddl
    add_predicate(domain, make_predicate(UNSATISFIABLE, []))
    if not BLOCK_ADDITIONS:
        negated_atom = pddl.NegatedAtom(UNSATISFIABLE, tuple())
        for action in domain.actions:
            if negated_atom not in action.precondition.parts:
                action.precondition = pddl.Conjunction([action.precondition, negated_atom]).simplified()
    return And(goal_expression, Not((UNSATISFIABLE,)))

def partition_plan(combined_plan, stream_result_from_name):
    stream_plan, action_plan = [], []
    for name, args in combined_plan:
        if name in stream_result_from_name:
            stream_plan.append(stream_result_from_name[name])
        else:
            action_plan.append((name, args))
    return stream_plan, action_plan
