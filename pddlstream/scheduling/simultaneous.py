from pddlstream.algorithm import solve_finite
from pddlstream.conversion import get_prefix, pddl_from_object, get_args, obj_from_pddl
from pddlstream.fast_downward import TOTAL_COST, OBJECT, Domain
from pddlstream.utils import INF
from pddlstream.stream import Function, Stream

def fd_from_fact(evaluation):
    import pddl
    predicate = get_prefix(evaluation)
    args = map(pddl_from_object, get_args(evaluation))
    return pddl.Atom(predicate, args)

#def evaluation_from_fd(fd):
#    args = tuple(map(obj_from_pddl, fd.args))
#    head = Head(fd.predicate, args)
#    return Evaluation(head, not fd.negated)


def fact_from_fd(fd):
    assert(not fd.negated)
    return (fd.predicate,) + tuple(map(obj_from_pddl, fd.args))


def get_stream_action(stream_result, name, effect_scale=1):
    #from pddl_parser.parsing_functions import parse_action
    import pddl

    parameters = []
    preconditions = [fd_from_fact(fact) for fact in stream_result.stream_instance.get_domain()]
    precondition = pddl.Conjunction(preconditions)
    effects = [pddl.Effect(parameters=[], condition=pddl.Truth(), literal=fd_from_fact(fact))
               for fact in stream_result.get_certified()]

    effort = effect_scale
    if effort == INF:
        return None
    fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
    expression = pddl.NumericConstant(effort) # Integer
    increase = pddl.Increase(fluent=fluent, expression=expression) # Can also be None

    return pddl.Action(name=name, parameters=parameters, num_external_parameters=len(parameters),
                    precondition=precondition, effects=effects, cost=increase)
    # TODO: previous problem seemed to be new predicates


def get_stream_actions(stream_results):
    stream_result_from_name = {}
    stream_actions = []
    for i, stream_result in enumerate(stream_results):
        if type(stream_result.stream_instance.stream) != Stream:
            continue
        name = '{}-{}'.format(stream_result.stream_instance.stream.name, i)
        stream_action = get_stream_action(stream_result, name)
        if stream_action is None:
            continue
        stream_result_from_name[name] = stream_result
        stream_actions.append(stream_action)
    return stream_actions, stream_result_from_name


def add_stream_actions(domain, stream_results):
    import pddl
    stream_actions, stream_result_from_name = get_stream_actions(stream_results)
    output_objects = []
    for stream_result in stream_result_from_name.values():
        output_objects += stream_result.output_objects
    new_constants = [pddl.TypedObject(pddl_from_object(obj), OBJECT) for obj in set(output_objects)]
    # to_untyped_strips
    # free_variables
    new_domain = Domain(domain.name, domain.requirements, domain.types, domain.type_dict,
                        domain.constants[:] + new_constants,
                        domain.predicates, domain.predicate_dict, domain.functions,
                        domain.actions[:] + stream_actions, domain.axioms)
    return new_domain, stream_result_from_name


def simultaneous_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    # TODO: can't make stream_actions for functions. Apply functions then retrace
    new_domain, stream_result_from_name = add_stream_actions(domain, stream_results)
    combined_plan, combined_cost = solve_finite(evaluations, goal_expression, new_domain, **kwargs)
    if combined_plan is None:
        return None, None, combined_cost # TODO: return plan cost
    stream_plan = []
    action_plan = []
    for name, args in combined_plan:
        if name in stream_result_from_name:
            stream_plan.append(stream_result_from_name[name])
        else:
            action_plan.append((name, args))
    return stream_plan, action_plan, combined_cost