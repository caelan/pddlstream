from collections import OrderedDict

from pddlstream.algorithms.downward import make_action, make_parameters, make_domain
from pddlstream.language.constants import Not
from pddlstream.language.conversion import pddl_from_object, substitute_expression
from pddlstream.language.statistics import check_effort
from pddlstream.language.function import FunctionResult
from pddlstream.language.stream import StreamResult
from pddlstream.utils import INF

BOUND_PREDICATE = '_bound'


def enforce_single_binding(result, preconditions, effects):
    binding_facts = [(BOUND_PREDICATE, pddl_from_object(out)) for out in result.output_objects]
    preconditions.extend(Not(fact) for fact in binding_facts)
    effects.extend(fact for fact in binding_facts)

def get_stream_actions(results, unique_binding=False, effort_scale=1, max_effort=INF, **kwargs):
    result_from_name = OrderedDict()
    stream_actions = []
    for result in results:
        #if not isinstance(stream_result, StreamResult):
        if type(result) == FunctionResult:
            continue
        effort = result.get_effort(**kwargs)
        if not check_effort(effort, max_effort):
            continue
        name = '{}-{}'.format(result.external.name, len(result_from_name))
        #name = '{}_{}_{}'.format(result.external.name, # No spaces & parens
        #                        ','.join(map(pddl_from_object, result.instance.input_objects)),
        #                        ','.join(map(pddl_from_object, result.output_objects)))
        assert name not in result_from_name
        result_from_name[name] = result

        preconditions = list(result.instance.get_domain())
        effects = list(result.get_certified()) + [result.stream_fact]
        if unique_binding:
            enforce_single_binding(result, preconditions, effects)
        cost = effort_scale * effort
        stream_actions.append(make_action(name, [], preconditions, effects, cost))
    return stream_actions, result_from_name

def add_stream_actions(domain, results, **kwargs):
    if not results:
        return domain, {}
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
