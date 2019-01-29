from __future__ import print_function

from pddlstream.algorithms.algorithm import parse_stream_pddl, evaluations_from_init
from pddlstream.algorithms.common import SolutionStore
from pddlstream.algorithms.downward import make_domain, make_predicate
from pddlstream.algorithms.recover_optimizers import retrace_instantiation
from pddlstream.algorithms.reorder import reorder_stream_plan
from pddlstream.algorithms.satisfaction import get_constraints
from pddlstream.algorithms.scheduling.postprocess import reschedule_stream_plan
from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.language.constants import is_parameter, get_length
from pddlstream.language.conversion import revert_solution, \
    evaluation_from_fact, replace_expression, get_prefix, get_args
from pddlstream.language.effort import compute_plan_effort
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.language.stream import Stream
from pddlstream.utils import INF, get_mapping


# TODO: version of this where I pass in a plan skeleton instead
# Might be easiest to do this by performing the search normally but constraining the actions used
# Can add effects that bind future parameters when going along

# TODO: investigate constraint satisfaction techniques for binding instead

def obj_from_parameterized_expression(parent): # obj_from_value_expression
    return replace_expression(parent, lambda o: OptimisticObject
                              .from_opt(o, o) if is_parameter(o) else Object.from_value(o))


def create_domain(constraints_list):
    predicate_dict = {}
    for fact in constraints_list: # TODO: consider removing this annoying check
        name = get_prefix(fact)
        if name not in predicate_dict:
            parameters = ['?x{}'.format(i) for i in range(len(get_args(fact)))]
            predicate_dict[name] = make_predicate(name, parameters)
    return make_domain(predicates=list(predicate_dict.values()))


def constraint_satisfaction(stream_pddl, stream_map, init, constraints, stream_info={},
                            max_sample_time=INF, **kwargs):
    # Approaches
    # 1) Existential quantification of bindings in goal conditions
    # 2) Backtrack useful streams and then schedule. Create arbitrary outputs for not mentioned.
    # 3) Construct all useful streams and then associate outputs with bindings
    #    Useful stream must satisfy at least one fact. How should these assignments be propagated though?
    #    Make an action that maps each stream result to unbound values?
    # TODO: include functions again for cost-sensitive satisfaction
    # TODO: deprecate this in favor of solve_pddlstream_satisfaction
    if not constraints:
        return {}, 0, init
    constraints = get_constraints(constraints)
    externals = parse_stream_pddl(stream_pddl, stream_map, stream_info)
    streams = list(filter(lambda e: isinstance(e, Stream), externals))
    #print('Streams:', streams)
    evaluations = evaluations_from_init(init)
    goal_facts = set(filter(lambda f: evaluation_from_fact(f) not in evaluations,
                            map(obj_from_parameterized_expression, constraints)))

    free_parameters = {o for f in goal_facts for o in get_args(f) if isinstance(o, OptimisticObject)}
    visited_facts = set()
    stream_results = []
    for fact in goal_facts:
        retrace_instantiation(fact, streams, evaluations, free_parameters, visited_facts, stream_results)
    print(stream_results)

    # TODO: consider other results if this fails
    store = SolutionStore(evaluations, max_time=INF, success_cost=INF, verbose=True) # TODO: include other info here?
    stream_plan = reschedule_stream_plan(evaluations, goal_facts, create_domain(goal_facts),
                                         stream_results, unique_binding=True, **kwargs)
    if stream_plan is None:
        return revert_solution(store.best_plan, store.best_cost, evaluations)
    stream_plan = reorder_stream_plan(stream_plan)
    print('Stream plan ({}, {:.1f}): {}'.format(
        get_length(stream_plan), compute_plan_effort(stream_plan), stream_plan))

    objects = set()
    for fact in goal_facts:
        objects.update(get_args(fact))
    free_parameters = list(filter(lambda o: isinstance(o, OptimisticObject), objects))
    action_plan = [('bindings', free_parameters)]

    queue = SkeletonQueue(store, goal_expression=None, domain=None)
    queue.process(stream_plan, action_plan, cost=0, complexity_limit=INF, max_time=max_sample_time)

    #write_stream_statistics(externals + synthesizers, verbose)
    plan, cost, init = revert_solution(store.best_plan, store.best_cost, evaluations)
    if plan is None:
        return plan, cost, init
    parameter_names = [o.value for o in free_parameters]
    [(_, parameter_values)] = plan
    bindings = get_mapping(parameter_names, parameter_values)
    return bindings, cost, init
