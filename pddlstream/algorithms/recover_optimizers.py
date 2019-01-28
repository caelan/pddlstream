import copy

from pddlstream.algorithms.common import INIT_EVALUATION
from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.algorithms.scheduling.utils import partition_external_plan
from pddlstream.language.constants import get_prefix, is_plan, get_args
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.language.optimizer import OPTIMIZER_STREAMS, OptimizerStream, VariableStream, ConstraintStream
from pddlstream.utils import neighbors_from_orders, get_mapping, get_connected_components

CLUSTER = True

def is_optimizer_result(result):
    return type(result.external) in OPTIMIZER_STREAMS


def get_optimizer(result):
    return result.external.optimizer if is_optimizer_result(result) else None


##################################################

def combine_optimizer_plan(stream_plan, functions):
    if not stream_plan:
        return stream_plan
    optimizer = get_optimizer(stream_plan[-1])
    if optimizer is None:
        return stream_plan
    function_plan = list(filter(lambda r: get_prefix(r.instance.external.head)
                                          in optimizer.objectives, functions))
    external_plan = stream_plan + function_plan
    if CLUSTER:
        partial_orders = get_partial_orders(external_plan)
        cluster_plans = get_connected_components(external_plan, partial_orders)
    else:
        cluster_plans = [external_plan]
    optimizer_plan = []
    for cluster_plan in cluster_plans:
        if all(isinstance(r, FunctionResult) for r in cluster_plan):
            continue
        if len(cluster_plan) == 1:
            optimizer_plan.append(cluster_plan[0])
            continue
        stream = OptimizerStream(optimizer, cluster_plan)
        instance = stream.get_instance(stream.input_objects, fluent_facts=stream.fluent_facts)
        result = instance.get_result(stream.output_objects)
        optimizer_plan.append(result)
    return optimizer_plan


def combine_optimizers_greedy(evaluations, external_plan):
    if not is_plan(external_plan):
        return external_plan
    # The key thing is that a variable must be grounded before it can used in a non-stream thing
    # TODO: construct variables in order
    # TODO: graph cut algorithm to minimize the number of constraints that are excluded
    # TODO: reorder to ensure that constraints are done first since they are likely to fail as tests
    incoming_edges, outgoing_edges = neighbors_from_orders(get_partial_orders(external_plan))
    queue = []
    functions = []
    for v in external_plan:
        if not incoming_edges[v]:
            (functions if isinstance(v, FunctionResult) else queue).append(v)
    current = []
    ordering = []
    while queue:
        optimizer = get_optimizer(current[-1]) if current else None
        for v in queue:
            if optimizer == get_optimizer(v):
                current.append(v)
                break
        else:
            ordering.extend(combine_optimizer_plan(current, functions))
            current = [queue[0]]
        v1 = current[-1]
        queue.remove(v1)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1)
            if not incoming_edges[v2]:
                (functions if isinstance(v2, FunctionResult) else queue).append(v2)
    ordering.extend(combine_optimizer_plan(current, functions))
    return ordering + functions


def sequence_results(evaluations, combined_results):
    current_facts = set()
    for result in combined_results:
        current_facts.update(filter(lambda f: evaluation_from_fact(f) in evaluations, result.get_domain()))
    combined_plan = []
    while combined_results:
        for result in combined_results:
            if set(result.get_domain()) <= current_facts:
                combined_plan.append(result)
                current_facts.update(result.get_certified())
                combined_results.remove(result)
                break
        else: # TODO: can also just try one cluster and return
            return None
    return combined_plan


def combine_optimizers(evaluations, external_plan):
    if not is_plan(external_plan):
        return external_plan
    stream_plan, function_plan = partition_external_plan(external_plan)
    optimizers = {get_optimizer(r) for r in stream_plan} # None is like a unique optimizer
    if len(optimizers - {None}) == 0:
        return external_plan

    print('Constraint plan: {}'.format(external_plan))
    combined_results = []
    for optimizer in optimizers:
        relevant_results = [r for r in stream_plan if get_optimizer(r) == optimizer]
        combined_results.extend(combine_optimizer_plan(relevant_results, function_plan))
    return sequence_results(evaluations, combined_results + function_plan)

##################################################

def retrace_instantiation(fact, streams, evaluations, free_parameters, visited_facts, planned_results):
    if (evaluation_from_fact(fact) in evaluations) or (fact in visited_facts):
        return
    visited_facts.add(fact)
    for stream in streams:
        for cert in stream.certified:
            if get_prefix(fact) == get_prefix(cert):
                mapping = get_mapping(get_args(cert), get_args(fact))  # Should be same anyways
                if not all(p in mapping for p in (stream.inputs + stream.outputs)):
                    # TODO: assumes another effect is sufficient for binding
                    # Create arbitrary objects for inputs/outputs that aren't mentioned
                    # Can lead to incorrect ordering
                    continue

                input_objects = tuple(mapping[p] for p in stream.inputs)
                output_objects = tuple(mapping[p] for p in stream.outputs)
                if not all(out in free_parameters for out in output_objects):
                    # Can only bind if free
                    continue
                instance = stream.get_instance(input_objects)
                for new_fact in instance.get_domain():
                    retrace_instantiation(new_fact, streams, evaluations, free_parameters,
                                          visited_facts, planned_results)
                planned_results.append(instance.get_result(output_objects))


def replan_with_optimizers(evaluations, external_plan, domain, optimizers):
    # TODO: return multiple plans?
    # TODO: can instead have multiple goal binding combinations
    # TODO: can replan using samplers as well
    if not is_plan(external_plan):
        return None
    optimizers = list(filter(lambda s: type(s) in OPTIMIZER_STREAMS, optimizers))
    if not optimizers:
        return None
    stream_plan, function_plan = partition_external_plan(external_plan)
    free_parameters = {o for r in stream_plan for o in r.output_objects}
    #free_parameters = {o for r in stream_plan for o in r.output_objects if isinstance(o, OptimisticObject)}
    initial_evaluations = {e: n for e, n in evaluations.items() if n.result == INIT_EVALUATION}
    #initial_evaluations = evaluations
    goal_facts = set()
    for result in stream_plan:
        goal_facts.update(filter(lambda f: evaluation_from_fact(f) not in
                                           initial_evaluations, result.get_certified()))

    visited_facts = set()
    new_results = []
    for fact in goal_facts:
        retrace_instantiation(fact, optimizers, initial_evaluations, free_parameters, visited_facts, new_results)
    variable_results = filter(lambda r: isinstance(r.external, VariableStream), new_results)
    constraint_results = filter(lambda r: isinstance(r.external, ConstraintStream), new_results)
    new_results = variable_results + constraint_results # TODO: ensure correct ordering

    #from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan
    #node_from_atom = get_achieving_streams(evaluations, stream_results) # TODO: make these lower effort
    #extract_stream_plan(node_from_atom, target_facts, stream_plan)

    optimizer_results = []
    for optimizer in {get_optimizer(r) for r in new_results}: # None is like a unique optimizer:
        relevant_results = [r for r in new_results if get_optimizer(r) == optimizer]
        optimizer_results.extend(combine_optimizer_plan(relevant_results, function_plan))
    #print(str_from_object(set(map(fact_from_evaluation, evaluations))))
    #print(str_from_object(set(goal_facts)))

    # TODO: can do the flexibly sized optimizers search
    from pddlstream.algorithms.scheduling.postprocess import reschedule_stream_plan
    optimizer_plan = reschedule_stream_plan(initial_evaluations, goal_facts, copy.copy(domain),
                                           (stream_plan + optimizer_results),
                                           unique_binding=True, unit_efforts=True)
    if not is_plan(optimizer_plan):
        return None
    return optimizer_plan + function_plan
