from __future__ import print_function

from itertools import product
from copy import deepcopy, copy

from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.reorder import separate_plan
from pddlstream.algorithms.scheduling.plan_streams import plan_streams
from pddlstream.algorithms.scheduling.utils import evaluations_from_stream_plan
from pddlstream.algorithms.constraints import add_plan_constraints, PlanConstraints, WILD
from pddlstream.language.constants import FAILED, INFEASIBLE, is_plan
from pddlstream.language.conversion import evaluation_from_fact, substitute_expression
from pddlstream.language.function import FunctionResult, Function
from pddlstream.language.stream import StreamResult, Result
from pddlstream.language.effort import compute_external_effort
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.utils import INF, safe_zip, get_mapping

CONSTRAIN_STREAMS = False
CONSTRAIN_PLANS = False # TODO: might cause some strange effects on continuous_tamp
MAX_DEPTH = INF # 1 | INF

def is_refined(stream_plan):
    # TODO: lazily expand the shared objects in some cases to prevent increase in size
    if not is_plan(stream_plan):
        return True
    return max([0] + [r.opt_index for r in stream_plan]) == 0
    #return all(result.opt_index == 0 for result in stream_plan)
    # TODO: some of these opt_index equal None

##################################################

def optimistic_process_instance(instantiator, instance):
    for result in instance.next_optimistic():
        new_facts = False
        complexity = instantiator.compute_complexity(instance)
        for fact in result.get_certified():
            new_facts |= instantiator.add_atom(evaluation_from_fact(fact), complexity)
        if isinstance(result, FunctionResult) or new_facts:
            yield result

def optimistic_process_function_queue(instantiator):
    while instantiator.function_queue:
        for result in optimistic_process_instance(instantiator, instantiator.pop_function()):
            yield result

def prune_high_effort_streams(streams, max_effort=INF, **effort_args):
    # TODO: convert streams to test streams with extremely high effort
    low_effort_streams = []
    for stream in streams:
        effort = compute_external_effort(stream, **effort_args)
        if isinstance(stream, Function) or (effort < max_effort): # exclusive
            low_effort_streams.append(stream)
    return low_effort_streams

def optimistic_process_streams(evaluations, streams, complexity_limit, **effort_args):
    optimistic_streams = prune_high_effort_streams(streams, **effort_args)
    instantiator = Instantiator(optimistic_streams)
    for evaluation, node in evaluations.items():
        if node.complexity <= complexity_limit:
            instantiator.add_atom(evaluation, node.complexity)
    results = []
    while instantiator.stream_queue and (instantiator.min_complexity() <= complexity_limit):
        results.extend(optimistic_process_instance(instantiator, instantiator.pop_stream()))
        # TODO: instantiate and solve to avoid repeated work
    results.extend(optimistic_process_function_queue(instantiator))
    exhausted = not instantiator
    return results, exhausted

##################################################

def optimistic_stream_instantiation(instance, bindings, evaluations, opt_evaluations,
                                    only_immediate=False):
    # TODO: combination for domain predicates
    new_instances = []
    for input_combo in product(*[bindings.get(i, [i]) for i in instance.input_objects]):
        mapping = get_mapping(instance.input_objects, input_combo)
        domain_evaluations = set(map(evaluation_from_fact, substitute_expression(
            instance.get_domain(), mapping))) # TODO: could just instantiate first
        if domain_evaluations <= opt_evaluations:
            new_instance = instance.external.get_instance(input_combo)
            if (new_instance.opt_index != 0) and (not only_immediate or (domain_evaluations <= evaluations)):
                new_instance.opt_index -= 1
            new_instances.append(new_instance)
    return new_instances

def optimistic_stream_evaluation(evaluations, stream_plan, use_bindings=True):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    evaluations = set(evaluations) # For subset testing
    opt_evaluations = set(evaluations)
    new_results = []
    bindings = {}
    for opt_result in stream_plan: # TODO: just refine the first step of the plan
        for new_instance in optimistic_stream_instantiation(
                opt_result.instance, (bindings if use_bindings else {}), evaluations, opt_evaluations):
            for new_result in new_instance.next_optimistic():
                opt_evaluations.update(map(evaluation_from_fact, new_result.get_certified()))
                new_results.append(new_result)
                if isinstance(new_result, StreamResult): # Could not add if same value
                    for opt, obj in safe_zip(opt_result.output_objects, new_result.output_objects):
                        bindings.setdefault(opt, []).append(obj)
    return new_results, bindings

##################################################

def compute_stream_results(evaluations, opt_results, externals, complexity_limit, **effort_args):
    # TODO: start from the original evaluations or use the stream plan preimage
    # TODO: only use streams in the states between the two actions
    # TODO: apply hierarchical planning to restrict the set of streams that considered on each subproblem
    # TODO: plan up to first action that only has one
    # TODO: revisit considering double bound streams
    functions = list(filter(lambda s: type(s) is Function, externals))
    opt_evaluations = evaluations_from_stream_plan(evaluations, opt_results)
    new_results, _ = optimistic_process_streams(opt_evaluations, functions, complexity_limit, **effort_args)
    return opt_results + new_results

def compute_skeleton_constraints(action_plan, bindings):
    skeleton = []
    groups = {arg: values for arg, values in bindings.items() if len(values) != 1}
    for name, args in action_plan:
        new_args = []
        for arg in args:
            if isinstance(arg, Object):
                new_args.append(arg)
            elif isinstance(arg, OptimisticObject):
                assert bindings.get(arg, [])
                if len(bindings[arg]) == 1:
                    new_args.append(bindings[arg][0])
                else:
                    #new_args.append(WILD)
                    new_args.append(arg)
            else:
                raise ValueError(arg)
        skeleton.append((name, new_args))
    # exact=False because we might need new actions
    return PlanConstraints(skeletons=[skeleton], groups=groups, exact=False, max_cost=INF)

def get_optimistic_solve_fn(goal_exp, domain, negative, max_cost=INF, **kwargs):
    # TODO: apply to hierarchical actions representations (will need to instantiate more actions)
    def fn(evaluations, results, constraints):
        if constraints is None:
            return plan_streams(evaluations, goal_exp, domain, results, negative,
                                max_cost=max_cost, **kwargs)
        #print(*relaxed_stream_plan(evaluations, goal_exp, domain, results, negative,
        #                               max_cost=max_cost, **kwargs))
        #constraints.dump()
        domain2 = deepcopy(domain)
        evaluations2 = copy(evaluations)
        goal_exp2 = add_plan_constraints(constraints, domain2, evaluations2, goal_exp, internal=True)
        max_cost2 = max_cost if constraints is None else min(max_cost, constraints.max_cost)
        combined_plan, cost = plan_streams(evaluations2, goal_exp2, domain2, results, negative,
                                           max_cost=max_cost2, **kwargs)
        #print(combined_plan, cost)
        #raw_input('Continue?')
        return combined_plan, cost
    return fn

##################################################

def hierarchical_plan_streams(evaluations, externals, results, optimistic_solve_fn, complexity_limit,
                              depth, constraints, **effort_args):
    if MAX_DEPTH <= depth:
        return None, INF, depth
    combined_plan, cost = optimistic_solve_fn(evaluations, results, constraints)
    if not is_plan(combined_plan):
        return combined_plan, cost, depth
    stream_plan, action_plan = separate_plan(combined_plan, stream_only=False)
    #dump_plans(stream_plan, action_plan, cost)
    #create_visualizations(evaluations, stream_plan, depth)
    #print(depth, get_length(stream_plan))
    if is_refined(stream_plan):
        return combined_plan, cost, depth
    new_results, bindings = optimistic_stream_evaluation(evaluations, stream_plan)
    if not CONSTRAIN_STREAMS and not CONSTRAIN_PLANS:
        return None, INF, depth + 1
    if CONSTRAIN_STREAMS:
        next_results = compute_stream_results(evaluations, new_results, externals, **effort_args)
    else:
        next_results, _ = optimistic_process_streams(evaluations, externals, complexity_limit, **effort_args)
    next_constraints = None
    if CONSTRAIN_PLANS:
        next_constraints = compute_skeleton_constraints(action_plan, bindings)
    return hierarchical_plan_streams(evaluations, externals, next_results, optimistic_solve_fn, complexity_limit,
                                     depth + 1, next_constraints, **effort_args)

def iterative_plan_streams(all_evaluations, externals, optimistic_solve_fn, complexity_limit, **effort_args):
    # Previously didn't have unique optimistic objects that could be constructed at arbitrary depths
    complexity_evals = {e: n for e, n in all_evaluations.items() if n.complexity <= complexity_limit}
    num_iterations = 0
    while True:
        num_iterations += 1
        results, exhausted = optimistic_process_streams(complexity_evals, externals, complexity_limit, **effort_args)
        combined_plan, cost, final_depth = hierarchical_plan_streams(
            complexity_evals, externals, results, optimistic_solve_fn, complexity_limit,
            depth=0, constraints=None, **effort_args)
        print('Attempt: {} | Results: {} | Depth: {} | Success: {}'.format(
            num_iterations, len(results), final_depth, combined_plan is not None))
        if is_plan(combined_plan):
            return combined_plan, cost
        if final_depth == 0:
            status = INFEASIBLE if exhausted else FAILED
            return status, cost
