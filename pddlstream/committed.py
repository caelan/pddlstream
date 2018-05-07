import time
from collections import defaultdict
from itertools import product

from pddlstream.algorithm import parse_problem, get_optimistic_constraints
from pddlstream.incremental import process_stream_queue
from pddlstream.context import ConstraintSolver, create_immediate_context
from pddlstream.conversion import revert_solution, evaluation_from_fact, substitute_expression
from pddlstream.function import Function
from pddlstream.instantiation import Instantiator
from pddlstream.object import Object
from pddlstream.scheduling.relaxed import relaxed_stream_plan
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan, evaluations_from_stream_plan
from pddlstream.stream import StreamResult
from pddlstream.utils import INF, elapsed_time
from pddlstream.visualization import clear_visualizations, create_visualizations

def disable_stream_instance(stream_instance, disabled):
    disabled.append(stream_instance)
    stream_instance.disabled = True

def reset_disabled(disabled):
    for stream_instance in disabled:
        stream_instance.disabled = False
    disabled[:] = []

##################################################

def optimistic_process_stream_queue(instantiator):
    stream_instance = instantiator.stream_queue.popleft()
    stream_results = stream_instance.next_optimistic()
    for stream_result in stream_results:
        for fact in stream_result.get_certified():
            instantiator.add_atom(evaluation_from_fact(fact))
    return stream_results

def ground_stream_instances(stream_instance, bindings, evaluations):
    # TODO: combination for domain predicates
    input_objects = [[i] if isinstance(i, Object) else bindings[i]
                    for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = dict(zip(stream_instance.input_objects, combo))
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping)))
        if domain <= evaluations:
            yield stream_instance.external.get_instance(combo)

##################################################

def update_info(externals, stream_info):
    for external in externals:
        if external.name in stream_info:
            external.info = stream_info[external.name]


def eagerly_evaluate(evaluations, externals, num_iterations, max_time, verbose):
    start_time = time.time()
    instantiator = Instantiator(evaluations, externals)
    for _ in range(num_iterations):
        for _ in range(len(instantiator.stream_queue)):
            if max_time <= elapsed_time(start_time):
                break
            process_stream_queue(instantiator, evaluations, verbose=verbose)

##################################################

def populate_results(evaluations, streams, max_time):
    #start_time = time.time()
    instantiator = Instantiator(evaluations, streams)
    stream_results = []
    while instantiator.stream_queue: # and (elapsed_time(start_time) < max_time):
        stream_results += optimistic_process_stream_queue(instantiator)
    return stream_results

##################################################

def process_stream_plan(evaluations, stream_plan, disabled, verbose,
                        quick_fail=False, layers=False, max_values=INF):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    # TODO: identify outputs bound to twice and don't sample for them
    opt_bindings = defaultdict(list)
    next_results = []
    failed = False
    for step, opt_result in enumerate(stream_plan):
        if failed and quick_fail:  # TODO: check if satisfies target certified
            break
        # TODO: could update all at once here
        # Could check opt_bindings to see if new bindings
        num_instances = max_values if (layers or all(isinstance(o, Object)
                                                     for o in opt_result.instance.input_objects)) else 0
        for i, instance in enumerate(ground_stream_instances(opt_result.instance, opt_bindings, evaluations)):
            if i < num_instances:
                results = instance.next_results(verbose=verbose, stream_plan=stream_plan[step:])
                disable_stream_instance(instance, disabled)
            else:
                results = instance.next_optimistic()
                next_results += results
            failed |= not results
            #if verbose and (not results):
            #    print('Stream failure')
            for result in results:
                if i < num_instances:
                    evaluations.update(map(evaluation_from_fact, result.get_certified()))
                if isinstance(result, StreamResult): # Could not add if same value
                    for opt, obj in zip(opt_result.output_objects, result.output_objects):
                        opt_bindings[opt].append(obj)
    if failed:
        return None
    return next_results

##################################################

def solve_committed(problem, max_time=INF, max_cost=INF, stream_info={},
                    commit=True, effort_weight=None, eager_iterations=1, visualize=False, verbose=True, **kwargs):
    # TODO: return to just using the highest level samplers at the start
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, externals = parse_problem(problem)
    update_info(externals, stream_info)
    eager_externals = filter(lambda e: e.info.eager, externals)
    constraint_solver = ConstraintSolver(problem[3])
    disabled = []
    if visualize:
        clear_visualizations()
    #functions = filter(lambda s: isinstance(s, Function), externals)
    functions = filter(lambda s: type(s) is Function, externals)
    streams = filter(lambda s: s not in functions, externals)
    stream_results = populate_results(evaluations, streams, max_time-elapsed_time(start_time))
    depth = 0
    while elapsed_time(start_time) < max_time:
        if stream_results is None:
            stream_plan, action_plan, cost = None, None, INF
        else:
            num_iterations += 1
            print('\nIteration: {} | Depth: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
                num_iterations, depth, len(evaluations), best_cost, elapsed_time(start_time)))
            # TODO: constrain to use previous plan to some degree
            eagerly_evaluate(evaluations, eager_externals, eager_iterations, max_time-elapsed_time(start_time), verbose)
            stream_results += populate_results(evaluations_from_stream_plan(evaluations, stream_results),
                                               functions, max_time-elapsed_time(start_time))
            solve_stream_plan = relaxed_stream_plan if effort_weight is None else simultaneous_stream_plan
            stream_plan, action_plan, cost = solve_stream_plan(evaluations, goal_expression,
                                                         domain, stream_results, max_cost=best_cost, **kwargs)
            print('Stream plan: {}\n'
                  'Action plan: {}'.format(stream_plan, action_plan))
        if stream_plan is None:
            if disabled or (depth != 0):
                if depth == 0:
                    reset_disabled(disabled)
                stream_results = populate_results(evaluations, streams, max_time - elapsed_time(start_time))
                depth = 0 # Recurse on problems
            else:
                break
        elif len(stream_plan) == 0:
            if cost < best_cost:
                best_plan = action_plan; best_cost = cost
                if best_cost < max_cost:
                    break
            stream_results = None
        else:
            if visualize:
                create_visualizations(evaluations, stream_plan, num_iterations)
            constraint_facts = constraint_solver.solve(get_optimistic_constraints(evaluations, stream_plan), verbose=verbose)
            evaluations.update(map(evaluation_from_fact, constraint_facts))
            if constraint_facts:
                stream_results = []
            else:
                stream_results = process_stream_plan(evaluations, stream_plan, disabled, verbose)
            if not commit:
                stream_results = None
            depth += 1
    return revert_solution(best_plan, best_cost, evaluations)