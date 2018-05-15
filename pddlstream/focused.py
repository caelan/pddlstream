from __future__ import print_function

import time
import os
from collections import defaultdict, deque, namedtuple
from itertools import product

from pddlstream.algorithm import parse_problem
from pddlstream.conversion import revert_solution, evaluation_from_fact, substitute_expression
from pddlstream.function import Function, Predicate
from pddlstream.function import PredicateResult
from pddlstream.incremental import process_stream_queue
from pddlstream.instantiation import Instantiator
from pddlstream.macro_stream import get_macro_stream_plan, MacroResult
from pddlstream.object import Object
from pddlstream.reorder import separate_plan, reorder_combined_plan, reorder_stream_plan
from pddlstream.scheduling.relaxed import relaxed_stream_plan, get_goal_instance
from pddlstream.scheduling.simultaneous import evaluations_from_stream_plan
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan
from pddlstream.stream import StreamResult
from pddlstream.utils import INF, elapsed_time, implies, write_pickle, read_pickle, ensure_dir
from pddlstream.visualization import clear_visualizations, create_visualizations


##################################################

# TODO: namedtuple
class ActionInfo(object):
    def __init__(self, terminal=False, p_success=None, overhead=None):
        """
        :param terminal: Indicates the action may require replanning after use
        :param p_success:
        """
        self.terminal = terminal # TODO: infer from p_success?
        if self.terminal:
            self.p_success, self.overhead = 1e-3, 0
        else:
            self.p_success, self.overhead = 1, INF
        if p_success is not None:
            self.p_success = p_success
        if overhead is not None:
            self.overhead = overhead
        # TODO: should overhead just be cost here then?

def get_action_info(action_info):
    action_execution = defaultdict(ActionInfo)
    for name, info in action_info.items():
        action_execution[name] = info
    return action_execution

def update_stream_info(externals, stream_info):
    for external in externals:
        if external.name in stream_info:
            external.info = stream_info[external.name]

##################################################

def eagerly_evaluate(evaluations, externals, num_iterations, max_time, verbose):
    start_time = time.time()
    instantiator = Instantiator(evaluations, externals)
    for _ in range(num_iterations):
        for _ in range(len(instantiator.stream_queue)):
            if max_time <= elapsed_time(start_time):
                break
            process_stream_queue(instantiator, evaluations, verbose=verbose)

def optimistic_process_stream_queue(instantiator):
    stream_instance = instantiator.stream_queue.popleft()
    stream_results = stream_instance.next_optimistic()
    for stream_result in stream_results:
        for fact in stream_result.get_certified():
            instantiator.add_atom(evaluation_from_fact(fact))
    return stream_results

def populate_results(evaluations, streams, initial_instances=[]):
    instantiator = Instantiator(evaluations, streams)
    for instance in initial_instances:
        instantiator.stream_queue.append(instance)
    stream_results = []
    while instantiator.stream_queue:
        stream_results += optimistic_process_stream_queue(instantiator)
    return stream_results

##################################################

def ground_stream_instances(stream_instance, bindings, evaluations, opt_evaluations):
    # TODO: combination for domain predicates
    evaluation_set = set(evaluations)
    combined_evaluations = evaluation_set | opt_evaluations
    real_instances = []
    opt_instances = []
    #input_objects = [[i] if isinstance(i, Object) else bindings[i]
    #                for i in stream_instance.input_objects]
    input_objects = [bindings.get(i, [i]) for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = dict(zip(stream_instance.input_objects, combo))
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping)))
        if domain <= combined_evaluations:
            instance = stream_instance.external.get_instance(combo)
            if domain <= evaluation_set:
                real_instances.append(instance)
            else:
                opt_instances.append(instance)
    return real_instances, opt_instances

##################################################

def disable_stream_instance(stream_instance, disabled):
    disabled.append(stream_instance)
    stream_instance.disabled = True

def reset_disabled(disabled):
    for stream_instance in disabled:
        stream_instance.disabled = False
    disabled[:] = []

##################################################

def process_stream_plan(evaluations, stream_plan, disabled, verbose,
                        quick_fail=True, layers=False, max_values=INF):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    streams_from_output = defaultdict(list)
    for result in stream_plan:
        if isinstance(result, StreamResult):
            for obj in result.output_objects:
                streams_from_output[obj].append(result)
    shared_output_streams = {s for streams in streams_from_output.values() if 1 < len(streams) for s in streams}
    #shared_output_streams = {}

    opt_bindings = defaultdict(list)
    opt_evaluations = set()
    opt_results = []
    failed = False
    stream_queue = deque(stream_plan)
    while stream_queue and implies(quick_fail, not failed):
        opt_result = stream_queue.popleft()
        real_instances, opt_instances = ground_stream_instances(opt_result.instance, opt_bindings,
                                                                evaluations, opt_evaluations)
        first_step = all(isinstance(o, Object) for o in opt_result.instance.input_objects)
        num_instances = min(len(real_instances), max_values) \
            if (layers or first_step or (opt_result not in shared_output_streams)) else 0
        opt_instances += real_instances[num_instances:]
        real_instances = real_instances[:num_instances]
        new_results = []
        local_failure = False
        for instance in real_instances:
            results = instance.next_results(verbose=verbose)
            for result in results:
                for fact in result.get_certified():
                    evaluation = evaluation_from_fact(fact)
                    if evaluation not in evaluations:
                        evaluations[evaluation] = result
            disable_stream_instance(instance, disabled)
            local_failure |= not results
            if isinstance(opt_result, PredicateResult) and not any(opt_result.value == r.value for r in results):
                local_failure = True # TODO: check for instance?
            new_results += results
        for instance in opt_instances:
            results = instance.next_optimistic()
            opt_evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            opt_results += results
            local_failure |= not results
            new_results += results
        for result in new_results:
            if isinstance(result, StreamResult): # Could not add if same value
                for opt, obj in zip(opt_result.output_objects, result.output_objects):
                    opt_bindings[opt].append(obj)
        if local_failure and isinstance(opt_result, MacroResult):
            stream_queue.extendleft(reversed(opt_result.decompose()))
            failed = False # TODO: check if satisfies target certified
        else:
            failed |= local_failure

    if verbose:
        print('Success: {}'.format(not failed))
    if failed:
        return None, None
    return opt_results, opt_bindings

##################################################

from pddlstream.reorder import separate_plan, reorder_combined_plan, reorder_stream_plan, \
    replace_derived, get_action_instances, topological_sort
from pddlstream.fast_downward import get_problem, fact_from_fd, task_from_domain_problem, get_init
from pddlstream.scheduling.relaxed import plan_preimage, recover_stream_plan
from pddlstream.conversion import evaluation_from_fact, pddl_from_object


def locally_optimize(evaluations, action_plan, goal_expression, domain, functions, negative, dynamic_streams, verbose):
    print('\nPostprocessing') # TODO: postprocess current skeleton as well

    task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs=False))
    plan_instances = get_action_instances(task, action_plan) + [get_goal_instance(task.goal)]
    replace_derived(task, set(), plan_instances)
    preimage = filter(lambda a: not a.negated, plan_preimage(plan_instances, []))

    stream_results = set()
    processed = set(map(fact_from_fd, preimage))
    queue = deque(processed)
    while queue:
        fact = queue.popleft()
        result = evaluations[evaluation_from_fact(fact)]
        if result is None:
            continue
        stream_results.add(result)
        for fact2 in result.instance.get_domain():
            if fact2 not in processed:
                queue.append(fact2)

    orders = set()
    for result in stream_results:
        for fact in result.instance.get_domain():
            result2 = evaluations[evaluation_from_fact(fact)]
            if result2 is not None:
                orders.add((result2, result))

    ordered_results = []
    for result in topological_sort(stream_results, orders):
        if isinstance(result, MacroResult):
            ordered_results.extend(result.decompose())
        else:
            ordered_results.append(result)

    opt_stream_plan = []
    opt_from_obj = {}
    for stream_result in ordered_results:
        input_objects = tuple(opt_from_obj.get(o, o) for o in stream_result.instance.input_objects)
        instance = stream_result.instance.external.get_instance(input_objects)
        assert(not instance.disabled)
        opt_results = instance.next_optimistic()
        if not opt_results:
            continue
        opt_result = opt_results[0]
        opt_stream_plan.append(opt_result)
        for obj, opt in zip(stream_result.output_objects, opt_result.output_objects):
            opt_from_obj[obj] = opt
    opt_stream_plan += populate_results(evaluations_from_stream_plan(evaluations, opt_stream_plan), functions)
    opt_action_plan = [(name, tuple(opt_from_obj.get(o, o) for o in args)) for name, args in action_plan]

    opt_evaluations = evaluations_from_stream_plan(evaluations, opt_stream_plan)
    problem = get_problem(opt_evaluations, goal_expression, domain, unit_costs=False)
    pddl_plan = [(name, map(pddl_from_object, args)) for name, args in opt_action_plan]
    stream_plan = recover_stream_plan(evaluations, opt_stream_plan, domain, problem, pddl_plan, negative, unit_costs=False)

    stream_plan = reorder_stream_plan(stream_plan)  # TODO: is this strictly redundant?
    stream_plan = get_macro_stream_plan(stream_plan, dynamic_streams)
    print('Stream plan: {}\n'
          'Action plan: {}'.format(stream_plan, opt_action_plan))

    disabled = []
    _, obj_from_opt = process_stream_plan(evaluations, stream_plan, disabled, verbose)
    if obj_from_opt is None:
        return action_plan
    # TODO: compare solution cost and validity?
    # TODO: select which binding
    # TODO: repeatedly do this
    new_action_plan = [(name, tuple(obj_from_opt.get(o, [o])[0] for o in args)) for name, args in opt_action_plan]
    return new_action_plan

##################################################

SamplingProblem = namedtuple('SamplingProblem', ['stream_plan', 'action_plan', 'cost']) # TODO: alternatively just preimage

DATA_DIR = 'data/'

def get_stream_data_filename(stream_name):
    return os.path.join(DATA_DIR, '{}.pp'.format(stream_name))

def load_stream_statistics(stream_name, externals):
    filename = get_stream_data_filename(stream_name)
    if not os.path.exists(filename):
        return
    data = read_pickle(filename)
    for external in externals:
        if external.name in data:
            statistics = data[external.name]
            external.total_calls += statistics['calls']
            external.total_overhead += statistics['overhead']
            external.total_successes += statistics['successes']

def write_stream_statistics(stream_name, externals):
    data = {}
    for external in externals:
        data[external.name] = {
            'calls': external.total_calls,
            'overhead': external.total_overhead,
            'successes': external.total_successes,
        }
        print(external.name, data[external.name])
    filename = get_stream_data_filename(stream_name)
    ensure_dir(filename)
    write_pickle(filename, data)

##################################################

def solve_focused(problem, stream_info={}, action_info={}, dynamic_streams=[],
                  max_time=INF, max_cost=INF, unit_costs=False,
                  commit=True, effort_weight=None, eager_layers=1,
                  visualize=False, verbose=True, postprocess=False, **search_kwargs):
    """
    Solves a PDDLStream problem by first hypothesizing stream outputs and then determining whether they exist
    :param problem: a PDDLStream problem
    :param action_info: a dictionary from stream name to ActionInfo for planning and execution
    :param stream_info: a dictionary from stream name to StreamInfo altering how individual streams are handled
    :param max_time: the maximum amount of time to apply streams
    :param max_cost: a strict upper bound on plan cost
    :param commit: if True, it commits to instantiating a particular partial plan-skeleton.
    :param effort_weight: a multiplier for stream effort compared to action costs
    :param eager_layers: the number of eager stream application layers per iteration
    :param visualize: if True, it draws the constraint network and stream plan as a graphviz file
    :param verbose: if True, this prints the result of each stream application
    :param search_kwargs: keyword args for the search subroutine
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    # TODO: return to just using the highest level samplers at the start
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, stream_name, externals = parse_problem(problem)
    action_info = get_action_info(action_info)
    update_stream_info(externals, stream_info)
    load_stream_statistics(stream_name, externals)
    eager_externals = filter(lambda e: e.info.eager, externals)
    disabled = []
    if visualize:
        clear_visualizations()
    #functions = filter(lambda s: isinstance(s, Function), externals)
    functions = filter(lambda s: type(s) is Function, externals)
    negative = filter(lambda s: type(s) is Predicate and s.is_negative(), externals)
    streams = filter(lambda s: s not in (functions + negative), externals)
    stream_results = []
    depth = 1
    #stream_results = populate_results(evaluations, streams)
    #depth = 0
    sampling_queue = deque()
    while elapsed_time(start_time) < max_time:
        search_time = time.time() # TODO: allocate more sampling effort to maintain the balance
        if stream_results is None:
            stream_plan, action_plan, cost = None, None, INF
        else:
            num_iterations += 1
            print('\nIteration: {} | Depth: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
                num_iterations, depth, len(evaluations), best_cost, elapsed_time(start_time)))
            # TODO: constrain to use previous plan to some degree
            eagerly_evaluate(evaluations, eager_externals, eager_layers, max_time - elapsed_time(start_time), verbose)
            stream_results += populate_results(evaluations_from_stream_plan(evaluations, stream_results), functions)
            # TODO: warning check if using simultaneous_stream_plan or relaxed_stream_plan with non-eager functions
            solve_stream_plan = relaxed_stream_plan if effort_weight is None else simultaneous_stream_plan
            #solve_stream_plan = sequential_stream_plan if effort_weight is None else simultaneous_stream_plan
            combined_plan, cost = solve_stream_plan(evaluations, goal_expression, domain, stream_results,
                                                               negative, max_cost=best_cost, unit_costs=unit_costs, **search_kwargs)
            combined_plan = reorder_combined_plan(evaluations, combined_plan, action_info, domain)
            print('Combined plan: {}'.format(combined_plan))
            stream_plan, action_plan = separate_plan(combined_plan, action_info)
            stream_plan = reorder_stream_plan(stream_plan) # TODO: is this strictly redundant?
            stream_plan = get_macro_stream_plan(stream_plan, dynamic_streams)
            print('Stream plan: {}\n'
                  'Action plan: {}'.format(stream_plan, action_plan))

        if stream_plan is None:
            if disabled or (depth != 0):
                if depth == 0:
                    reset_disabled(disabled)
                stream_results = populate_results(evaluations, streams)
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
            sampling_queue.append(SamplingProblem(stream_plan, action_plan, cost))
            if visualize:
                create_visualizations(evaluations, stream_plan, num_iterations)
            option = True
            if option:
                # TODO: can instantiate all but subtract stream_results
                # TODO: can even pass a subset of the fluent state
                # TODO: can just compute the stream plan preimage
                # TODO: replan constraining the initial state and plan skeleton
                # TODO: reuse subproblems
                # TODO: always start from the initial state (i.e. don't update)
                old_evaluations = set(evaluations)
                stream_results, _ = process_stream_plan(evaluations, stream_plan, disabled, verbose)
                new_evaluations = set(evaluations) - old_evaluations
                if stream_results is not None:
                    new_instances = [r.instance for r in stream_results]
                    stream_results = populate_results(new_evaluations, streams, new_instances)
            if not commit:
                stream_results = None
            depth += 1

    reset_disabled(disabled)
    if postprocess and (not unit_costs) and (best_plan is not None):
        best_plan = locally_optimize(evaluations, best_plan, goal_expression, domain,
                                     functions, negative, dynamic_streams, verbose)
    write_stream_statistics(stream_name, externals)
    return revert_solution(best_plan, best_cost, evaluations)