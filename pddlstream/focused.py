from __future__ import print_function

import os
import time
from collections import defaultdict, deque, namedtuple

from pddlstream.algorithm import parse_problem
from pddlstream.conversion import revert_solution
from pddlstream.function import Function, Predicate
from pddlstream.incremental import process_stream_queue
from pddlstream.instantiation import Instantiator
from pddlstream.macro_stream import get_macro_stream_plan
from pddlstream.postprocess import locally_optimize, populate_results, process_stream_plan
from pddlstream.reorder import separate_plan, reorder_combined_plan, reorder_stream_plan
from pddlstream.scheduling.relaxed import relaxed_stream_plan
from pddlstream.scheduling.simultaneous import evaluations_from_stream_plan
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan
from pddlstream.utils import INF, elapsed_time, write_pickle, read_pickle, ensure_dir
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

def reset_disabled(disabled):
    for stream_instance in disabled:
        stream_instance.disabled = False
    disabled[:] = []

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
    print('\nExternal Statistics')
    data = {}
    for external in externals:
        data[external.name] = {
            'calls': external.total_calls,
            'overhead': external.total_overhead,
            'successes': external.total_successes,
        }
        print(external.name, external.get_p_success(), external.get_overhead(), external.get_effort()) #, data[external.name])
    filename = get_stream_data_filename(stream_name)
    ensure_dir(filename)
    write_pickle(filename, data)
    print('Wrote:', filename)

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