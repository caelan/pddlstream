#!/usr/bin/env python

from __future__ import print_function

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, Or, Exists, print_solution, PARAMETER, Fact
from pddlstream.language.stream import StreamInfo, Stream, DEBUG
from pddlstream.language.generator import from_fn, from_test
from pddlstream.utils import outgoing_from_edges, incoming_from_edges, neighbors_from_orders
from pddlstream.algorithms.algorithm import parse_stream_pddl, evaluations_from_init
from pddlstream.algorithms.common import SolutionStore
from pddlstream.algorithms.disable_skeleton import create_disabled_axioms, extract_disabled_clusters
from pddlstream.algorithms.downward import make_domain, make_predicate, add_predicate
from pddlstream.algorithms.recover_optimizers import retrace_instantiation, combine_optimizers
from pddlstream.algorithms.reorder import reorder_stream_plan
from pddlstream.algorithms.scheduling.postprocess import reschedule_stream_plan
# from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.language.constants import is_parameter, get_length, partition_facts, Assignment, OptPlan
from pddlstream.language.conversion import revert_solution, \
    evaluation_from_fact, replace_expression, get_prefix, get_args
from pddlstream.language.function import Function
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.language.statistics import write_stream_statistics, compute_plan_effort
from pddlstream.language.stream import Stream
from pddlstream.algorithms.visualization import visualize_constraints
from pddlstream.algorithms.satisfaction import parse_value
from pddlstream.utils import INF, get_mapping, elapsed_time, str_from_object, safe_zip, safe_apply_mapping

from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.advanced import enforce_simultaneous, identify_non_producers
from pddlstream.algorithms.common import SolutionStore
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.disabled import push_disabled, reenable_disabled, process_stream_plan
from pddlstream.algorithms.disable_skeleton import create_disabled_axioms
#from pddlstream.algorithms.downward import has_costs
from pddlstream.algorithms.incremental import process_stream_queue
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.refinement import iterative_plan_streams, get_optimistic_solve_fn
from pddlstream.algorithms.scheduling.plan_streams import OptSolution
from pddlstream.algorithms.reorder import reorder_stream_plan
from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.algorithms.visualization import reset_visualizations, create_visualizations, \
    has_pygraphviz, log_plans
from pddlstream.language.constants import is_plan, get_length, str_from_plan, INFEASIBLE
from pddlstream.language.fluent import compile_fluent_streams
from pddlstream.language.function import Function, Predicate
from pddlstream.language.optimizer import ComponentStream
from pddlstream.algorithms.recover_optimizers import combine_optimizers
from pddlstream.language.statistics import load_stream_statistics, \
    write_stream_statistics, compute_plan_effort
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.utils import INF, implies, str_from_object, safe_zip

# TODO: move other advanced out of their directories if only a single file

PREDICATE = 'P'
NAME_TEMPLATE = '{}' # s{}
PREDICATE_TEMPLATE = '{}-{}'
PARAM_TEMPLATE = '?{}-{}' # PARAMETER +

##################################################

def main(reorder=False, visualize=False):
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    streams = ['{}'.format(i) for i in range(5)]
    info = {
        # Intentionally, misleading the stream
        'increment': StreamInfo(p_success=0.01, overhead=1),
        'decrement': StreamInfo(p_success=1, overhead=1),
    }

    orders = {
        (streams[0], streams[2]),
        (streams[1], streams[3]),
        (streams[2], streams[4]),
        (streams[3], streams[4]),
    }
    incoming_from_edges, outgoing_from_edges = neighbors_from_orders(orders)

    param_from_order = {order: PARAM_TEMPLATE.format(*order) for order in orders}
    fact_from_order = {order: (PREDICATE, param_from_order[order]) for order in orders}
    object_from_param = {param: parse_value(param) for param in param_from_order.values()}

    opt_plan = OptPlan(action_plan=[], preimage_facts=[])
    cost = 0
    stream_plan = []
    for s in streams:
        stream = Stream(
            name=s,
            gen_fn=DEBUG, #from_test(universe_test),
            inputs=[param_from_order[s2, s] for s2 in incoming_from_edges[s]],
            domain=[fact_from_order[s2, s] for s2 in incoming_from_edges[s]],
            fluents=[],
            outputs=[param_from_order[s, s2] for s2 in outgoing_from_edges[s]],
            certified=[fact_from_order[s, s2] for s2 in outgoing_from_edges[s]],
            info=StreamInfo(p_success=1, overhead=0, verbose=True),
        )
        # TODO: dump streams
        print(stream)

        input_objects = safe_apply_mapping(stream.inputs, object_from_param)
        instance = stream.get_instance(input_objects)
        print(instance)
        output_objects = safe_apply_mapping(stream.outputs, object_from_param)
        result = instance.get_result(output_objects)
        print(result)
        stream_plan.append(result)
        print()

    evaluations = {}
    store = SolutionStore(evaluations=evaluations, max_time=INF, success_cost=INF, verbose=True)
    skeleton_queue = SkeletonQueue(store, domain=None, disable=True)

    ################

    # stream_plan = replan_with_optimizers(evaluations, stream_plan, domain, externals) or stream_plan
    stream_plan = combine_optimizers(evaluations, stream_plan)
    # stream_plan = get_synthetic_stream_plan(stream_plan, # evaluations
    #                                       [s for s in synthesizers if not s.post_only])
    # stream_plan = recover_optimistic_outputs(stream_plan)
    if reorder:
        # TODO: this blows up memory wise for long stream plans
        stream_plan = reorder_stream_plan(store, stream_plan)

    num_optimistic = sum(r.optimistic for r in stream_plan) if stream_plan else 0
    action_plan = opt_plan.action_plan if is_plan(opt_plan) else opt_plan
    print('Stream plan ({}, {}, {:.3f}): {}\nAction plan ({}, {:.3f}): {}'.format(
        get_length(stream_plan), num_optimistic, compute_plan_effort(stream_plan), stream_plan,
        get_length(action_plan), cost, str_from_plan(action_plan)))
    if is_plan(stream_plan) and visualize:
        num_iterations = 0
        log_plans(stream_plan, action_plan, num_iterations)
        create_visualizations(evaluations, stream_plan, num_iterations)

    ################

    # # optimizer_plan = replan_with_optimizers(evaluations, stream_plan, domain, optimizers)
    # optimizer_plan = None
    # if optimizer_plan is not None:
    #     # TODO: post process a bound plan
    #     print('Optimizer plan ({}, {:.3f}): {}'.format(
    #         get_length(optimizer_plan), compute_plan_effort(optimizer_plan), optimizer_plan))
    #     skeleton_queue.new_skeleton(optimizer_plan, opt_plan, cost)

    skeleton_queue.process(stream_plan, opt_plan, cost, complexity_limit=INF, max_time=INF) # is INFEASIBLE:

    solution = store.extract_solution()
    print(solution)
    print_solution(solution)

if __name__ == '__main__':
    main()
