#!/usr/bin/env python

from __future__ import print_function

from itertools import chain, repeat

import random

from pddlstream.algorithms.common import SolutionStore
from pddlstream.algorithms.meta import create_parser
from pddlstream.algorithms.recover_optimizers import combine_optimizers
from pddlstream.algorithms.reorder import reorder_stream_plan
from pddlstream.algorithms.satisfaction import parse_value
# from pddlstream.algorithms.downward import has_costs
from pddlstream.algorithms.scheduling.plan_streams import OptSolution
from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.algorithms.visualization import create_visualizations, \
    log_plans, visualize_stream_plan
from pddlstream.language.constants import OptPlan, is_plan, get_length, str_from_plan, print_solution, Output
from pddlstream.language.statistics import compute_plan_effort
from pddlstream.language.function import Function
from pddlstream.language.stream import Stream, StreamInfo, DEBUG
from pddlstream.language.generator import from_gen, universe_test, from_gen_fn
from pddlstream.utils import INF, SEPARATOR, neighbors_from_orders, safe_apply_mapping, inf_generator, irange, safe_zip

# TODO: move other advanced out of their directories if only a single file

PREDICATE = 'P'
NAME_TEMPLATE = 's{}' # s{}
#PREDICATE_TEMPLATE = '{}-{}'
PARAM_TEMPLATE = '?{}-{}' # PARAMETER +
VALUE = True
N = 1000

##################################################

def create_problem1():
    streams = [NAME_TEMPLATE.format(i) for i in range(5)]
    orders = {
        (streams[0], streams[1]),
        (streams[2], streams[3]),
        (streams[1], streams[4]),
        (streams[3], streams[4]),
    }
    info = {
        # TODO: update to use itertools
        streams[2]: StreamInfo(p_success=0.),
        #streams[3]: StreamInfo(p_success=0.),
        #streams[4]: StreamInfo(p_success=0.),
    }
    #tests = {n: lambda *args, **kwargs: random.random() < 1. for n in streams} # TODO: sampler_from_random?

    return streams, orders, info

def create_problem2(chain1=2, chain2=2):
    # TODO: store execution traces
    streams = [NAME_TEMPLATE.format(i) for i in range(chain1 + chain2)]
    orders = set(get_pairs(streams[:chain1])) | \
             set(get_pairs(streams[chain1:]))
    # info = {
    #     streams[1]: StreamInfo(p_success=0.),
    #     streams[2]: StreamInfo(p_success=0.),
    #     streams[3]: StreamInfo(p_success=0.),
    # }
    # info = [
    #     repeat(True, times=N),
    #     chain(repeat(True, times=N), repeat(True, times=N)),
    #     repeat(False, times=N),
    # ]
    info = [
        {'p_success': 1., 'max_attempts': INF},
        {'p_success': 0.2, 'max_attempts': 1},
        {'p_success': 1., 'max_attempts': INF},
        {'p_success': 0., 'max_attempts': INF},
    ]

    return streams, orders, info

##################################################

def get_pairs(sequence):
    sequence = list(sequence)
    return safe_zip(sequence[:-1], sequence[1:])

def constant_generator(constant, **kwargs):
    return (constant for _ in inf_generator(**kwargs))

def outcome_generator(p_success=1., start_index=1, stop_index=INF):
    # TODO: debug values that use info
    for i in inf_generator():
        yield (start_index <= i < stop_index) and (random.random() < p_success)

def get_gen(generator, outputs=[], **kwargs):
    history = []
    #generator = outcome_generator(**kwargs)
    for outcome in generator:
        values = ['{}-{}'.format(output[1:], len(history)) for output in outputs]
        history.append(outcome)
        yield values if outcome else None
        input()
    #return (output if random.random() < p_success else None for _ in inf_generator())

def get_gen_fn(outputs=[], p_success=1., max_attempts=INF):
    history = []

    def gen_fn(*args, **kwargs):
        for _ in irange(max_attempts):
            if random.random() < p_success:
                values = ['{}-{}'.format(output[1:], len(history)) for output in outputs]
                history.append(values)
                yield values
            else:
                yield None
    return gen_fn

##################################################

def opt_from_graph(names, orders, infos={}):
    param_from_order = {order: PARAM_TEMPLATE.format(*order) for order in orders}
    fact_from_order = {order: (PREDICATE, param_from_order[order]) for order in orders}
    object_from_param = {param: parse_value(param) for param in param_from_order.values()}

    incoming_from_edges, outgoing_from_edges = neighbors_from_orders(orders)
    stream_plan = []
    for i, n in enumerate(names):
        #info = infos.get(n, StreamInfo(p_success=1, overhead=0, verbose=True))
        inputs = [param_from_order[n2, n] for n2 in incoming_from_edges[n]]
        outputs = [param_from_order[n, n2] for n2 in outgoing_from_edges[n]]
        #gen = get_gen(outputs=outputs, p_success=info.p_success)
        #gen = get_gen(infos[i], outputs=outputs)
        stream = Stream(
            name=n,
            #gen_fn=DEBUG,
            #gen_fn=from_gen(gen),
            gen_fn=from_gen_fn(get_gen_fn(outputs=outputs, **infos[i])),
            inputs=inputs,
            domain=[fact_from_order[n2, n] for n2 in incoming_from_edges[n]],
            fluents=[],
            outputs=outputs,
            certified=[fact_from_order[n, n2] for n2 in outgoing_from_edges[n]],
            #info=info,
            info=StreamInfo(),
        )
        # TODO: dump names

        print()
        print(stream)
        input_objects = safe_apply_mapping(stream.inputs, object_from_param)
        instance = stream.get_instance(input_objects)
        print(instance)
        output_objects = safe_apply_mapping(stream.outputs, object_from_param)
        result = instance.get_result(output_objects)
        print(result)
        stream_plan.append(result)

    opt_plan = OptPlan(action_plan=[], preimage_facts=[])
    opt_solution = OptSolution(stream_plan, opt_plan, cost=1)
    return opt_solution

##################################################

def solve_skeleton(evaluations={}, opt_solutions=[], max_time=INF, success_cost=0,
                   max_complexity=INF, reorder=False, visualize=True):
    store = SolutionStore(evaluations=evaluations, max_time=max_time, success_cost=success_cost, verbose=True)
    skeleton_queue = SkeletonQueue(store, domain=None, disable=True)

    for opt_solution in opt_solutions:
        # TODO: add and then process later
        stream_plan, opt_plan, cost = opt_solution

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
            #create_visualizations(evaluations, stream_plan, num_iterations)
            visualize_stream_plan(stream_plan)

        ################

        # # optimizer_plan = replan_with_optimizers(evaluations, stream_plan, domain, optimizers)
        # optimizer_plan = None
        # if optimizer_plan is not None:
        #     # TODO: post process a bound plan
        #     print('Optimizer plan ({}, {:.3f}): {}'.format(
        #         get_length(optimizer_plan), compute_plan_effort(optimizer_plan), optimizer_plan))
        #     skeleton_queue.new_skeleton(optimizer_plan, opt_plan, cost)

        # TODO: these are quite different
        #skeleton_queue.process(stream_plan, opt_plan, cost, complexity_limit=INF, max_time=INF)
        skeleton_queue.process(stream_plan, opt_plan, cost, complexity_limit=max_complexity, max_time=0)
    return store.extract_solution()

def main():
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    streams, orders, info = create_problem2() # create_problem1 | create_problem2
    opt_solution = opt_from_graph(streams, orders, info)
    print(SEPARATOR)

    solution = solve_skeleton(opt_solutions=[opt_solution])
    print(SEPARATOR)

    print(solution) # TODO: print the stream plan
    print_solution(solution)

if __name__ == '__main__':
    main()
