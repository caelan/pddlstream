import time
from collections import defaultdict, namedtuple
from itertools import product
from heapq import heappush, heappop
from functools import cmp_to_key

from pddlstream.algorithm import parse_problem, get_optimistic_constraints
from pddlstream.incremental import process_stream_queue
from pddlstream.context import ConstraintSolver
from pddlstream.conversion import revert_solution, evaluation_from_fact, substitute_expression
from pddlstream.function import Function, Predicate, PredicateResult
from pddlstream.instantiation import Instantiator
from pddlstream.object import Object
from pddlstream.scheduling.relaxed import relaxed_stream_plan
from pddlstream.scheduling.sequential import sequential_stream_plan
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan, evaluations_from_stream_plan
from pddlstream.stream import StreamResult
from pddlstream.utils import INF, elapsed_time
from pddlstream.visualization import clear_visualizations, create_visualizations, get_partial_orders

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

def cmp(v1, v2):
    # TODO: dynamic programming solution to this?
    i1 = v1.instance.external.info
    i2 = v2.instance.external.info
    if ((i1.p_success <= i2.p_success) and (i1.overhead < i2.overhead)) or \
            ((i1.p_success < i2.p_success) and (i1.overhead <= i2.overhead)):
        return -1
    if ((i2.p_success <= i1.p_success) and (i2.overhead < i1.overhead)) or \
            ((i2.p_success < i1.p_success) and (i2.overhead <= i1.overhead)):
        return +1
    # TODO: include context here as a weak constraint
    # TODO: actions as a weak constraint
    # TODO: works in the absence of parital orders
    # TODO: actions are extremely unlikely to work
    return 0

def get_stream_dag(stream_plan):
    orders = get_partial_orders(stream_plan)
    incoming_edges = defaultdict(set)
    outgoing_edges = defaultdict(set)
    for v1, v2 in orders:
        incoming_edges[v2].add(v1)
        outgoing_edges[v1].add(v2)
    return incoming_edges, outgoing_edges

def topological_sort2(stream_plan):
    # TODO: order streams with streams
    if stream_plan is None:
        return None
    incoming_edges, outgoing_edges = get_stream_dag(stream_plan)
    # TODO: factor other streams depending on these (i.e. if eval enables something that is likely to fail)
    key = cmp_to_key(cmp)
    #key = lambda r: r.instance.external.info.effort

    # Each thing has a evaluation cost and a probability of continuing vs moving to a terminal state
    # TODO: maybe combine these by pruning using the expected cost of the future
    ordering = []
    queue = []
    for v in stream_plan:
        if not incoming_edges[v]:
            heappush(queue, (key(v), v))
    while queue:
        _, v1 = heappop(queue)
        ordering.append(v1)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1)
            if not incoming_edges[v2]:
                heappush(queue, (key(v2), v2))
    print(compute_expected_cost(ordering))
    return ordering

def topological_sort(stream_plan):
    if stream_plan is None:
        return None
    incoming_edges, outgoing_edges = get_stream_dag(stream_plan)
    queue = []
    reversed_ordering = []
    visited = set()
    future_infos = {}
    for v in stream_plan:
        if not outgoing_edges[v]:
            info = v.instance.external.info
            future_infos[v] = (-info.p_success, -info.overhead)
            heappush(queue, (future_infos[v], v))
    while queue:
        _, v1 = heappop(queue)
        reversed_ordering.append(v1)
        visited.add(v1)
        for v2 in incoming_edges[v1]:
            if outgoing_edges[v2] <= visited:
                info = v2.instance.external.info
                future_infos[v2] = (-info.p_success, -info.overhead)
                # TODO: could factor in outgoing_edges[v2]
                heappush(queue, (future_infos[v2], v2))
    ordering = list(reversed(reversed_ordering))
    print(compute_expected_cost(ordering))
    return ordering


def compute_expected_cost(stream_plan):
    expected_cost = 0
    for result in reversed(stream_plan):
        info = result.instance.external.info
        expected_cost = info.overhead + info.p_success * expected_cost
        print(result, expected_cost)
    return expected_cost

Subproblem = namedtuple('Subproblem', ['cost', 'head', 'subset'])

# 2^N rather than N!
def dynamic_programming(stream_plan):
    if stream_plan is None:
        return None
    incoming_edges, outgoing_edges = get_stream_dag(stream_plan)
    # TODO: dynamic programming over sets or vertices fixed in order?
    # The probability operates over the full future set

    def valid_combine(v, subset):
        return outgoing_edges[v] <= subset  # Instead if prereqs not in set
        #return not (incoming_edges[v] & subset) # I think these are the same

    # Need partial ordering (acyclic) on subproblems to avoid priority queue
    # Naturally acyclic because of subsets.
    queue = [(0, frozenset())]
    expected_cost = {frozenset(): Subproblem(0, None, None)}
    visited = set()
    while queue:
        _, subset = heappop(queue)
        if subset in visited:
            continue
        visited.add(subset)
        for v in set(stream_plan) - subset: # TODO: this is where I can prune things
            if valid_combine(v, subset):
                new_subset = frozenset([v]) | subset
                info = v.instance.external.info
                new_cost = info.overhead + info.p_success*expected_cost[subset].cost # Add new element to front
                if (new_subset not in expected_cost) or (new_cost < expected_cost[new_subset].cost):
                    expected_cost[new_subset] = Subproblem(new_cost, v, subset)
                    # TODO: do partial ordering here by ensure all other subproblems are solved
                    #heappush(queue, (new_cost, new_subset))
                    heappush(queue, (len(new_subset), new_subset))
                #for v2 in new_subset:
                #    stuff = new_subset = frozenset([v2])
                #    if (new_subset not in expected_cost) and valid_combine(v2, stuff):
                #        break
                #else:
                #    heappush(queue, (expected_cost[new_subset].cost, new_subset))


                    # TODO: if no overhead, this just becomes a log shortest path problem? I don't think so really
    # TODO: single source root
    # The only decisions are when there are several things in the queue

    subset = frozenset(stream_plan)
    print(expected_cost[subset].cost)
    ordering = []
    while True:
        subproblem = expected_cost[subset]
        if subproblem.head is None:
            break
        ordering.append(subproblem.head)
        subset = subproblem.subset
    print(ordering)
    print(compute_expected_cost(ordering))
    return ordering









##################################################

def ground_stream_instances(stream_instance, bindings, evaluations, opt_evaluations):
    # TODO: combination for domain predicates
    combined_evaluations = evaluations | opt_evaluations
    real_instances = []
    opt_instances = []
    input_objects = [[i] if isinstance(i, Object) else bindings[i]
                    for i in stream_instance.input_objects]
    for combo in product(*input_objects):
        mapping = dict(zip(stream_instance.input_objects, combo))
        domain = set(map(evaluation_from_fact, substitute_expression(
            stream_instance.get_domain(), mapping)))
        if domain <= combined_evaluations:
            instance = stream_instance.external.get_instance(combo)
            if domain <= evaluations:
                real_instances.append(instance)
            else:
                opt_instances.append(instance)
    return real_instances, opt_instances

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
                        quick_fail=True, layers=False, max_values=INF):
    # TODO: can also use the instantiator and operate directly on the outputs
    # TODO: could bind by just using new_evaluations
    streams_from_output = defaultdict(list)
    for result in stream_plan:
        if isinstance(result, StreamResult):
            for obj in result.output_objects:
                streams_from_output[obj].append(result)
    shared_output_streams = {s for streams in streams_from_output.values() if 1 < len(streams) for s in streams}

    opt_bindings = defaultdict(list)
    opt_evaluations = set()
    opt_results = []
    failed = False
    for step, opt_result in enumerate(stream_plan):
        if failed and quick_fail:  # TODO: check if satisfies target certified
            break
        # Could check opt_bindings to see if new bindings
        real_instances, opt_instances = ground_stream_instances(opt_result.instance, opt_bindings, evaluations, opt_evaluations)
        #num_instances = min(len(real_instances), max_values) if (layers or all(isinstance(o, Object)
        #                                             for o in opt_result.instance.input_objects)) else 0
        num_instances = min(len(real_instances), max_values) \
            if (layers or (step == 0) or (opt_result not in shared_output_streams)) else 0
        opt_instances += real_instances[num_instances:]
        real_instances = real_instances[:num_instances]
        new_results = []
        for instance in real_instances:
            results = instance.next_results(verbose=verbose, stream_plan=stream_plan[step:])
            evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            disable_stream_instance(instance, disabled)
            failed |= not results
            if isinstance(opt_result, PredicateResult) and not any(opt_result.value == r.value for r in results):
                failed = True # TODO: check for instance?
            new_results += results
        for instance in opt_instances:
            results = instance.next_optimistic()
            opt_evaluations.update(evaluation_from_fact(f) for r in results for f in r.get_certified())
            opt_results += results
            failed |= not results
            new_results += results
        for result in new_results:
            if isinstance(result, StreamResult): # Could not add if same value
                for opt, obj in zip(opt_result.output_objects, result.output_objects):
                    opt_bindings[opt].append(obj)
    if verbose:
        print('Success: {}'.format(not failed))
    if failed:
        return None
    return opt_results

##################################################

def solve_focused(problem, max_time=INF, max_cost=INF, stream_info={},
                  commit=True, effort_weight=None, eager_layers=1,
                  visualize=False, verbose=True, **search_kwargs):
    """
    Solves a PDDLStream problem by first hypothesizing stream outputs and then determining whether they exist
    :param problem: a PDDLStream problem
    :param max_time: the maximum amount of time to apply streams
    :param max_cost: a strict upper bound on plan cost
    :param stream_info: a dictionary from stream name to StreamInfo altering how individual streams are handled
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
    evaluations, goal_expression, domain, externals = parse_problem(problem)
    update_info(externals, stream_info)
    eager_externals = filter(lambda e: e.info.eager, externals)
    constraint_solver = ConstraintSolver(problem[3])
    disabled = []
    if visualize:
        clear_visualizations()
    #functions = filter(lambda s: isinstance(s, Function), externals)
    functions = filter(lambda s: type(s) is Function, externals)
    negative = filter(lambda s: type(s) is Predicate and s.is_negative(), externals)
    streams = filter(lambda s: s not in (functions + negative), externals)
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
            eagerly_evaluate(evaluations, eager_externals, eager_layers, max_time - elapsed_time(start_time), verbose)
            stream_results += populate_results(evaluations_from_stream_plan(evaluations, stream_results),
                                               functions, max_time-elapsed_time(start_time))
            # TODO: warning check if using simultaneous_stream_plan or relaxed_stream_plan with non-eager functions
            solve_stream_plan = relaxed_stream_plan if effort_weight is None else simultaneous_stream_plan
            #solve_stream_plan = sequential_stream_plan if effort_weight is None else simultaneous_stream_plan
            stream_plan, action_plan, cost = solve_stream_plan(evaluations, goal_expression, domain, stream_results,
                                                               negative, max_cost=best_cost, **search_kwargs)
            print(compute_expected_cost(stream_plan))
            print(topological_sort2(stream_plan))
            print(topological_sort(stream_plan))
            #stream_plan = topological_sort(stream_plan)
            dynamic_programming(stream_plan)
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