import time
from itertools import product
from collections import defaultdict, OrderedDict, deque

from pddlstream.algorithm import parse_problem, get_optimistic_constraints
from pddlstream.context import ConstraintSolver
from pddlstream.conversion import revert_solution, evaluation_from_fact, substitute_expression, Minimize, Equal
from pddlstream.function import Function, Predicate
from pddlstream.incremental import process_stream_queue
from pddlstream.instantiation import Instantiator
from pddlstream.object import Object
from pddlstream.reorder import reorder_stream_plan, get_combined_orders, \
    separate_plan, reorder_combined_plan, neighbors_from_orders, get_partial_orders
from pddlstream.scheduling.relaxed import relaxed_stream_plan
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan
from pddlstream.stream import StreamResult, Stream
from pddlstream.utils import INF, elapsed_time
from pddlstream.visualization import clear_visualizations, create_visualizations
from pddlstream.scheduling.simultaneous import evaluations_from_stream_plan
from pddlstream.utils import INF
from pddlstream.function import PredicateResult, FunctionResult

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
            self.p_success, self.overhead = 1e-3, 1
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

def populate_results(evaluations, streams, max_time):
    #start_time = time.time()
    instantiator = Instantiator(evaluations, streams)
    stream_results = []
    while instantiator.stream_queue: # and (elapsed_time(start_time) < max_time):
        stream_results += optimistic_process_stream_queue(instantiator)
    return stream_results

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

def method(stream_plan, dynamic_streams):
    if stream_plan is None:
        return None
    orders = get_partial_orders(stream_plan)
    for order in list(orders):
        orders.add(order[::-1])
    neighbors, _ = neighbors_from_orders(orders)
    # TODO: what if many possibilities?

    processed = set()
    new_stream_plan = []
    for v in stream_plan: # Processing in order is important
        if v in processed:
            continue
        processed.add(v)
        # TODO: assert that it has at least one thing in it

        for dynamic in dynamic_streams:
            # TODO: something could be an input and output of a cut...
            if v.instance.external.name not in dynamic.streams:
                continue
            # TODO: need to ensure all are covered I think?
            # TODO: don't do if no streams within

            results = [v]
            queue = deque([v])
            while queue:
                v1 = queue.popleft()
                for v2 in neighbors[v1]:
                    if (v2 not in processed) and (v2.instance.external.name in dynamic.streams):
                        results.append(v2)
                        queue.append(v2)
                        processed.add(v2)

            print(results)


            param_from_obj = {}
            inputs = []
            domain = set()
            outputs = []
            certified = set()
            input_objects = []
            output_objects = []
            functions = set()
            # TODO: always have the option of not making a mega stream
            for result in results: # TODO: branch on sequences here
                # TODO: obtain cluster, top sort, then process
                # TODO: could just do no instance inputs (no need for an instance...)
                # TODO: could return map or tuple
                local_mapping = {}
                stream = result.instance.external
                for inp, input_object in zip(stream.inputs, result.instance.input_objects):
                    # TODO: make fixed objects constant
                    #if isinstance()
                    if input_object not in param_from_obj:
                        param_from_obj[input_object] = '?i{}'.format(len(inputs))
                        inputs.append(param_from_obj[input_object])
                        input_objects.append(input_object)
                    local_mapping[inp] = param_from_obj[input_object]
                print(local_mapping)

                if isinstance(result, PredicateResult):
                    #functions.append(Equal(stream.head, result.value))
                    mapping = {inp: param_from_obj[inp] for inp in result.instance.input_objects}
                    functions.update(substitute_expression(result.get_certified(), mapping))
                elif isinstance(result, PredicateResult):
                    functions.add(substitute_expression(Minimize(stream.head), local_mapping))
                else:
                    #for inp in stream.inputs: # TODO: only do optimistic parameters?
                    #    #if inp not in local_mapping:
                    #    #    # TODO: this isn't right
                    #    local_mapping[inp] = '?i{}'.format(len(inputs))
                    #    inputs.append(local_mapping[inp])

                    domain.update(substitute_expression(stream.domain, local_mapping))
                    for out, output_object in zip(stream.outputs, result.output_objects):
                        if output_object not in param_from_obj:
                            param_from_obj[output_object] = '?o{}'.format(len(outputs))
                            outputs.append(param_from_obj[output_object])
                            output_objects.append(output_object)

                        local_mapping[out] = param_from_obj[output_object]

                    # TODO: what if the input parameters for one are the outputs of another in the cluster
                    #for out in stream.outputs:
                    #    #if out not in local_mapping:
                    #    local_mapping[out] = '?o{}'.format(len(outputs))
                    #    outputs.append(local_mapping[out])
                    certified.update(substitute_expression(stream.certified, local_mapping))

            def gen_fn(*input_values):
                print(input_values)
                mapping = dict(zip(inputs, input_values))
                targets = substitute_expression(certified | functions, mapping)
                return dynamic.gen_fn(outputs, targets)

            mega_stream = Stream(dynamic.name, gen_fn,
                   inputs=tuple(inputs), domain=domain,
                   outputs=tuple(outputs), certified=certified)
            mega_instance = mega_stream.get_instance(input_objects)
            mega_result = StreamResult(mega_instance, output_objects)
            new_stream_plan.append(mega_result)
            new_stream_plan += filter(lambda s: isinstance(s, FunctionResult), results)

            print(mega_stream)
            print(mega_result)
            raw_input('awefawef')

            break
        else:
            new_stream_plan.append(v)

    print(new_stream_plan)


    return new_stream_plan

def solve_focused(problem, stream_info={}, action_info={}, dynamic_streams=[],
                  max_time=INF, max_cost=INF,
                  commit=True, effort_weight=None, eager_layers=1,
                  visualize=False, verbose=True, **search_kwargs):
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
    evaluations, goal_expression, domain, externals = parse_problem(problem)
    action_info = get_action_info(action_info)
    update_stream_info(externals, stream_info)
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
            combined_plan, cost = solve_stream_plan(evaluations, goal_expression, domain, stream_results,
                                                               negative, max_cost=best_cost, **search_kwargs)
            combined_plan = reorder_combined_plan(evaluations, combined_plan, action_info, domain)
            print('Combined plan: {}'.format(combined_plan))
            stream_plan, action_plan = separate_plan(combined_plan, action_info)
            #stream_plan = reorder_streams(stream_plan)
            print('Stream plan: {}\n'
                  'Action plan: {}'.format(stream_plan, action_plan))
            stream_plan = method(stream_plan, dynamic_streams)

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
    # TODO: modify streams here
    return revert_solution(best_plan, best_cost, evaluations)