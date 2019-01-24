import copy
from collections import Counter, deque

from pddlstream.algorithms.downward import make_axiom
from pddlstream.algorithms.common import INIT_EVALUATION
from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.algorithms.scheduling.utils import partition_external_plan
from pddlstream.language.constants import get_prefix, get_args, get_parameter_name, Fact, concatenate, is_plan
from pddlstream.language.conversion import substitute_expression, list_from_conjunction, evaluation_from_fact
from pddlstream.language.external import parse_lisp_list, get_procedure_fn
from pddlstream.language.function import FunctionResult
from pddlstream.language.object import OptimisticObject, Object
from pddlstream.language.stream import OptValue, StreamInfo, Stream, StreamInstance, StreamResult, \
    PartialInputs, NEGATIVE_SUFFIX
from pddlstream.language.synthesizer import get_cluster_values
from pddlstream.utils import neighbors_from_orders, INF, get_mapping, safe_zip, str_from_object

# TODO: could also just block a skeleton itself by adding it as a state variable

DEFAULT_SIMULTANEOUS = False
DEFAULT_UNIQUE = True # TODO: would it ever even make sense to do shared here?
OPTIMIZER_AXIOM = False
BLOCK_ADDITIONS = False

class OptimizerOutput(object):
    def __init__(self, assignments=[], facts=[], infeasible=[]): # infeasible=None
        self.assignments = assignments
        self.facts = facts
        self.infeasible = infeasible
    def __repr__(self):
        #return '{}{}'.format(self.__class__.__name__, str_from_object(self.__dict__))
        return str_from_object(self.__dict__)

class Optimizer(object):
    def __init__(self, name, procedure, info):
        self.name = name
        self.procedure = procedure
        self.info = info
        self.variables = []
        self.constraints = []
        self.objectives = []
        self.streams = []
    def get_streams(self):
        return self.variables + self.constraints
    def __repr__(self):
        return '{}'.format(self.name) #, self.streams)

##################################################

def get_list_gen_fn(stream, procedure, inputs, outputs, certified, fluents={}, hint={}):
    def list_gen_fn(*input_values):
        mapping = get_mapping(inputs, input_values)
        targets = substitute_expression(certified, mapping)
        for output in procedure(outputs, targets, hint=hint):
            if isinstance(output, OptimizerOutput):
                # TODO: would be better to just extend StreamInstance
                stream.infeasible.extend(output.infeasible)
                yield output.assignments
            else:
                yield output
    return list_gen_fn

def get_effort_fn(optimizer_name):
    # TODO: higher effort is the variable cannot be free for the testing process
    # This might happen if the variable is certified to have a property after construction
    def effort_fn(*input_values):
        free_indices = [i for i, value in enumerate(input_values) if isinstance(value, OptValue)
                        and value.stream.startswith(optimizer_name)]
        if not free_indices:
            return INF
        return 1
    return effort_fn

##################################################

class OptimizerInfo(StreamInfo):
    def __init__(self, planable=False, p_success=None, overhead=None):
        super(OptimizerInfo, self).__init__(p_success=p_success, overhead=overhead)
        self.planable = planable # TODO: this isn't currently used
        # TODO: post-processing

class VariableStream(Stream):
    def __init__(self, optimizer, variable, inputs, domain, certified, info):
        self.optimizer = optimizer
        self.variable = variable
        self.infeasible = []
        outputs = [variable]
        name = '{}-{}'.format(optimizer.name, get_parameter_name(variable))
        gen_fn = get_list_gen_fn(self, optimizer.procedure, inputs, outputs, certified)
        #gen_fn = empty_gen()
        #info = StreamInfo(effort_fn=get_effort_fn(optimizer_name, inputs, outputs))
        #info = StreamInfo(opt_gen_fn=PartialInputs(unique=DEFAULT_UNIQUE, num=DEFAULT_NUM))
        # Each stream could certify a stream-specific fact as well
        # TODO: will I need to adjust simultaneous here as well?
        info = StreamInfo(opt_gen_fn=PartialInputs(unique=DEFAULT_UNIQUE), simultaneous=DEFAULT_SIMULTANEOUS)
        self.stream_fact = Fact('_{}'.format(name), concatenate(inputs, outputs)) # TODO: just add to certified?
        super(VariableStream, self).__init__(name, gen_fn, inputs, domain,
                                             outputs, certified, info)

class ConstraintStream(Stream):
    def __init__(self, optimizer, constraint, domain, fluents):
        self.optimizer = optimizer
        self.constraint = constraint
        self.infeasible = []
        inputs = get_args(constraint)
        outputs = []
        certified = [constraint]
        name = '{}-{}'.format(optimizer.name, get_prefix(constraint))
        gen_fn = get_list_gen_fn(self, optimizer.procedure, inputs, outputs, certified)
        #gen_fn = empty_gen()
        info = StreamInfo(effort_fn=get_effort_fn(optimizer.name), simultaneous=DEFAULT_SIMULTANEOUS)
        self.stream_fact = Fact('_{}'.format(name), concatenate(inputs, outputs))
        super(ConstraintStream, self).__init__(name, gen_fn, inputs, domain,
                                               outputs, certified, info, fluents=fluents)

OPTIMIZER_STREAMS = [VariableStream, ConstraintStream]

##################################################

UNSATISFIABLE = 'unsatisfiable{}'.format(NEGATIVE_SUFFIX)

class OptimizerInstance(StreamInstance):
    def __init__(self, stream, input_objects, fluent_facts):
        super(OptimizerInstance, self).__init__(stream, input_objects, fluent_facts)
        #self.disabled_axiom = None
    def disable(self, evaluations, domain):
        #assert not self.disabled
        super(StreamInstance, self).disable(evaluations, domain) # StreamInstance instead of OptimizerInstance
        # TODO: re-enable?
        # TODO: might need to block separate clusters at once in order to ensure that it captures the true behavior
        #index = len(self.external.disabled_instances)
        #self.external.disabled_instances.append(self)
        instance_counts = Counter(r.instance for r in self.external.stream_plan
                                  if isinstance(r.external, VariableStream))
        for instance, num in instance_counts.items(): # TODO: wait until the full plan has failed
            instance.num_optimistic = max(instance.num_optimistic, num + 1)
        if OPTIMIZER_AXIOM:
            self._add_disabled_axiom(domain)
    def get_constraints(self):
        output_mapping = get_mapping(self.external.outputs, self.external.output_objects)
        output_mapping.update(self.get_mapping())
        #constraints = substitute_expression(self.external.certified, output_mapping)
        constraints = []
        for i, result in enumerate(self.external.stream_plan):
            macro_fact = substitute_expression(result.external.stream_fact, self.external.macro_from_micro[i])
            constraints.append(substitute_expression(macro_fact, output_mapping))
        #print(self.external.infeasible)
        #print(self.external.certified)
        # TODO: I think I should be able to just disable the fluent fact from being used in that context
        return constraints
    def get_free_objects(self):
       constraints = self.get_constraints()
       objects = set()
       for fact in constraints:
           objects.update(get_args(fact))
       return list(filter(lambda o: isinstance(o, OptimisticObject), objects))
    def _add_disabled_axiom(self, domain):
        # TODO: be careful about the shared objects as parameters
        #free_objects = self.get_free_objects()
        free_objects = self.external.output_objects
        parameters = ['?p{}'.format(i) for i in range(len(free_objects))]
        preconditions = substitute_expression(self.get_constraints(), get_mapping(free_objects, parameters))
        self._disabled_axiom = make_axiom(parameters, preconditions, (UNSATISFIABLE,))
        domain.axioms.append(self._disabled_axiom)
    def enable(self, evaluations, domain):
        super(OptimizerInstance, self).enable(evaluations, domain)
        raise NotImplementedError()

class OptimizerStream(Stream):
    _Instance = OptimizerInstance
    def __init__(self, optimizer, external_plan):
        optimizer.streams.append(self)
        self.optimizer = optimizer
        self.infeasible = []
        self.stream_plan, self.function_plan = partition_external_plan(external_plan)
        inputs, domain, outputs, certified, functions, self.macro_from_micro, \
            self.input_objects, self.output_objects, self.fluent_facts = get_cluster_values(external_plan)

        self.hint = {}
        for result, mapping in safe_zip(self.stream_plan, self.macro_from_micro):
            if isinstance(result, StreamResult):
                for param, obj in safe_zip(result.external.outputs, result.output_objects):
                    if isinstance(obj, Object):
                        self.hint[mapping[param]] = obj.value
        target_facts = sorted(certified | functions)
        gen_fn = get_list_gen_fn(self, optimizer.procedure, inputs, outputs,
                                 target_facts, hint=self.hint)
        super(OptimizerStream, self).__init__(optimizer.name, gen_fn, inputs, domain, outputs,
                                              certified, optimizer.info)
    @property
    def instance(self):
        return self.get_instance(self.input_objects, fluent_facts=self.fluent_facts)


##################################################

def parse_variable(optimizer, lisp_list, info):
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {':variable', ':inputs', ':domain', ':graph'}
    return VariableStream(optimizer,
                          value_from_attribute[':variable'], # TODO: assume unique?
                          value_from_attribute.get(':inputs', []),
                          list_from_conjunction(value_from_attribute.get(':domain')),
                          list_from_conjunction(value_from_attribute.get(':graph')), info)

def parse_constraint(optimizer, lisp_list):
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {':constraint', ':necessary', ':fluents'}
    return ConstraintStream(optimizer,
                            value_from_attribute[':constraint'],
                            list_from_conjunction(value_from_attribute[':necessary']),
                            value_from_attribute.get(':fluents', []))

# TODO: convert optimizer into a set of streams? Already present within test stream

def parse_optimizer(lisp_list, procedures, infos):
    _, optimizer_name = lisp_list[:2]
    procedure = get_procedure_fn(procedures, optimizer_name)
    info = infos.get(optimizer_name, OptimizerInfo())
    optimizer = Optimizer(optimizer_name, procedure, info)
    for sub_list in lisp_list[2:]:
        form = sub_list[0]
        if form == ':variable':
            optimizer.variables.append(parse_variable(optimizer, sub_list, info))
        elif form == ':constraint':
            optimizer.constraints.append(parse_constraint(optimizer, sub_list))
        elif form == ':objective':
            optimizer.objectives.append(sub_list[1])
        else:
            raise ValueError(form)
    return optimizer.get_streams()

##################################################

def is_optimizer_result(result):
    return type(result.external) in OPTIMIZER_STREAMS

def get_optimizer(result):
    return result.external.optimizer if is_optimizer_result(result) else None

def get_connected_components(vertices, edges):
    #return [vertices]
    incoming, outgoing = neighbors_from_orders(edges)
    clusters = []
    processed = set()
    for v0 in vertices:
        if v0 in processed:
            continue
        processed.add(v0)
        cluster = {v0}
        queue = deque([v0])
        while queue:
            v1 = queue.popleft()
            for v2 in (incoming[v1] | outgoing[v1]):
                if v2 not in processed:
                    processed.add(v2)
                    cluster.add(v2)
                    queue.append(v2)
        clusters.append([v for v in vertices if v in cluster])
    return clusters

def combine_optimizer_plan(stream_plan, functions):
    if not stream_plan:
        return stream_plan
    optimizer = get_optimizer(stream_plan[-1])
    if optimizer is None:
        return stream_plan
    function_plan = list(filter(lambda r: get_prefix(r.instance.external.head)
                                          in optimizer.objectives, functions))
    external_plan = stream_plan + function_plan
    optimizer_plan = []
    for cluster_plan in get_connected_components(external_plan, get_partial_orders(external_plan)):
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

##################################################

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

##################################################

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
