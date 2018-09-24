from collections import Counter, deque

from pddlstream.language.constants import get_prefix, get_args, get_parameter_name
from pddlstream.language.conversion import substitute_expression, list_from_conjunction, evaluation_from_fact
from pddlstream.language.external import parse_lisp_list, get_procedure_fn
from pddlstream.language.stream import OptValue, StreamInfo, Stream, StreamInstance, PartialInputs, DEFAULT_UNIQUE
from pddlstream.language.generator import empty_gen
from pddlstream.utils import get_mapping, INF, neighbors_from_orders
from pddlstream.utils import elapsed_time, INF, get_mapping, find_unique, HeapElement
from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.algorithms.downward import fd_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.language.synthesizer import get_cluster_values


# TODO: augment state with the set of constraints required on the path
# TODO: create additional variables in the event that the search fails
# TODO: make more samples corresponding to the most number used in a failed cluster
# TODO: could also just block a skeleton itself by adding it as a state variable
# TODO: fluents for what is not active in the state and add those to consider subsets

class Optimizer(object):
    def __init__(self, name, procedure, info):
        self.name = name
        self.procedure = procedure
        self.info = info
        self.variables = []
        self.constraints = []
        self.objectives = []
    def get_streams(self):
        return self.variables + self.constraints
    def __repr__(self):
        return '{}'.format(self.name) #, self.streams)

class VariableStream(Stream):
    def __init__(self, optimizer, variable, inputs, domain, certified):
        self.optimizer = optimizer
        self.variable = variable
        outputs = [variable]
        name = '{}-{}'.format(optimizer.name, get_parameter_name(variable))
        gen_fn = get_gen_fn(optimizer.procedure, inputs, outputs, certified)
        #gen_fn = empty_gen()
        # info = StreamInfo(effort_fn=get_effort_fn(optimizer_name, inputs, outputs))
        #info = StreamInfo(opt_gen_fn=PartialInputs(unique=DEFAULT_UNIQUE, num=DEFAULT_NUM))
        info = StreamInfo()
        super(VariableStream, self).__init__(name, gen_fn, inputs, domain, outputs, certified, info)

class ConstraintStream(Stream):
    def __init__(self, optimizer, constraint, domain):
        self.optimizer = optimizer
        self.constraint = constraint
        inputs = get_args(constraint)
        outputs = []
        certified = [constraint]
        name = '{}-{}'.format(optimizer.name, get_prefix(constraint))
        gen_fn = get_gen_fn(optimizer.procedure, inputs, outputs, certified)
        #gen_fn = empty_gen()
        info = StreamInfo(effort_fn=get_effort_fn(optimizer.name))
        super(ConstraintStream, self).__init__(name, gen_fn, inputs, domain, outputs, certified, info)

##################################################

UNSATISFIABLE = 'unsatisfiable-negative'

class OptimizerInstance(StreamInstance):
    def __init__(self, stream, input_objects, fluent_facts):
        assert not fluent_facts
        super(OptimizerInstance, self).__init__(stream, input_objects, fluent_facts)
    def get_blocked_fact(self):
        if self.external.is_fluent():
            assert self.axiom_predicate is not None
            return (self.axiom_predicate,) + self.input_objects
        return (self.external.blocked_predicate,) + self.input_objects
    def disable(self, evaluations, domain):
        #assert not self.disabled
        # Prevent further progress via any action, prevent achieved goal
        super(StreamInstance, self).disable(evaluations, domain)
        # TODO: re-enable?
        # TODO: block stream-specific predicates relating to these
        # TODO: block equivalent combinations of things
        # TODO: don't block variables? Might have a variable that is a function of another though...
        # TODO: might need to block separate clusters at once in order to ensure that it captures the true behavior
        #index = len(self.external.disabled_instances)
        #self.external.disabled_instances.append(self)
        #self.axiom_predicate = '_ax{}-{}'.format(self.external.blocked_predicate, index)
        mapping = get_mapping(self.external.outputs, self.external.output_objects)
        mapping.update(self.get_mapping())
        constraints = substitute_expression(self.external.certified, mapping)
        instance_counts = Counter(r.instance for r in self.external.stream_plan
                                  if isinstance(r.external, VariableStream))
        for instance, num in instance_counts.items():
            instance.num_optimistic = max(instance.num_optimistic, num + 1)
        # TODO: don't do this until the full plan has failed
        import pddl
        parameters = tuple()
        precondition = pddl.Conjunction(list(map(fd_from_fact, constraints)))
        domain.axioms.append(pddl.Axiom(name=UNSATISFIABLE, parameters=parameters,
                                        num_external_parameters=len(parameters),
                                        condition=precondition))

class OptimizerStream(Stream):
    _Instance = OptimizerInstance
    def __init__(self, optimizer, stream_plan):
        self.optimizer = optimizer
        self.stream_plan = stream_plan
        inputs, domain, outputs, certified, functions, _, \
            self.input_objects, self.output_objects = get_cluster_values(stream_plan)
        gen_fn = get_gen_fn(optimizer.procedure, inputs, outputs, certified | functions)
        super(OptimizerStream, self).__init__(optimizer.name, gen_fn, inputs, domain, outputs, certified, optimizer.info)

##################################################

def get_gen_fn(procedure, inputs, outputs, certified):
    def gen_fn(*input_values):
        mapping = get_mapping(inputs, input_values)
        targets = substitute_expression(certified, mapping)
        return procedure(outputs, targets)
    return gen_fn

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

def parse_variable(optimizer, lisp_list):
    value_from_attribute = parse_lisp_list(lisp_list)
    return VariableStream(optimizer,
                          value_from_attribute[':variable'], # TODO: assume unique?
                          value_from_attribute.get(':inputs', []),
                          list_from_conjunction(value_from_attribute.get(':domain')),
                          list_from_conjunction(value_from_attribute.get(':graph')))

def parse_constraint(optimizer, lisp_list):
    value_from_attribute = parse_lisp_list(lisp_list)
    return ConstraintStream(optimizer,
                            value_from_attribute[':constraint'],
                            list_from_conjunction(value_from_attribute[':necessary']))

# TODO: convert optimizer into a set of streams? Already present within test stream

def parse_optimizer(lisp_list, stream_map, stream_info):
    _, optimizer_name = lisp_list[:2]
    procedure = get_procedure_fn(stream_map, optimizer_name)
    info = stream_info.get(optimizer_name, StreamInfo())
    optimizer = Optimizer(optimizer_name, procedure, info)
    for sub_list in lisp_list[2:]:
        form = sub_list[0]
        if form == ':variable':
            optimizer.variables.append(parse_variable(optimizer, sub_list))
        elif form == ':constraint':
            optimizer.constraints.append(parse_constraint(optimizer, sub_list))
        elif form == ':objective':
            optimizer.objectives.append(sub_list[1])
        else:
            raise ValueError(form)
    return optimizer.get_streams()

##################################################

def get_optimizer(result):
    external = result.instance.external
    if isinstance(external, VariableStream) or isinstance(external, ConstraintStream):
        return external.optimizer
    return None

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
        instance = stream.get_instance(stream.input_objects)
        optimizer_plan.append(stream._Result(instance, stream.output_objects))
    return optimizer_plan

# TODO: this process needs to be improved still

def combine_optimizers_greedy(evaluations, external_plan):
    if external_plan is None:
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

def combine_optimizers(evaluations, external_plan):
    if external_plan is None:
        return external_plan
    optimizers = {get_optimizer(r) for r in external_plan} # None is like a unique optimizer
    if not optimizers:
        return external_plan
    function_plan = list(filter(lambda r: isinstance(r, FunctionResult), external_plan))
    stream_plan = list(filter(lambda r: r not in function_plan, external_plan))

    combined_results = []
    for optimizer in optimizers:
        relevant_results = [r for r in stream_plan if get_optimizer(r) == optimizer]
        combined_results.extend(combine_optimizer_plan(relevant_results, function_plan))
    combined_results.extend(function_plan)

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
