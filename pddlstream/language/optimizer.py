from collections import defaultdict

from pddlstream.algorithms.scheduling.utils import partition_external_plan
from pddlstream.language.constants import get_prefix, get_args, get_parameter_name, is_parameter, Minimize
from pddlstream.language.conversion import substitute_expression, list_from_conjunction
from pddlstream.language.external import parse_lisp_list, get_procedure_fn
from pddlstream.language.function import PredicateResult, FunctionResult
from pddlstream.language.object import Object, SharedOptValue
from pddlstream.language.stream import StreamInfo, Stream, StreamInstance, StreamResult, \
    PartialInputs, NEGATIVE_SUFFIX, WildOutput
from pddlstream.language.generator import get_next
from pddlstream.utils import INF, get_mapping, safe_zip, str_from_object
from pddlstream.algorithms.reorder import get_stream_plan_components, get_partial_orders

DEFAULT_SIMULTANEOUS = False
DEFAULT_UNIQUE = True # TODO: would it ever even make sense to do shared here?
# TODO: revert to my previous specification where streams can simply be fused

class OptimizerOutput(object):
    def __init__(self, assignments=[], facts=[], infeasible=[]): # infeasible=None
        self.assignments = list(assignments)
        self.facts = list(facts)
        self.infeasible = list(map(frozenset, infeasible))
    def to_wild(self):
        return WildOutput(self.assignments, self.facts)
    def __bool__(self):
        return bool(self.assignments)
    __nonzero__ = __bool__
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

class ComponentStream(Stream):
    def __init__(self, optimizer, *args):
        self.optimizer = optimizer
        super(ComponentStream, self).__init__(*args)

##################################################

def get_list_gen_fn(procedure, inputs, outputs, certified, hint={}):
    # TODO: prevent outputs of the sampler from being used as inputs (only consider initial values)
    def list_gen_fn(*input_values):
        mapping = get_mapping(inputs, input_values)
        targets = substitute_expression(certified, mapping)
        return procedure(outputs, targets, hint=hint)
    return list_gen_fn

def get_effort_fn(optimizer_name):
    # TODO: higher effort is the variable cannot be free for the testing process
    # This might happen if the variable is certified to have a property after construction
    def effort_fn(*input_values):
        parameter_indices = [i for i, value in enumerate(input_values) if is_parameter(value)]
        optimizer_indices = [i for i, value in enumerate(input_values) if isinstance(value, SharedOptValue)
                              if input_values[i].stream.startswith(optimizer_name)]
        #if not parameter_indices and not optimizer_indices:
        #    return INF
        return 1
    return effort_fn

def prune_dominated(collections):
    for i, collection1 in enumerate(collections):
        if all((i == j) or not (collection2 <= collection1)
               for j, collection2 in enumerate(collections)):
            yield collection1

##################################################

class OptimizerInfo(StreamInfo):
    def __init__(self, planable=False, p_success=None, overhead=None):
        super(OptimizerInfo, self).__init__(p_success=p_success, overhead=overhead)
        self.planable = planable # TODO: this isn't currently used
        # TODO: post-processing

class VariableStream(ComponentStream):
    # TODO: allow generation of two variables
    def __init__(self, optimizer, variables, inputs, domain, certified, infos):
        name = '{}-{}'.format(optimizer.name, '-'.join(map(get_parameter_name, variables)))
        gen_fn = get_list_gen_fn(optimizer.procedure, inputs, variables, certified)
        # TODO: need to convert OptimizerOutput
        #gen_fn = empty_gen()
        #info = StreamInfo(effort=get_effort_fn(optimizer_name, inputs, outputs))
        #info = StreamInfo(opt_gen_fn=PartialInputs(unique=DEFAULT_UNIQUE, num=DEFAULT_NUM))
        info = infos.get(name, None)
        if info is None:
            info = StreamInfo(opt_gen_fn=PartialInputs(unique=DEFAULT_UNIQUE),
                              simultaneous=DEFAULT_SIMULTANEOUS)
        super(VariableStream, self).__init__(optimizer, name, gen_fn, inputs, domain,
                                             variables, certified, info)

class ConstraintStream(ComponentStream):
    def __init__(self, optimizer, constraint, domain, infos):
        # TODO: could support fluents and compile them into conditional effects
        inputs = get_args(constraint)
        outputs = []
        certified = [constraint]
        name = '{}-{}'.format(optimizer.name, get_prefix(constraint))
        gen_fn = get_list_gen_fn(optimizer.procedure, inputs, outputs, certified)
        #gen_fn = empty_gen()
        info = infos.get(name, None)
        if info is None:
            info = StreamInfo(effort=get_effort_fn(optimizer.name),
                              simultaneous=DEFAULT_SIMULTANEOUS)
        super(ConstraintStream, self).__init__(optimizer, name, gen_fn, inputs, domain,
                                               outputs, certified, info)

##################################################

VARIABLES = ':variables'
CONSTRAINT = ':constraint'

def parse_variable(optimizer, lisp_list, infos):
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {VARIABLES, ':inputs', ':domain', ':graph'}
    return VariableStream(optimizer,
                          value_from_attribute[VARIABLES], # TODO: assume unique?
                          value_from_attribute.get(':inputs', []),
                          list_from_conjunction(value_from_attribute.get(':domain')),
                          list_from_conjunction(value_from_attribute.get(':graph')),
                          infos)

def parse_constraint(optimizer, lisp_list, infos):
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {CONSTRAINT, ':necessary'} # , ':fluents'}
    return ConstraintStream(optimizer,
                            value_from_attribute[CONSTRAINT],
                            list_from_conjunction(value_from_attribute[':necessary']),
                            infos)

# TODO: convert optimizer into a set of streams? Already present within test stream

def parse_optimizer(lisp_list, procedures, infos):
    _, optimizer_name = lisp_list[:2]
    procedure = get_procedure_fn(procedures, optimizer_name)
    optimizer_info = infos.get(optimizer_name, OptimizerInfo())
    optimizer = Optimizer(optimizer_name, procedure, optimizer_info)
    for sub_list in lisp_list[2:]:
        form = sub_list[0]
        if form == VARIABLES:
            optimizer.variables.append(parse_variable(optimizer, sub_list, infos))
        elif form == CONSTRAINT:
            optimizer.constraints.append(parse_constraint(optimizer, sub_list, infos))
        elif form == ':objective':
            optimizer.objectives.append(sub_list[1])
        else:
            raise ValueError(form)
    return optimizer.get_streams()

##################################################

UNSATISFIABLE = 'unsatisfiable{}'.format(NEGATIVE_SUFFIX)

class OptimizerResult(StreamResult):
    def get_components(self):
        return self.external.stream_plan
    def get_objectives(self):
        return substitute_expression(self.external.objectives, self.get_mapping())
    def get_unsatisfiable(self):
        return self.instance.get_unsatisfiable()

class OptimizerInstance(StreamInstance):
    _Result = OptimizerResult
    def __init__(self, stream, input_objects, fluent_facts):
        super(OptimizerInstance, self).__init__(stream, input_objects, fluent_facts)
        all_constraints = frozenset(range(len(self.external.certified)))
        self.infeasible = {all_constraints}
        # TODO: might need to block separate clusters at once in order to ensure that it captures the true behavior
        # TODO: connected components on facts
        # TODO: cluster connected components in the infeasible set
        # TODO: compute things dependent on a stream and treat like an optimizer
        # Also make an option to just treat everything like an optimizer
    def _next_outputs(self):
        self._create_generator()
        output, self.enumerated = get_next(self._generator, default=[])
        if not isinstance(output, OptimizerOutput):
            output = OptimizerOutput(assignments=output)
        self.infeasible.update(output.infeasible)
        # TODO: instead replace each time
        return output.to_wild()
    def get_unsatisfiable(self):
        constraints = substitute_expression(self.external.certified, self.external.mapping)
        index_from_constraint = {c: i for i, c in enumerate(constraints)}
        # TODO: compute connected components
        result_from_index = defaultdict(set)
        for result in self.external.stream_plan:
            for fact in result.get_certified():
                if fact in index_from_constraint:
                    result_from_index[index_from_constraint[fact]].add(result)
        # TODO: add implied results
        #orders = get_partial_orders(self.external.stream_plan)
        return [{result for index in cluster for result in result_from_index[index]}
                for cluster in prune_dominated(self.infeasible)]

class OptimizerStream(Stream):
    _Instance = OptimizerInstance
    def __init__(self, optimizer, external_plan):
        optimizer.streams.append(self)
        self.optimizer = optimizer
        self.stream_plan, self.function_plan = partition_external_plan(external_plan)
        inputs, domain, outputs, certified, functions, self.macro_from_micro, \
            self.input_objects, self.output_objects, self.fluent_facts = get_cluster_values(external_plan)

        hint = {}
        for result, mapping in safe_zip(self.stream_plan, self.macro_from_micro):
            if isinstance(result, StreamResult):
                for param, obj in safe_zip(result.external.outputs, result.output_objects):
                    if isinstance(obj, Object):
                        hint[mapping[param]] = obj.value
        self.objectives = certified + functions
        gen_fn = get_list_gen_fn(optimizer.procedure, inputs, outputs, self.objectives, hint=hint)
        #assert len(self.get_cluster_plans()) == 1
        super(OptimizerStream, self).__init__(optimizer.name, gen_fn, inputs, domain, outputs,
                                              certified, optimizer.info)
    @property
    def mapping(self):
        return get_mapping(self.inputs + self.outputs,
                           self.input_objects + self.output_objects)
    def get_cluster_plans(self):
        # TODO: split the optimizer into clusters when provably independent
        return get_stream_plan_components(self.stream_plan + self.function_plan)
    @property
    def instance(self):
        return self.get_instance(self.input_objects, fluent_facts=self.fluent_facts)

##################################################

def add_result_inputs(result, param_from_obj, local_mapping, inputs, input_objects):
    for param, obj in zip(result.instance.external.inputs, result.instance.input_objects):
        # TODO: only do optimistic parameters?
        if obj not in param_from_obj:
            param_from_obj[obj] = '?i{}'.format(len(inputs)) # '?_i{}'
            inputs.append(param_from_obj[obj])
            input_objects.append(obj)
        local_mapping[param] = param_from_obj[obj]


def add_result_outputs(result, param_from_obj, local_mapping, outputs, output_objects):
    for param, obj in zip(result.instance.external.outputs, result.output_objects):
        if obj not in param_from_obj:
            param_from_obj[obj] = '?o{}'.format(len(outputs))
            outputs.append(param_from_obj[obj])
            output_objects.append(obj)
        local_mapping[param] = param_from_obj[obj]

def get_cluster_values(stream_plan):
    param_from_obj = {}
    macro_from_micro = []
    inputs, domain, outputs, certified, functions = [], set(), [], set(), set()
    input_objects, output_objects = [], []
    fluent_facts = []
    for result in stream_plan:
        local_mapping = {}  # global_from_local
        stream = result.instance.external
        add_result_inputs(result, param_from_obj, local_mapping, inputs, input_objects)
        domain.update(set(substitute_expression(stream.domain, local_mapping)) - certified)
        if isinstance(result, PredicateResult):
            # functions.append(Equal(stream.head, result.value))
            # TODO: do I need the new mapping here?
            mapping = {inp: param_from_obj[inp] for inp in result.instance.input_objects}
            functions.update(substitute_expression(result.get_certified(), mapping))
        elif isinstance(result, FunctionResult):
            functions.add(substitute_expression(Minimize(stream.head), local_mapping))
        else:
            fluent_facts.extend(result.instance.fluent_facts)
            add_result_outputs(result, param_from_obj, local_mapping, outputs, output_objects)
            certified.update(substitute_expression(stream.certified, local_mapping))
            macro_from_micro.append(local_mapping) # TODO: append for functions as well?
    #assert not fluent_facts
    return inputs, sorted(domain), outputs, sorted(certified), sorted(functions), \
           macro_from_micro, input_objects, output_objects, fluent_facts
