from collections import Counter

from pddlstream.algorithms.downward import make_axiom
from pddlstream.algorithms.scheduling.utils import partition_external_plan
from pddlstream.language.constants import get_prefix, get_args, get_parameter_name, Fact, concatenate, Minimize
from pddlstream.language.conversion import substitute_expression, list_from_conjunction
from pddlstream.language.external import parse_lisp_list, get_procedure_fn
from pddlstream.language.function import PredicateResult, FunctionResult
from pddlstream.language.object import OptimisticObject, Object
from pddlstream.language.stream import OptValue, StreamInfo, Stream, StreamInstance, StreamResult, \
    PartialInputs, NEGATIVE_SUFFIX
from pddlstream.utils import INF, get_mapping, safe_zip, str_from_object, get_connected_components
from pddlstream.algorithms.reorder import get_partial_orders

# TODO: could also just block a skeleton itself by adding it as a state variable

DEFAULT_SIMULTANEOUS = False
DEFAULT_UNIQUE = True # TODO: would it ever even make sense to do shared here?
OPTIMIZER_AXIOM = True
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
    # TODO: prevent outputs of the sampler from being used as inputs (only consider initial values)
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
    def __init__(self, optimizer, variable, inputs, domain, certified, infos):
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
        info = infos.get(name, None)
        if info is None:
            info = StreamInfo(opt_gen_fn=PartialInputs(unique=DEFAULT_UNIQUE),
                              simultaneous=DEFAULT_SIMULTANEOUS)
        super(VariableStream, self).__init__(name, gen_fn, inputs, domain,
                                             outputs, certified, info)

class ConstraintStream(Stream):
    def __init__(self, optimizer, constraint, domain, fluents, infos):
        self.optimizer = optimizer
        self.constraint = constraint
        self.infeasible = []
        inputs = get_args(constraint)
        outputs = []
        certified = [constraint]
        name = '{}-{}'.format(optimizer.name, get_prefix(constraint))
        gen_fn = get_list_gen_fn(self, optimizer.procedure, inputs, outputs, certified)
        #gen_fn = empty_gen()
        info = infos.get(name, None)
        if info is None:
            info = StreamInfo(effort_fn=get_effort_fn(optimizer.name),
                              simultaneous=DEFAULT_SIMULTANEOUS)
        super(ConstraintStream, self).__init__(name, gen_fn, inputs, domain,
                                               outputs, certified, info, fluents=fluents)

OPTIMIZER_STREAMS = [VariableStream, ConstraintStream]

##################################################

def parse_variable(optimizer, lisp_list, infos):
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {':variable', ':inputs', ':domain', ':graph'}
    return VariableStream(optimizer,
                          value_from_attribute[':variable'], # TODO: assume unique?
                          value_from_attribute.get(':inputs', []),
                          list_from_conjunction(value_from_attribute.get(':domain')),
                          list_from_conjunction(value_from_attribute.get(':graph')),
                          infos)

def parse_constraint(optimizer, lisp_list, infos):
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {':constraint', ':necessary', ':fluents'}
    return ConstraintStream(optimizer,
                            value_from_attribute[':constraint'],
                            list_from_conjunction(value_from_attribute[':necessary']),
                            value_from_attribute.get(':fluents', []),
                            infos)

# TODO: convert optimizer into a set of streams? Already present within test stream

def parse_optimizer(lisp_list, procedures, infos):
    _, optimizer_name = lisp_list[:2]
    procedure = get_procedure_fn(procedures, optimizer_name)
    optimizer_info = infos.get(optimizer_name, OptimizerInfo())
    optimizer = Optimizer(optimizer_name, procedure, optimizer_info)
    for sub_list in lisp_list[2:]:
        form = sub_list[0]
        if form == ':variable':
            optimizer.variables.append(parse_variable(optimizer, sub_list, infos))
        elif form == ':constraint':
            optimizer.constraints.append(parse_constraint(optimizer, sub_list, infos))
        elif form == ':objective':
            optimizer.objectives.append(sub_list[1])
        else:
            raise ValueError(form)
    return optimizer.get_streams()

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
    def _free_objects(self):
       constraints = self.get_constraints()
       objects = set()
       for fact in constraints:
           objects.update(get_args(fact))
       return list(filter(lambda o: isinstance(o, OptimisticObject), objects))
    def _add_disabled_axiom(self, domain):
        # TODO: be careful about the shared objects as parameters
        #free_objects = self._free_objects()
        free_objects = self.external.output_objects
        parameters = ['?p{}'.format(i) for i in range(len(free_objects))]
        preconditions = substitute_expression(self.get_constraints(), get_mapping(free_objects, parameters))
        self._disabled_axiom = make_axiom(parameters, preconditions, (UNSATISFIABLE,))
        domain.axioms.append(self._disabled_axiom)
        # TODO: need to block functions & predicates
        # TODO: how do I block negative streams used in conditional effects?
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
        #assert len(self.get_cluster_plans()) == 1
        super(OptimizerStream, self).__init__(optimizer.name, gen_fn, inputs, domain, outputs,
                                              certified, optimizer.info)
    def get_cluster_plans(self):
        # TODO: split the optimizer into clusters when provably independent
        external_plan = self.stream_plan + self.function_plan
        partial_orders = get_partial_orders(external_plan)
        return get_connected_components(external_plan, partial_orders)
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
    return inputs, domain, outputs, certified, functions, \
           macro_from_micro, input_objects, output_objects, fluent_facts
