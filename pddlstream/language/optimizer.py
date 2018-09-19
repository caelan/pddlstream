from itertools import product

from pddlstream.language.constants import get_prefix, get_args, get_parameter_name
from pddlstream.language.conversion import substitute_expression, list_from_conjunction
from pddlstream.language.external import parse_lisp_list, get_procedure_fn
from pddlstream.language.stream import OptValue, StreamInfo, Stream
from pddlstream.language.generator import empty_gen
from pddlstream.utils import get_mapping, INF

def get_gen_fn(procedure, inputs, outputs, certified):
    def gen_fn(*input_values):
        mapping = get_mapping(inputs, input_values)
        targets = substitute_expression(certified, mapping)
        return procedure(outputs, targets)
    return gen_fn

def parse_variable(optimizer_name, procedure, lisp_list, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list)
    variable = value_from_attribute[':variable'] # TODO: assume unique?
    outputs = [variable]
    inputs = value_from_attribute.get(':inputs', [])
    domain = list_from_conjunction(value_from_attribute[':domain'])
    certified = list_from_conjunction(value_from_attribute[':graph'])
    stream_name = '{}_{}'.format(optimizer_name, get_parameter_name(variable))
    gen_fn = get_gen_fn(procedure, inputs, outputs, certified) # TODO: shouldn't need this...
    #gen_fn = empty_gen()
    #info = StreamInfo(effort_fn=get_effort_fn(optimizer_name, inputs, outputs))
    info = StreamInfo()
    return Stream(stream_name, gen_fn, inputs, domain, outputs, certified, info)

def parse_constraint(optimizer_name, procedure, lisp_list, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list)
    outputs = []
    constraint = value_from_attribute[':constraint']
    certified = [constraint]
    inputs = get_args(constraint)
    domain = list_from_conjunction(value_from_attribute[':necessary'])
    stream_name = '{}_{}'.format(optimizer_name, get_prefix(constraint))
    gen_fn = get_gen_fn(procedure, inputs, outputs, certified) # TODO: shouldn't need this...
    #gen_fn = empty_gen()
    #info = StreamInfo(effort_fn=get_effort_fn(optimizer_name, inputs, outputs))
    info = StreamInfo()
    return Stream(stream_name, gen_fn, inputs, domain, outputs, certified, info)

# TODO: convert optimizer into a set of streams?

def parse_optimizer(lisp_list, stream_map, stream_info):
    _, optimizer_name = lisp_list[:2]
    procedure = get_procedure_fn(stream_map, optimizer_name)
    streams = []
    for sub_list in lisp_list[2:]:
        form = sub_list[0]
        if form == ':variable':
            streams.append(parse_variable(optimizer_name, procedure, sub_list, stream_info))
        elif form == ':constraint':
            streams.append(parse_constraint(optimizer_name, procedure, sub_list, stream_info))
        else:
            raise ValueError(form)
    return streams

##################################################

# TODO: opt gen function
# TODO: custom optimistic objects

def get_effort_fn(optimizer_name, inputs, outputs):
    def effort_fn(*input_values):
        free_indices = [i for i, value in enumerate(input_values) if isinstance(value, OptValue)
                        and value.stream.startswith(optimizer_name)]
        num_free = len(outputs) + len(free_indices)
        #print(free_indices)
        #print(input_values)
        #raw_input('awef')
        if num_free == 0:
            return INF
        return 1
    return effort_fn

def parse_constraint2(optimizer_name, procedure, lisp_list, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list)
    fixed = value_from_attribute.get(':fixed', [])
    mutex = value_from_attribute.get(':mutex', []) # Associating a variable to a constraint
    #free = value_from_attribute.get(':free', [])
    [constraint] = list_from_conjunction(value_from_attribute[':constraint'])
    constraint_name = get_prefix(constraint)
    necessary = list_from_conjunction(value_from_attribute[':necessary'])
    streams = []
    #for i, combo in enumerate(product([False, True], repeat=len(free))):
    #    outputs = [p for p, include in zip(free, combo) if include]
    # if not outputs:
    #    continue
    outputs_list = [tuple()] + [(p,) for p in mutex]
    for outputs in outputs_list:
        inputs = [p for p in get_args(constraint) if p not in outputs]
        certified = [constraint] + [f for f in necessary if any(p in outputs for p in get_args(f))]
        domain = [f for f in necessary if f not in certified]
        # TODO: don't include constraints without free params
        combo = [p in outputs for p in get_args(constraint)]
        stream_name = '{}_{}_{}'.format(optimizer_name, constraint_name, ''.join(map(str, map(int, combo))))
        info = StreamInfo(effort_fn=get_effort_fn(optimizer_name, inputs, outputs))
        #info = StreamInfo()
        gen_fn = get_gen_fn(procedure, inputs, outputs, certified)
        streams.append(Stream(stream_name, gen_fn, inputs, domain, outputs, certified, info))
        # TODO: prune implied facts
        # TODO: finite cost when inputs are from the same constraint
    return streams

def parse_optimizer2(lisp_list, stream_map, stream_info):
    _, name = lisp_list[:2]
    procedure = get_procedure_fn(stream_map, name)
    streams = []
    for constraint in lisp_list[2:]:
        streams.extend(parse_constraint2(name, procedure, constraint, stream_info))
    return streams