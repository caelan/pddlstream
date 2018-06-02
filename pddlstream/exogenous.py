from collections import defaultdict
from itertools import count

from pddlstream.conversion import get_prefix, get_args, evaluation_from_fact, \
    is_atom, Evaluation, Head
from pddlstream.downward import fd_from_fact, TOTAL_COST
from pddlstream.stream import Stream, from_fn
from pddlstream.utils import int_ceil
from pddlstream.object import Object

# TODO: can do this whole story within the focused algorithm as well

class FutureValue(object):
    # TODO: use this instead of debug value?
    _output_counts = defaultdict(count)
    def __init__(self, stream, input_values, output_parameter):
        self.stream = stream
        self.input_values = input_values
        self.output_parameter = output_parameter
        self.index = next(self._output_counts[output_parameter])
        # TODO: hash this?
    def __repr__(self):
        return '@{}{}'.format(self.output_parameter[1:], self.index)

# def replace_gen_fn(stream):
#     future_gen_fn = from_fn(lambda *args: tuple(FutureValue(stream.name, args, o) for o in stream.outputs))
#     gen_fn = stream.gen_fn
#     def new_gen_fn(*input_values):
#         if any(isinstance(value, FutureValue) for value in input_values):
#             return future_gen_fn(*input_values)
#         return gen_fn(*input_values)
#     stream.gen_fn = new_gen_fn

def get_fluents(domain):
    fluent_predicates = set()
    for action in domain.actions:
        for effect in action.effects:
            fluent_predicates.add(effect.literal.predicate)
    for axiom in domain.axioms:
        fluent_predicates.add(axiom.name)
    return fluent_predicates

##################################################

def augment_evaluations(evaluations, future_map):
    for evaluation in filter(is_atom, evaluations):
        name = evaluation.head.function
        if name in future_map:
            new_head = Head(future_map[name], evaluation.head.args)
            new_evaluation = Evaluation(new_head, evaluation.value)
            evaluations[new_evaluation] = None

def rename_atom(atom, mapping):
    name = get_prefix(atom)
    if name not in mapping:
        return atom
    return (mapping[name],) + get_args(atom)

def create_static_stream(stream, evaluations, fluent_predicates, get_future):
    def fn(*input_values):
        # The stream certified predicates become fluents here
        input_objects = tuple(map(Object.from_value, input_values))
        instance = stream.get_instance(input_objects)
        if all(evaluation_from_fact(f) in evaluations for f in instance.get_domain()):
            return None
        return tuple(FutureValue(stream.name, input_values, o) for o in stream.outputs)

    stream_name = 'future-{}'.format(stream.name)
    gen_fn = from_fn(fn)
    static_domain = list(filter(lambda a: get_prefix(a) not in fluent_predicates, stream.domain))
    new_domain = list(map(get_future, static_domain))
    stream_atom = ('{}-result'.format(stream.name),) + tuple(stream.inputs + stream.outputs)
    new_certified = [stream_atom] + list(map(get_future, stream.certified))
    return Stream(stream_name, gen_fn, stream.inputs, new_domain,
                          stream.outputs, new_certified, stream.info)

def compile_to_exogenous_actions(evaluations, domain, streams):
    import pddl
    # TODO: automatically derive fluents
    # TODO: version of this that operates on fluents of length one?
    # TODO: better instantiation when have full parameters
    # TODO: conversion from stream cost to real cost units?
    # TODO: any predicates derived would need to be replaced as well
    fluent_predicates = get_fluents(domain)
    domain_predicates = {get_prefix(a) for s in streams for a in s.domain}
    if not (domain_predicates & fluent_predicates):
        return

    certified_predicates = {get_prefix(a) for s in streams for a in s.certified}
    future_map = {p: 'f-{}'.format(p) for p in certified_predicates}
    augment_evaluations(evaluations, future_map)
    rename_future = lambda a: rename_atom(a, future_map)
    for stream in list(streams):
        if not isinstance(stream, Stream):
            raise NotImplementedError(stream)
        # TODO: could also just have conditions asserting that one of the fluent conditions fails
        streams.append(create_static_stream(stream, evaluations, fluent_predicates, rename_future))
        stream_atom = streams[-1].certified[0]
        parameters = [pddl.TypedObject(p, 'object') for p in get_args(stream_atom)]
        # TODO: add to predicates as well?
        domain.predicate_dict[get_prefix(stream_atom)] = pddl.Predicate(get_prefix(stream_atom), parameters)
        precondition = pddl.Conjunction(tuple(map(fd_from_fact, (stream_atom,) + tuple(stream.domain))))
        effects = [pddl.Effect(parameters=[], condition=pddl.Truth(),
                               literal=fd_from_fact(fact)) for fact in stream.certified]
        effort = 1 # TODO: use stream info
        #effort = 1 if unit_cost else result.instance.get_effort()
        #if effort == INF:
        #    continue
        fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
        expression = pddl.NumericConstant(int_ceil(effort)) # Integer
        cost = pddl.Increase(fluent=fluent, expression=expression) # Can also be None
        domain.actions.append(pddl.Action(name='call-{}'.format(stream.name),
                                          parameters=parameters,
                                          num_external_parameters=len(parameters),
                                          precondition=precondition, effects=effects, cost=cost))
        stream.certified = tuple(set(stream.certified) |
                                 set(map(rename_future, stream.certified)))

##################################################

def replace_predicates(predicate_map, expression):
    import pddl.conditions
    if isinstance(expression, pddl.conditions.ConstantCondition):
        return expression
    if isinstance(expression, pddl.conditions.JunctorCondition):
        new_parts = [replace_predicates(predicate_map, p) for p in expression.parts]
        return expression.__class__(new_parts)
    if isinstance(expression, pddl.conditions.QuantifiedCondition):
        new_parts = [replace_predicates(predicate_map, p) for p in expression.parts]
        return expression.__class__(expression.parameters, new_parts)
    if isinstance(expression, pddl.conditions.Literal):
        new_predicate = predicate_map.get(expression.predicate, expression.predicate)
        return expression.__class__(new_predicate, expression.args)
    raise ValueError(expression)

def compile_to_exogenous_axioms(evaluations, domain, streams):
    import pddl
    fluent_predicates = get_fluents(domain)
    domain_predicates = {get_prefix(a) for s in streams for a in s.domain}
    if not (domain_predicates & fluent_predicates):
        return

    certified_predicates = {get_prefix(a) for s in streams for a in s.certified}
    future_map = {p: 'f-{}'.format(p) for p in certified_predicates}
    augment_evaluations(evaluations, future_map)
    rename_future = lambda a: rename_atom(a, future_map)
    derived_map = {p: 'd-{}'.format(p) for p in certified_predicates}
    rename_derived = lambda a: rename_atom(a, derived_map)

    for action in domain.actions:
        action.precondition = replace_predicates(derived_map, action.precondition)
        for effect in action.effects:
            assert(isinstance(effect, pddl.Effect))
            effect.condition = replace_predicates(derived_map, effect.condition)
    for axiom in domain.axioms:
        axiom.condition = replace_predicates(derived_map, axiom.condition)

    #fluent_predicates.update(certified_predicates)
    for stream in list(streams):
        if not isinstance(stream, Stream):
            raise NotImplementedError(stream)
        streams.append(create_static_stream(stream, evaluations, fluent_predicates, rename_future))
        precondition = pddl.Conjunction(tuple(map(fd_from_fact, streams[-1].certified[:1] +
                                                  tuple(map(rename_derived, stream.domain)))))
        for fact in stream.certified:
            derived_fact = fd_from_fact(rename_derived(fact))
            external_params = derived_fact.args
            internal_params = tuple(p for p in (stream.inputs + stream.outputs)
                                        if p not in derived_fact.args)
            parameters = tuple(pddl.TypedObject(p, 'object')
                                     for p in (external_params + internal_params))
            #precondition = pddl.Conjunction(tuple(map(fd_from_fact, [stream_atom] +
            #                                        list(map(rename_derived, stream.domain)))))
            #precondition = pddl.Disjunction([fd_from_fact(fact), precondition]) # TODO: quantifier
            domain.axioms.extend([
                pddl.Axiom(name=derived_fact.predicate, parameters=parameters,
                           num_external_parameters=len(external_params),
                           condition=precondition),
                pddl.Axiom(name=derived_fact.predicate,  parameters=parameters[:len(external_params)],
                           num_external_parameters=len(external_params),
                           condition=fd_from_fact(fact))])
        stream.certified = tuple(set(stream.certified) |
                                 set(map(rename_future, stream.certified)))

##################################################

def compile_to_exogenous(evaluations, domain, streams, use_axioms=False):
    if use_axioms:
        return compile_to_exogenous_axioms(evaluations, domain, streams)
    return compile_to_exogenous_actions(evaluations, domain, streams)
