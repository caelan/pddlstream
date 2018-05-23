from collections import defaultdict
from itertools import count

from pddlstream.conversion import get_prefix, get_args, objects_from_evaluations, is_atom, Evaluation, Head
from pddlstream.downward import fd_from_fact, TOTAL_COST
from pddlstream.stream import Stream, from_fn
from pddlstream.utils import int_ceil


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


def replace_gen_fn(stream):
    future_gen_fn = from_fn(lambda *args: tuple(FutureValue(stream.name, args, o) for o in stream.outputs))
    gen_fn = stream.gen_fn
    def new_gen_fn(*input_values):
        if any(isinstance(value, FutureValue) for value in input_values):
            return future_gen_fn(*input_values)
        return gen_fn(*input_values)
    stream.gen_fn = new_gen_fn

def compile_to_exogenous(evaluations, domain, streams):
    import pddl
    # TODO: automatically derive fluents
    # TODO: prevent application for not latent
    # TODO: version of this that operates on fluents of length one?
    # TODO: better instantiation when have full parameters
    # TODO: conversion from stream cost to real cost units?
    # TODO: more generically would create something for each parameter

    fluent_predicates = set()
    for action in domain.actions:
        for effect in action.effects:
            fluent_predicates.add(effect.literal.predicate)
    for axiom in domain.axioms:
        fluent_predicates.add(axiom.name)

    domain_predicates = {get_prefix(a) for s in streams for a in s.domain}
    if not (domain_predicates & fluent_predicates):
        return
    print(fluent_predicates)
    print(domain_predicates)
    # TODO: can also map to real and fake
    # TODO: any predicates derived would need to be replaced as well

    #objects = objects_from_evaluations(evaluations)
    #print(objects)

    certified_predicates = {get_prefix(a) for s in streams for a in s.certified}
    print(certified_predicates)

    def get_evaluation_future(atom):
        if not is_atom(atom):
            return atom
        if atom.head.function not in certified_predicates:
            return atom
        new_function = 'f-{}'.format(atom.head.function)
        new_head = Head(new_function, atom.head.args)
        return Evaluation(new_head, atom.value)

    def get_future(atom):
        name = get_prefix(atom)
        if name not in certified_predicates:
            return atom
        new_name = 'f-{}'.format(name)
        return (new_name,) + get_args(atom)

    print(len(evaluations))
    for evaluation in evaluations.keys():
        evaluations[get_evaluation_future(evaluation)] = None
    print(len(evaluations))

    for stream in list(streams):
        if not isinstance(stream, Stream):
            raise NotImplementedError(stream)
        fluent_domain = list(filter(lambda a: get_prefix(a) in fluent_predicates, stream.domain))
        static_domain = list(set(stream.domain) - set(fluent_domain))
        # TODO: could also just have conditions asserting that one of the fluent conditions fails

        stream_name = 'future-{}'.format(stream.name)
        gen_fn = from_fn(lambda *args: tuple(FutureValue(stream.name, args, o) for o in stream.outputs))
        parameters = tuple(stream.inputs + stream.outputs)
        new_domain = list(map(get_future, static_domain))
        stream_atom = ('{}-result'.format(stream.name),) + parameters
        new_graph = [stream_atom] + list(map(get_future, stream.certified))
        streams.append(Stream(stream_name, gen_fn, stream.inputs, new_domain,
                              stream.outputs, new_graph, stream.info))

        action_parameters = [pddl.TypedObject(p, 'object') for p in parameters]
        action_name = 'call-{}'.format(stream.name)
        precondition = pddl.Conjunction(tuple(map(fd_from_fact, new_graph + list(stream.domain))))
        effects = [pddl.Effect(parameters=[], condition=pddl.Truth(),
                               literal=fd_from_fact(fact)) for fact in stream.certified]
        effort = 1 # TODO: use stream info
        #effort = 1 if unit_cost else result.instance.get_effort()
        #if effort == INF:
        #    continue
        fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
        expression = pddl.NumericConstant(int_ceil(effort)) # Integer
        cost = pddl.Increase(fluent=fluent, expression=expression) # Can also be None
        domain.actions.append(pddl.Action(name=action_name, parameters=action_parameters,
                                 num_external_parameters=len(action_parameters),
                                 precondition=precondition, effects=effects, cost=cost))

        #replace_gen_fn(stream)
        stream.certified = tuple(set(stream.certified) | set(map(get_future, stream.certified)))
