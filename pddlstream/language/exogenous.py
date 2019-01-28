from collections import defaultdict
from itertools import count

from pddlstream.algorithms.common import add_fact, INTERNAL_EVALUATION
from pddlstream.algorithms.downward import make_predicate, add_predicate, make_action, make_axiom, get_fluents
from pddlstream.language.constants import Head, Evaluation, get_prefix, get_args
from pddlstream.language.conversion import evaluation_from_fact, \
    is_atom, fact_from_evaluation
from pddlstream.language.generator import from_fn
from pddlstream.language.object import Object
from pddlstream.language.stream import Stream


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

##################################################

def augment_evaluations(evaluations, future_map):
    for evaluation in list(filter(is_atom, evaluations)):
        name = evaluation.head.function
        if name in future_map:
            new_head = Head(future_map[name], evaluation.head.args)
            new_evaluation = Evaluation(new_head, evaluation.value)
            add_fact(evaluations, fact_from_evaluation(new_evaluation),
                     result=INTERNAL_EVALUATION, complexity=0)

def rename_atom(atom, mapping):
    name = get_prefix(atom)
    if name not in mapping:
        return atom
    return (mapping[name],) + get_args(atom)

def create_static_stream(stream, evaluations, fluent_predicates, get_future):
    def static_fn(*input_values):
        instance = stream.get_instance(tuple(map(Object.from_value, input_values)))
        if all(evaluation_from_fact(f) in evaluations for f in instance.get_domain()):
            return None
        return tuple(FutureValue(stream.name, input_values, o) for o in stream.outputs)

    #opt_evaluations = None
    def static_opt_gen_fn(*input_values):
        instance = stream.get_instance(tuple(map(Object.from_value, input_values)))
        if all(evaluation_from_fact(f) in evaluations for f in instance.get_domain()):
            return
        for output_values in stream.opt_gen_fn(*input_values):
            yield output_values
        # TODO: need to replace regular opt_gen_fn to update opt_evaluations
        # if I want to prevent switch from normal to static in opt

    # Focused algorithm naturally biases against using future because of axiom layers
    stream_name = 'future-{}'.format(stream.name)
    gen_fn = from_fn(static_fn)
    static_domain = list(filter(lambda a: get_prefix(a) not in fluent_predicates, stream.domain))
    new_domain = list(map(get_future, static_domain))
    stream_atom = ('{}-result'.format(stream.name),) + tuple(stream.inputs + stream.outputs)
    new_certified = [stream_atom] + list(map(get_future, stream.certified))
    static_stream = Stream(stream_name, gen_fn, stream.inputs, new_domain,
                          stream.outputs, new_certified, stream.info)
    static_stream.opt_gen_fn = static_opt_gen_fn
    return static_stream

def compile_to_exogenous_actions(evaluations, domain, streams):
    # TODO: automatically derive fluents
    # TODO: version of this that operates on fluents of length one?
    # TODO: better instantiation when have full parameters
    # TODO: conversion from stream cost to real cost units?
    # TODO: any predicates derived would need to be replaced as well
    fluent_predicates = get_fluents(domain)
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
        add_predicate(domain, make_predicate(get_prefix(stream_atom), get_args(stream_atom)))
        preconditions = [stream_atom] + list(stream.domain)
        effort = 1 # TODO: use stream info
        #effort = 1 if unit_cost else result.instance.get_effort()
        #if effort == INF:
        #    continue
        domain.actions.append(make_action(
            name='call-{}'.format(stream.name),
            parameters=get_args(stream_atom),
            preconditions=preconditions,
            effects=stream.certified,
            cost=effort))
        stream.certified = tuple(set(stream.certified) |
                                 set(map(rename_future, stream.certified)))

##################################################

def replace_literals(replace_fn, expression):
    import pddl.conditions
    if isinstance(expression, pddl.conditions.ConstantCondition):
        return expression # TODO: replace constants?
    if isinstance(expression, pddl.conditions.JunctorCondition):
        new_parts = [replace_literals(replace_fn, p) for p in expression.parts]
        return expression.__class__(new_parts)
    if isinstance(expression, pddl.conditions.QuantifiedCondition):
        new_parts = [replace_literals(replace_fn, p) for p in expression.parts]
        return expression.__class__(expression.parameters, new_parts)
    if isinstance(expression, pddl.conditions.Literal):
        return replace_fn(expression)
    raise ValueError(expression)

def replace_predicates(predicate_map, expression):
    def replace_fn(literal):
        new_predicate = predicate_map.get(literal.predicate, literal.predicate)
        return literal.__class__(new_predicate, literal.args)
    return replace_literals(replace_fn, expression)

def compile_to_exogenous_axioms(evaluations, domain, streams):
    # TODO: no attribute certified
    import pddl
    fluent_predicates = get_fluents(domain)
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
        stream_atom = streams[-1].certified[0]
        add_predicate(domain, make_predicate(get_prefix(stream_atom), get_args(stream_atom)))
        preconditions = [stream_atom] + list(map(rename_derived, stream.domain))
        for certified_fact in stream.certified:
            derived_fact = rename_derived(certified_fact)
            external_params = get_args(derived_fact)
            internal_params = tuple(p for p in (stream.inputs + stream.outputs)
                                    if p not in get_args(derived_fact))
            domain.axioms.extend([
                make_axiom(
                    parameters=external_params,
                    preconditions=[certified_fact],
                    derived=derived_fact),
                make_axiom(
                    parameters=external_params+internal_params,
                    preconditions=preconditions,
                    derived=derived_fact),
            ])
        stream.certified = tuple(set(stream.certified) |
                                 set(map(rename_future, stream.certified)))

##################################################

def get_exogenous_predicates(domain, streams):
    fluent_predicates = get_fluents(domain)
    domain_predicates = {get_prefix(a) for s in streams for a in s.domain}
    return list(domain_predicates & fluent_predicates)

def compile_to_exogenous(evaluations, domain, streams, use_axioms=True):
    exogenous_predicates = get_exogenous_predicates(domain, streams)
    if not exogenous_predicates:
        return False
    print('Warning! The following predicates are mentioned in both action effects '
          'and stream domain conditions: {}'.format(exogenous_predicates))
    if use_axioms:
        compile_to_exogenous_axioms(evaluations, domain, streams)
    else:
        compile_to_exogenous_actions(evaluations, domain, streams)
    return True
