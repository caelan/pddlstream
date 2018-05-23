from collections import defaultdict
from itertools import count

from pddlstream.conversion import get_prefix
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


def compile_to_exogenous(domain, streams):
    import pddl
    # TODO: automatically derive fluents
    # TODO: prevent application for not latent
    # TODO: version of this that operates on fluents of length one?
    # TODO: better instantiation when have full parameters

    fluent_predicates = set()
    for action in domain.actions:
        for effect in action.effects:
            fluent_predicates.add(effect.literal.predicate)
    for axiom in domain.axioms:
        fluent_predicates.add(axiom.name)
    print(fluent_predicates)

    for stream in streams:
        fluent_domain = list(filter(lambda a: get_prefix(a) in fluent_predicates, stream.domain))
        if not fluent_domain:
            continue
        if not isinstance(stream, Stream):
            raise NotImplementedError(stream)
        static_domain = list(set(stream.domain) - set(fluent_domain))
        #stream.domain = tuple(static_domain)

        stream_name = 'future-{}'.format(stream.name)
        gen_fn = from_fn(lambda *args: tuple(FutureValue(stream.name, args, o) for o in stream.outputs))
        parameters = tuple(stream.inputs + stream.outputs)
        new_graph = [('{}-result'.format(stream.name),) + parameters]
        streams.append(Stream(stream_name, gen_fn, stream.inputs, static_domain,
                                  stream.outputs, new_graph, stream.info))

        action_parameters = [pddl.TypedObject(p, 'object') for p in parameters]
        action_name = 'call-{}'.format(stream.name)
        precondition = pddl.Conjunction(tuple(map(fd_from_fact, new_graph + fluent_domain)))
        effects = [pddl.Effect(parameters=[], condition=pddl.Truth(), literal=fd_from_fact(fact)) for fact in stream.certified]
        effort = 1
        #effort = 1 if unit_cost else result.instance.get_effort()
        #if effort == INF:
        #    continue
        fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
        expression = pddl.NumericConstant(int_ceil(effort)) # Integer
        cost = pddl.Increase(fluent=fluent, expression=expression) # Can also be None
        domain.actions.append(pddl.Action(name=action_name, parameters=action_parameters,
                                 num_external_parameters=len(action_parameters),
                                 precondition=precondition, effects=effects, cost=cost))
        # TODO: modify other streams to only take real values?