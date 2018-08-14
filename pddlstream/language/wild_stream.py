from collections import Sequence

from pddlstream.language.conversion import list_from_conjunction, get_formula_operators, \
    AND, obj_from_value_expression, str_from_tuple
from pddlstream.language.external import get_procedure_fn, parse_lisp_list
from pddlstream.language.generator import get_next
from pddlstream.language.stream import Stream, StreamInstance, StreamResult

# TODO: implement a pushing example

def wild_gen_fn_from_gen_fn(gen_fn):
    def wild_gen_fn(*args, **kwargs):
        for output_list in gen_fn(*args, **kwargs):
            fact_list = []
            yield output_list, fact_list
    return wild_gen_fn

def gen_fn_from_wild_gen_fn(wild_gen_fn):
    def gen_fn(*args, **kwargs):
        for output_list, _ in wild_gen_fn(*args, **kwargs):
            yield output_list
    return wild_gen_fn

##################################################

# TODO: could also make only wild facts and automatically identify output tuples satisfying certified
# TODO: might want to move to this model in general

class WildInstance(StreamInstance):
    def __init__(self, stream, input_objects):
        super(WildInstance, self).__init__(stream, input_objects)
        self.new_facts = []
        self.num_wild = 0

    def next_facts(self, verbose=False):
        # TODO: more cleanly implement this feature
        if verbose:
            print('Wild {}-{}) {}:{}->{}'.format(self.num_wild, self.num_calls, self.external.name,
                                            str_from_tuple(self.get_input_values()),
                                            self.new_facts))
        new_facts = list(map(obj_from_value_expression, self.new_facts))
        self.new_facts = []
        self.num_wild += 1
        return new_facts

    def _next_outputs(self):
        if self._generator is None:
            input_values = self.get_input_values()
            try:
                self._generator = self.external.gen_fn(*input_values)
            except TypeError as err:
                print('Wild stream [{}] expects {} inputs'.format(self.external.name, len(input_values)))
                raise err
        pair, self.enumerated = get_next(self._generator)
        if len(pair) != 2:
            raise RuntimeError('Wild stream [{}] does not generate pairs of output values and wild facts'.format(self.external.name))
        new_values, new_facts = pair
        if not isinstance(new_facts, Sequence):
            raise ValueError('Output wild facts for stream [{}] is not a sequence: {}'.format(self.external.name, new_values))
        self.new_facts.extend(new_facts)
        return new_values

class WildStream(Stream):
    _Instance = WildInstance
    _Result = StreamResult

##################################################

def parse_wild_stream(lisp_list, stream_map, stream_info):
    # TODO: refactor this
    value_from_attribute = parse_lisp_list(lisp_list)
    assert set(value_from_attribute) <= {':wild-stream', ':inputs', ':domain', ':fluents', ':outputs', ':certified'}
    name = value_from_attribute[':wild-stream']
    domain = value_from_attribute.get(':domain', None)
    # TODO: dnf_from_positive_formula(value_from_attribute.get(':domain', []))
    if not (get_formula_operators(domain) <= {AND}):
        # TODO: allow positive DNF
        raise ValueError('Stream [{}] domain must be a conjunction'.format(name))
    certified = value_from_attribute.get(':certified', None)
    if not (get_formula_operators(certified) <= {AND}):
        raise ValueError('Stream [{}] certified must be a conjunction'.format(name))
    return WildStream(name, get_procedure_fn(stream_map, name),
                      value_from_attribute.get(':inputs', []),
                      list_from_conjunction(domain),
                      value_from_attribute.get(':outputs', []),
                      list_from_conjunction(certified),
                      stream_info.get(name, None),
                      value_from_attribute.get(':fluents', []))  # TODO: None
