from pddlstream.language.stream import Stream, StreamInfo
from pddlstream.language.external import parse_lisp_list
from pddlstream.language.generator import from_test
from pddlstream.language.conversion import list_from_conjunction, substitute_expression, get_args, is_parameter

# TODO: could signal a rule by making its gen_fn just the constant True
# TODO: could apply the rule in the initial state once but then couldn't support unexpected facts

rules = [] # TODO: no global

def parse_rule(lisp_list, stream_map, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list[1:])
    assert set(value_from_attribute) <= {':inputs', ':domain', ':certified'}
    # TODO: if len(certified) == 1, augment existing streams
    rules.append(Stream(name='rule{}'.format(len(rules)),
                        gen_fn=from_test(lambda *args: True),
                        inputs=value_from_attribute.get(':inputs', []),
                        domain=list_from_conjunction(value_from_attribute.get(':domain', [])),
                        fluents=[],
                        outputs=[],
                        certified=list_from_conjunction(value_from_attribute.get(':certified', [])),
                        info=StreamInfo(eager=True, p_success=1, overhead=0)))
    return rules[-1]
    # TODO: could make p_success=0 to prevent use in search
