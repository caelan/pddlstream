from collections import deque

from pddlstream.language.constants import get_prefix, get_args
from pddlstream.language.stream import Stream, StreamInfo
from pddlstream.language.external import parse_lisp_list
from pddlstream.language.generator import from_test
from pddlstream.language.conversion import list_from_conjunction, substitute_expression

# TODO: could signal a rule by making its gen_fn just the constant True
# TODO: could apply the rule in the initial state once but then couldn't support unexpected facts
# TODO: prune unnecessary preconditions using rules
from pddlstream.utils import get_mapping

RULES = [] # TODO: no global

def parse_rule(lisp_list, stream_map, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list[1:])
    assert set(value_from_attribute) <= {':inputs', ':domain', ':certified'}
    # TODO: if len(certified) == 1, augment existing streams
    RULES.append(Stream(name='rule{}'.format(len(RULES)),
                        gen_fn=from_test(lambda *args: True),
                        inputs=value_from_attribute.get(':inputs', []),
                        domain=list_from_conjunction(value_from_attribute.get(':domain', [])),
                        fluents=[],
                        outputs=[],
                        certified=list_from_conjunction(value_from_attribute.get(':certified', [])),
                        info=StreamInfo(eager=True, p_success=1, overhead=0)))
    return RULES[-1]
    # TODO: could make p_success=0 to prevent use in search

##################################################

def apply_rules_to_streams(rules, streams):
    # TODO: can actually this with multiple condition if stream certified contains all
    # TODO: do also when no domain conditions
    processed_rules = deque(rules)
    while processed_rules:
        rule = processed_rules.popleft()
        if len(rule.domain) != 1:
            continue
        [rule_fact] = rule.domain
        rule.info.p_success = 0 # Need not be applied
        for stream in streams:
            if not isinstance(stream, Stream):
                continue
            for stream_fact in stream.certified:
                if get_prefix(rule_fact) == get_prefix(stream_fact):
                    mapping = get_mapping(get_args(rule_fact), get_args(stream_fact))
                    new_facts = set(substitute_expression(rule.certified, mapping)) - set(stream.certified)
                    stream.certified = stream.certified + tuple(new_facts)
                    if new_facts and (stream in rules):
                            processed_rules.append(stream)