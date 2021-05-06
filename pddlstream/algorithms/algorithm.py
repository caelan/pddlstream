from collections import Counter

from pddlstream.algorithms.common import evaluations_from_init, SOLUTIONS
from pddlstream.algorithms.constraints import add_plan_constraints
from pddlstream.algorithms.downward import parse_lisp, parse_goal, has_costs, set_unit_costs, normalize_domain_goal
from pddlstream.language.temporal import parse_domain, SimplifiedDomain
from pddlstream.language.constants import get_prefix, get_args
from pddlstream.language.conversion import obj_from_value_expression
from pddlstream.language.exogenous import compile_to_exogenous
from pddlstream.language.external import External
from pddlstream.language.function import parse_function, parse_predicate
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.language.optimizer import parse_optimizer
from pddlstream.language.rule import parse_rule, apply_rules_to_streams, RULES
from pddlstream.language.stream import parse_stream, Stream, StreamInstance
from pddlstream.utils import INF


# TODO: rename file to parsing

def parse_constants(domain, constant_map):
    obj_from_constant = {}
    for constant in domain.constants:
        if constant.name.startswith(Object._prefix): # TODO: check other prefixes
            raise NotImplementedError('Constants are not currently allowed to begin with {}'.format(Object._prefix))
        if constant.name not in constant_map:
            raise ValueError('Undefined constant {}'.format(constant.name))
        value = constant_map.get(constant.name, constant.name)
        obj_from_constant[constant.name] = Object(value, name=constant.name) # TODO: remap names
        # TODO: add object predicate

    for name in constant_map:
        for constant in domain.constants:
            if constant.name == name:
                break
        else:
            raise ValueError('Constant map value {} not mentioned in domain :constants'.format(name))
    del domain.constants[:] # So not set twice
    return obj_from_constant

def check_problem(domain, streams, obj_from_constant):
    for action in (domain.actions + domain.axioms):
        for p, c in Counter(action.parameters).items():
            if c != 1:
                raise ValueError('Parameter [{}] for action [{}] is not unique'.format(p.name, action.name))
        # TODO: check that no undeclared parameters & constants
        #action.dump()

    undeclared_predicates = set()
    for stream in streams:
        # TODO: domain.functions
        facts = list(stream.domain)
        if isinstance(stream, Stream):
            facts.extend(stream.certified)
        for fact in facts:
            name = get_prefix(fact)
            if name not in domain.predicate_dict:
                undeclared_predicates.add(name)
            elif len(get_args(fact)) != domain.predicate_dict[name].get_arity(): # predicate used with wrong arity: {}
                print('Warning! predicate used with wrong arity in stream [{}]: {}'.format(stream.name, fact))
        # for constant in stream.constants:
        #     if constant not in obj_from_constant:
        #         raise ValueError('Undefined constant in stream [{}]: {}'.format(stream.name, constant))
    if undeclared_predicates:
        print('Warning! Undeclared predicates: {}'.format(
            sorted(undeclared_predicates))) # Undeclared predicate: {}

def reset_globals():
    # TODO: maintain these dictionaries in an object
    Object.reset()
    OptimisticObject.reset()
    RULES[:] = []
    SOLUTIONS[:] = []

def parse_problem(problem, stream_info={}, constraints=None, unit_costs=False, unit_efforts=False):
    # TODO: just return the problem if already written programmatically
    #reset_globals() # Prevents use of satisfaction.py
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = problem

    domain = parse_domain(domain_pddl) # TODO: normalize here
    #domain = domain_pddl
    if len(domain.types) != 1:
        raise NotImplementedError('Types are not currently supported')
    if unit_costs:
        set_unit_costs(domain)
    if not has_costs(domain):
        # TODO: set effort_weight to 1 if no costs
        print('Warning! All actions have no cost. Recommend setting unit_costs=True')
    obj_from_constant = parse_constants(domain, constant_map) # Keep before parse_stream_pddl

    streams = parse_stream_pddl(stream_pddl, stream_map, stream_info=stream_info,
                                unit_costs=unit_costs, unit_efforts=unit_efforts)
    check_problem(domain, streams, obj_from_constant)

    evaluations = evaluations_from_init(init)
    goal_exp = obj_from_value_expression(goal)

    if isinstance(domain, SimplifiedDomain):
        #assert isinstance(domain, str) # raw PDDL is returned
        _ = {name: Object(value, name=name) for name, value in constant_map.items()}
        return evaluations, goal_exp, domain, streams

    goal_exp = add_plan_constraints(constraints, domain, evaluations, goal_exp)
    parse_goal(goal_exp, domain) # Just to check that it parses
    normalize_domain_goal(domain, goal_exp) # TODO: does not normalize goal_exp

    compile_to_exogenous(evaluations, domain, streams)
    return evaluations, goal_exp, domain, streams

##################################################

def parse_streams(streams, rules, stream_pddl, procedure_map, procedure_info, use_functions=True):
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, pddl_name = next(stream_iter)
    assert('stream' == pddl_type)
    for lisp_list in stream_iter:
        name = lisp_list[0] # TODO: refactor at this point
        if name == ':stream':
            externals = [parse_stream(lisp_list, procedure_map, procedure_info)]
        elif name == ':rule':
            externals = [parse_rule(lisp_list, procedure_map, procedure_info)]
        elif name == ':function':
            if not use_functions:
                continue
            externals = [parse_function(lisp_list, procedure_map, procedure_info)]
        elif name == ':predicate': # Cannot just use args if want a bound
            externals = [parse_predicate(lisp_list, procedure_map, procedure_info)]
        elif name == ':optimizer':
            externals = parse_optimizer(lisp_list, procedure_map, procedure_info)
        else:
            raise ValueError(name)
        for external in externals:
            if any(e.name == external.name for e in streams):
                raise ValueError('Stream [{}] is not unique'.format(external.name))
            if name == ':rule':
                rules.append(external)
            external.pddl_name = pddl_name # TODO: move within constructors
            streams.append(external)

def set_unit_efforts(externals):
    for external in externals:
        if external.get_effort() < INF:
            external.info.effort = 1

NO_INFO = None
RELATIONAL_INFO = 'relational_info' # structural_info
STATISTICS_INFO = 'statistics_info'

def parse_stream_pddl(stream_pddl, stream_map, stream_info={}, unit_costs=False, unit_efforts=False):
    if stream_info is None: # NO_INFO
        stream_info = {}
    externals = []
    if stream_pddl is None:
        return externals # No streams
    if isinstance(stream_pddl, str):
        stream_pddl = [stream_pddl]
    if all(isinstance(e, External) for e in stream_pddl):
        return stream_pddl
    if isinstance(stream_map, dict): # DEBUG_MODES
        stream_map = {k.lower(): v for k, v in stream_map.items()}
    stream_info = {k.lower(): v for k, v in stream_info.items()}
    rules = []
    for pddl in stream_pddl:
        # TODO: check which functions are actually used and prune the rest
        parse_streams(externals, rules, pddl, stream_map, stream_info, use_functions=not unit_costs)
    apply_rules_to_streams(rules, externals)
    if unit_efforts:
        set_unit_efforts(externals)
    return externals

##################################################

def remove_blocked(evaluations, domain, instance, new_results):
    # TODO: finish refactoring this
    if new_results and isinstance(instance, StreamInstance):
        instance.enable(evaluations, domain)
