import time
from collections import OrderedDict, deque, namedtuple, Counter

from pddlstream.algorithms.downward import parse_domain, get_problem, task_from_domain_problem, \
    parse_lisp, sas_from_pddl, parse_goal
from pddlstream.algorithms.search import abstrips_solve_from_task
from pddlstream.language.constants import get_prefix, get_args
from pddlstream.language.conversion import obj_from_value_expression, obj_from_pddl_plan, \
    evaluation_from_fact, substitute_expression
from pddlstream.language.exogenous import compile_to_exogenous, replace_literals
from pddlstream.language.external import External, DEBUG, get_plan_effort
from pddlstream.language.function import parse_function, parse_predicate, Function, Predicate
from pddlstream.language.object import Object
from pddlstream.language.rule import parse_rule
from pddlstream.language.stream import parse_stream, Stream
from pddlstream.utils import elapsed_time, INF, get_mapping, find_unique, get_length, str_from_plan
from pddlstream.language.optimizer import parse_optimizer, VariableStream, ConstraintStream

# TODO: way of programmatically specifying streams/actions

INITIAL_EVALUATION = None

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
    for action in domain.actions + domain.axioms:
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
        for constant in stream.constants:
            if constant not in obj_from_constant:
                raise ValueError('Undefined constant in stream [{}]: {}'.format(stream.name, constant))
    if undeclared_predicates:
        print('Warning! Undeclared predicates: {}'.format(
            sorted(undeclared_predicates))) # Undeclared predicate: {}

def evaluations_from_init(init):
    return OrderedDict((evaluation_from_fact(obj_from_value_expression(f)), INITIAL_EVALUATION) for f in init)

def parse_problem(problem, stream_info={}):
    # TODO: just return the problem if already written programmatically
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = problem
    domain = parse_domain(domain_pddl)
    if len(domain.types) != 1:
        raise NotImplementedError('Types are not currently supported')
    obj_from_constant = parse_constants(domain, constant_map)
    streams = parse_stream_pddl(stream_pddl, stream_map, stream_info)
    check_problem(domain, streams, obj_from_constant)

    evaluations = evaluations_from_init(init)
    goal_expression = obj_from_value_expression(goal)
    parse_goal(goal_expression, domain) # Just to check that it parses
    #normalize_domain_goal(domain, goal_expression)

    # TODO: refactor the following?
    compile_to_exogenous(evaluations, domain, streams)
    compile_fluent_streams(domain, streams)
    enforce_simultaneous(domain, streams)
    return evaluations, goal_expression, domain, streams

##################################################

def get_predicates(expression):
    import pddl.conditions
    if isinstance(expression, pddl.conditions.ConstantCondition):
        return set()
    if isinstance(expression, pddl.conditions.JunctorCondition) or \
            isinstance(expression, pddl.conditions.QuantifiedCondition):
        predicates = set()
        for part in expression.parts:
            predicates.update(get_predicates(part))
        return predicates
    if isinstance(expression, pddl.conditions.Literal):
        return {expression.predicate}
    raise ValueError(expression)

def enforce_simultaneous(domain, externals):
    axiom_predicates = set()
    for axiom in domain.axioms:
        axiom_predicates.update(get_predicates(axiom.condition))
    for external in externals:
        if (type(external) in [VariableStream, ConstraintStream]) and not external.info.simultaneous:
            predicates = {get_prefix(fact) for fact in external.certified}
            if predicates & axiom_predicates:
                external.info.simultaneous = True
                #print(external, (predicates & axiom_predicates))

##################################################

def has_costs(domain):
    for action in domain.actions:
        if action.cost is not None:
            return True
    return False

def solve_finite(evaluations, goal_expression, domain, unit_costs=None, debug=False, **kwargs):
    if unit_costs is None:
        unit_costs = not has_costs(domain)
    problem = get_problem(evaluations, goal_expression, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    sas_task = sas_from_pddl(task, debug=debug)
    plan_pddl, cost = abstrips_solve_from_task(sas_task, debug=debug, **kwargs)
    return obj_from_pddl_plan(plan_pddl), cost

##################################################

Solution = namedtuple('Solution', ['plan', 'cost'])

class SolutionStore(object):
    def __init__(self, max_time, max_cost, verbose):
        # TODO: store evaluations here as well as map from head to value?
        self.start_time = time.time()
        self.max_time = max_time
        #self.cost_fn = get_length if unit_costs else None
        self.max_cost = max_cost
        self.verbose = verbose
        self.best_plan = None
        self.best_cost = INF
        #self.best_cost = self.cost_fn(self.best_plan)
        self.solutions = []
    def add_plan(self, plan, cost):
        # TODO: double-check that this is a solution
        self.solutions.append(Solution(plan, cost))
        if cost < self.best_cost:
            self.best_plan = plan
            self.best_cost = cost
    def is_solved(self):
        return self.best_cost < self.max_cost
    def elapsed_time(self):
        return elapsed_time(self.start_time)
    def is_timeout(self):
        return self.max_time <= self.elapsed_time()
    def is_terminated(self):
        return self.is_solved() or self.is_timeout()

def add_facts(evaluations, fact, result=None):
    new_evaluations = []
    for fact in fact:
        evaluation = evaluation_from_fact(fact)
        if evaluation not in evaluations:
            evaluations[evaluation] = result
            new_evaluations.append(evaluation)
    return new_evaluations

def add_certified(evaluations, result):
    return add_facts(evaluations, result.get_certified(), result=result)

##################################################

def get_domain_predicates(external):
    return set(map(get_prefix, external.domain))

def get_certified_predicates(external):
    if isinstance(external, Stream):
        return set(map(get_prefix, external.certified))
    if isinstance(external, Function):
        return {get_prefix(external.head)}
    raise ValueError(external)

def get_non_producers(externals):
    # TODO: handle case where no domain conditions
    pairs = set()
    for external1 in externals:
        for external2 in externals:
            if get_certified_predicates(external1) & get_domain_predicates(external2):
                pairs.add((external1, external2))
    producers = {e1 for e1, _ in pairs}
    non_producers = set(externals) - producers
    # TODO: these are streams that be evaluated at the end as tests
    return non_producers

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

def parse_streams(streams, rules, stream_pddl, procedure_map, procedure_info):
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, pddl_name = next(stream_iter)
    assert('stream' == pddl_type)
    for lisp_list in stream_iter:
        name = lisp_list[0] # TODO: refactor at this point
        if name in (':stream', ':wild-stream'):
            externals = [parse_stream(lisp_list, procedure_map, procedure_info)]
        elif name == ':rule':
            externals = [parse_rule(lisp_list, procedure_map, procedure_info)]
        elif name == ':function':
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

def parse_stream_pddl(pddl_list, procedures, infos):
    streams = []
    if pddl_list is None:
        return streams
    if isinstance(pddl_list, str):
        pddl_list = [pddl_list]
    #if all(isinstance(e, External) for e in stream_pddl):
    #    return stream_pddl
    if procedures != DEBUG:
        procedures = {k.lower(): v for k, v in procedures.items()}
    infos = {k.lower(): v for k, v in infos.items()}
    rules = []
    for pddl in pddl_list:
        parse_streams(streams, rules, pddl, procedures, infos)
    apply_rules_to_streams(rules, streams)
    return streams

##################################################

def compile_fluent_streams(domain, externals):
    state_streams = list(filter(lambda e: isinstance(e, Stream) and
                                          (e.is_negated() or e.is_fluent()), externals))
    predicate_map = {}
    for stream in state_streams:
        for fact in stream.certified:
            predicate = get_prefix(fact)
            assert predicate not in predicate_map # TODO: could make a conjunction condition instead
            predicate_map[predicate] = stream
    if not predicate_map:
        return state_streams

    # TODO: could make free parameters free
    # TODO: allow functions on top the produced values?
    # TODO: check that generated values are not used in the effects of any actions
    # TODO: could treat like a normal stream that generates values (but with no inputs required/needed)
    def fn(literal):
        if literal.predicate not in predicate_map:
            return literal
        # TODO: other checks on only inputs
        stream = predicate_map[literal.predicate]
        certified = find_unique(lambda f: get_prefix(f) == literal.predicate, stream.certified)
        mapping = get_mapping(get_args(certified), literal.args)
        #assert all(arg in mapping for arg in stream.inputs) # Certified must contain all inputs
        if not all(arg in mapping for arg in stream.inputs):
            # TODO: this excludes typing. This is not entirely safe
            return literal
        blocked_args = tuple(mapping[arg] for arg in stream.inputs)
        blocked_literal = literal.__class__(stream.blocked_predicate, blocked_args).negate()
        if stream.is_negated():
            # TODO: add stream conditions here
            return blocked_literal
        return pddl.Conjunction([literal, blocked_literal])

    import pddl
    for action in domain.actions:
        action.precondition = replace_literals(fn, action.precondition).simplified()
        # TODO: throw an error if the effect would be altered
        for effect in action.effects:
            if not isinstance(effect.condition, pddl.Truth):
                raise NotImplementedError(effect.condition)
            #assert(isinstance(effect, pddl.Effect))
            #effect.condition = replace_literals(fn, effect.condition)
    for axiom in domain.axioms:
        axiom.condition = replace_literals(fn, axiom.condition).simplified()
    return state_streams


def dump_plans(stream_plan, action_plan, cost):
    print('Stream plan ({}, {:.1f}): {}\nAction plan ({}, {}): {}'.format(get_length(stream_plan),
                                                                          get_plan_effort(stream_plan),
                                                                          stream_plan,
                                                                          get_length(action_plan), cost,
                                                                          str_from_plan(action_plan)))


def partition_externals(externals):
    functions = list(filter(lambda s: type(s) is Function, externals))
    predicates = list(filter(lambda s: type(s) is Predicate, externals)) # and s.is_negative()
    negated_streams = list(filter(lambda s: (type(s) is Stream) and s.is_negated(), externals)) # and s.is_negative()
    negative = predicates + negated_streams
    streams = list(filter(lambda s: s not in (functions + negative), externals))
    #optimizers = list(filter(lambda s: type(s) in [VariableStream, ConstraintStream], externals))
    return streams, functions, negative #, optimizers
