import time
from collections import OrderedDict, defaultdict

from pddlstream.algorithms.downward import parse_domain, get_problem, task_from_domain_problem, \
    solve_from_task, parse_lisp
from pddlstream.language.exogenous import compile_to_exogenous
from pddlstream.language.conversion import evaluations_from_init, obj_from_value_expression, obj_from_pddl_plan, \
    evaluation_from_fact, get_prefix
from pddlstream.language.external import External, DEBUG
from pddlstream.language.function import parse_function, parse_predicate, Function, Predicate
from pddlstream.language.object import Object
from pddlstream.language.stream import parse_stream, Stream
from pddlstream.language.state_stream import parse_state_stream, StateStream
from pddlstream.utils import elapsed_time, INF

# TODO: way of programmatically specifying streams/actions

def parse_constants(domain, constant_map):
    for constant in domain.constants:
        if constant.name.startswith(Object._prefix):
            # TODO: remap names
            raise NotImplementedError('Constants are not currently allowed to begin with {}'.format(Object._prefix))
        if constant.name not in constant_map:
            raise ValueError('Undefined constant {}'.format(constant.name))
        value = constant_map.get(constant.name, constant.name)
        obj = Object(value, name=constant.name)
        # TODO: add object predicate
    for name in constant_map:
        for constant in domain.constants:
            if constant.name == name:
                break
        else:
            raise ValueError('Constant map {} not mentioned in domain'.format(name))
    del domain.constants[:] # So not set twice

INITIAL_EVALUATION = None

def parse_problem(problem, stream_info={}):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = problem
    domain = parse_domain(domain_pddl)
    if len(domain.types) != 1:
        raise NotImplementedError('Types are not currently supported')
    parse_constants(domain, constant_map)
    stream_name, streams = parse_stream_pddl(stream_pddl, stream_map, stream_info)
    evaluations = OrderedDict((e, INITIAL_EVALUATION) for e in evaluations_from_init(init))
    goal_expression = obj_from_value_expression(goal)
    compile_to_exogenous(evaluations, domain, streams)
    return evaluations, goal_expression, domain, stream_name, streams

##################################################

def has_costs(domain):
    for action in domain.actions:
        if action.cost is not None:
            return True
    return False

def solve_finite(evaluations, goal_expression, domain, unit_costs=None, **kwargs):
    if unit_costs is None:
        unit_costs = not has_costs(domain)
    problem = get_problem(evaluations, goal_expression, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    plan_pddl, cost = solve_from_task(task, **kwargs)
    return obj_from_pddl_plan(plan_pddl), cost


def neighbors_from_orders(orders):
    incoming_edges = defaultdict(set)
    outgoing_edges = defaultdict(set)
    for v1, v2 in orders:
        incoming_edges[v2].add(v1)
        outgoing_edges[v1].add(v2)
    return incoming_edges, outgoing_edges

##################################################

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
    def add_plan(self, plan, cost):
        # TODO: list of plans
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


def add_certified(evaluations, result):
    new_evaluations = []
    for fact in result.get_certified():
        evaluation = evaluation_from_fact(fact)
        if evaluation not in evaluations:
            evaluations[evaluation] = result
            new_evaluations.append(evaluation)
    return new_evaluations

##################################################

def get_domain_predicates(external):
    return set(map(get_prefix, external.domain))

def get_certified_predicates(external):
    if type(external) in (Stream, StateStream):
        return set(map(get_prefix, external.certified))
    if type(external) in (Function, Predicate):
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

def parse_stream_pddl(stream_pddl, stream_map, stream_info):
    streams = []
    if stream_pddl is None:
        return None, streams
    if all(isinstance(e, External) for e in stream_pddl):
        return None, stream_pddl
    if stream_map != DEBUG:
        stream_map = {k.lower(): v for k, v in stream_map.items()}
    stream_info = {k.lower(): v for k, v in stream_info.items()}
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, stream_name = next(stream_iter)
    assert('stream' == pddl_type)

    for lisp_list in stream_iter:
        name = lisp_list[0]
        if name == ':stream':
            external = parse_stream(lisp_list, stream_map, stream_info)
        elif name == ':state-stream':
            external = parse_state_stream(lisp_list, stream_map, stream_info)
        elif name == ':wild':
            raise NotImplementedError(name)
        elif name == ':rule':
            # TODO: implement rules
            # TODO: add eager stream if multiple conditions otherwise apply and add to stream effects
            raise NotImplementedError(name)
        elif name == ':function':
            external = parse_function(lisp_list, stream_map, stream_info)
        elif name == ':predicate': # Cannot just use args if want a bound
            external = parse_predicate(lisp_list, stream_map, stream_info)
        else:
            raise ValueError(name)
        if any(e.name == external.name for e in streams):
            raise ValueError('Stream [{}] is not unique'.format(external.name))
        streams.append(external)
    return stream_name, streams