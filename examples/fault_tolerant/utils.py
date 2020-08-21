import os
import random

from pddlstream.algorithms.algorithm import get_predicates
from pddlstream.algorithms.downward import parse_sequential_domain, get_literals, get_precondition, get_fluents, \
    is_literal, is_assignment
from pddlstream.algorithms.scheduling.diverse import generic_union
from pddlstream.language.constants import Fact, Not, Equal, StreamAction
from pddlstream.utils import find_unique, get_mapping, hash_or_id


def list_paths(directory):
    if not os.path.exists(directory):
        return []
    return [os.path.abspath(os.path.join(directory, f)) for f in sorted(os.listdir(directory))]


def object_facts_from_str(s):
    objs, ty = s.strip().rsplit(' - ', 1)
    return [(ty, obj) for obj in objs.split(' ')]


def fact_from_str(s):
    return tuple(s.strip('( )').split(' '))


def int_from_str(s):
    return int(s.replace('number', ''))


def extract_static(domain_pddl, plans, probabilities):
    domain = parse_sequential_domain(domain_pddl)
    #static_predicates = get_static_predicates(domain)
    static_sets = []
    for plan in plans:
        # TODO: more general preimage
        preimage = set()
        for name, args in plan:
            action = find_unique(lambda a: a.name == name, domain.actions)
            mapping = get_mapping([param.name for param in action.parameters], args)
            for literal in get_literals(get_precondition(action)):
                if literal.predicate in probabilities: #(literal.predicate in static_predicates:
                    preimage.add(literal.rename_variables(mapping))
        static_sets.append(frozenset(preimage))
    return static_sets


def get_static_predicates(domain):
    fluent_predicates = get_fluents(domain)
    assert not domain.axioms  # TODO: axioms
    predicates = {predicate for action in domain.actions
                  for predicate in get_predicates(get_precondition(action))}  # get_literals
    return predicates - fluent_predicates


class hashabledict(dict):
    def __setitem__(self, key, value):
        raise RuntimeError()
    # assumes immutable once hashed
    def __hash__(self):
        return hash(frozenset(self.items()))


def fact_from_fd(literal):
    if is_literal(literal):
        atom = Fact(literal.predicate, literal.args)
        if literal.negated:
            return Not(atom)
        return atom
    if is_assignment(literal):
        func = (literal.fluent.symbol,) + literal.fluent.args
        return Equal(func, literal.expression.value)
    raise NotImplementedError(literal)


def extract_streams(plan):
    return frozenset(stream for stream in plan if isinstance(stream, StreamAction))


def simulate_successes(stochastic_fns, solutions, n_simulations):
    successes = 0
    plans = [plan for plan, _, _ in solutions]
    if not plans:
        return successes
    for _ in range(n_simulations):
        # TODO: compare with exact computation from p_disjunction
        outcomes = {}
        for stream in generic_union(*map(extract_streams, plans)):
            name, inputs, outputs = stream
            assert not outputs
            outcomes[stream] = stochastic_fns[name](*inputs)
            # total = sum(stochastic_fns[name](*inputs) for _ in range(n_simulations))
            # print(float(total) / n_simulations)
        for plan in plans:
            if all(outcomes[stream] for stream in extract_streams(plan)):
                successes += 1
                break
    return successes


def test_from_bernoulli_fn(bernoulli_fn):
    return lambda *args, **kwargs: random.random() < bernoulli_fn(*args, **kwargs)


class CachedFn(object):
    def __init__(self, fn):
        self.fn = fn
        self.cache = {}
    def __call__(self, *args): #, **kwargs):
        key = tuple(map(hash_or_id, args)) # Add the type
        if key not in self.cache:
            self.cache[key] = self.fn(*args)
        return self.cache[key]
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.fn)