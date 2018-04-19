from __future__ import print_function

from collections import namedtuple, defaultdict, deque
from itertools import product
from fast_downward import parse_lisp, parse_domain, run_fast_downward
from problem import Object

EQ = '=' # nxor
AND = 'and'
OR = 'or'
NOT = 'not'
EXISTS = 'exists'
FORALL = 'forall'
CONNECTIVES = (EQ, AND, OR, NOT)
QUANTIFIERS = (FORALL, EXISTS)
OPERATORS = CONNECTIVES + QUANTIFIERS

#TotalCost = 'total-cost'
TOTAL_COST = 'total-cost'
DOMAIN_NAME = 'pddlstream'
PROBLEM_NAME = DOMAIN_NAME

STREAM_ATTRIBUTES = [':stream', ':inputs', ':domain', ':outputs', ':certified']

Problem = namedtuple('Problem', ['init', 'goal', 'domain', 'streams', 'constants'])
Head = namedtuple('Head', ['function', 'args'])
Evaluation = namedtuple('Evaluation', ['head', 'value'])
Stream = namedtuple('Stream', ['name', 'gen_fn', 'inputs', 'domain', 'outputs', 'certified'])

class Stream(object):
    def __init__(self, name, gen_fn, inputs, domain, outputs, certified):
        self.name = name
        self.gen_fn = gen_fn
        self.inputs = tuple(inputs)
        self.domain = tuple(domain)
        self.outputs = tuple(outputs)
        self.certified = tuple(certified)
        self.instances = {}
    def get_instance(self, input_values):
        input_values = tuple(input_values)
        if input_values not in self.instances:
            self.instances[input_values] = StreamInstance(self, input_values)
        return self.instances[input_values]
    def __repr__(self):
        return '{}:{}->{}'.format(self.name, self.inputs, self.outputs)


class StreamInstance(object):
    def __init__(self, stream, input_values):
        self.stream = stream
        self.input_values = tuple(input_values)
        self._generator = None
        self.enumerated = False
        self.calls = 0
    def mapping(self, output_values=None):
        pairs = zip(self.stream.inputs, self.input_values)
        if output_values is not None:
            assert(len(self.stream.outputs) == len(output_values))
            pairs += zip(self.stream.outputs, output_values)
        return dict(pairs)
    def next_outputs(self):
        assert not self.enumerated
        if self._generator is None:
            self._generator = self.stream.gen_fn(*(iv.value for iv in self.input_values))
        self.calls += 1
        #if self.stream.max_calls <= self.calls:
        #    self.enumerated = True
        try:
            return next(self._generator)
        except StopIteration:
            self.enumerated = True
        return []
    def __repr__(self):
        return '{}:{}->{}'.format(self.stream.name, self.input_values, self.stream.outputs)


#CONSTANTS = ':constants'
# = ':objects'


#def constant(name):
#    return '{{{}}}'.format(name)
#    #return '{{{}}}'.format(name)


#class Atom(object):
#    pass


#def partition(array, i):
#    return array[:i], array[i:]


def convert_head(atom):
    name, args = atom[0], atom[1:]
    return tuple([name] + map(Object.from_value, args))


convert_atom = convert_head


def is_head(expression):
    prefix = expression[0]
    return prefix not in OPERATORS


def substitute_expression(parent, mapping):
    if isinstance(parent, str) or isinstance(parent, Object):
        return mapping.get(parent, parent)
    return tuple(substitute_expression(child, mapping) for child in parent)

def convert_expression(expression):
    prefix = expression[0]
    if prefix in CONNECTIVES:
        children = expression[1:]
        return tuple([prefix] + map(convert_expression, children))
    elif prefix in QUANTIFIERS:
        assert(len(expression) == 3)
        parameters = expression[1]
        child = expression[2]
        return prefix, parameters, convert_expression(child)
    return convert_atom(expression)


def pddl_from_object(obj):
    return obj.pddl


def pddl_from_expression(tree):
    if isinstance(tree, Object):
        return pddl_from_object(tree)
    if isinstance(tree, str):
        return tree
    return '({})'.format(' '.join(map(pddl_from_expression, tree)))


def pddl_from_evaluation(evaluation):
    head = (evaluation.head.function,) + tuple(evaluation.head.args)
    if is_atom(evaluation):
        return pddl_from_expression(head)
    if is_negated_atom(evaluation):
        return None
    expression = (EQ, head, str(evaluation.value))
    return pddl_from_expression(expression)


def pddl_from_evaluations(evaluations):
    return '\n\t\t'.join(sorted(filter(lambda s: s is not None,
                                       map(pddl_from_evaluation, evaluations))))


def pddl_from_objects(objects):
    return ' '.join(sorted(map(pddl_from_object, objects)))


def get_prefix(expression):
    return expression[0]

def get_args(head):
    return head[1:]

def evaluations_from_init(init):
    evaluations = []
    for fact in init:
        prefix = get_prefix(fact)
        if prefix == EQ:
            head, value = fact[1:]
        elif prefix == NOT:
            head = fact[1]
            value = False
        else:
            head = fact
            value = True
        func, args = get_prefix(head), get_args(head)
        head = Head(func.lower(), tuple(map(Object.from_value, args)))
        evaluations.append(Evaluation(head, value))
    return evaluations

def values_from_objects(objects):
    return tuple(obj.value for obj in objects)

def init_from_evaluations(evaluations):
    init = []
    for evaluation in evaluations:
        head = (evaluation.head.function, values_from_objects(evaluation.head.args))
        if is_atom(evaluation):
            init.append(head)
        elif is_negated_atom(evaluation):
            init.append((NOT, head))
        else:
            init.append((EQ, head, evaluation.value))
    return init

def is_atom(evaluation):
    return evaluation.value is True

def is_negated_atom(evaluation):
    return evaluation.value is False

def atoms_from_evaluations(evaluations):
    return map(lambda e: e.head, filter(is_atom, evaluations))

def objects_from_evaluations(evaluations):
    # TODO: assumes object predicates
    objects = set()
    for evaluation in evaluations:
        objects.update(evaluation.head.args)
    return objects

def get_pddl_problem(init_evaluations, goal_expression,
                     problem_name=DOMAIN_NAME, domain_name=PROBLEM_NAME, objective=(TOTAL_COST,)):
    # TODO: mako or some more elegant way of creating this
    objects = objects_from_evaluations(init_evaluations)
    s = '(define (problem {})\n' \
           '\t(:domain {})\n' \
           '\t(:objects {})\n' \
           '\t(:init {})\n' \
           '\t(:goal {})'.format(problem_name, domain_name,
                                 pddl_from_objects(objects),
                                 pddl_from_evaluations(init_evaluations),
                                 pddl_from_expression(goal_expression))
    if objective is not None:
        s += '\n\t(:metric minimize {})'.format(
            pddl_from_expression(objective))
    return s + ')\n'


def obj_from_pddl_plan(pddl_plan):
    if pddl_plan is None:
        return None
    return [(action, map(Object.from_name, args)) for action, args in pddl_plan]


def value_from_obj_plan(obj_plan):
    if obj_plan is None:
        return None
    return [(action, values_from_objects(args)) for action, args in obj_plan]


def list_from_conjunction(expression):
    if not expression:
        return []
    prefix = get_prefix(expression)
    assert(prefix not in (QUANTIFIERS + (NOT, OR, EQ)))
    if prefix == AND:
        children = []
        for child in expression[1:]:
            children += list_from_conjunction(child)
        return children
    return [tuple(expression)]

def parse_stream(stream_pddl, stream_map):
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, stream_name = next(stream_iter)
    assert('stream' == pddl_type)

    streams = []
    for stream in stream_iter:
        attributes = [stream[i] for i in range(0, len(stream), 2)]
        assert(STREAM_ATTRIBUTES == attributes)
        name, inputs, domain, outputs, certified = [stream[i] for i in range(1, len(stream), 2)]
        if name not in stream_map:
            raise ValueError('Undefined stream conditional generator: {}'.format(name))
        streams.append(Stream(name, stream_map[name],
                     tuple(inputs), list_from_conjunction(domain),
                     tuple(outputs), list_from_conjunction(certified)))
    return streams

def get_mapping(atoms1, atoms2, initial={}):
    # TODO unify this stuff
    assert len(atoms1) == len(atoms2)
    mapping = initial.copy()
    for a1, a2 in zip(atoms1, atoms2):
        assert(get_prefix(a1) == get_prefix(a2))
        for arg1, arg2 in zip(get_args(a1), a2[1]): # TODO: this is because eval vs predicate
            if mapping.get(arg1, arg2) == arg2:
                mapping[arg1] = arg2
            else:
                return None
    return mapping

class Instantiator(object): # Dynamic Stream Instantiator
    def __init__(self, evaluations, streams):
        # TODO: filter eager
        #self.streams_from_atom = defaultdict(list)
        self.streams = streams
        self.stream_instances = set()
        self.stream_queue = deque()
        self.atoms = set()
        self.atoms_from_domain = defaultdict(list)
        for stream in self.streams:
            if not stream.inputs:
                self._add_instance(stream.get_instance(tuple()))
        for atom in atoms_from_evaluations(evaluations):
            self.add_atom(atom)

    #def __next__(self):
    #    pass

    def __iter__(self):
        while self.stream_queue:
            stream_instance = self.stream_queue.popleft()
            # TODO: remove from set?
            yield stream_instance

    def _add_instance(self, stream_instance):
        if stream_instance in self.stream_instances:
            return False
        self.stream_instances.add(stream_instance)
        self.stream_queue.append(stream_instance)
        return True

    def add_atom(self, atom):
        if atom in self.atoms:
            return False
        self.atoms.add(atom)
        # TODO: doing this in a way that will eventually allow constants

        for i, stream in enumerate(self.streams):
            for j, domain_atom in enumerate(stream.domain):
                if get_prefix(atom) != get_prefix(domain_atom):
                    continue
                assert(len(get_args(atom)) == len(get_args(domain_atom)))
                if any(isinstance(b, Object) and (a != b) for (a, b) in
                       zip(get_args(atom), get_args(domain_atom))):
                    continue
                self.atoms_from_domain[(i, j)].append((stream, i))
                values = [self.atoms_from_domain[(i, k)] if j != k else [atom]
                          for k, a in enumerate(stream.domain)]
                for combo in product(*values):
                    mapping = get_mapping(stream.domain, combo)
                    if mapping is None:
                        continue
                    input_values = tuple(mapping[p] for p in stream.inputs)
                    self._add_instance(stream.get_instance(input_values))
        return True

##################################################

def solve_finite(problem, **kwargs):
    init, goal, domain_pddl, stream_pddl, stream_map, constant_map = problem
    evaluations = evaluations_from_init(init)
    goal_expression = convert_expression(goal)
    domain = parse_domain(domain_pddl)
    assert(len(domain.types) == 1)
    assert(not domain.constants)

    problem_pddl = get_pddl_problem(evaluations, goal_expression,
                                    domain_name=domain.name)
    plan = value_from_obj_plan(obj_from_pddl_plan(
        run_fast_downward(domain_pddl, problem_pddl)))
    return plan, init

##################################################

def solve_exhaustive(problem, **kwargs):
    init, goal, domain_pddl, stream_pddl, stream_map, constant_map = problem
    domain = parse_domain(domain_pddl)
    assert (len(domain.types) == 1)
    assert (not domain.constants)

    evaluations = evaluations_from_init(init)
    streams = parse_stream(stream_pddl, stream_map)
    print(streams)

    instantiator = Instantiator(evaluations, streams)
    for stream_instance in instantiator:
        output_values_list = stream_instance.next_outputs()
        print('{}:{}->{}'.format(stream_instance.stream.name,
                                 stream_instance.input_values, output_values_list))
        for output_values in output_values_list:
            converted_values = map(Object.from_value, output_values)
            certified_atoms = substitute_expression(stream_instance.stream.certified,
                                                    stream_instance.mapping(converted_values))
            for atom in certified_atoms:
                head = Head(get_prefix(atom), get_args(atom))
                # TODO: use an existing method here?
                evaluation = Evaluation(head, True)
                instantiator.add_atom(head)
                evaluations.append(evaluation)
        if not stream_instance.enumerated:
            instantiator.stream_queue.append(stream_instance)

    goal_expression = convert_expression(goal)
    problem_pddl = get_pddl_problem(evaluations, goal_expression,
                                    domain_name=domain.name)
    plan = value_from_obj_plan(obj_from_pddl_plan(
        run_fast_downward(domain_pddl, problem_pddl)))
    return plan, init_from_evaluations(evaluations)


# Basic functions for parsing PDDL (Lisp) files.

# @ # $ % [] {} <> || \/
# What if we have #p1 and #p11

# https://docs.python.org/3.4/library/string.html
# mako/templating?

#class Not(object):

# TODO: start by requiring that all objects have a substituion


#def pddl_from_head(head):
#    return pddl_from_expression((head.name,) + head.args)

#def And(*expressions):
#    return (AND,) + expressions

# TODO: I think we do want everything to be an object at the end of the day
