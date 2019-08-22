import time
from collections import Iterator, namedtuple, deque
from itertools import count

from pddlstream.utils import INF, elapsed_time

# TODO: indicate wild stream output just from the output form
# TODO: depth limited and cycle-free optimistic objects

class BoundedGenerator(Iterator):
    """
    A generator with a fixed length.
    The generator tracks its number of calls, allowing it to terminate with one fewer call
    """
    def __init__(self, generator, max_calls=INF):
        self.generator = generator
        self.max_calls = max_calls
        self.history = []
    @property
    def calls(self):
        return len(self.history)
    @property
    def enumerated(self):
        return self.max_calls <= self.calls
    def next(self):
        if self.enumerated:
            raise StopIteration()
        self.history.append(next(self.generator))
        return self.history[-1]
    __next__ = next


def get_next(generator, default=[]):
    new_values = default
    enumerated = False
    try:
        new_values = next(generator)
    except StopIteration:
        enumerated = True
    if isinstance(generator, BoundedGenerator):
        enumerated |= generator.enumerated
    return new_values, enumerated

##################################################

# Methods that convert some procedure -> function to a generator of lists

def from_list_gen_fn(list_gen_fn):
    # Purposefully redundant for now
    return list_gen_fn


def from_gen_fn(gen_fn):
    return from_list_gen_fn(lambda *args, **kwargs: ([] if ov is None else [ov]
                                                     for ov in gen_fn(*args, **kwargs)))


def from_sampler(sampler, max_attempts=INF):
    def gen_fn(*input_values):
        attempts = count()
        while next(attempts) < max_attempts:
            yield sampler(*input_values)
    return from_gen_fn(gen_fn)

##################################################

# Methods that convert some procedure -> function to a BoundedGenerator

def from_list_fn(list_fn):
    #return lambda *args, **kwargs: iter([list_fn(*args, **kwargs)])
    return lambda *args, **kwargs: BoundedGenerator(iter([list_fn(*args, **kwargs)]), max_calls=1)


def from_fn(fn):
    def list_fn(*args, **kwargs):
        outputs = fn(*args, **kwargs)
        return [] if outputs is None else [outputs]
    return from_list_fn(list_fn)

def outputs_from_boolean(boolean):
    return tuple() if boolean else None


def from_test(test):
    return from_fn(lambda *args, **kwargs: outputs_from_boolean(test(*args, **kwargs)))


def from_constant(constant):
    return from_fn(fn_from_constant(constant))


def negate_test(test):
    return lambda *args, **kwargs: not test(*args, **kwargs)


def empty_gen():
    return lambda *args, **kwargs: iter([])
#empty_gen = lambda *args, **kwargs: iter([])

##################################################

# Methods that convert some procedure -> function

def fn_from_constant(constant):
    return lambda *args, **kwargs: constant

universe_test = fn_from_constant(True)
empty_test = fn_from_constant(False)

##################################################

def accelerate_list_gen_fn(list_gen_fn, num_elements=1, max_attempts=1, max_time=INF):
    """
    Accelerates a list_gen_fn by eagerly generating num_elements at a time if possible
    """
    def new_list_gen_fn(*inputs):
        generator = list_gen_fn(*inputs)
        terminated = False
        while not terminated:
            start_time = time.time()
            elements = []
            for i in range(max_attempts):
                if terminated or (num_elements <= len(elements)) or (max_time <= elapsed_time(start_time)):
                    break
                new_elements, terminated = get_next(generator)
                elements.extend(new_elements)
            yield elements
    return new_list_gen_fn

##################################################

Composed = namedtuple('Composed', ['outputs', 'step', 'generator'])


def compose_gen_fns(*gen_fns):
    assert gen_fns
    # Assumes consistent ordering of inputs/outputs
    # Samplers are a special case where only the first needs to be a generator
    # TODO: specify info about what to compose
    # TODO: alternatively, make a new stream that composes several
    def gen_fn(*inputs):
        queue = deque([Composed([], 0, gen_fns[0](*inputs))])
        while queue:
            composed = queue.popleft()
            new_outputs_list, terminated = get_next(composed.generator)
            for new_outputs in new_outputs_list:
                outputs = composed.outputs + new_outputs
                if composed.step == (len(gen_fns) - 1):
                    yield outputs
                else:
                    next_step = composed.step + 1
                    generator = gen_fns[next_step](*(inputs + composed.output_values))
                    queue.append(Composed(outputs, next_step, generator))
            if not new_outputs_list:
                yield None
            if not terminated:
                queue.append(composed)
    return gen_fn


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