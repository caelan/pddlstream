import os

from pddlstream.algorithms.downward import get_literals, get_conjunctive_parts, pddl_from_instance
from pddlstream.language.conversion import obj_from_pddl
from pddlstream.language.fluent import get_predicate_map, remap_certified
from pddlstream.language.stream import Stream
from pddlstream.utils import INF

PYPLANNERS_VAR = 'PYPLANNERS_PATH'


def get_pyplanners_path():
    #raise RuntimeError('Environment variable {} is not defined.'.format(ENV_VAR))
    return os.environ.get(PYPLANNERS_VAR, None)


def has_attachments(domain):
    return any(getattr(action, 'attachments', {}) for action in domain.actions)


def compile_fluents_as_attachments(domain, externals):
    import pddl
    state_streams = set(filter(lambda e: isinstance(e, Stream) and e.is_fluent(), externals)) # is_special
    if not state_streams:
        return externals
    predicate_map = get_predicate_map(state_streams)
    if predicate_map and (get_pyplanners_path() is None):
        raise NotImplementedError('Algorithm does not support fluent streams: {}'.format(
            [stream.name for stream in predicate_map.values()]))
    for action in domain.actions:
        for effect in action.effects:
            # TODO: conditional effects
            if any(literal.predicate in predicate_map for literal in get_literals(effect.condition)):
                raise ValueError(effect)
        action.attachments = {}
        preconditions = []
        for literal in get_conjunctive_parts(action.precondition):
            if not isinstance(literal, pddl.Literal):
                raise NotImplementedError(literal)
            if literal.predicate in predicate_map:
                stream = predicate_map[literal.predicate]
                if not stream.is_test():
                    raise NotImplementedError(stream)
                assert remap_certified(literal, stream) is not None
                action.attachments[literal] = stream
            else:
                preconditions.append(literal)
        action.precondition = pddl.Conjunction(preconditions).simplified()
        #fn = lambda l: pddl.Truth() if l.predicate in predicate_map else l
        #action.precondition = replace_literals(fn, action.precondition).simplified()
        #action.dump()
    return [external for external in externals if external not in state_streams]

##################################################

def get_attachment_test(action_instance):
    from pddlstream.algorithms.scheduling.apply_fluents import get_fluent_instance
    from pddlstream.language.fluent import remap_certified
    # TODO: ensure no OptimisticObjects
    def test(state):
        for literal, stream in action_instance.action.attachments.items():
            param_from_inp = remap_certified(literal, stream)
            input_objects = tuple(obj_from_pddl(
                action_instance.var_mapping[param_from_inp[inp]]) for inp in stream.inputs)
            stream_instance = get_fluent_instance(stream, input_objects, state)
            failure = not stream_instance.all_results()
            if literal.negated != failure:
                return False
        return True
    return test


def solve_pyplanners(instantiated):
    # TODO: could operate on SAS instead
    # https://github.com/caelan/pyplanners
    # https://github.mit.edu/caelan/stripstream/blob/c8c6cd1d6bd5e2e8e31cd5603e28a8e0d7bb2cdc/stripstream/algorithms/search/pyplanners.py
    import sys
    pyplanners_path = get_pyplanners_path()
    if pyplanners_path not in sys.path:
        sys.path.append(pyplanners_path)
    from strips.states import State, PartialState
    from strips.operators import Action, Axiom
    from strips.utils import default_derived_plan
    import pddl

    if instantiated is None:
        return None, INF
    py_actions = []
    for action in instantiated.actions:
        py_action = Action({'fd_action': action})
        py_action.conditions = set(action.precondition)
        py_action.effects = set()
        for condition, effect in action.del_effects:
            assert not condition
            py_action.effects.add(effect.negate())
        for condition, effect in action.add_effects:
            assert not condition
            py_action.effects.add(effect)
        py_action.cost = action.cost
        py_action.test = get_attachment_test(action)
        py_actions.append(py_action)
    py_axioms = []
    for axiom in instantiated.axioms:
        py_axiom = Axiom({'fd_axiom_id': id(axiom)}) # Not hashable for some reason
        py_axiom.conditions = set(axiom.condition)
        py_axiom.effects = {axiom.effect}
        py_axioms.append(py_axiom)
    goal = PartialState(instantiated.goal_list)

    fluents = {f.positive() for f in goal.conditions}
    for py_operator in py_actions + py_axioms:
        fluents.update(f.positive() for f in py_operator.conditions)

    initial = State(atom for atom in instantiated.task.init
                    if isinstance(atom, pddl.Atom) and (atom in fluents))
    plan, state_space = default_derived_plan(initial, goal, py_actions, py_axioms)
    if plan is None:
        return None, INF
    actions = [pddl_from_instance(action.fd_action) for action in plan]
    return actions, plan.cost