import os
import sys

from pddlstream.algorithms.algorithm import get_predicates
from pddlstream.algorithms.downward import get_literals, get_conjunctive_parts, fd_from_fact, EQ, make_object, \
    pddl_from_instance, DEFAULT_MAX_TIME
from pddlstream.language.object import Object
from pddlstream.language.conversion import obj_from_pddl, substitute_fact
from pddlstream.language.fluent import get_predicate_map, remap_certified
from pddlstream.language.stream import Stream
from pddlstream.utils import INF, invert_dict, get_mapping

# Intuition: static facts about whether this state satisfies a condition
# The state can be seen as a hidden parameter with a precondition that you are at it

PYPLANNERS_VAR = 'PYPLANNERS_PATH'
PLACEHOLDER = Object.from_value('~')


def get_pyplanners_path():
    return os.environ.get(PYPLANNERS_VAR, None)


def has_attachments(domain):
    return any(getattr(action, 'attachments', {}) for action in domain.actions)


def compile_fluents_as_attachments(domain, externals):
    import pddl
    state_streams = set(filter(lambda e: isinstance(e, Stream) and e.is_fluent, externals)) # is_special
    if not state_streams:
        return externals

    predicate_map = get_predicate_map(state_streams)
    if predicate_map and (get_pyplanners_path() is None):
        # TODO: fluent streams with outputs
        # Could convert the free parameter to a constant
        raise NotImplementedError('Algorithm does not support fluent streams: {}'.format(
            [stream.name for stream in state_streams]))

    domain.constants.append(make_object(PLACEHOLDER.pddl))
    for action in domain.actions:
        for effect in action.effects:
            # TODO: conditional effects
            if any(literal.predicate in predicate_map for literal in get_literals(effect.condition)):
                raise ValueError('Attachments cannot be in action effects: {}'.format(effect))
        action.attachments = {}
        preconditions = set()
        for literal in get_conjunctive_parts(action.precondition):
            #if not isinstance(literal, pddl.Literal):
            #    raise NotImplementedError('Only literals are supported: {}'.format(literal))
            if not get_predicates(literal) & set(predicate_map):
                preconditions.add(literal)
                continue
            if not isinstance(literal, pddl.Literal):
                raise NotImplementedError(literal)
            # Drops the original precondition
            stream = predicate_map[literal.predicate]
            mapping = remap_certified(literal, stream)
            assert mapping is not None
            action.attachments[literal] = stream
            preconditions.update(pddl.Atom(EQ, (mapping[out], PLACEHOLDER.pddl))
                                 for out in stream.outputs)
            preconditions.update(fd_from_fact(substitute_fact(fact, mapping))
                                 for fact in stream.domain)
        action.precondition = pddl.Conjunction(preconditions).simplified()
        #fn = lambda l: pddl.Truth() if l.predicate in predicate_map else l
        #action.precondition = replace_literals(fn, action.precondition).simplified()
        #action.dump()
    return [external for external in externals if external not in state_streams]

##################################################

def get_attachment_test(action_instance):
    from pddlstream.algorithms.scheduling.apply_fluents import get_fluent_instance
    from pddlstream.language.fluent import remap_certified
    # TODO: support for focused (need to resolve after binding)
    # TODO: ensure no OptimisticObjects

    def test(state):
        # TODO: action parameters don't depend on the state
        # TODO: counter for how many times the action has been remapped
        if not hasattr(action_instance.action, 'attachments'):
            return True
        for literal, stream in action_instance.action.attachments.items():
            param_from_inp = remap_certified(literal, stream)
            input_objects = tuple(obj_from_pddl(
                action_instance.var_mapping[param_from_inp[inp]]) for inp in stream.inputs)
            stream_instance = get_fluent_instance(stream, input_objects, state)  # Output automatically cached
            results = stream_instance.first_results(num=1)
            #results = stream_instance.all_results()
            failure = not results
            if literal.negated != failure:
                return False
            #args = action_instance.name.strip('()').split(' ')
            #idx_from_param = {p.name: i for i, p in enumerate(action_instance.action.parameters)}
            param_from_out = remap_certified(literal, stream)
            result = results[0] # Arbitrary
            out_from_obj = invert_dict(result.mapping)
            for obj in result.output_objects:
                param = param_from_out[out_from_obj[obj]]
                action_instance.var_mapping[param] = obj.pddl
                # idx = idx_from_param[param]
                # args[1+idx] = obj.pddl
            #action_instance.name = '({})'.format(' '.join(args))
        return True
    return test


def solve_pyplanners(instantiated, planner=None, max_planner_time=DEFAULT_MAX_TIME, max_cost=INF):
    if instantiated is None:
        return None, INF

    # https://github.mit.edu/caelan/stripstream/blob/c8c6cd1d6bd5e2e8e31cd5603e28a8e0d7bb2cdc/stripstream/algorithms/search/pyplanners.py
    pyplanners_path = get_pyplanners_path()
    if pyplanners_path is None:
        raise RuntimeError('Must clone https://github.com/caelan/pyplanners '
                           'and set the environment variable {} to its path'.format(PYPLANNERS_VAR))
    if pyplanners_path not in sys.path:
        sys.path.append(pyplanners_path)

    # TODO: could operate on translated SAS instead
    from strips.states import State, PartialState
    from strips.operators import Action, Axiom
    from strips.utils import solve_strips, default_derived_plan
    import pddl

    default_planner = {
        'search': 'eager',
        'evaluator': 'greedy',
        'heuristic': 'ff',
        'successors': 'all',
    }
    if isinstance(planner, dict):
        default_planner.update(planner)

    py_actions = []
    for action in instantiated.actions:
        #action.dump()
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
        #axiom.dump()
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

    plan, state_space = solve_strips(initial, goal, py_actions, py_axioms,
                                     max_time=max_planner_time, max_cost=max_cost, **default_planner)
    if plan is None:
        return None, INF
    actions = [pddl_from_instance(action.fd_action) for action in plan]
    #print(actions)
    return actions, plan.cost
