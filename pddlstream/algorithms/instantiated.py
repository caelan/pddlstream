from pddlstream.algorithms.downward import get_conditional_effects, get_precondition, is_applicable, \
    apply_axiom, apply_action
from pddlstream.algorithms.instantiate_task import sas_from_instantiated
from pddlstream.algorithms.scheduling.plan_streams import RENAME_ACTIONS, rename_instantiated_actions, \
    get_plan_cost
from pddlstream.algorithms.search import solve_from_task
from pddlstream.retired.successor_generator import get_fluents
from pddlstream.utils import INF, Verbose


def sanitize_state(state):
    import pddl
    return {atom for atom in state if isinstance(atom, pddl.Atom) and not atom.negated}


def get_initial_state(task):
    return sanitize_state(task.init)


def compute_conditions(instantiated):
    conditions = set(instantiated.goal_list)
    for operator in instantiated.axioms + instantiated.actions:
        conditions.update(get_precondition(operator))
        for conditionals, _ in get_conditional_effects(operator):
            conditions.update(conditionals)
    return conditions # Signed


def compute_fluents(instantiated):
    #from pddlstream.algorithms.instantiate_task import get_goal_instance
    initial = get_initial_state(instantiated.task)
    return get_fluents(initial, instantiated.actions)


def compute_derived(instantiated):
    return {axiom.effect.positive() for axiom in instantiated.axioms}

################################################################################

def reachable_literals(instantiated):
    literals = set(instantiated.task.init) # TODO: return initially negated fluent
    for axiom in instantiated.axioms: # TODO: assumes added in order
        literals.add(axiom.effect)
    for action in instantiated.actions:
        for conditions, effect in get_conditional_effects(action):
            #if conditions_hold(state, conditions): # TODO: conditional effects
            literals.add(effect)
    return literals


def derive_axioms(state, axioms):
    # TODO: more efficient version that hashes conditions
    derived_state = set(state)
    applied_indices = set()
    while True:
        for idx, axiom in enumerate(axioms):
            if (idx not in applied_indices) and is_applicable(derived_state, axiom):
                apply_axiom(derived_state, axiom)
                applied_indices.add(idx)
                break
        else:
            break
    return derived_state


def applicable_actions(state, axioms, actions):
    derived_state = derive_axioms(state, axioms)
    for action in actions:
        if is_applicable(derived_state, action):
            yield action


def apply_plan(initial_state, plan): #, axioms=[]):
    state = set(initial_state)
    for action in plan:
        #assert is_applicable(state, action)
        apply_action(state, action)
    return state
    # if not axioms:
    #     return state


def state_difference(state2, state1):
    return [atom.negate() for atom in state1 if atom not in state2] + \
           [atom for atom in state2 if atom not in state1]

################################################################################

def solve_instantiated(instantiated, rename=RENAME_ACTIONS, debug=False, **kwargs):
    #from pddlstream.algorithms.scheduling.plan_streams import solve_optimistic_sequential
    plan, cost = None, INF
    if instantiated is None:
        return plan, cost

    original_actions = instantiated.actions[:]
    cost_from_action = {action: action.cost for action in original_actions} # TODO: costs are transformed
    action_from_name = rename_instantiated_actions(instantiated, rename) # rename=RENAME_ACTIONS
    # TODO: the action unsatisfiable conditions are pruned
    with Verbose(debug):
        sas_task = sas_from_instantiated(instantiated)
        # sas_task.metric = task.use_min_cost_metric
        sas_task.metric = True
    instantiated.actions[:] = original_actions

    # TODO: apply renaming to hierarchy as well
    # solve_from_task | serialized_solve_from_task | abstrips_solve_from_task | abstrips_solve_from_task_sequential
    renamed_plan, _ = solve_from_task(sas_task, debug=debug, **kwargs)
    if renamed_plan is None:
        return plan, cost

    plan = [action_from_name[name if rename else '({} {})'.format(name, ' '.join(args))]
            for name, args in renamed_plan]
    cost = get_plan_cost(plan, cost_from_action)
    return plan, cost
