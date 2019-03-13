import os
from collections import namedtuple, defaultdict
from time import time

from pddlstream.algorithms.downward import get_literals, get_precondition, get_fluents, get_function_assignments, \
    TRANSLATE_OUTPUT, parse_domain, parse_problem, task_from_domain_problem, GOAL_NAME
from pddlstream.algorithms.relation import Relation, compute_order, solve_satisfaction
from pddlstream.language.constants import is_parameter
from pddlstream.utils import flatten, apply_mapping, MockSet, elapsed_time, clear_dir, Verbose

import pddl
import instantiate
import translate
import normalize

InstantiatedTask = namedtuple('InstantiatedTask', ['task', 'atoms', 'actions', 'axioms',
                                                   'reachable_action_params', 'goal_list'])


def instantiate_goal(goal):
    # HACK! Goals should be treated differently.
    if isinstance(goal, pddl.Conjunction):
        goal_list = goal.parts
    else:
        goal_list = [goal]
    for item in goal_list:
        assert isinstance(item, pddl.Literal)
    return goal_list

##################################################

def get_goal_instance(goal):
    return pddl.PropositionalAction(GOAL_NAME, instantiate_goal(goal), [], None)


def instantiate_atoms(atoms_from_cond):
    conditions, atoms = zip(*atoms_from_cond.items())
    relations = [Relation(conditions[index].args, atoms[index])
                 for index in compute_order(conditions, atoms)]
    solution = solve_satisfaction(relations)
    for element in solution.body:
        yield solution.get_mapping(element)


def instantiate_condition(action, is_static, args_from_predicate):
    static_conditions = list(filter(is_static, get_literals(get_precondition(action))))
    static_objects = set(flatten(atom.args for atom in static_conditions))
    static_parameters = set(filter(is_parameter, static_objects))
    static_constants = static_objects - static_parameters
    if static_constants:
        raise NotImplementedError(static_constants)
    parameters = {p.name for p in action.parameters}
    if not (parameters <= static_parameters):
        raise NotImplementedError(parameters)
    atoms_from_cond = {condition: args_from_predicate[condition.predicate]
                       for condition in static_conditions}
    return instantiate_atoms(atoms_from_cond)


def get_reachable_action_params(instantiated_actions):
    reachable_action_params = defaultdict(list)
    for inst_action in instantiated_actions:
        action = inst_action.action
        parameters = [p.name for p in action.parameters]
        args = apply_mapping(parameters, inst_action.var_mapping)
        reachable_action_params[action].append(args) # TODO: does this actually do anything
    return reachable_action_params


def instantiate_domain(task):
    fluent_predicates = get_fluents(task)
    is_static = lambda a: isinstance(a, pddl.Atom) and (a.predicate not in fluent_predicates)

    fluent_facts = MockSet(lambda a: not is_static(a))
    init_facts = set(task.init)
    function_assignments = get_function_assignments(task)
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)

    predicate_to_atoms = defaultdict(set)
    args_from_predicate = defaultdict(set)
    for atom in filter(is_static, task.init):  # TODO: compute which predicates might involve constants
        predicate_to_atoms[atom.predicate].add(atom)
        args_from_predicate[atom.predicate].add(atom.args)

    instantiated_actions = []
    for action in task.actions:
        for variable_mapping in instantiate_condition(action, is_static,
                                                      args_from_predicate):
            inst_action = action.instantiate(variable_mapping, init_facts, fluent_facts, type_to_objects,
                                             task.use_min_cost_metric, function_assignments, predicate_to_atoms)
            if inst_action:
                instantiated_actions.append(inst_action)
    instantiated_axioms = []
    for axiom in task.axioms:
        for variable_mapping in instantiate_condition(axiom, is_static,
                                                      args_from_predicate):
            inst_axiom = axiom.instantiate(variable_mapping, init_facts, fluent_facts)
            if inst_axiom:
                instantiated_axioms.append(inst_axiom)
    return instantiated_actions, instantiated_axioms


def instantiate_task(task):
    start_time = time()
    print()
    normalize.normalize(task)
    relaxed_reachable, atoms, actions, axioms, reachable_action_params = instantiate.explore(task)
    if not relaxed_reachable:
        return None
    #instantiated_actions, instantiated_axioms = instantiate_domain(task)
    #reachable_action_params = get_reachable_action_params(instantiated_actions)
    #print(len(actions), len(instantiated_actions))
    #print(len(axioms), len(axioms))
    goal_list = instantiate_goal(task.goal)
    print('Instantiation time:', elapsed_time(start_time))
    return InstantiatedTask(task, atoms, actions, axioms, reachable_action_params, goal_list)

##################################################

def sas_from_instantiated(instantiated_task):
    import timers
    import fact_groups
    import options
    import simplify
    import variable_order
    from translate import translate_task, unsolvable_sas_task, strips_to_sas_dictionary, \
        build_implied_facts, build_mutex_key, solvable_sas_task
    start_time = time()
    print()

    if not instantiated_task:
        return unsolvable_sas_task("No relaxed solution")
    task, atoms, actions, axioms, reachable_action_params, goal_list = instantiated_task

    with timers.timing("Computing fact groups", block=True):
        groups, mutex_groups, translation_key = fact_groups.compute_groups(
            task, atoms, reachable_action_params)

    with timers.timing("Building STRIPS to SAS dictionary"):
        ranges, strips_to_sas = strips_to_sas_dictionary(
            groups, assert_partial=options.use_partial_encoding)

    with timers.timing("Building dictionary for full mutex groups"):
        mutex_ranges, mutex_dict = strips_to_sas_dictionary(
            mutex_groups, assert_partial=False)

    if options.add_implied_preconditions:
        with timers.timing("Building implied facts dictionary..."):
            implied_facts = build_implied_facts(strips_to_sas, groups,
                                                mutex_groups)
    else:
        implied_facts = {}

    with timers.timing("Building mutex information", block=True):
        mutex_key = build_mutex_key(strips_to_sas, mutex_groups)

    with timers.timing("Translating task", block=True):
        sas_task = translate_task(
            strips_to_sas, ranges, translation_key,
            mutex_dict, mutex_ranges, mutex_key,
            task.init, goal_list, actions, axioms, task.use_min_cost_metric,
            implied_facts)

    if options.filter_unreachable_facts:
        with timers.timing("Detecting unreachable propositions", block=True):
            try:
                simplify.filter_unreachable_propositions(sas_task)
            except simplify.Impossible:
                return unsolvable_sas_task("Simplified to trivially false goal")
            except simplify.TriviallySolvable:
                return solvable_sas_task("Simplified to empty goal")

    if options.reorder_variables or options.filter_unimportant_vars:
        with timers.timing("Reordering and filtering variables", block=True):
            variable_order.find_and_apply_variable_order(
                sas_task, options.reorder_variables,
                options.filter_unimportant_vars)

    translate.dump_statistics(sas_task)
    print('Translation time:', elapsed_time(start_time))
    return sas_task

##################################################

def write_sas_task(sas_task, temp_dir):
    clear_dir(temp_dir)
    translate_path = os.path.join(temp_dir, TRANSLATE_OUTPUT)
    with open(os.path.join(temp_dir, TRANSLATE_OUTPUT), "w") as output_file:
        sas_task.output(output_file)
    return translate_path


def sas_from_pddl(task, debug=False):
    #normalize.normalize(task)
    #sas_task = translate.pddl_to_sas(task)
    with Verbose(debug):
        instantiated = instantiate_task(task)
        sas_task = sas_from_instantiated(instantiated)
        sas_task.metric = task.use_min_cost_metric # TODO: are these sometimes not equal?
    return sas_task


def translate_and_write_pddl(domain_pddl, problem_pddl, temp_dir, verbose):
    domain = parse_domain(domain_pddl)
    problem = parse_problem(domain, problem_pddl)
    task = task_from_domain_problem(domain, problem)
    sas_task = sas_from_pddl(task)
    write_sas_task(sas_task, temp_dir)
    return task
