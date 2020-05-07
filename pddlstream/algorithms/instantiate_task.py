from __future__ import print_function

import os
from collections import namedtuple, defaultdict, deque
from time import time

from pddlstream.algorithms.downward import get_literals, get_precondition, get_fluents, get_function_assignments, \
    TRANSLATE_OUTPUT, parse_sequential_domain, parse_problem, task_from_domain_problem, GOAL_NAME, literal_holds, \
    get_conjunctive_parts, get_conditional_effects
from pddlstream.algorithms.relation import Relation, compute_order, solve_satisfaction
from pddlstream.language.constants import is_parameter
from pddlstream.utils import flatten, apply_mapping, MockSet, elapsed_time, Verbose, safe_remove, ensure_dir

import pddl
import instantiate
import translate
import normalize

FD_INSTANTIATE = True

InstantiatedTask = namedtuple('InstantiatedTask', ['task', 'atoms', 'actions', 'axioms',
                                                   'reachable_action_params', 'goal_list'])


def instantiate_goal(goal):
    goal_list = get_conjunctive_parts(goal)
    assert all(isinstance(item, pddl.Literal) for item in goal_list)
    return goal_list


def get_goal_instance(goal):
    return pddl.PropositionalAction(GOAL_NAME, instantiate_goal(goal), [], None)

##################################################

def get_constants(atom):
    return tuple((i, a) for i, a in enumerate(atom.args) if not is_parameter(a))

def instantiate_condition(action, is_static, args_from_predicate):
    parameters = {p.name for p in action.parameters}
    #if not parameters:
    #    yield {}
    #    return
    static_conditions = list(filter(is_static, get_literals(get_precondition(action))))
    static_parameters = set(filter(is_parameter, flatten(atom.args for atom in static_conditions)))
    if not (parameters <= static_parameters):
        raise NotImplementedError(parameters)
    atoms_from_cond = {condition: args_from_predicate[condition.predicate, get_constants(condition)]
                       for condition in static_conditions}
    conditions, atoms = zip(*atoms_from_cond.items())
    relations = [Relation(conditions[index].args, atoms[index])
                 for index in compute_order(conditions, atoms)]
    solution = solve_satisfaction(relations)
    for element in solution.body:
        yield solution.get_mapping(element)

def get_reachable_action_params(instantiated_actions):
    reachable_action_params = defaultdict(list)
    for inst_action in instantiated_actions:
        action = inst_action.action
        parameters = [p.name for p in action.parameters]
        args = apply_mapping(parameters, inst_action.var_mapping)
        reachable_action_params[action].append(args) # TODO: does this actually do anything
    return reachable_action_params

##################################################

def filter_negated(conditions, negated_from_name):
    return list(filter(lambda a: a.predicate not in negated_from_name, conditions))


def get_achieving_axioms(state, operators, negated_from_name={}):
    # TODO: order by stream effort
    # marking algorithm for propositional Horn logic
    unprocessed_from_literal = defaultdict(list)
    operator_from_literal = {}
    remaining_from_stream = {}
    reachable_operators = set() # TODO: only keep facts

    queue = deque()
    def process_axiom(op, effect):
        reachable_operators.add(id(op))
        if effect not in operator_from_literal:
            operator_from_literal[effect] = op
            queue.append(effect)

    # TODO: could produce a list of all derived conditions
    for op in operators:
        preconditions = get_precondition(op)
        for cond, effect in get_conditional_effects(op):
            conditions = cond + preconditions
            remaining_from_stream[id(op), effect] = 0
            for literal in filter_negated(conditions, negated_from_name):
                if literal_holds(state, literal):
                    operator_from_literal[literal] = None
                else:
                    remaining_from_stream[id(op), effect] += 1
                    unprocessed_from_literal[literal].append((op, effect))
            if remaining_from_stream[id(op), effect] == 0:
                process_axiom(op, effect)

    while queue:
        literal = queue.popleft()
        for op, effect in unprocessed_from_literal[literal]:
            remaining_from_stream[id(op), effect] -= 1
            if remaining_from_stream[id(op), effect] == 0:
                process_axiom(op, effect)
    return operator_from_literal, [op for op in operators if id(op) in reachable_operators]

##################################################

def instantiate_domain(task, prune_static=True):
    fluent_predicates = get_fluents(task)
    is_static = lambda a: isinstance(a, pddl.Atom) and (a.predicate not in fluent_predicates)

    fluent_facts = MockSet(lambda a: not prune_static or not is_static(a))
    init_facts = set(task.init)
    function_assignments = get_function_assignments(task)
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)

    constants_from_predicate = defaultdict(set)
    for action in task.actions + task.axioms:
        for atom in filter(is_static, get_literals(get_precondition(action))):
            constants = tuple((i, a) for i, a in enumerate(atom.args) if not is_parameter(a))
            constants_from_predicate[atom.predicate].add(constants)

    predicate_to_atoms = defaultdict(set)
    args_from_predicate = defaultdict(set)
    for atom in filter(is_static, task.init):  # TODO: compute which predicates might involve constants
        predicate_to_atoms[atom.predicate].add(atom)
        args_from_predicate[atom.predicate].add(atom.args)
        for constants in constants_from_predicate[atom.predicate]:
            if all(atom.args[i] == o for i, o in constants):
                args_from_predicate[atom.predicate, constants].add(atom.args)

    instantiated_actions = []
    for action in task.actions:
        for variable_mapping in instantiate_condition(action, is_static, args_from_predicate):
            inst_action = action.instantiate(variable_mapping, init_facts, fluent_facts, type_to_objects,
                                             task.use_min_cost_metric, function_assignments, predicate_to_atoms)
            if inst_action:
                instantiated_actions.append(inst_action)
    instantiated_axioms = []
    for axiom in task.axioms:
        for variable_mapping in instantiate_condition(axiom, is_static, args_from_predicate):
            inst_axiom = axiom.instantiate(variable_mapping, init_facts, fluent_facts)
            if inst_axiom:
                instantiated_axioms.append(inst_axiom)

    reachable_facts, reachable_operators = get_achieving_axioms(init_facts, instantiated_actions + instantiated_axioms)
    atoms = {atom for atom in (init_facts | set(reachable_facts)) if isinstance(atom, pddl.Atom)}
    relaxed_reachable = all(literal_holds(init_facts, goal) or goal in reachable_facts
                            for goal in instantiate_goal(task.goal))
    reachable_actions = [action for action in reachable_operators
                         if isinstance(action, pddl.PropositionalAction)]
    reachable_axioms = [axiom for axiom in reachable_operators
                        if isinstance(axiom, pddl.PropositionalAxiom)]
    return relaxed_reachable, atoms, reachable_actions, reachable_axioms

##################################################

def instantiate_task(task, check_infeasible=True, **kwargs):
    start_time = time()
    print()
    normalize.normalize(task)
    if FD_INSTANTIATE:
        relaxed_reachable, atoms, actions, axioms, reachable_action_params = instantiate.explore(task)
    else:
        relaxed_reachable, atoms, actions, axioms = instantiate_domain(task, **kwargs)
        reachable_action_params = get_reachable_action_params(actions)
    #for atom in sorted(filter(lambda a: isinstance(a, pddl.Literal), set(task.init) | set(atoms)),
    #                   key=lambda a: a.predicate):
    #    print(fact_from_fd(atom))
    #print(axioms)
    #for i, action in enumerate(sorted(actions, key=lambda a: a.name)):
    #    print(i, transform_action_args(pddl_from_instance(action), obj_from_pddl))
    print('Infeasible:', not relaxed_reachable)
    print('Instantiation time:', elapsed_time(start_time))
    if check_infeasible and not relaxed_reachable:
        return None
    goal_list = instantiate_goal(task.goal)
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
    translate_path = os.path.join(temp_dir, TRANSLATE_OUTPUT)
    #clear_dir(temp_dir)
    safe_remove(translate_path)
    ensure_dir(translate_path)
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
    domain = parse_sequential_domain(domain_pddl)
    problem = parse_problem(domain, problem_pddl)
    task = task_from_domain_problem(domain, problem)
    sas_task = sas_from_pddl(task)
    write_sas_task(sas_task, temp_dir)
    return task
