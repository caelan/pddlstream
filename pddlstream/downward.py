from __future__ import print_function

import os
import re
import sys
from collections import namedtuple
from time import time

from pddlstream.conversion import is_atom, is_negated_atom, objects_from_evaluations, pddl_from_object, \
    pddl_list_from_expression, get_prefix, get_args, obj_from_pddl, NOT, EQ
from pddlstream.utils import read, write, safe_rm_dir, INF, Verbose, TmpCWD, clear_dir, get_file_path

#FD_PATH = os.environ['FD_PATH']
FD_PATH = get_file_path(__file__, '../FastDownward/builds/release32/')
FD_BIN = os.path.join(FD_PATH, 'bin')
TRANSLATE_PATH = os.path.join(FD_BIN, 'translate')

DOMAIN_INPUT = 'domain.pddl'
PROBLEM_INPUT = 'problem.pddl'
TRANSLATE_FLAGS = [] # '--negative-axioms'
sys.argv = sys.argv[:1] + TRANSLATE_FLAGS + [DOMAIN_INPUT, PROBLEM_INPUT]
sys.path.append(TRANSLATE_PATH)

import translate
import pddl
import pddl_parser.lisp_parser
import normalize
import pddl_parser
import axiom_rules
import instantiate
import fact_groups
import timers
import options
import simplify
import variable_order
from pddl_parser.parsing_functions import parse_domain_pddl, parse_task_pddl, parse_condition, check_for_duplicates


TEMP_DIR = 'temp/'
TRANSLATE_OUTPUT = 'output.sas'
SEARCH_OUTPUT = 'sas_plan'
SEARCH_COMMAND = 'downward --internal-plan-file %s %s < %s'

# TODO: be careful when doing costs. Might not be admissible if use plus one for heuristic
# TODO: use goal_serialization / hierarchy on the inside loop of these algorithms

OBJECT = 'object'
TOTAL_COST = 'total-cost' # TotalCost

SEARCH_OPTIONS = {
    # Optimal
    'dijkstra': '--heuristic "h=blind(transform=adapt_costs(cost_type=NORMAL))" '
                '--search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'max-astar': '--heuristic "h=hmax(transform=adapt_costs(cost_type=NORMAL))"'
                 ' --search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'lmcut-astar': '--heuristic "h=lmcut(transform=adapt_costs(cost_type=NORMAL))"'
                 ' --search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',

    # Suboptimal
    'ff-astar': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
                '--search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'ff-wastar1': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=true,boost=100,w=1,'
                  'preferred_successors_first=true,cost_type=NORMAL,max_time=%s,bound=%s)"',
    'ff-wastar3': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=100,w=3,'
                  'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',
    'ff-wastar5': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=100,w=5,'
                  'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',

    'cea-wastar1': '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" '
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w=1,'
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',
    'cea-wastar3': '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" '
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w=3,'
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',
    'cea-wastar5': '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" '
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w=5,'
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"',

    'ff-eager': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                '--search "eager_greedy([hff],max_time=%s,bound=%s)"',
    'ff-eager-pref': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                     '--search "eager_greedy([hff],preferred=[hff],max_time=%s,bound=%s)"',
    'ff-lazy': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
               '--search "lazy_greedy([hff],preferred=[hff],max_time=%s,bound=%s)"',
}


##################################################

def parse_lisp(lisp):
    return pddl_parser.lisp_parser.parse_nested_list(lisp.splitlines())

Domain = namedtuple('Domain', ['name', 'requirements', 'types', 'type_dict', 'constants',
                               'predicates', 'predicate_dict', 'functions', 'actions', 'axioms'])

def parse_domain(domain_pddl):
    return Domain(*parse_domain_pddl(parse_lisp(domain_pddl)))

Problem = namedtuple('Problem', ['task_name', 'task_domain_name', 'task_requirements', 'objects', 'init',
                               'goal', 'use_metric'])

def parse_problem(domain, problem_pddl):
    return Problem(*parse_task_pddl(parse_lisp(problem_pddl), domain.type_dict, domain.predicate_dict))

##################################################

def fd_from_fact(fact):
    # TODO: convert to evaluation?
    prefix = get_prefix(fact)
    if prefix == NOT:
        return fd_from_fact(fact[1]).negate()
    if prefix == EQ:
        _, head, value = fact
        predicate = get_prefix(head)
        args = map(pddl_from_object, get_args(head))
        fluent = pddl.f_expression.PrimitiveNumericExpression(symbol=predicate, args=args)
        expression = pddl.f_expression.NumericConstant(value)
        return pddl.f_expression.Assign(fluent, expression)
    args = map(pddl_from_object, get_args(fact))
    return pddl.Atom(prefix, args)

def fact_from_fd(fd):
    assert(not fd.negated)
    return (fd.predicate,) + tuple(map(obj_from_pddl, fd.args))

def get_init(init_evaluations, negated=False):
    # TODO: this doesn't include =
    init = []
    for evaluation in init_evaluations:
        name = evaluation.head.function
        args = tuple(map(pddl_from_object, evaluation.head.args))
        if is_atom(evaluation):
            init.append(pddl.Atom(name, args))
        elif negated and is_negated_atom(evaluation):
            init.append(pddl.NegatedAtom(name, args))
        else:
            fluent = pddl.f_expression.PrimitiveNumericExpression(symbol=name, args=args)
            expression = pddl.f_expression.NumericConstant(evaluation.value)  # Integer
            init.append(pddl.f_expression.Assign(fluent, expression))
    return init

def get_problem(init_evaluations, goal_expression, domain, unit_costs):
    objects = map(pddl_from_object, objects_from_evaluations(init_evaluations))
    typed_objects = list({pddl.TypedObject(obj, OBJECT) for obj in objects} - set(domain.constants))
    init = get_init(init_evaluations)
    goal = parse_condition(pddl_list_from_expression(goal_expression),
                           domain.type_dict, domain.predicate_dict)
    return Problem(task_name=domain.name, task_domain_name=domain.name, objects=typed_objects,
                   task_requirements=pddl.tasks.Requirements([]), init=init, goal=goal, use_metric=not unit_costs)


def task_from_domain_problem(domain, problem):
    domain_name, domain_requirements, types, type_dict, constants, predicates, predicate_dict, functions, actions, axioms \
                 = domain
    task_name, task_domain_name, task_requirements, objects, init, goal, use_metric = problem

    assert domain_name == task_domain_name
    requirements = pddl.Requirements(sorted(set(
                domain_requirements.requirements +
                task_requirements.requirements)))
    objects = constants + objects
    check_for_duplicates(
        [o.name for o in objects],
        errmsg="error: duplicate object %r",
        finalmsg="please check :constants and :objects definitions")
    init += [pddl.Atom("=", (obj.name, obj.name)) for obj in objects]

    return pddl.Task(
        domain_name, task_name, requirements, types, objects,
        predicates, functions, init, goal, actions, axioms, use_metric)

##################################################

def get_literals(condition):
    import pddl
    if isinstance(condition, pddl.Literal):
        return [condition]
    if isinstance(condition, pddl.Conjunction):
        literals = []
        for c in condition.parts():
            literals.extend(get_literals(c))
        return literals
    raise ValueError(condition)

##################################################

def translate_paths(domain_path, problem_path):
    #pddl_parser.parse_pddl_file('domain', domain_path)
    #pddl_parser.parse_pddl_file('task', problem_path)
    task = pddl_parser.open(
        domain_filename=domain_path, task_filename=problem_path)
    normalize.normalize(task)
    return task

def translate_task(task, temp_dir):
    #sas_task = pddl_to_sas(instantiate_task(task))
    normalize.normalize(task)
    sas_task = translate.pddl_to_sas(task)
    #try:
    #    sas_task = translate.pddl_to_sas(task)
    #except AssertionError:
    #    raise AssertionError('A function is not defined for some grounding of an action')
    translate.dump_statistics(sas_task)
    clear_dir(temp_dir)
    with open(os.path.join(temp_dir, TRANSLATE_OUTPUT), "w") as output_file:
        sas_task.output(output_file)
    return sas_task

# def run_translate(temp_dir, verbose):
#     t0 = time()
#     with Verbose(verbose):
#         print('\nTranslate command: import translate; translate.main()')
#         with TmpCWD(os.path.join(os.getcwd(), temp_dir)):
#             translate.main()
#         print('Translate runtime:', time() - t0)

def translate_pddl(domain_pddl, problem_pddl, temp_dir, verbose):
    domain = parse_domain(domain_pddl)
    problem = parse_problem(domain, problem_pddl)
    task = task_from_domain_problem(domain, problem)
    with Verbose(verbose):
        translate_task(task, temp_dir)

##################################################

def run_search(temp_dir, planner='max-astar', max_time=INF, max_cost=INF, debug=False):
    if max_time == INF:
        max_time = 'infinity'
    else:
        max_time = int(max_time)
    if max_cost == INF:
        max_cost = 'infinity'
    else:
        max_cost = int(max_cost)

    t0 = time()
    search = os.path.join(FD_BIN, SEARCH_COMMAND)
    planner_config = SEARCH_OPTIONS[planner] % (max_time, max_cost)
    command = search % (temp_dir + SEARCH_OUTPUT, planner_config, temp_dir + TRANSLATE_OUTPUT)
    if debug:
        print('\nSearch command:', command)
    p = os.popen(command)  # NOTE - cannot pipe input easily with subprocess
    output = p.read()
    if debug:
        print(output[:-1])
        print('Search runtime:', time() - t0)
    if not os.path.exists(temp_dir + SEARCH_OUTPUT):
        return None
    return read(temp_dir + SEARCH_OUTPUT)


##################################################

def parse_solution(solution):
    #action_regex = r'\((\w+(\s+\w+)\)' # TODO: regex
    cost = INF
    if solution is None:
        return None, cost
    cost_regex = r'cost\s*=\s*(\d+)'
    matches = re.findall(cost_regex, solution)
    if matches:
        cost = int(matches[0])
    lines = solution.split('\n')[:-2]  # Last line is newline, second to last is cost
    plan = []
    for line in lines:
        entries = line.strip('( )').split(' ')
        plan.append((entries[0], tuple(entries[1:])))
    return plan, cost


def write_pddl(domain_pddl=None, problem_pddl=None, temp_dir=TEMP_DIR):
    clear_dir(temp_dir)
    domain_path = os.path.join(temp_dir, DOMAIN_INPUT)
    if domain_pddl is not None:
        write(domain_path, domain_pddl)
    problem_path = os.path.join(temp_dir, PROBLEM_INPUT)
    if problem_pddl is not None:
        write(problem_path, problem_pddl)
    return domain_path, problem_path

##################################################

def solve_from_pddl(domain_pddl, problem_pddl, temp_dir=TEMP_DIR, clean=False, debug=False, **kwargs):
    start_time = time()
    write_pddl(domain_pddl, problem_pddl, temp_dir)
    #run_translate(temp_dir, verbose)
    translate_pddl(domain_pddl, problem_pddl, temp_dir, debug)
    solution = run_search(temp_dir, debug=debug, **kwargs)
    if clean:
        safe_rm_dir(temp_dir)
    if debug:
        print('Total runtime:', time() - start_time)
    return parse_solution(solution)

def solve_from_task(task, temp_dir=TEMP_DIR, clean=False, debug=False, **kwargs):
    start_time = time()
    with Verbose(debug):
        translate_task(task, temp_dir)
        solution = run_search(temp_dir, debug=True, **kwargs)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    return parse_solution(solution)

##################################################

GroundTask = namedtuple('GroundTask', ['task', 'atoms', 'actions', 'reachable_action_params', 'original_axioms',
                                       'axioms', 'axiom_init', 'axiom_layer_dict', 'goal_list'])

def instantiate_task(task, simplify_axioms=False):
    # TODO: map parameters to actions and then select which list of atomic supports it
    normalize.normalize(task)
    relaxed_reachable, atoms, actions, original_axioms, reachable_action_params = instantiate.explore(task)
    if not relaxed_reachable:
       return None
    #goal_list = get_literals(task.goal)
    goal_list = task.goal.parts if isinstance(task.goal, pddl.Conjunction) else [task.goal]
    # TODO: this removes the axioms for some reason
    if simplify_axioms:
        axioms, axiom_init, axiom_layer_dict = axiom_rules.handle_axioms(actions, original_axioms, goal_list)
    else:
        axioms, axiom_init, axiom_layer_dict = [], [], {}
    return GroundTask(task, atoms, actions, reachable_action_params, original_axioms,
                      axioms, axiom_init, axiom_layer_dict, goal_list)

def conditions_hold(state, conditions):
    return all((cond in state) != cond.negated for cond in conditions)

def is_applicable(state, action):
    if isinstance(action, pddl.PropositionalAction):
        return conditions_hold(state, action.precondition)
    elif isinstance(action, pddl.PropositionalAxiom):
        return conditions_hold(state, action.condition)
    else:
        raise ValueError(action)

def apply_action(state, action):
    assert(isinstance(action, pddl.PropositionalAction))
    for conditions, effect in action.del_effects:
        if conditions_hold(state, conditions) and (effect in state):
            state.remove(effect)
    for conditions, effect in action.add_effects:
        if conditions_hold(state, conditions):
            state.add(effect)

def apply_axiom(state, axiom):
    assert(isinstance(state, pddl.PropositionalAxiom))
    state.add(axiom.effect)

def apply_lifted_action(state, action):
    assert(isinstance(state, pddl.Action))
    assert(not action.parameters)
    for effect in state.effects:
        assert(not effect.parameters)

def plan_cost(plan):
    cost = 0
    for action in plan:
        cost += action.cost
    return cost

##################################################

def pddl_to_sas(ground_task):
    with timers.timing("Computing fact groups", block=True):
        groups, mutex_groups, translation_key = fact_groups.compute_groups(
            ground_task.task, ground_task.atoms, ground_task.reachable_action_params)

    with timers.timing("Building STRIPS to SAS dictionary"):
        ranges, strips_to_sas = translate.strips_to_sas_dictionary(
            groups, assert_partial=options.use_partial_encoding)

    with timers.timing("Building dictionary for full mutex groups"):
        mutex_ranges, mutex_dict = translate.strips_to_sas_dictionary(
            mutex_groups, assert_partial=False)

    if options.add_implied_preconditions:
        with timers.timing("Building implied facts dictionary..."):
            implied_facts = translate.build_implied_facts(strips_to_sas, groups,
                                                mutex_groups)
    else:
        implied_facts = {}

    with timers.timing("Building mutex information", block=True):
        mutex_key = translate.build_mutex_key(strips_to_sas, mutex_groups)

    with timers.timing("Translating task", block=True):
        sas_task = translate.translate_task(
            strips_to_sas, ranges, translation_key,
            mutex_dict, mutex_ranges, mutex_key,
            ground_task.task.init, ground_task.goal_list, ground_task.actions, ground_task.original_axioms,
            ground_task.task.use_min_cost_metric,
            implied_facts)

    print("%d effect conditions simplified" %
          translate.simplified_effect_condition_counter)
    print("%d implied preconditions added" %
          translate.added_implied_precondition_counter)

    if options.filter_unreachable_facts:
        with timers.timing("Detecting unreachable propositions", block=True):
            try:
                simplify.filter_unreachable_propositions(sas_task)
            except simplify.Impossible:
                return ground_task.unsolvable_sas_task("Simplified to trivially false goal")
            except simplify.TriviallySolvable:
                return ground_task.solvable_sas_task("Simplified to empty goal")

    if options.reorder_variables or options.filter_unimportant_vars:
        with timers.timing("Reordering and filtering variables", block=True):
            variable_order.find_and_apply_variable_order(
                sas_task, options.reorder_variables,
                options.filter_unimportant_vars)

    return sas_task
