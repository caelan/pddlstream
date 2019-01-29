from __future__ import print_function

import os
import re
import sys
from collections import namedtuple
from time import time

from pddlstream.language.constants import EQ, NOT, Head, Evaluation, get_prefix, get_args, OBJECT, TOTAL_COST
from pddlstream.language.conversion import is_atom, is_negated_atom, objects_from_evaluations, pddl_from_object, \
    pddl_list_from_expression, obj_from_pddl
from pddlstream.utils import read, write, INF, Verbose, clear_dir, get_file_path, MockSet, find_unique, int_ceil

filepath = os.path.abspath(__file__)
if ' ' in filepath:
    raise RuntimeError('The path to pddlstream cannot include spaces')
FD_PATH = None
for release in ['release64', 'release32']: # TODO: list the directory
    path = get_file_path(__file__, '../../FastDownward/builds/{}/'.format(release))
    if os.path.exists(path):
        FD_PATH = path
        break
if FD_PATH is None:
    # TODO: could also just automatically compile
    raise RuntimeError('Please compile FastDownward first [.../pddlstream$ ./FastDownward/build.py]')
FD_BIN = os.path.join(FD_PATH, 'bin')
TRANSLATE_PATH = os.path.join(FD_BIN, 'translate')

DOMAIN_INPUT = 'domain.pddl'
PROBLEM_INPUT = 'problem.pddl'
TRANSLATE_FLAGS = ['--negative-axioms']
original_argv = sys.argv[:]
sys.argv = sys.argv[:1] + TRANSLATE_FLAGS + [DOMAIN_INPUT, PROBLEM_INPUT]
sys.path.append(TRANSLATE_PATH)
# TODO: max translate time

import pddl.f_expression
import translate
import pddl
import instantiate
import pddl_parser.lisp_parser
import normalize
import pddl_parser
from pddl_parser.parsing_functions import parse_domain_pddl, parse_task_pddl, \
    parse_condition, check_for_duplicates
sys.argv = original_argv

TEMP_DIR = 'temp/'
TRANSLATE_OUTPUT = 'output.sas'
SEARCH_OUTPUT = 'sas_plan'
SEARCH_COMMAND = 'downward --internal-plan-file {} {} < {}'
INFINITY = 'infinity'
GOAL_NAME = '@goal' # @goal-reachable

# TODO: be careful when doing costs. Might not be admissible if use plus one for heuristic
# TODO: modify parsing_functions to support multiple costs

# bound (int): exclusive depth bound on g-values. Cutoffs are always performed according to the real cost.
# (i.e. solutions must be strictly better than the bound)

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
    'ff-eager': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                '--search "eager_greedy([h],max_time=%s,bound=%s)"',
    'ff-eager-pref': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                     '--search "eager_greedy([h],preferred=[h],max_time=%s,bound=%s)"',
    'ff-lazy': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
               '--search "lazy_greedy([h],preferred=[h],max_time=%s,bound=%s)"',
    'goal-lazy': '--heuristic "h=goalcount(transform=no_transform())" '
                 '--search "lazy_greedy([h],randomize_successors=True,max_time=%s,bound=%s)"',
    'add-random-lazy': '--heuristic "h=add(transform=adapt_costs(cost_type=PLUSONE))" '
                       '--search "lazy_greedy([h],randomize_successors=True,max_time=%s,bound=%s)"',

    'ff-lazy-tiebreak': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
                  '--search "lazy(tiebreaking([h, g()]),preferred=[h],reopen_closed=false,'
                  'randomize_successors=True,cost_type=NORMAL,max_time=%s,bound=%s)"',
    
    'ff-ehc': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
              '--search "ehc(h,preferred=[h],preferred_usage=RANK_PREFERRED_FIRST,'
              'cost_type=NORMAL,max_time=%s,bound=%s)"',
    # The key difference is that ehc resets the open list upon finding an improvement
}
# TODO: do I want to sort operators in FD hill-climbing search?
# Greedily prioritize operators with less cost. Useful when prioritizing actions that have no stream cost

for w in range(1, 1+5):
    SEARCH_OPTIONS['ff-wastar{}'.format(w)] = '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" ' \
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=true,boost=100,w={},' \
                  'preferred_successors_first=true,cost_type=NORMAL,max_time=%s,bound=%s)"'.format(w)
    SEARCH_OPTIONS['cea-wastar{}'.format(w)] = '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" ' \
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w={},' \
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"'.format(w)

# TODO: throw a warning if max_planner_time is met
DEFAULT_MAX_TIME = 30 # INF
DEFAULT_PLANNER = 'ff-astar'

##################################################

# WARNING: overflow on h^add! Costs clamped to 100000000
MAX_FD_COST = 1e8

def round_cost(cost):
    cost_scale = get_cost_scale()
    return int(cost_scale * cost) / cost_scale

def get_cost_scale():
    return pddl.f_expression.COST_SCALE

def set_cost_scale(cost_scale):
    pddl.f_expression.COST_SCALE = cost_scale

def scale_cost(cost):
    return int_ceil(get_cost_scale() * float(cost))

set_cost_scale(1000) # TODO: make unit costs be equivalent to cost scale = 0

##################################################

def parse_lisp(lisp):
    return pddl_parser.lisp_parser.parse_nested_list(lisp.splitlines())

Domain = namedtuple('Domain', ['name', 'requirements', 'types', 'type_dict', 'constants',
                               'predicates', 'predicate_dict', 'functions', 'actions', 'axioms'])

def parse_domain(domain_pddl):
    if isinstance(domain_pddl, Domain):
        return domain_pddl
    domain = Domain(*parse_domain_pddl(parse_lisp(domain_pddl)))
    #for action in domain.actions:
    #    if (action.cost is not None) and isinstance(action.cost, pddl.Increase) and isinstance(action.cost.expression, pddl.NumericConstant):
    #        action.cost.expression.value = scale_cost(action.cost.expression.value)
    return domain

def has_costs(domain):
    for action in domain.actions:
        if action.cost is not None:
            return True
    return False

Problem = namedtuple('Problem', ['task_name', 'task_domain_name', 'task_requirements',
                                 'objects', 'init', 'goal', 'use_metric'])

def parse_problem(domain, problem_pddl):
    if isinstance(problem_pddl, Problem):
        return problem_pddl
    return Problem(*parse_task_pddl(parse_lisp(problem_pddl), domain.type_dict, domain.predicate_dict))

#def parse_action(lisp_list):
#    action = [':action', 'test'
#              ':parameters', [],
#              ':precondition', [],
#              ':effect', []]
#    parse_action(action)
#    pddl_parser.parsing_functions.parse_action(lisp_list, [], {})
#    return pddl.Action

##################################################

# fact -> evaluation -> fd

def fd_from_fact(fact):
    # TODO: convert to evaluation?
    prefix = get_prefix(fact)
    if prefix == NOT:
        return fd_from_fact(fact[1]).negate()
    #if prefix == EQ:
    #    _, head, value = fact
    #    predicate = get_prefix(head)
    #    args = list(map(pddl_from_object, get_args(head)))
    #    fluent = pddl.f_expression.PrimitiveNumericExpression(symbol=predicate, args=args)
    #    expression = pddl.f_expression.NumericConstant(value)
    #    return pddl.f_expression.Assign(fluent, expression)
    args = list(map(pddl_from_object, get_args(fact)))
    return pddl.Atom(prefix, args)

def fact_from_fd(fd):
    assert(isinstance(fd, pddl.Literal) and not fd.negated)
    return (fd.predicate,) + tuple(map(obj_from_pddl, fd.args))

def evaluation_from_fd(fd):
    if isinstance(fd, pddl.Literal):
        head = Head(fd.predicate, tuple(map(obj_from_pddl, fd.args)))
        return Evaluation(head, not fd.negated)
    if isinstance(fd, pddl.f_expression.Assign):
        raise not NotImplementedError()
        #head = Head(fd.fluent.symbol, tuple(map(obj_from_pddl, fd.fluent.args)))
        #return Evaluation(head, float(fd.expression.value) / COST_SCALE) # Need to be careful
    raise ValueError(fd)

def fd_from_evaluation(evaluation):
    name = evaluation.head.function
    args = tuple(map(pddl_from_object, evaluation.head.args))
    if is_atom(evaluation):
        return pddl.Atom(name, args)
    elif is_negated_atom(evaluation):
        return pddl.NegatedAtom(name, args)
    fluent = pddl.f_expression.PrimitiveNumericExpression(symbol=name, args=args)
    expression = pddl.f_expression.NumericConstant(evaluation.value)
    return pddl.f_expression.Assign(fluent, expression)

##################################################

def parse_goal(goal_expression, domain):
    #try:
    #    pass
    #except SystemExit as e:
    #    return False
    return parse_condition(pddl_list_from_expression(goal_expression),
                           domain.type_dict, domain.predicate_dict).simplified()

def get_problem(init_evaluations, goal_expression, domain, unit_costs=False):
    objects = objects_from_evaluations(init_evaluations)
    typed_objects = list({pddl.TypedObject(pddl_from_object(obj), OBJECT) for obj in objects} - set(domain.constants))
    # TODO: this doesn't include =
    init = [fd_from_evaluation(e) for e in init_evaluations if not is_negated_atom(e)]
    goal = parse_goal(goal_expression, domain)
    return Problem(task_name=domain.name, task_domain_name=domain.name, objects=typed_objects,
                   task_requirements=pddl.tasks.Requirements([]), init=init, goal=goal, use_metric=not unit_costs)


def task_from_domain_problem(domain, problem):
    # TODO: prune eval
    domain_name, domain_requirements, types, type_dict, constants, \
        predicates, predicate_dict, functions, actions, axioms = domain
    task_name, task_domain_name, task_requirements, objects, init, goal, use_metric = problem

    assert domain_name == task_domain_name
    requirements = pddl.Requirements(sorted(set(domain_requirements.requirements +
                                                task_requirements.requirements)))
    objects = constants + objects
    check_for_duplicates([o.name for o in objects],
        errmsg="error: duplicate object %r",
        finalmsg="please check :constants and :objects definitions")
    init.extend(pddl.Atom(EQ, (obj.name, obj.name)) for obj in objects)

    task = pddl.Task(domain_name, task_name, requirements, types, objects,
                     predicates, functions, init, goal, actions, axioms, use_metric)
    normalize.normalize(task)
    return task

##################################################

def get_fluents(domain):
    fluent_predicates = set()
    for action in domain.actions:
        for effect in action.effects:
            fluent_predicates.add(effect.literal.predicate)
    for axiom in domain.axioms:
        fluent_predicates.add(axiom.name)
    return fluent_predicates

def get_literals(condition):
    if isinstance(condition, pddl.Truth):
        return []
    if isinstance(condition, pddl.Literal):
        return [condition]
    if isinstance(condition, pddl.Conjunction):
        literals = []
        for c in condition.parts:
            literals.extend(get_literals(c))
        return literals
    raise ValueError(condition)

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
        sas_task = sas_from_instantiated(instantiate_task(task))
        sas_task.metric = task.use_min_cost_metric # TODO: are these sometimes not equal?
    return sas_task

def translate_and_write_pddl(domain_pddl, problem_pddl, temp_dir, verbose):
    domain = parse_domain(domain_pddl)
    problem = parse_problem(domain, problem_pddl)
    task = task_from_domain_problem(domain, problem)
    sas_task = sas_from_pddl(task)
    write_sas_task(sas_task, temp_dir)
    return task

#def normalize_domain_goal(domain, goal):
#    task = pddl.Task(None, None, None, None, None,
#                     None, None, [], goal, domain.actions, domain.axioms, None)
#    normalize.normalize(task)

##################################################

def convert_cost(cost):
    if cost == INF:
        return INFINITY
    return int(cost)

def run_search(temp_dir, planner=DEFAULT_PLANNER, max_planner_time=DEFAULT_MAX_TIME, max_cost=INF, debug=False):
    max_time = convert_cost(max_planner_time)
    max_cost = INFINITY if max_cost == INF else scale_cost(max_cost)
    start_time = time()
    search = os.path.join(FD_BIN, SEARCH_COMMAND)
    planner_config = SEARCH_OPTIONS[planner] % (max_time, max_cost)
    command = search.format(temp_dir + SEARCH_OUTPUT, planner_config, temp_dir + TRANSLATE_OUTPUT)
    if debug:
        print('Search command:', command)
    p = os.popen(command)  # NOTE - cannot pipe input easily with subprocess
    output = p.read()
    if debug:
        print(output[:-1])
        print('Search runtime:', time() - start_time)
    if not os.path.exists(temp_dir + SEARCH_OUTPUT):
        return None
    return read(temp_dir + SEARCH_OUTPUT)

##################################################

def parse_action(line):
    entries = line.strip('( )').split(' ')
    name = entries[0]
    args = tuple(entries[1:])
    return (name, args)

def parse_solution(solution):
    #action_regex = r'\((\w+(\s+\w+)\)' # TODO: regex
    cost = INF
    if solution is None:
        return None, cost
    cost_regex = r'cost\s*=\s*(\d+)'
    matches = re.findall(cost_regex, solution)
    if matches:
        cost = float(matches[0]) / get_cost_scale()
    # TODO: recover the actual cost of the plan from the evaluations
    lines = solution.split('\n')[:-2]  # Last line is newline, second to last is cost
    plan = list(map(parse_action, lines))
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

def literal_holds(state, literal):
    #return (literal in state) != literal.negated
    return (literal.positive() in state) != literal.negated

def conditions_hold(state, conditions):
    return all(literal_holds(state, cond) for cond in conditions)

def is_applicable(state, action):
    if isinstance(action, pddl.PropositionalAction):
        return conditions_hold(state, action.precondition)
    elif isinstance(action, pddl.PropositionalAxiom):
        return conditions_hold(state, action.condition)
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

def is_valid_plan(initial_state, plan): #, goal):
    state = set(initial_state)
    for action in plan:
        if not is_applicable(state, action):
            return False
        apply_action(state, action)
    return True

#def apply_lifted_action(state, action):
#    assert(isinstance(state, pddl.Action))
#    assert(not action.parameters)
#    for effect in state.effects:
#        assert(not effect.parameters)

def plan_cost(plan):
    cost = 0
    for action in plan:
        cost += action.cost
    return cost

def substitute_derived(axiom_plan, action_instance):
    # TODO: what if the propositional axiom has conditional derived
    axiom_pre = {p for ax in axiom_plan for p in ax.condition}
    axiom_eff = {ax.effect for ax in axiom_plan}
    action_instance.precondition = list((set(action_instance.precondition) | axiom_pre) - axiom_eff)

##################################################

def get_action_instances(task, action_plan):
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    function_assignments = {f.fluent: f.expression for f in task.init
                            if isinstance(f, pddl.f_expression.FunctionAssignment)}
    fluent_facts = MockSet()
    init_facts = set()
    action_instances = []
    for name, objects in action_plan:
        # TODO: what if more than one action of the same name due to normalization?
        # Normalized actions have same effects, so I just have to pick one
        action = find_unique(lambda a: a.name == name, task.actions)
        args = list(map(pddl_from_object, objects))
        assert (len(action.parameters) == len(args))
        variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        instance = action.instantiate(variable_mapping, init_facts,
                                      fluent_facts, type_to_objects,
                                      task.use_min_cost_metric, function_assignments)
        assert (instance is not None)
        action_instances.append(instance)
    return action_instances


def get_goal_instance(goal):
    return pddl.PropositionalAction(GOAL_NAME, instantiate_goal(goal), [], None)

##################################################

def add_preimage_condition(condition, preimage, i):
    for literal in condition:
        #preimage[literal] = preimage.get(literal, set()) | {i}
        preimage.setdefault(literal, set()).add(i)
    #preimage.update(condition)


def add_preimage_effect(effect, preimage):
    preimage.pop(effect, None)
    #if effect in preimage:
    #    # Fluent effects kept, static dropped
    #    preimage.remove(effect)


def action_preimage(action, preimage, i):
    for conditions, effect in (action.add_effects + action.del_effects):
        assert(not conditions)
        # TODO: can later select which conditional effects are used
        # TODO: might need to truely decide whether one should hold or not for a preimage
        # Maybe I should do that here
        add_preimage_effect(effect, preimage)
    add_preimage_condition(action.precondition, preimage, i)


def axiom_preimage(axiom, preimage, i):
    add_preimage_effect(axiom.effect, preimage)
    add_preimage_condition(axiom.condition, preimage, i)


def plan_preimage(combined_plan, goal):
    #preimage = set(goal)
    action_plan = [action for action in combined_plan if isinstance(action, pddl.PropositionalAction)]
    step = len(action_plan)
    preimage = {condition: {step} for condition in goal}
    for operator in reversed(combined_plan):
        if isinstance(operator, pddl.PropositionalAction):
            step -= 1
            action_preimage(operator, preimage, step)
        elif isinstance(operator, pddl.PropositionalAxiom):
            axiom_preimage(operator, preimage, step)
        else:
            raise ValueError(operator)
    return preimage

##################################################

def add_predicate(domain, predicate):
    if predicate.name in domain.predicate_dict:
        return False
    domain.predicates.append(predicate)
    domain.predicate_dict[predicate.name] = predicate
    return True


def make_object(obj, type=OBJECT):
    return pddl.TypedObject(obj, type)


def make_parameters(parameters, **kwargs):
    return tuple(make_object(p, **kwargs) for p in parameters)


def make_predicate(name, parameters):
    return pddl.Predicate(name, make_parameters(parameters))


def make_preconditions(preconditions):
    return pddl.Conjunction(list(map(fd_from_fact, preconditions)))


def make_effects(effects):
    return [pddl.Effect(parameters=[], condition=pddl.Truth(),
                        literal=fd_from_fact(fact)) for fact in effects]

def make_cost(cost):
    if cost is None:
        return cost
    fluent = pddl.PrimitiveNumericExpression(symbol=TOTAL_COST, args=[])
    try:
        expression = pddl.NumericConstant(cost)
    except TypeError:
        expression = pddl.PrimitiveNumericExpression(
            symbol=get_prefix(cost), args=list(map(pddl_from_object, get_args(cost))))
    return pddl.Increase(fluent=fluent, expression=expression)


def make_action(name, parameters, preconditions, effects, cost=None):
    # Usually all parameters are external
    return pddl.Action(name=name,
                       parameters=make_parameters(parameters),
                       num_external_parameters=len(parameters),
                       precondition=make_preconditions(preconditions),
                       effects=make_effects(effects),
                       cost=make_cost(cost))


def make_axiom(parameters, preconditions, derived):
    predicate = get_prefix(derived)
    external_parameters = list(get_args(derived))
    internal_parameters = [p for p in parameters if p not in external_parameters]
    parameters = external_parameters + internal_parameters
    return pddl.Axiom(name=predicate,
                      parameters=make_parameters(parameters),
                      num_external_parameters=len(external_parameters),
                      condition=make_preconditions(preconditions))


def make_domain(constants=[], predicates=[], functions=[], actions=[], axioms=[]):
    import pddl_parser
    types = [pddl.Type(OBJECT)]
    pddl_parser.parsing_functions.set_supertypes(types)
    return Domain(name='', requirements=pddl.Requirements([]),
             types=types, type_dict={ty.name: ty for ty in types}, constants=constants,
             predicates=predicates, predicate_dict={p.name: p for p in predicates},
             functions=functions, actions=actions, axioms=axioms)

##################################################

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

def instantiate_task(task):
    # TODO: my own action instantiation
    normalize.normalize(task)
    relaxed_reachable, atoms, actions, axioms, reachable_action_params = instantiate.explore(task)
    if not relaxed_reachable:
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
    return sas_task
