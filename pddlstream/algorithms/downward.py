from __future__ import print_function

import os
import re
import sys
from collections import namedtuple
from time import time

from pddlstream.language.conversion import is_atom, is_negated_atom, objects_from_evaluations, pddl_from_object, \
    pddl_list_from_expression, obj_from_pddl
from pddlstream.language.constants import EQ, NOT, Head, Evaluation, get_prefix, get_args
from pddlstream.utils import read, write, safe_rm_dir, INF, Verbose, clear_dir, get_file_path, MockSet, find_unique

# TODO: possible bug when path has a space or period
FD_PATH = get_file_path(__file__, '../../FastDownward/builds/release32/')
FD_BIN = os.path.join(FD_PATH, 'bin')
TRANSLATE_PATH = os.path.join(FD_BIN, 'translate')

DOMAIN_INPUT = 'domain.pddl'
PROBLEM_INPUT = 'problem.pddl'
TRANSLATE_FLAGS = [] # '--negative-axioms'
sys.argv = sys.argv[:1] + TRANSLATE_FLAGS + [DOMAIN_INPUT, PROBLEM_INPUT]
sys.path.append(TRANSLATE_PATH)

import translate
import pddl
import instantiate
import pddl_parser.lisp_parser
import normalize
import pddl_parser
from pddl_parser.parsing_functions import parse_domain_pddl, parse_task_pddl, \
    parse_condition, check_for_duplicates

TEMP_DIR = 'temp/'
TRANSLATE_OUTPUT = 'output.sas'
SEARCH_OUTPUT = 'sas_plan'
SEARCH_COMMAND = 'downward --internal-plan-file %s %s < %s'

# TODO: be careful when doing costs. Might not be admissible if use plus one for heuristic
# TODO: use goal_serialization / hierarchy on the inside loop of these algorithms

OBJECT = 'object'
TOTAL_COST = 'total-cost' # TotalCost
INFINITY = 'infinity'

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
}

for w in [1, 3, 5]:
    SEARCH_OPTIONS['ff-wastar{}'.format(w)] = '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" ' \
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=true,boost=100,w={},' \
                  'preferred_successors_first=true,cost_type=NORMAL,max_time=%s,bound=%s)"'.format(w)
    SEARCH_OPTIONS['cea-wastar{}'.format(w)] = '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" ' \
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w=5,' \
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"'.format(w)

DEFAULT_MAX_TIME = 30 # INF
DEFAULT_PLANNER = 'ff-astar'

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

#def parse_action(lisp_list):
#    action = [':action', 'test'
#              ':parameters', [],
#              ':precondition', [],
#              ':effect', []]
#    parse_action(action)
#    pddl_parser.parsing_functions.parse_action(lisp_list, [], {})
#    return
#
#def create_action(lisp_list):
#    raise NotImplementedError()
#    #return pddl.Action

##################################################

# fact -> evaluation -> fd

def fd_from_fact(fact):
    # TODO: convert to evaluation?
    prefix = get_prefix(fact)
    if prefix == NOT:
        return fd_from_fact(fact[1]).negate()
    if prefix == EQ:
        _, head, value = fact
        predicate = get_prefix(head)
        args = list(map(pddl_from_object, get_args(head)))
        fluent = pddl.f_expression.PrimitiveNumericExpression(symbol=predicate, args=args)
        expression = pddl.f_expression.NumericConstant(value)
        return pddl.f_expression.Assign(fluent, expression)
    args = list(map(pddl_from_object, get_args(fact)))
    return pddl.Atom(prefix, args)

def fact_from_fd(fd):
    assert(not fd.negated)
    return (fd.predicate,) + tuple(map(obj_from_pddl, fd.args))

def evaluation_from_fd(fd):
    if isinstance(fd, pddl.Literal):
        head = Head(fd.predicate, tuple(map(obj_from_pddl, fd.args)))
        return Evaluation(head, not fd.negated)
    if isinstance(fd, pddl.f_expression.Assign):
        head = Head(fd.fluent.symbol, tuple(map(obj_from_pddl, fd.fluent.args)))
        return Evaluation(head, fd.expression.value)
    raise ValueError(fd)

def fd_from_evaluation(evaluation):
    name = evaluation.head.function
    args = tuple(map(pddl_from_object, evaluation.head.args))
    if is_atom(evaluation):
        return pddl.Atom(name, args)
    elif is_negated_atom(evaluation):
        return pddl.NegatedAtom(name, args)
    fluent = pddl.f_expression.PrimitiveNumericExpression(symbol=name, args=args)
    expression = pddl.f_expression.NumericConstant(evaluation.value)  # Integer
    return pddl.f_expression.Assign(fluent, expression)

##################################################

def get_problem(init_evaluations, goal_expression, domain, unit_costs):
    objects = objects_from_evaluations(init_evaluations)
    typed_objects = list({pddl.TypedObject(pddl_from_object(obj), OBJECT) for obj in objects} - set(domain.constants))
    # TODO: this doesn't include =
    init = [fd_from_evaluation(e) for e in init_evaluations if not is_negated_atom(e)]
    goal = parse_condition(pddl_list_from_expression(goal_expression),
                           domain.type_dict, domain.predicate_dict)
    return Problem(task_name=domain.name, task_domain_name=domain.name, objects=typed_objects,
                   task_requirements=pddl.tasks.Requirements([]), init=init, goal=goal, use_metric=not unit_costs)


def task_from_domain_problem(domain, problem):
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
    init += [pddl.Atom("=", (obj.name, obj.name)) for obj in objects]

    task = pddl.Task(domain_name, task_name, requirements, types, objects,
                     predicates, functions, init, goal, actions, axioms, use_metric)
    normalize.normalize(task)
    return task

##################################################

def get_literals(condition):
    if isinstance(condition, pddl.Literal):
        return [condition]
    if isinstance(condition, pddl.Conjunction):
        literals = []
        for c in condition.parts:
            literals.extend(get_literals(c))
        return literals
    raise ValueError(condition)

##################################################

def write_task(sas_task, temp_dir):
    clear_dir(temp_dir)
    translate_path = os.path.join(temp_dir, TRANSLATE_OUTPUT)
    with open(os.path.join(temp_dir, TRANSLATE_OUTPUT), "w") as output_file:
        sas_task.output(output_file)
    return translate_path

def translate_task(task):
    normalize.normalize(task)
    sas_task = translate.pddl_to_sas(task)
    translate.dump_statistics(sas_task)
    return sas_task

def translate_and_write_pddl(domain_pddl, problem_pddl, temp_dir, verbose):
    domain = parse_domain(domain_pddl)
    problem = parse_problem(domain, problem_pddl)
    task = task_from_domain_problem(domain, problem)
    with Verbose(verbose):
        write_task(translate_task(task), temp_dir)

##################################################

def run_search(temp_dir, planner=DEFAULT_PLANNER, max_planner_time=DEFAULT_MAX_TIME, max_cost=INF, debug=False):
    max_time = INFINITY if max_planner_time == INF else int(max_planner_time)
    max_cost = INFINITY if max_cost == INF else int(max_cost)
    start_time = time()
    search = os.path.join(FD_BIN, SEARCH_COMMAND)
    planner_config = SEARCH_OPTIONS[planner] % (max_time, max_cost)
    command = search % (temp_dir + SEARCH_OUTPUT, planner_config, temp_dir + TRANSLATE_OUTPUT)
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
    # TODO: combine
    start_time = time()
    with Verbose(debug):
        write_pddl(domain_pddl, problem_pddl, temp_dir)
        #run_translate(temp_dir, verbose)
        translate_and_write_pddl(domain_pddl, problem_pddl, temp_dir, debug)
        solution = run_search(temp_dir, debug=debug, **kwargs)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    return parse_solution(solution)

##################################################

def conditions_hold(state, conditions):
    return all((cond in state) != cond.negated for cond in conditions)

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
    import pddl
    #name = '@goal-reachable'
    name = '@goal'
    precondition =  goal.parts if isinstance(goal, pddl.Conjunction) else [goal]
    #precondition = get_literals(goal)
    return pddl.PropositionalAction(name, precondition, [], None)

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