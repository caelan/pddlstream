from __future__ import print_function

import os
import re
import sys
from collections import namedtuple
from time import time

from pddlstream.language.conversion import is_atom, is_negated_atom, objects_from_evaluations, pddl_from_object, \
    pddl_list_from_expression, get_prefix, get_args, obj_from_pddl, NOT, EQ
from pddlstream.utils import read, write, safe_rm_dir, INF, Verbose, clear_dir, get_file_path

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
    'ff-eager': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                '--search "eager_greedy([hff],max_time=%s,bound=%s)"',
    'ff-eager-pref': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                     '--search "eager_greedy([hff],preferred=[hff],max_time=%s,bound=%s)"',
    'ff-lazy': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
               '--search "lazy_greedy([hff],preferred=[hff],max_time=%s,bound=%s)"',
}

for w in [1, 3, 5]:
    SEARCH_OPTIONS['ff-wastar{}'.format(w)] = '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" ' \
                  '--search "lazy_wastar([h],preferred=[h],reopen_closed=true,boost=100,w={},' \
                  'preferred_successors_first=true,cost_type=NORMAL,max_time=%s,bound=%s)"'.format(w)
    SEARCH_OPTIONS['cea-wastar{}'.format(w)] = '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" ' \
                   '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w=5,' \
                   'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"'.format(w)

# TODO: goal serialization

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
    import pddl
    if isinstance(condition, pddl.Literal):
        return [condition]
    if isinstance(condition, pddl.Conjunction):
        literals = []
        for c in condition.parts:
            literals.extend(get_literals(c))
        return literals
    raise ValueError(condition)

##################################################

def translate_and_write_task(task, temp_dir):
    normalize.normalize(task)
    sas_task = translate.pddl_to_sas(task)
    translate.dump_statistics(sas_task)
    clear_dir(temp_dir)
    with open(os.path.join(temp_dir, TRANSLATE_OUTPUT), "w") as output_file:
        sas_task.output(output_file)
    return sas_task

def translate_and_write_pddl(domain_pddl, problem_pddl, temp_dir, verbose):
    domain = parse_domain(domain_pddl)
    problem = parse_problem(domain, problem_pddl)
    task = task_from_domain_problem(domain, problem)
    with Verbose(verbose):
        translate_and_write_task(task, temp_dir)

##################################################

def run_search(temp_dir, planner='max-astar', max_time=INF, max_cost=INF, debug=False):
    max_time = 'infinity' if max_time == INF else int(max_time)
    max_cost = 'infinity' if max_cost == INF else int(max_cost)
    start_time = time()
    search = os.path.join(FD_BIN, SEARCH_COMMAND)
    planner_config = SEARCH_OPTIONS[planner] % (max_time, max_cost)
    command = search % (temp_dir + SEARCH_OUTPUT, planner_config, temp_dir + TRANSLATE_OUTPUT)
    if debug:
        print('\nSearch command:', command)
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
    start_time = time()
    write_pddl(domain_pddl, problem_pddl, temp_dir)
    #run_translate(temp_dir, verbose)
    translate_and_write_pddl(domain_pddl, problem_pddl, temp_dir, debug)
    solution = run_search(temp_dir, debug=debug, **kwargs)
    if clean:
        safe_rm_dir(temp_dir)
    if debug:
        print('Total runtime:', time() - start_time)
    return parse_solution(solution)

def solve_from_task(task, temp_dir=TEMP_DIR, clean=False, debug=False, **kwargs):
    start_time = time()
    with Verbose(debug):
        translate_and_write_task(task, temp_dir)
        solution = run_search(temp_dir, debug=True, **kwargs)
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    return parse_solution(solution)

import sas_tasks

def apply_sas_operator(init, op):
    for var, pre, post, cond in op.pre_post:
        assert (pre == -1) or (init.values[var] == pre)
        assert not cond
        init.values[var] = post

def serialized_solve_from_task(task, temp_dir=TEMP_DIR, clean=False, debug=False, **kwargs):
    start_time = time()
    with Verbose(debug):
        sas_task = translate_and_write_task(task, temp_dir)
        op_from_name = {op.name: op for op in sas_task.operators} # No need to keep repeats
        goals = sas_task.goal.pairs[:]
        full_plan = []
        full_cost = 0
        for i in range(len(goals)):
            sas_task.goal = sas_tasks.SASGoal(goals[:i+1])
            solution = run_search(temp_dir, debug=True, **kwargs)
            plan, cost = parse_solution(solution)
            if plan is None:
                return None, INF
            full_plan.extend(plan)
            full_cost += cost
            for action, args in plan:
                name = '({})'.format(' '.join((action,) + args))
                apply_sas_operator(sas_task.init, op_from_name[name])
        if clean:
            safe_rm_dir(temp_dir)
        print('Total runtime:', time() - start_time)
    return full_plan, full_cost

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
