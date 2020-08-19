from __future__ import print_function

import os
import re
import sys
import subprocess
from collections import namedtuple, defaultdict
from time import time

from pddlstream.language.constants import EQ, NOT, Head, Evaluation, get_prefix, get_args, OBJECT, TOTAL_COST, Action, Not
from pddlstream.language.conversion import is_atom, is_negated_atom, objects_from_evaluations, pddl_from_object, \
    pddl_list_from_expression, obj_from_pddl
from pddlstream.utils import read, write, INF, get_file_path, MockSet, find_unique, int_ceil, \
    safe_remove, safe_zip, ensure_dir, safe_rm_dir, implies, get_python_version, elapsed_time, safe_listdir
from pddlstream.language.write_pddl import get_problem_pddl

DIVERSE_PLANNERS = ['forbid', 'kstar', 'symk']

#CERBERUS_PATH = '/home/caelan/Programs/cerberus' # Check if this path exists
#CERBERUS_PATH = '/home/caelan/Programs/fd-redblack-ipc2018' # Check if this path exists
# Does not support derived predicates

##################################################

#FORBID_PATH = '/Users/caelan/Programs/external/IBM/ForbidIterative'
#FORBID_TEMPLATE = 'plan.py --planner topk --number-of-plans {max_plans} --symmetries ' \
#                  '--domain {domain} --problem {problem}'
# --planner topk,topq,topkq,diverse
# topk: many plans that could be equivalent
# topq: equivalence classes on plans (to prevent all possible reorderings) all plans up to that cost
# topkq: generate reorders options
# diverse: ignores solution quality
# IJCAI submission: additional planner for topq and then diverse subset K* (multiple explicit paths)
# ForbidIterative planners for top-k, top-quality, and diverse planning problems
# https://zenodo.org/record/3246774

##################################################

FORBID_TEMPLATE = 'plan.py --planner unordered_topq --overall-time-limit {max_time} --quality-bound {max_cost} --symmetries ' \
                  '--domain {domain} --problem {problem} --use-local-folder --clean-local-folder' # ' --upper-bound-on-number-of-plans {max_plans}'
# [--overall-time-limit OVERALL_TIME_LIMIT]
# [--planner {topk,topk_via_unordered_topq,unordered_topq,extended_unordered_topq,topq_via_topk,topq_via_unordered_topq,diverse}]
# --reordering generates multiple plans from a single one
# --quality_bound is a multiplicative factor of the first (best) plan cost
# --symmetries substitutes different variable values when generating plans
# Because using optimal search, plans are generated successively with increasing cost

# Does not support derived predicates, disjunctive conditions
# Assumes python2

##################################################

KSATAR_TEMPLATE = './fast-downward.py --build release64 {domain} {problem} ' \
                  '--search "kstar(blind(),q={max_cost},k={max_plans},max_time={max_time},skip_reorderings=true)"' # ,bound={max_cost}
# blind, hmax, lmcut
# TODO: kstar segfaults on OS X occasionally

##################################################

# https://github.com/speckdavid/symk
SYMK_TEMPLATE = './fast-downward.py {domain} {problem} ' \
                '--search "symq-bd(plan_selection=unordered(num_plans={max_plans}),quality={max_cost})"'
# plan_selection=top_k, plan_selection=unordered
# Quality 1<=q<=infinity is a multiplier that is multiplied to the cost of the cheapest solution.
# For example, q=1 reports only the cheapest plans, where quality=infinity corresponds to the top-k planning.

##################################################

filepath = os.path.abspath(__file__)
if ' ' in filepath:
    raise RuntimeError('The path to pddlstream cannot include spaces')

def find_build(fd_path):
    for release in ['release64', 'release32']:  # TODO: list the directory
        path = os.path.join(fd_path, 'builds/{}/'.format(release))
        if os.path.exists(path):
            return path
    # TODO: could also just automatically compile
    raise RuntimeError('Please compile FastDownward first [.../pddlstream$ ./FastDownward/build.py]')

FD_PATH = get_file_path(__file__, '../../FastDownward/')
TRANSLATE_PATH = os.path.join(find_build(FD_PATH), 'bin/translate')
FD_BIN = os.path.join(find_build(FD_PATH), 'bin') # CERBERUS_PATH if USE_CERBERUS else

DOMAIN_INPUT = 'domain.pddl'
PROBLEM_INPUT = 'problem.pddl'
TRANSLATE_FLAGS = ['--negative-axioms'] # [] if USE_CERBERUS else
original_argv = sys.argv[:]
sys.argv = sys.argv[:1] + TRANSLATE_FLAGS + [DOMAIN_INPUT, PROBLEM_INPUT]
sys.path.append(TRANSLATE_PATH)
# TODO: max translate time

import pddl.f_expression
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

##################################################

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

    'ff-eager-tiebreak': '--heuristic "h=ff(transform=no_transform())" '
                         '--search "eager(tiebreaking([h, g()]),reopen_closed=false,'
                         'cost_type=NORMAL,max_time=%s,bound=%s, f_eval=sum([g(), h]))"', # preferred=[h],
    'ff-lazy-tiebreak': '--heuristic "h=ff(transform=no_transform())" '
                         '--search "lazy(tiebreaking([h, g()]),reopen_closed=false,'
                         'randomize_successors=True,cost_type=NORMAL,max_time=%s,bound=%s)"',  # preferred=[h],
    # TODO: eagerly evaluate goal count but lazily compute relaxed plan

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

# if USE_CERBERUS:
#     # --internal-previous-portfolio-plans
#     #import imp
#     #plan_path = os.path.join(CERBERUS_PATH, 'plan.py')
#     #plan = imp.load_source('plan', plan_path)
#     sys.path.append(CERBERUS_PATH)
#     import importlib
#     mod = importlib.import_module("plan-agl") # plan | plan-agl | plan-cbo | plan-sat
#     #SEARCH_OPTIONS['cerberus'] = ' '.join(p.strip() for s in mod.config_string() for p in s.split('\n')) # .replace('\n', ' ')
#     SEARCH_OPTIONS['cerberus'] = ' '.join(s if s.startswith('--') else '"{}"'.format(s)
#                                           for s in mod.config_string())

# TODO: throw a warning if max_planner_time is met
DEFAULT_MAX_TIME = 30 # INF
DEFAULT_PLANNER = 'ff-astar'
DEFAULT_MAX_PLANS = 1e6 # effectively infinity

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
    if cost == INF:
        return INF
    return int_ceil(get_cost_scale() * float(cost))

def convert_cost(cost):
    if cost == INF:
        return INFINITY
    return int_ceil(cost)

set_cost_scale(1e3) # TODO: make unit costs be equivalent to cost scale = 0

##################################################

def parse_lisp(lisp):
    return pddl_parser.lisp_parser.parse_nested_list(lisp.splitlines())

# TODO: dynamically generate type_dict and predicate_dict
Domain = namedtuple('Domain', ['name', 'requirements', 'types', 'type_dict', 'constants',
                               'predicates', 'predicate_dict', 'functions', 'actions', 'axioms', 'pddl'])

def parse_sequential_domain(domain_pddl):
    if isinstance(domain_pddl, Domain):
        return domain_pddl
    args = list(parse_domain_pddl(parse_lisp(domain_pddl))) + [domain_pddl]
    domain = Domain(*args)
    # for action in domain.actions:
    #    if (action.cost is not None) and isinstance(action.cost, pddl.Increase) and isinstance(action.cost.expression, pddl.NumericConstant):
    #        action.cost.expression.value = scale_cost(action.cost.expression.value)
    return domain

def has_costs(domain):
    for action in domain.actions:
        if action.cost is not None:
            return True
    return False

Problem = namedtuple('Problem', ['task_name', 'task_domain_name', 'task_requirements',
                                 'objects', 'init', 'goal', 'use_metric', 'pddl'])

def parse_problem(domain, problem_pddl):
    if isinstance(problem_pddl, Problem):
        return problem_pddl
    args = list(parse_task_pddl(parse_lisp(problem_pddl), domain.type_dict, domain.predicate_dict)) + [problem_pddl]
    return Problem(*args)

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
    assert(is_literal(fd))
    atom = (fd.predicate,) + tuple(map(obj_from_pddl, fd.args))
    return Not(atom) if fd.negated else atom

def evaluation_from_fd(fd):
    if is_literal(fd):
        head = Head(fd.predicate, tuple(map(obj_from_pddl, fd.args)))
        return Evaluation(head, not fd.negated)
    if is_assignment(fd):
        raise NotImplementedError(fd)
        #head = Head(fd.fluent.symbol, tuple(map(obj_from_pddl, fd.fluent.args)))
        #return Evaluation(head, float(fd.expression.value) / get_cost_scale())  # Need to be careful due to rounding
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

def parse_goal(goal_exp, domain):
    #try:
    #    pass
    #except SystemExit as e:
    #    return False
    return parse_condition(pddl_list_from_expression(goal_exp),
                           domain.type_dict, domain.predicate_dict).simplified()

def get_problem(evaluations, goal_exp, domain, unit_costs=False):
    objects = objects_from_evaluations(evaluations)
    typed_objects = list({pddl.TypedObject(pddl_from_object(obj), OBJECT) for obj in objects} - set(domain.constants))
    # TODO: this doesn't include =
    init = [fd_from_evaluation(e) for e in evaluations if not is_negated_atom(e)]
    goal = pddl.Truth() if goal_exp is None else parse_goal(goal_exp, domain)
    problem_pddl = get_problem_pddl(evaluations, goal_exp, domain.pddl, temporal=False)
    write_pddl(domain.pddl, problem_pddl)
    return Problem(task_name=domain.name, task_domain_name=domain.name,
                   objects=sorted(typed_objects, key=lambda o: o.name),
                   task_requirements=pddl.tasks.Requirements([]), init=init, goal=goal,
                   use_metric=not unit_costs, pddl=problem_pddl)

IDENTICAL = "identical" # lowercase is critical (!= instead?)

def task_from_domain_problem(domain, problem):
    # TODO: prune evaluation that aren't needed in actions
    #domain_name, domain_requirements, types, type_dict, constants, \
    #    predicates, predicate_dict, functions, actions, axioms = domain
    task_name, task_domain_name, task_requirements, objects, init, goal, use_metric, problem_pddl = problem

    assert domain.name == task_domain_name
    requirements = pddl.Requirements(sorted(set(domain.requirements.requirements +
                                                task_requirements.requirements)))
    objects = domain.constants + objects
    check_for_duplicates([o.name for o in objects],
        errmsg="error: duplicate object %r",
        finalmsg="please check :constants and :objects definitions")
    init.extend(pddl.Atom(EQ, (obj.name, obj.name)) for obj in objects)
    # TODO: optimistically evaluate (not (= ?o1 ?o2))
    # for fd_obj in objects:
    #     obj = obj_from_pddl(fd_obj.name) # TODO: not instantiated for solve_from_pddl
    #     if obj.is_unique():
    #         init.append(pddl.Atom(IDENTICAL, (fd_obj.name, fd_obj.name)))
    #     else:
    #         assert obj.is_shared()
    task = pddl.Task(domain.name, task_name, requirements, domain.types, objects,
                     domain.predicates, domain.functions, init, goal,
                     domain.actions, domain.axioms, use_metric)
    normalize.normalize(task)
    # task.add_axiom
    return task

##################################################

def get_derived_predicates(axioms):
    axioms_from_name = defaultdict(list)
    for axiom in axioms:
        axioms_from_name[axiom.name].append(axiom)
    return axioms_from_name

def get_fluents(domain):
    fluent_predicates = set(get_derived_predicates(domain.axioms))
    for action in domain.actions:
        for effect in action.effects:
            fluent_predicates.add(effect.literal.predicate)
    return fluent_predicates

def is_literal(condition):
    return isinstance(condition, pddl.Literal)

def is_assignment(condition):
    return isinstance(condition, pddl.f_expression.Assign)

def get_literals(condition):
    if is_literal(condition):
        return [condition]
    if isinstance(condition, pddl.Truth):
        return []
    if isinstance(condition, pddl.Conjunction):
        literals = []
        for c in condition.parts:
            literals.extend(get_literals(c))
        return literals
    raise ValueError(condition)

def get_conjunctive_parts(condition):
    return condition.parts if isinstance(condition, pddl.Conjunction) else [condition]

def get_disjunctive_parts(condition):
    return condition.parts if isinstance(condition, pddl.Disjunction) else [condition]

##################################################

#def normalize_domain_goal(domain, goal):
#    task = pddl.Task(None, None, None, None, None,
#                     None, None, [], goal, domain.actions, domain.axioms, None)
#    normalize.normalize(task)

DIVERSE_DIR = 'found_plans'

def clean_planner(planner):
    # if planner == 'forbid':
    #     for filename in safe_listdir(FORBID_PATH):  # Deletes existing forbid plans and output.sas
    #         if filename.startswith(SEARCH_OUTPUT) or filename.startswith('reformulated_output.sas'):
    #             safe_remove(os.path.join(FORBID_PATH, filename))
    if planner in DIVERSE_PLANNERS:
        kstar_plans_path = os.path.join(os.getcwd(), DIVERSE_DIR)
        safe_remove(os.path.join(os.getcwd(), TRANSLATE_OUTPUT))
        safe_rm_dir(kstar_plans_path)

def run_search(temp_dir, planner=DEFAULT_PLANNER, max_planner_time=DEFAULT_MAX_TIME,
               max_cost=INF, max_plans=DEFAULT_MAX_PLANS, debug=False): # max_plans=1
    max_time = convert_cost(max_planner_time)
    max_cost = convert_cost(scale_cost(max_cost))
    start_time = time()
    search = os.path.join(FD_BIN, SEARCH_COMMAND)

    domain_path = os.path.abspath(os.path.join(temp_dir, DOMAIN_INPUT))
    problem_path = os.path.abspath(os.path.join(temp_dir, PROBLEM_INPUT))

    clean_planner(planner)
    for filename in safe_listdir(temp_dir): # Deletes existing temp plans
        if filename.startswith(SEARCH_OUTPUT):
            safe_remove(os.path.join(temp_dir, filename))

    max_plans = int(min(DEFAULT_MAX_PLANS, max_plans))
    if planner == 'forbid':
        assert max_planner_time < INF
        command = os.path.join(os.environ['FORBID_PATH'], FORBID_TEMPLATE).format(
            max_time=max_planner_time, max_cost=max_cost, #max_plans=max_plans,
            domain=domain_path, problem=problem_path)
    elif planner == 'kstar':
        command = os.path.join(os.environ['KSTAR_PATH'], KSATAR_TEMPLATE).format(
            domain=domain_path, problem=problem_path, max_plans=max_plans,
            max_time=max_planner_time, max_cost=max_cost)
    elif planner == 'symk':
        command = os.path.join(os.environ['SYMK_PATH'], SYMK_TEMPLATE).format(
            domain=domain_path, problem=problem_path, max_plans=max_plans, max_cost=max_cost)
    # elif planner == 'cerberus':
    #     planner_config = SEARCH_OPTIONS[planner] # Check if max_time, max_cost exist
    #     command = search.format(temp_dir + SEARCH_OUTPUT, planner_config, temp_dir + TRANSLATE_OUTPUT)
    else:
        planner_config = SEARCH_OPTIONS[planner] % (max_time, max_cost)
        search_output = '{}.1'.format(SEARCH_OUTPUT)
        command = search.format(temp_dir + search_output, planner_config, temp_dir + TRANSLATE_OUTPUT)

    # https://stackoverflow.com/questions/1689505/python-ulimit-and-nice-for-subprocess-call-subprocess-popen
    # https://serverfault.com/questions/540904/ulimit-n-not-persisting-tried-everything
    # https://stackoverflow.com/questions/57536172/cannot-call-ubuntu-ulimit-from-python-subprocess-without-using-shell-option
    # https://docs.python.org/3/library/resource.html
    # TODO: max memory
    limit = 'unlimited' if max_time == 'inf' else max_time
    command = 'ulimit -t {}; {}'.format(limit, command)
    if debug:
        print('Search command:', command)

    # os.popen is deprecated
    # run, call, check_call, check_output
    #with subprocess.Popen(command.split(), stdout=subprocess.PIPE, shell=True, cwd=None) as proc:
    #    output = proc.stdout.read()
    # CalledProcessError
    #try:
    #    output = subprocess.check_output(command, shell=True, cwd=None) #, timeout=None)
    #except subprocess.CalledProcessError as e:
    #    print(e)

    #try:
    #proc = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, cwd=None, close_fds=True)
    #output, error = proc.communicate()
    # except subprocess.TimeoutExpired as e:
    #     pass

    # TODO: import psutil
    # https://github.com/caelan/SS-Replan/blob/6b57adeaa7094926199301fbf2ca1a70ea8e7927/run_experiment.py#L160

    output = None
    # BUFFER = 5 # 0 | 5
    # from examples.pybullet.utils.pybullet_tools.utils import timeout
    # with timeout(max_planner_time + BUFFER): # TODO: throws CalledProcessError if value is larger
    #assert get_python_version() == 3
    try:
        # https://stackoverflow.com/questions/1191374/using-module-subprocess-with-timeout
        output = subprocess.check_output(command, cwd=None, shell=True, stderr=subprocess.STDOUT) #, timeout=max_planner_time)
    except subprocess.CalledProcessError:
        pass

    diverse_plans_path = os.path.join(os.getcwd(), DIVERSE_DIR)
    if planner == 'forbid':
        forbid_plans_path = os.path.join(diverse_plans_path, 'done')
        for filename in safe_listdir(forbid_plans_path): # Copies plans from forbid to temp
            if filename.startswith(SEARCH_OUTPUT):
                os.rename(os.path.join(forbid_plans_path, filename), os.path.join(temp_dir, filename))
    elif planner in ['kstar', 'symk']:
        for filename in safe_listdir(diverse_plans_path):
            if filename.startswith(SEARCH_OUTPUT):
                os.rename(os.path.join(diverse_plans_path, filename), os.path.join(temp_dir, filename))
    clean_planner(planner)

    if debug:
        # if implies(USE_FORBID, 'TimeoutExpired' not in output):
        if output is not None:
            print(output.decode(encoding='UTF-8')[:-1])
        print('Search runtime:', elapsed_time(start_time))

    plan_files = sorted((f for f in safe_listdir(temp_dir) if f.startswith(SEARCH_OUTPUT)),
                        key=lambda f: int(f.split('.')[1]))
    print('Plans:', plan_files)
    return parse_solutions(temp_dir, plan_files)

##################################################

def parse_action(line):
    entries = line.strip('( )').split(' ')
    name = entries[0]
    args = tuple(entries[1:])
    return Action(name, args)

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

def parse_solutions(temp_dir, plan_files):
    solutions = [parse_solution(read(os.path.join(temp_dir, plan_file)))
                 for plan_file in plan_files]
    print('Found {} plans'.format(len(solutions))) # Best cost
    return sorted(solutions, key=lambda pair: pair[1])

def write_pddl(domain_pddl=None, problem_pddl=None, clean=False, temp_dir=TEMP_DIR):
    if clean:
        safe_rm_dir(temp_dir) # TODO: why is this here?
    ensure_dir(temp_dir)
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

def get_precondition(operator):
    if isinstance(operator, pddl.Action) or isinstance(operator, pddl.PropositionalAction):
        return operator.precondition
    elif isinstance(operator, pddl.Axiom) or isinstance(operator, pddl.PropositionalAxiom):
        return operator.condition
    raise ValueError(operator)

def get_conditional_effects(operator):
    if isinstance(operator, pddl.PropositionalAction):
        return [(cond, effect.negate()) for cond, effect in operator.del_effects] + \
               [(cond, effect) for cond, effect in operator.add_effects]
    elif isinstance(operator, pddl.PropositionalAxiom):
        return [([], operator.effect)]
    raise ValueError(operator)

def get_effects(operator):
    # TODO: conditional effects
    return [effect for _, effect in get_conditional_effects(operator)]

def is_applicable(state, action):
    return conditions_hold(state, get_precondition(action))

def apply_action(state, action):
    assert(isinstance(action, pddl.PropositionalAction))
    # TODO: signed literals
    for conditions, effect in action.del_effects:
        if conditions_hold(state, conditions):
            state.discard(effect)
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

def get_function_assignments(task):
    return {f.fluent: f.expression for f in task.init
            if isinstance(f, pddl.f_expression.FunctionAssignment)}

def get_action_instances(task, action_plan):
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    function_assignments = get_function_assignments(task)
    predicate_to_atoms = instantiate.get_atoms_by_predicate(task.init)
    fluent_facts = MockSet()
    init_facts = set()
    action_instances = []
    for name, objects in action_plan:
        # TODO: what if more than one action of the same name due to normalization?
        # Normalized actions have same effects, so I just have to pick one
        # TODO: conditional effects and internal parameters
        action = find_unique(lambda a: a.name == name, task.actions)
        args = list(map(pddl_from_object, objects))
        variable_mapping = {p.name: a for p, a in safe_zip(action.parameters, args)}
        instance = action.instantiate(variable_mapping, init_facts, fluent_facts, type_to_objects,
                                      task.use_min_cost_metric, function_assignments, predicate_to_atoms)
        assert (instance is not None)
        action_instances.append(instance)
    return action_instances

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


def has_conditional_effects(action_instance):
    for conditions, effect in (action_instance.add_effects + action_instance.del_effects):
        if conditions:
            return True
    return False


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


def plan_preimage(combined_plan, goal=[]):
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
    types = [pddl.Type(OBJECT)]
    pddl_parser.parsing_functions.set_supertypes(types)
    return Domain(name='', requirements=pddl.Requirements([]),
             types=types, type_dict={ty.name: ty for ty in types}, constants=constants,
             predicates=predicates, predicate_dict={p.name: p for p in predicates},
             functions=functions, actions=actions, axioms=axioms, pddl=None)


def pddl_from_instance(instance):
    action = instance.action
    args = [instance.var_mapping[p.name]
            for p in action.parameters[:action.num_external_parameters]]
    return Action(action.name, args)
