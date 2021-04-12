import argparse
import time

from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.common import SolutionStore
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, fact_from_fd
from pddlstream.algorithms.incremental import solve_incremental, process_stream_queue
from pddlstream.algorithms.focused import solve_focused, solve_focused_original, solve_binding, solve_adaptive
from pddlstream.algorithms.instantiate_task import instantiate_task, convert_instantiated
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.scheduling.reinstantiate import reinstantiate_axiom
from pddlstream.language.constants import is_plan, Certificate, PDDLProblem, get_prefix, get_args
from pddlstream.language.conversion import values_from_objects, value_from_obj_expression, NOT
from pddlstream.language.external import DEBUG
from pddlstream.language.temporal import SimplifiedDomain, parse_domain
from pddlstream.utils import elapsed_time, INF, Verbose

FOCUSED_ALGORITHMS = ['focused', 'binding', 'adaptive']
ALGORITHMS = ['incremental'] + FOCUSED_ALGORITHMS
DEFAULT_ALGORITHM = 'adaptive'

##################################################

def create_parser():
    # https://docs.python.org/3/library/argparse.html#the-add-argument-method
    parser = argparse.ArgumentParser() # Automatically includes help
    parser.add_argument('-a', '--algorithm', type=str, default=DEFAULT_ALGORITHM, choices=ALGORITHMS, required=False,
                        help='Specifies the PDDLStream algorithm to use')
    parser.add_argument('-u', '--unit', action='store_true', help='Uses unit costs') # --unit_costs
    # args = parser.parse_args()
    # print('Arguments:', args)
    # TODO: search planner, debug
    # TODO: method that calls solve with args
    return parser

##################################################

def solve(problem, algorithm=DEFAULT_ALGORITHM, constraints=PlanConstraints(),
          stream_info={}, replan_actions=set(),
          unit_costs=False, success_cost=INF,
          max_time=INF, max_iterations=INF, max_memory=INF,
          initial_complexity=0, complexity_step=1, max_complexity=INF,
          max_skeletons=INF, search_sample_ratio=0, max_failures=0,
          unit_efforts=False, max_effort=INF, effort_weight=None, reorder=True,
          #temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=[],
          #planner=DEFAULT_PLANNER, max_planner_time=DEFAULT_MAX_TIME, max_cost=INF, debug=False
          visualize=False, verbose=True, **search_kwargs):
    """
    Solves a PDDLStream problem generically using one of the available algorithms
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the set of legal solutions
    :param stream_info: a dictionary from stream name to StreamInfo altering how individual streams are handled
    :param replan_actions: the actions declared to induce replanning for the purpose of deferred stream evaluation

    :param unit_costs: use unit action costs rather than numeric costs
    :param success_cost: the exclusive (strict) upper bound on plan cost to successfully terminate
    :param max_time: the maximum runtime
    :param max_iterations: the maximum number of search iterations
    :param max_memory: the maximum amount of memory

    :param initial_complexity: the initial stream complexity limit
    :param complexity_step: the increase in the stream complexity limit per iteration
    :param max_complexity: the maximum stream complexity limit

    :param max_skeletons: the maximum number of plan skeletons (max_skeletons=None indicates not adaptive)
    :param search_sample_ratio: the desired ratio of sample time / search time when max_skeletons!=None
    :param max_failures: the maximum number of stream failures before switching phases when max_skeletons=None

    :param unit_efforts: use unit stream efforts rather than estimated numeric efforts
    :param max_effort: the maximum amount of stream effort
    :param effort_weight: a multiplier for stream effort compared to action costs
    :param reorder: if True, reorder stream plans to minimize the expected sampling overhead

    :param visualize: if True, draw the constraint network and stream plan as a graphviz file
    :param verbose: if True, print the result of each stream application
    :param search_kwargs: keyword args for the search subroutine

    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan (INF if no plan), and evaluations is init expanded
        using stream applications
    """

    # TODO: print the arguments using locals()
    # TODO: could instead make common arguments kwargs but then they could have different default values
    if algorithm == 'incremental':
        return solve_incremental(
            problem=problem, constraints=constraints,
            unit_costs=unit_costs, success_cost=success_cost,
            max_iterations=max_iterations, max_time=max_time, max_memory=max_memory,
            initial_complexity=initial_complexity, complexity_step=complexity_step, max_complexity=max_complexity,
            verbose=verbose, **search_kwargs)

    # if algorithm == 'abstract_focused': # meta_focused | meta_focused
    #     return solve_focused(
    #         problem, constraints=constraints,
    #         stream_info=stream_info, replan_actions=replan_actions,
    #         unit_costs=unit_costs, success_cost=success_cost,
    #         max_time=INF, max_iterations=INF, max_memory=INF,
    #         initial_complexity=initial_complexity, complexity_step=complexity_step, #max_complexity=max_complexity,
    #         max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
    #         bind=bind, max_failures=max_failures,
    #         unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
    #         visualize=visualize, verbose=verbose, **search_kwargs)

    fail_fast = (max_failures < INF)
    if algorithm == 'focused':
        return solve_focused_original(
            problem, constraints=constraints,
            stream_info=stream_info, replan_actions=replan_actions,
            unit_costs=unit_costs, success_cost=success_cost,
            max_time=INF, max_iterations=INF, max_memory=INF,
            initial_complexity=initial_complexity, complexity_step=complexity_step, max_complexity=max_complexity,
            #max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
            fail_fast=fail_fast, # bind=bind, max_failures=max_failures,
            unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
            visualize=visualize, verbose=verbose, **search_kwargs)

    if algorithm == 'binding':
        return solve_binding(
            problem, constraints=constraints,
            stream_info=stream_info, replan_actions=replan_actions,
            unit_costs=unit_costs, success_cost=success_cost,
            max_time=INF, max_iterations=INF, max_memory=INF,
            initial_complexity=initial_complexity, complexity_step=complexity_step, max_complexity=max_complexity,
            #max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
            fail_fast=fail_fast, # bind=bind, max_failures=max_failures,
            unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
            visualize=visualize, verbose=verbose, **search_kwargs)

    if algorithm == 'adaptive':
        return solve_adaptive(
            problem, constraints=constraints,
            stream_info=stream_info, replan_actions=replan_actions,
            unit_costs=unit_costs, success_cost=success_cost,
            max_time=INF, max_iterations=INF, max_memory=INF,
            initial_complexity=initial_complexity, complexity_step=complexity_step, max_complexity=max_complexity,
            max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
            #bind=bind, max_failures=max_failures,
            unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
            visualize=visualize, verbose=verbose, **search_kwargs)
    raise NotImplementedError(algorithm)

##################################################

def restart(problem, planner_fn, max_time=INF, max_restarts=0, abort=False):
    # TODO: iteratively lower the cost bound
    # TODO: a sequence of different planner configurations
    # TODO: reset objects and/or streams

    assert max_restarts >= 0
    start_time = time.time()
    for attempt in range(1+max_restarts):
        if elapsed_time(start_time) > max_time:
            break
        solution = planner_fn(problem) # Or include the problem in the lambda
        plan, cost, certificate = solution
        if is_plan(plan):
            return solution

    certificate = Certificate(all_facts=[], preimage_facts=[]) # TODO: aggregate
    return None, INF, certificate

##################################################

def create_actionless_problem(problem, use_streams=False, new_goal=None):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = problem
    if not use_streams:
        stream_pddl = None
    if new_goal is None:
        new_goal = goal
    domain = parse_domain(domain_pddl) # TODO: Constant map value @base not mentioned in domain :constants
    domain.actions[:] = [] # No actions
    return PDDLProblem(domain, constant_map, stream_pddl, stream_map, init, new_goal)

##################################################

def examine_instantiated(problem, constraints=PlanConstraints(), unit_costs=False, max_time=INF,
                         verbose=False, debug=False, **search_kwargs):
    # TODO: refactor to an analysis file
    domain_pddl, constant_map, stream_pddl, _, init, goal = problem
    stream_map = DEBUG
    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    evaluations, goal_exp, domain, externals = parse_problem(
        problem, constraints=constraints, unit_costs=unit_costs)
    store = SolutionStore(evaluations, max_time, success_cost=INF, verbose=verbose)
    #externals = compile_fluents_as_attachments(domain, externals) #

    #from pddlstream.algorithms.refinement import iterative_plan_streams, optimistic_process_streams
    #results, exhausted = optimistic_process_streams(complexity_evals, externals, complexity_limit=INF) #, **effort_args)

    instantiator = Instantiator(externals, evaluations)
    process_stream_queue(instantiator, store, complexity_limit=INF, verbose=verbose) # TODO: optimistic instead

    #plan, cost = solve_finite(evaluations, goal_exp, domain, max_cost=max_cost, **search_args)
    assert not isinstance(domain, SimplifiedDomain)
    problem = get_problem(evaluations, goal_exp, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    with Verbose(debug):
        instantiated = instantiate_task(task)
        if instantiated is None:
            return None
        instantiated.axioms[:] = [reinstantiate_axiom(axiom) for axiom in instantiated.axioms]
        instantiated = convert_instantiated(instantiated)
    return instantiated
    # sas_task = sas_from_pddl(task, debug=debug)

##################################################

INTERNAL_AXIOM = 'new-axiom'

def iterate_subgoals(goals, axiom_from_effect):
    necessary = set()
    possible = set()
    for goal in goals:
        if goal in axiom_from_effect:
            necessary.update(set.intersection(*[set(axiom.condition) for axiom in axiom_from_effect[goal]]))
            #print(len(axiom_from_effect[goal]) == 1)  # Universal
            for axiom in axiom_from_effect[goal]:
                possible.update(axiom.condition)  # Add goal as well?
        else:
            necessary.add(goal)
    print('Necessary:', necessary)
    print('Possible:', possible - necessary)
    return possible


def recurse_subgoals(goals, axiom_from_effect):
    possible = set()

    def recurse(goal):
        if goal in possible:
            return
        possible.add(goal)
        if goal in axiom_from_effect:
            for axiom in axiom_from_effect[goal]:
                for subgoal in axiom.condition:
                    recurse(subgoal)

    for goal in goals:
        recurse(goal)
    return possible


def analyze_goal(problem):
    # from pddlstream.algorithms.scheduling.recover_axioms import recover_axioms_plans
    instantiated = examine_instantiated(problem, unit_costs=False)
    if instantiated is None:
        return None

    # TODO: axiom_rules.handle_axioms(...)
    print('Goals:', instantiated.goal_list)
    #print('Axioms:', instantiated.axioms)
    axiom_from_effect = {}
    for axiom in instantiated.axioms:
        #axiom = reinstantiate_axiom(axiom)
        #axiom.dump()
        axiom_from_effect.setdefault(axiom.effect, []).append(axiom)

    # TODO: could also remove fluents and instantiate given the initial state
    # Or don't remove but assume that they hold but are fluent
    #possible = iterate_subgoals(instantiated.goal_list, axiom_from_effect)
    possible = recurse_subgoals(instantiated.goal_list, axiom_from_effect)
    possible = sorted([subgoal for subgoal in possible if not subgoal.predicate.startswith('new-axiom')],
                      key=lambda g: g.predicate)
    possible = list(map(fact_from_fd, possible))
    possible = list(map(value_from_obj_expression, possible))
    print('Possible:', possible)
    return possible # TODO: decompose into simplified components
