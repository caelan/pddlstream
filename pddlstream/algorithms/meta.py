import argparse
import time

from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.common import SolutionStore
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem
from pddlstream.algorithms.incremental import solve_incremental, process_stream_queue
from pddlstream.algorithms.focused import solve_focused, solve_focused_original, solve_binding, solve_adaptive
from pddlstream.algorithms.instantiate_task import instantiate_task, convert_instantiated
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.language.constants import is_plan, Certificate, PDDLProblem
from pddlstream.language.external import DEBUG
from pddlstream.language.temporal import SimplifiedDomain
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
          max_skeletons=INF, search_sample_ratio=0, bind=True, max_failures=0,
          unit_efforts=False, max_effort=INF, effort_weight=None, reorder=True,
          visualize=False, verbose=True, **search_kwargs):

    # TODO: print the arguments using locals()
    # TODO: could instead make common arguments kwargs but then they could have different default values
    if algorithm == 'incremental':
        return solve_incremental(
            problem=problem, constraints=constraints,
            unit_costs=unit_costs, success_cost=success_cost,
            max_iterations=max_iterations, max_time=max_time, max_memory=max_memory,
            initial_complexity=initial_complexity, complexity_step=complexity_step, max_complexity=max_complexity,
            verbose=verbose, **search_kwargs)

    if algorithm == 'abstract_focused': # meta_focused | meta_focused
        return solve_focused(
            problem, constraints=constraints,
            stream_info=stream_info, replan_actions=replan_actions,
            unit_costs=unit_costs, success_cost=success_cost,
            max_time=INF, max_iterations=INF, max_memory=INF,
            initial_complexity=initial_complexity, complexity_step=complexity_step, #max_complexity=max_complexity,
            max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
            bind=bind, max_failures=max_failures,
            unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
            visualize=visualize, verbose=verbose, **search_kwargs)

    if algorithm == 'focused':
        return solve_focused_original(
            problem, constraints=constraints,
            stream_info=stream_info, replan_actions=replan_actions,
            unit_costs=unit_costs, success_cost=success_cost,
            max_time=INF, max_iterations=INF, max_memory=INF,
            initial_complexity=initial_complexity, complexity_step=complexity_step, #max_complexity=max_complexity,
            #max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
            fail_fast=(max_failures < INF), # bind=bind, max_failures=max_failures,
            unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
            visualize=visualize, verbose=verbose, **search_kwargs)

    if algorithm == 'binding':
        return solve_binding(
            problem, constraints=constraints,
            stream_info=stream_info, replan_actions=replan_actions,
            unit_costs=unit_costs, success_cost=success_cost,
            max_time=INF, max_iterations=INF, max_memory=INF,
            initial_complexity=initial_complexity, complexity_step=complexity_step, #max_complexity=max_complexity,
            #max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
            fail_fast=(max_failures < INF), # bind=bind, max_failures=max_failures,
            unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
            visualize=visualize, verbose=verbose, **search_kwargs)

    if algorithm == 'adaptive':
        return solve_adaptive(
            problem, constraints=constraints,
            stream_info=stream_info, replan_actions=replan_actions,
            unit_costs=unit_costs, success_cost=success_cost,
            max_time=INF, max_iterations=INF, max_memory=INF,
            initial_complexity=initial_complexity, complexity_step=complexity_step, #max_complexity=max_complexity,
            max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
            #bind=bind, max_failures=max_failures,
            unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
            visualize=visualize, verbose=verbose, **search_kwargs)
    raise NotImplementedError(algorithm)

##################################################

def restart(problem, planner_fn, max_time=INF, max_restarts=0):
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

def examine_instantiated(problem, constraints=PlanConstraints(), unit_costs=False, max_time=INF, verbose=False, **search_kwargs):
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
    process_stream_queue(instantiator, store, complexity_limit=INF, verbose=verbose)

    #plan, cost = solve_finite(evaluations, goal_exp, domain, max_cost=max_cost, **search_args)
    debug = False
    assert not isinstance(domain, SimplifiedDomain)
    problem = get_problem(evaluations, goal_exp, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    with Verbose(debug):
        instantiated = instantiate_task(task)
        if instantiated is None:
            return None
        instantiated = convert_instantiated(instantiated)
    return instantiated

    #max_cost = min(store.best_cost, constraints.max_cost)
    # sas_task = sas_from_pddl(task, debug=debug)
    # pddl_plan, cost = abstrips_solve_from_task(sas_task, max_cost=max_cost, debug=debug, **search_args)
    # plan = obj_from_pddl_plan(pddl_plan)
    # if is_plan(plan):
    #     store.add_plan(plan, cost)
    # return store.extract_solution()
