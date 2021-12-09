import argparse
import time

from collections import defaultdict

from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.common import evaluations_from_init
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, fact_from_fd, fd_from_fact, \
    fd_from_evaluations, INTERNAL_AXIOM
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.focused import solve_focused_original, solve_binding, solve_adaptive, get_negative_externals
from pddlstream.algorithms.instantiate_task import instantiate_task, convert_instantiated
from pddlstream.algorithms.refinement import optimistic_process_streams
from pddlstream.algorithms.scheduling.reinstantiate import reinstantiate_axiom
from pddlstream.algorithms.scheduling.recover_streams import evaluations_from_stream_plan
from pddlstream.language.constants import is_plan, Certificate, PDDLProblem, get_prefix, Solution
from pddlstream.language.conversion import value_from_obj_expression, EQ
from pddlstream.language.external import DEBUG, SHARED_DEBUG
from pddlstream.language.stream import PartialInputs
from pddlstream.language.temporal import SimplifiedDomain
from pddlstream.utils import elapsed_time, INF, Verbose, irange, SEPARATOR

FOCUSED_ALGORITHMS = ['focused', 'binding', 'adaptive']
ALGORITHMS = ['incremental'] + FOCUSED_ALGORITHMS
DEFAULT_ALGORITHM = 'adaptive'

##################################################

def create_parser(default_algorithm=DEFAULT_ALGORITHM):
    # https://docs.python.org/3/library/argparse.html#the-add-argument-method
    parser = argparse.ArgumentParser() # Automatically includes help
    parser.add_argument('-a', '--algorithm', type=str, default=default_algorithm, choices=ALGORITHMS, required=False,
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
          max_skeletons=INF, search_sample_ratio=1, max_failures=0,
          unit_efforts=False, max_effort=INF, effort_weight=None, reorder=True,
          #temp_dir=TEMP_DIR, clean=False, debug=False, hierarchy=[],
          #planner=DEFAULT_PLANNER, max_planner_time=DEFAULT_MAX_TIME, max_cost=INF, debug=False
          visualize=False, verbose=True, **search_kwargs):
    """
    Solves a PDDLStream problem generically using one of the available algorithms
    :param problem: a PDDLStream problem
    :param algorithm: a PDDLStream algorithm name
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
    # TODO: portfolios of PDDLStream algorithms
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
    #         max_time=max_time, max_iterations=max_iterations, max_memory=max_memory,
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
            max_time=max_time, max_iterations=max_iterations, max_memory=max_memory,
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
            max_time=max_time, max_iterations=max_iterations, max_memory=max_memory,
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
            max_time=max_time, max_iterations=max_iterations, max_memory=max_memory,
            initial_complexity=initial_complexity, complexity_step=complexity_step, max_complexity=max_complexity,
            max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
            #bind=bind, max_failures=max_failures,
            unit_efforts=unit_efforts, max_effort=max_effort, effort_weight=effort_weight, reorder=reorder,
            visualize=visualize, verbose=verbose, **search_kwargs)
    raise NotImplementedError(algorithm)

##################################################

def solve_restart(problem, max_time=INF, max_restarts=0, iteration_time=INF, abort=True, **kwargs):
    # TODO: iteratively lower the cost bound
    # TODO: a sequence of different planner configurations
    # TODO: reset objects and/or streams
    if (max_restarts >= 1) and (iteration_time == INF):
        iteration_time = min(2 * 60, iteration_time)
    assert (max_restarts == 0) or (iteration_time != INF)

    assert max_restarts >= 0
    start_time = time.time()
    for attempt in irange(1+max_restarts):
        iteration_start_time = time.time()
        if elapsed_time(start_time) > max_time:
            break
        if attempt >= 1:
            print(SEPARATOR)
        #solution = planner_fn(problem) # Or include the problem in the lambda
        remaining_time = min(iteration_time, max_time-elapsed_time(start_time))
        solution = solve(problem, max_time=remaining_time, **kwargs)
        plan, cost, certificate = solution
        if is_plan(plan): # TODO: INFEASIBLE
            return solution
        if abort and (elapsed_time(iteration_start_time) < remaining_time):
            break # TODO: return the cause of failure

    certificate = Certificate(all_facts=[], preimage_facts=[]) # TODO: aggregate
    return Solution(None, INF, certificate)

##################################################

def set_unique(externals):
    for external in externals:
        external.info.opt_gen_fn = PartialInputs(unique=True)
        external.num_opt_fns = 0

def examine_instantiated(problem, unique=False, normalize=True, unit_costs=False, verbose=False, debug=False, **kwargs):
    # TODO: refactor to an analysis file
    domain_pddl, constant_map, stream_pddl, _, init, goal = problem
    stream_map = DEBUG if unique else SHARED_DEBUG # DEBUG_MODES
    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    evaluations, goal_exp, domain, externals = parse_problem(problem, **kwargs)
    assert not isinstance(domain, SimplifiedDomain)
    negative = get_negative_externals(externals)
    externals = list(filter(lambda s: s not in negative, externals))

    # store = SolutionStore(evaluations, max_time, success_cost=INF, verbose=verbose)
    # instantiator = Instantiator(externals, evaluations)
    # process_stream_queue(instantiator, store, complexity_limit=INF, verbose=verbose)
    # results = [] # TODO: extract from process_stream_queue

    #set_unique(externals)
    # domain.actions[:] = [] # TODO: only instantiate axioms
    # TODO: drop all fluents and instantiate
    # TODO: relaxed planning version of this
    results, exhausted = optimistic_process_streams(evaluations, externals, complexity_limit=INF, max_effort=None)

    evaluations = evaluations_from_stream_plan(evaluations, results, max_effort=None)
    problem = get_problem(evaluations, goal_exp, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    with Verbose(debug):
        instantiated = instantiate_task(task, check_infeasible=False)
        if instantiated is None:
            return results, None
        # TODO: reinstantiate actions?
        instantiated.axioms[:] = [reinstantiate_axiom(axiom) for axiom in instantiated.axioms]
        if normalize:
            instantiated = convert_instantiated(instantiated)
    return results, instantiated
    # sas_task = sas_from_pddl(task, debug=debug)

##################################################

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


def recurse_subgoals(goals, condition_from_effect):
    possible = set()

    def recurse(goal):
        if goal in possible:
            return
        possible.add(goal)
        for condition in condition_from_effect[goal]:
            recurse(condition)

    for goal in goals:
        recurse(goal)
    return possible


def analyze_goal(problem, use_actions=False, use_axioms=True, use_streams=True, blocked_predicates=[], **kwargs):
    # TODO: instantiate all goal partial states
    # TODO: remove actions/axioms that never could achieve a subgoal
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = problem
    evaluations = evaluations_from_init(init)
    init = set(fd_from_evaluations(evaluations))

    # from pddlstream.algorithms.scheduling.recover_axioms import recover_axioms_plans
    results, instantiated = examine_instantiated(problem, **kwargs) # TODO: only do if the goals are derived
    if instantiated is None:
        return None
    #optimistic_init = set(instantiated.task.init)

    # This is like backchaining in a relaxed space
    condition_from_effect = defaultdict(set)
    if use_actions:
        # TODO: selectively ignore some conditions (e.g. HandEmpty)
        # TODO: refactor into separate method
        for action in instantiated.actions:
            for conditional, effect in action.add_effects:
                for condition in (action.precondition + conditional):
                    if condition.predicate not in blocked_predicates:
                        condition_from_effect[effect].add(condition)
            for conditional, effect in action.del_effects:
                for condition in (action.precondition + conditional):
                    if condition.predicate not in blocked_predicates:
                        condition_from_effect[effect.negate()].add(condition)
    if use_axioms:
        # TODO: axiom_rules.handle_axioms(...)
        # print('Axioms:', instantiated.axioms)
        for axiom in instantiated.axioms:
            #axiom = reinstantiate_axiom(axiom)
            #axiom.dump()
            for condition in axiom.condition:
                condition_from_effect[axiom.effect].add(condition)
    if use_streams:
        for result in results:
            for effect in result.certified:
                if get_prefix(effect) == EQ:
                    continue
                for condition in result.domain:
                    condition_from_effect[fd_from_fact(effect)].add(fd_from_fact(condition))

    print('Goals:', list(map(fact_from_fd, instantiated.goal_list)))
    #all_subgoals = iterate_subgoals(instantiated.goal_list, axiom_from_effect)
    all_subgoals = recurse_subgoals(instantiated.goal_list, condition_from_effect)
    filtered_subgoals = [subgoal for subgoal in all_subgoals if subgoal in init] # TODO: return the goals as well?
    external_subgoals = [value_from_obj_expression(fact_from_fd(subgoal))
                         for subgoal in sorted(filtered_subgoals, key=lambda g: g.predicate)
                         if not subgoal.predicate.startswith(INTERNAL_AXIOM)]
    print('Initial:', external_subgoals)

    return external_subgoals # TODO: decompose into simplified components
