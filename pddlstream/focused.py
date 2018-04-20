from pddlstream.conversion import get_pddl_problem, value_from_obj_plan, \
    obj_from_pddl_plan, substitute_expression, Head, get_prefix, get_args, Evaluation, \
    init_from_evaluations, evaluations_from_init, convert_expression, values_from_objects, objects_from_values
from pddlstream.fast_downward import run_fast_downward, parse_domain
from pddlstream.instantiation import Instantiator
from pddlstream.problem import Object
from pddlstream.stream import parse_stream
from pddlstream.utils import INF, elapsed_time
import time

from pddlstream.incremental import parse_problem, solve_finite, revert_solution

def solve_focused(problem, max_time=INF, **kwargs):
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    domain_pddl = problem[2]
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))








    plan, cost = solve_finite(evaluations, goal_expression, domain, domain_pddl, **kwargs)
    return revert_solution(plan, cost, evaluations)