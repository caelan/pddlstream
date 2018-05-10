import time

from experimental.context import ConstraintSolver
from experimental.focused import reset_disabled, process_immediate_stream_plan, \
    get_optimistic_constraints
from pddlstream.algorithm import parse_problem
from pddlstream.conversion import revert_solution, evaluation_from_fact
from pddlstream.focused import optimistic_process_stream_queue
from pddlstream.instantiation import Instantiator
from pddlstream.scheduling.sequential import sequential_stream_plan
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan
from pddlstream.utils import INF, elapsed_time
from pddlstream.visualization import clear_visualizations, create_visualizations


# Next set of stream_results
# 1) All non-disabled (focused)
# 2) Previous stream plan (will fail if any composition)
# 3) New stream results (some might not be performable). In some sense this is similar to the subtraction.
# 4) All non-disabled - previously considered
# 5) Stream results "useful" along the plan
# 6) Subset of evaluations

# TODO: maintain a FIFO/FILO queue of things you can try and pop until empty (can even avoid readding)

def solve_committed(problem, max_time=INF, effort_weight=None, visualize=False, verbose=True, **kwargs):
    # TODO: constrain plan skeleton
    # TODO: constrain ususable samples
    # TODO: recursively consider previously exposed binding levels
    # TODO: parameter for how many times to consider a plan skeleton

    # TODO: constrain to use previous plan skeleton
    # TODO: only use stream instances on plan
    # TODO: identify subset of state to include to further constrain (requires inverting axioms)
    # TODO: recurse to previous problems
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    constraint_solver = ConstraintSolver(problem[3])
    disabled = []
    if visualize:
        clear_visualizations()
    committed = False
    instantiator = Instantiator(evaluations, streams)
    #stream_results = []
    #while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
    #    stream_results += optimistic_process_stream_queue(instantiator, prioritized=False)
    # TODO: queue to always consider functions
    # TODO: can always append functions
    # Subproblems are which streams you can use
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('\nIteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        stream_results = []
        while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
            stream_results += optimistic_process_stream_queue(instantiator)
        solve_stream_plan = sequential_stream_plan if effort_weight is None else simultaneous_stream_plan
        #solve_stream_plan = relaxed_stream_plan
        # TODO: constrain to use previous plan to some degree
        stream_plan, action_plan, cost = solve_stream_plan(evaluations, goal_expression,
                                                     domain, stream_results, **kwargs)
        print('Stream plan: {}\n'
              'Action plan: {}'.format(stream_plan, action_plan))
        if stream_plan is None:
            if committed or disabled:
                if not committed:
                    reset_disabled(disabled)
                committed = False
                instantiator = Instantiator(evaluations, streams)
            else:
                break
        elif (len(stream_plan) == 0) and (cost < best_cost):
            best_plan = action_plan; best_cost = cost
            break
        else:
            if visualize:
                create_visualizations(evaluations, stream_plan, num_iterations)
            # TODO: use set of intended stream instances here instead
            #stream_results = []
            committed = True
            constraint_facts = constraint_solver.solve(get_optimistic_constraints(evaluations, stream_plan), verbose=verbose)
            if constraint_facts:
                new_evaluations = map(evaluation_from_fact, constraint_facts)
                evaluations.update(new_evaluations)
            else:
                #new_evaluations = process_stream_plan(evaluations, stream_plan, disabled, verbose)
                new_evaluations = process_immediate_stream_plan(evaluations, stream_plan, disabled, verbose)
                for evaluation in new_evaluations:
                    instantiator.add_atom(evaluation) # TODO: return things to try next
                #while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
                #    stream_results += optimistic_process_stream_queue(instantiator, prioritized=False)
                #stream_results = stream_plan # TODO: would need to prune disabled
                # TODO: don't include streams that aren't performable?
                # TODO: could also only include the previous stream plan
                # TODO: need to be careful if I only instantiate one that I am not unable to find a plan
                # TODO: need to always propagate this a little
    return revert_solution(best_plan, best_cost, evaluations)