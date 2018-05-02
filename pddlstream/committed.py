import time

from pddlstream.scheduling.sequential import sequential_stream_plan
from pddlstream.scheduling.relaxed import relaxed_stream_plan

from pddlstream.algorithm import parse_problem, optimistic_process_stream_queue
from pddlstream.conversion import revert_solution, evaluation_from_fact
from pddlstream.focused import reset_disabled, process_stream_plan, process_immediate_stream_plan, \
    ConstraintSolver, get_optimistic_constraints
from pddlstream.instantiation import Instantiator
from pddlstream.scheduling.simultaneous import simultaneous_stream_plan
from pddlstream.utils import INF, elapsed_time


# TODO: display a plan skeleton as a constraint graph

def solve_committed(problem, max_time=INF, effort_weight=None, verbose=True, **kwargs):
    # TODO: constrain plan skeleton
    # TODO: constrain ususable samples
    # TODO: recursively consider previously exposed binding levels
    # TODO: parameter for how many times to consider a plan skeleton
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    constraint_solver = ConstraintSolver(problem[3])
    disabled = []

    committed = False
    instantiator = Instantiator(evaluations, streams)
    while elapsed_time(start_time) < max_time:
        num_iterations += 1
        print('\nIteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        stream_results = []
        while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
            stream_results += optimistic_process_stream_queue(instantiator, prioritized=False)
        solve_stream_plan = sequential_stream_plan if effort_weight is None else simultaneous_stream_plan
        #solve_stream_plan = relaxed_stream_plan
        stream_plan, action_plan, cost = solve_stream_plan(evaluations, goal_expression,
                                                     domain, stream_results, **kwargs)
        print('Stream plan: {}\n'
              'Action plan: {}'.format(stream_plan, action_plan))
        if stream_plan is None:
            #if instantiator.stream_instances:
            #if len(instantiator.stream_instances) < len(instantiator.stream_queue):
            if committed or disabled:
                if not committed:
                    reset_disabled(disabled)
                committed = False
                instantiator = Instantiator(evaluations, streams)
                #instantiator.stream_instances.clear()
            else:
                break
        elif (len(stream_plan) == 0) and (cost < best_cost):
                best_plan = action_plan;
                best_cost = cost
                break
        else:
            # TODO: use set of intended stream instances here instead
            committed = True
            constraint_facts = constraint_solver.solve(get_optimistic_constraints(evaluations, stream_plan), verbose=verbose)
            if constraint_facts:
                new_evaluations = map(evaluation_from_fact, constraint_facts)
                evaluations.update(new_evaluations)
                #for evaluation in new_evaluations: # TODO: can include this here or not
                #    instantiator.add_atom(evaluation)
                # TODO: return things to try next
            else:
                #new_evaluations = process_stream_plan(evaluations, stream_plan, disabled, verbose)
                new_evaluations = process_immediate_stream_plan(evaluations, stream_plan, disabled, verbose)
                for evaluation in new_evaluations:
                    instantiator.add_atom(evaluation)
                # TODO: constrain to use previous plan skeleton
                # TODO: only use stream instances on plan
                # TODO: identify subset of state to include to further constrain (requires inverting axioms)
            #if not new_evaluations:
            #    instantiator.stream_instances.clear()
    return revert_solution(best_plan, best_cost, evaluations)