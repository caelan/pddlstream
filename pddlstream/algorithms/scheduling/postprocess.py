from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, sas_from_pddl
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan
from pddlstream.algorithms.scheduling.simultaneous import get_stream_actions
from pddlstream.algorithms.search import solve_from_task
from pddlstream.language.constants import And
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.utils import flatten

#RESCHEDULE_PLANNER = 'ff-astar'
RESCHEDULE_PLANNER = 'lmcut-astar'
#RESCHEDULE_PLANNER = 'ff-lazy'

def reschedule_stream_plan(evaluations, target_facts, domain, stream_results, unique_binding=False,
                           unit_efforts=True):
    # TODO: search in space of partially ordered plans
    # TODO: constrain selection order to be alphabetical?
    goal_expression = And(*target_facts)
    reschedule_problem = get_problem(evaluations, goal_expression, domain, unit_costs=unit_efforts)
    reschedule_task = task_from_domain_problem(domain, reschedule_problem)
    reschedule_task.actions, stream_result_from_name = get_stream_actions(
        stream_results, unique_binding=unique_binding, unit_efforts=unit_efforts)
    #reschedule_task.axioms = [] # TODO: ensure that the constants are added in the event that axioms are needed?
    sas_task = sas_from_pddl(reschedule_task)
    stream_names, effort = solve_from_task(sas_task, planner=RESCHEDULE_PLANNER, max_planner_time=10, debug=False)
    if stream_names is None:
        return None
    stream_plan = [stream_result_from_name[name] for name, _ in stream_names]
    return stream_plan

##################################################

def shorten_stream_plan(evaluations, stream_plan, target_facts):
    all_subgoals = set(target_facts) | set(flatten(r.instance.get_domain() for r in stream_plan))
    evaluation_subgoals = set(filter(evaluations.__contains__, map(evaluation_from_fact, all_subgoals)))
    open_subgoals = set(filter(lambda f: evaluation_from_fact(f) not in evaluations, all_subgoals))
    results_from_fact = {}
    for result in stream_plan:
        for fact in result.get_certified():
            results_from_fact.setdefault(fact, []).append(result)

    for removed_result in reversed(stream_plan): # TODO: only do in order?
        certified_subgoals = open_subgoals & set(removed_result.get_certified())
        if not certified_subgoals: # Could combine with following
            new_stream_plan = stream_plan[:]
            new_stream_plan.remove(removed_result)
            return new_stream_plan
        if all(2 <= len(results_from_fact[fact]) for fact in certified_subgoals):
            node_from_atom = get_achieving_streams(evaluation_subgoals, set(stream_plan) - {removed_result})
            if all(fact in node_from_atom for fact in target_facts):
                new_stream_plan = []
                extract_stream_plan(node_from_atom, target_facts, new_stream_plan)
                return new_stream_plan
    return None


def prune_stream_plan(evaluations, stream_plan, target_facts):
    while True:
        new_stream_plan = shorten_stream_plan(evaluations, stream_plan, target_facts)
        if new_stream_plan is None:
            break
        stream_plan = new_stream_plan
    return stream_plan