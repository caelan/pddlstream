import os
from collections import defaultdict
from collections import namedtuple
from heapq import heappush, heappop

from pddlstream.conversion import obj_from_pddl_plan, is_atom, evaluation_from_fact, fact_from_evaluation
from pddlstream.fast_downward import get_problem, task_from_domain_problem, get_init
from pddlstream.fast_downward import instantiate_task, run_search, safe_rm_dir, parse_solution, \
    pddl_to_sas, clear_dir, TEMP_DIR, TRANSLATE_OUTPUT
from pddlstream.relaxed_scheduling import fact_from_fd
from pddlstream.utils import Verbose, find
from pddlstream.visualization import visualize_constraints, visualize_stream_plan


# TODO: reuse the ground problem when solving for sequential subgoals

def action_preimage(action, preimage):
    for conditions, effect in action.add_effects + action.del_effects:
        assert(not conditions)
        if effect in preimage:
            preimage.remove(effect)
    preimage.update(action.precondition)


Node = namedtuple('Node', ['effort', 'stream_result']) # TODO: level

def get_stream_effort(evaluations, stream_results, op=sum):
    # TODO: could do this with bound_stream_instances instead
    unprocessed_from_atom = defaultdict(list)
    node_from_atom = {None: Node(0, None)}
    conditions_from_stream = {}
    remaining_from_stream = {}
    for stream_result in stream_results:
        conditions_from_stream[stream_result] = stream_result.stream_instance.get_domain() + (None,)
        remaining_from_stream[stream_result] = len(conditions_from_stream[stream_result])
        for atom in conditions_from_stream[stream_result]:
            unprocessed_from_atom[atom].append(stream_result)
    for atom in evaluations:
        if is_atom(atom):
            node_from_atom[fact_from_evaluation(atom)] = Node(0, None)

    queue = [(node.effort, atom) for atom, node in node_from_atom.items()]
    while queue:
        _, atom = heappop(queue)
        if atom not in unprocessed_from_atom:
            continue
        for stream_result in unprocessed_from_atom[atom]:
            remaining_from_stream[stream_result] -= 1
            if remaining_from_stream[stream_result]:
                continue
            effort = 1
            total_effort = op(node_from_atom[cond].effort for cond in conditions_from_stream[stream_result]) + effort
            for new_atom in stream_result.get_certified():
                if (new_atom not in node_from_atom) or (total_effort < node_from_atom[new_atom].effort):
                    node_from_atom[new_atom] = Node(total_effort, stream_result)
                    heappush(queue, (total_effort, new_atom))
        del unprocessed_from_atom[atom]
    del node_from_atom[None]
    return node_from_atom

def extract_stream_plan(node_from_atom, facts, stream_plan):
    for fact in facts:
        stream_result = node_from_atom[fact].stream_result
        if (stream_result is None) or (stream_result in stream_plan):
            continue
        extract_stream_plan(node_from_atom, stream_result.stream_instance.get_domain(), stream_plan)
        stream_plan.append(stream_result)

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, **kwargs):
    opt_evaluations = set(evaluations)
    for stream_result in stream_results:
        for fact in stream_result.get_certified():
            opt_evaluations.add(evaluation_from_fact(fact))
    task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain))

    with Verbose(False):
        ground_task = instantiate_task(task)
        sas_task = pddl_to_sas(ground_task)
        clear_dir(TEMP_DIR)
        with open(os.path.join(TEMP_DIR, TRANSLATE_OUTPUT), "w") as output_file:
            sas_task.output(output_file)
    solution = run_search(TEMP_DIR, verbose=False, **kwargs)
    safe_rm_dir(TEMP_DIR)
    action_plan, _ = parse_solution(solution)
    if action_plan is None:
        return None, action_plan

    for axiom in ground_task.axioms:
        print(axiom.condition)
        print(axiom.effect)
    assert(not ground_task.axioms)

    """
    #regex = r'(\(\w+(?:\s\w+)*\))'
    #print(re.findall(regex, solution))
    ground_from_name = defaultdict(list)
    for action in ground_task.actions:
        ground_from_name[action.name].append(action)
    action_plan = []
    for name in solution.split('\n')[:-2]:
        assert(len(ground_from_name[name]) == 1)
        # TODO: later select a particular ground action that satisfies the conditions
        action_plan.append(ground_from_name[name][0])
    
    print(plan_cost(action_plan))
    state = set(task.init)
    axiom_plan = []
    # TODO: remove conditional effects
    for action in action_plan:
        axiom_plan.append([])
        assert(is_applicable(state, action))
        apply_action(state, action)
        print(state)
    """

    import pddl
    import pddl_to_prolog
    import build_model
    import instantiate
    real_init = get_init(evaluations)
    opt_facts = set(task.init) - set(real_init)
    with Verbose(False):
        model = build_model.compute_model(pddl_to_prolog.translate(task))
        fluent_facts = instantiate.get_fluent_facts(task, model) | opt_facts
    task.init = real_init
    init_facts = set(task.init)
    function_assignments = {fact.fluent: fact.expression for fact in init_facts
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    fd_plan = []
    for name, args in action_plan:
        # TODO: use reachable params to instantiate (includes external)
        action = find(lambda a: a.name == name, domain.actions)
        assert(len(action.parameters) == len(args))
        variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        fd_plan.append(action.instantiate(variable_mapping, init_facts,
                                         fluent_facts, type_to_objects,
                                         task.use_min_cost_metric, function_assignments))

    preimage = set(ground_task.goal_list)
    for action in reversed(fd_plan):
        action_preimage(action, preimage)
    preimage -= init_facts
    preimage = filter(lambda l: not l.negated, preimage)
    # TODO: need to include all conditions
    # TODO: need to invert axioms back

    # TODO: prune with rules
    # TODO: linearization that takes into account satisfied goals at each level
    # TODO: backtrace streams and axioms
    # TODO: can optimize for streams & axioms all at once
    visualize_constraints(preimage)

    node_from_atom = get_stream_effort(evaluations, stream_results)
    stream_plan = []
    extract_stream_plan(node_from_atom, map(fact_from_fd, preimage), stream_plan)
    visualize_stream_plan(stream_plan)

    raw_input('continue?')

    return stream_plan, obj_from_pddl_plan(action_plan)
