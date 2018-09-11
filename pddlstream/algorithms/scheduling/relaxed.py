from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, apply_action, fact_from_fd, \
    conditions_hold, fd_from_fact, get_goal_instance, plan_preimage, get_literals
from pddlstream.algorithms.scheduling.recover_axioms import get_achieving_axioms, extract_axioms, \
    get_derived_predicates, get_necessary_axioms, instantiate_necessary_axioms
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan
from pddlstream.algorithms.scheduling.simultaneous import evaluations_from_stream_plan, extract_function_results, \
    get_results_from_head, get_stream_actions
from pddlstream.algorithms.search import abstrips_solve_from_task, solve_from_task
from pddlstream.language.conversion import obj_from_pddl_plan, obj_from_pddl, evaluation_from_fact
from pddlstream.language.constants import And, is_parameter
from pddlstream.language.function import PredicateResult, Predicate
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.utils import Verbose, MockSet, INF, flatten

from collections import defaultdict

DO_RESCHEDULE = False
RESCHEDULE_PLANNER = 'ff-astar' # TODO: investigate other admissible heuristics

# TODO: reuse the ground problem when solving for sequential subgoals

#def get_predicate_result():
#    pass # TODO: refactor

def instantiate_actions(opt_task, type_to_objects, function_assignments, action_plan):
    action_instances = []
    for name, args in action_plan: # TODO: negative atoms in actions
        candidates = []
        for action in opt_task.actions:
            if action.name != name:
                continue
            if len(action.parameters) != len(args):
                raise NotImplementedError('Existential quantifiers are not currently '
                                          'supported in preconditions: {}'.format(name))
            variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
            action_instance = action.instantiate(variable_mapping, set(), MockSet(), type_to_objects,
                                          opt_task.use_min_cost_metric, function_assignments)
            assert (action_instance is not None)
            candidates.append(((action, args), action_instance))
        if not candidates:
            raise RuntimeError('Could not find an applicable action {}'.format(name))
        action_instances.append(candidates)
    action_instances.append([(None, get_goal_instance(opt_task.goal))])
    return action_instances

def simplify_conditional_effects(real_state, opt_state, action_instance, axioms_from_name):
    # TODO: compute required stream facts in a forward way and allow opt facts that are already known required
    for effects in [action_instance.add_effects, action_instance.del_effects]:
        for i, (conditions, effect) in reversed(list(enumerate(effects))):
            if any(c.predicate in axioms_from_name for c in conditions):
                raise NotImplementedError('Conditional effects cannot currently involve derived predicates')
            if conditions_hold(real_state, conditions):
                # Holds in real state
                effects[i] = ([], effect)
            elif not conditions_hold(opt_state, conditions):
                # Does not hold in optimistic state
                effects.pop(i)
            else:
                # TODO: handle more general case where can choose to achieve particular conditional effects
                raise NotImplementedError('Conditional effects cannot currently involve certified predicates')

def convert_negative(negative_preimage, negative_from_name, step_from_atom, real_states):
    negative_plan = set()
    for literal in negative_preimage:
        negative = negative_from_name[literal.predicate]
        if isinstance(negative, Predicate):
            predicate_instance = negative.get_instance(map(obj_from_pddl, literal.args))
            value = not literal.negated
            if predicate_instance.enumerated:
                assert (predicate_instance.value == value)
            else:
                negative_plan.add(PredicateResult(predicate_instance, value,
                                                  opt_index=predicate_instance.opt_index))
        elif isinstance(negative, Stream):
            #assert not negative.is_fluent()
            object_from_input = dict(zip(negative.inputs, map(obj_from_pddl, literal.args)))
            input_objects = tuple(object_from_input[inp] for inp in negative.inputs)

            fluent_facts_list = []
            if negative.is_fluent():
                for state_index in step_from_atom[literal]:
                    fluent_facts_list.append(list(map(fact_from_fd,
                        filter(lambda f: isinstance(f, pddl.Atom) and (f.predicate in negative.fluents),
                               real_states[state_index]))))
            else:
                fluent_facts_list.append(frozenset())

            for fluent_facts in fluent_facts_list:
                negative_instance = negative.get_instance(input_objects, fluent_facts=fluent_facts)
                if not negative_instance.successes:
                    negative_plan.add(StreamResult(negative_instance, tuple(),
                                                   opt_index=negative_instance.opt_index))
        else:
            raise ValueError(negative)
    return negative_plan

def convert_fluent_streams(stream_plan, real_states, steps_from_stream):
    import pddl
    new_stream_plan = []
    for result in stream_plan:
        external = result.instance.external
        if (result.opt_index != 0) or (not external.is_fluent()):
            new_stream_plan.append(result)
            continue
        if len(steps_from_stream[result]) != 1:
            raise NotImplementedError() # Pass all fluents and make two axioms
        # TODO: can handle case where no outputs easily
        [state_index] = steps_from_stream[result]
        fluent_facts = list(map(fact_from_fd, filter(lambda f: isinstance(f, pddl.Atom) and (f.predicate in external.fluents), real_states[state_index])))
        new_instance = external.get_instance(result.instance.input_objects, fluent_facts=fluent_facts)
        new_stream_plan.append(external._Result(new_instance, result.output_objects, opt_index=result.opt_index))
    return new_stream_plan

def reschedule_stream_plan(evaluations, preimage_facts, domain, stream_results):
    # TODO: search in space of partially ordered plans
    # TODO: constrain selection order to be alphabetical?
    reschedule_problem = get_problem(evaluations, And(*preimage_facts), domain, unit_costs=True)
    reschedule_task = task_from_domain_problem(domain, reschedule_problem)
    reschedule_task.actions, stream_result_from_name = get_stream_actions(stream_results)
    #reschedule_task.axioms = [] # TODO: ensure that the constants are added in the even that axioms are needed?
    new_plan, _ = solve_from_task(reschedule_task, planner=RESCHEDULE_PLANNER, max_time=10, debug=True)
    return [stream_result_from_name[name] for name, _ in new_plan]

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

def extract_axiom_plan(opt_task, real_state, opt_state, action_instance, negative_from_name):
    import pddl_to_prolog
    import build_model
    import axiom_rules
    import instantiate
    import pddl

    original_axioms = opt_task.axioms
    axiom_from_action = get_necessary_axioms(action_instance, original_axioms, negative_from_name)
    if not axiom_from_action:
        return []
    conditions_from_predicate = defaultdict(set)
    for axiom, var_mapping in axiom_from_action.values():
        for literal in get_literals(axiom.condition):
            conditions_from_predicate[literal.predicate].add(
                literal.rename_variables(var_mapping))

    # TODO: store map from predicate to atom
    opt_task.init = {atom for atom in opt_state if isinstance(atom, pddl.Atom) and
                     any(all(is_parameter(a2) or (a1 == a2) for a1, a2 in zip(atom.args, atom2.args))
                         for atom2 in conditions_from_predicate[atom.predicate])}
    opt_task.axioms = []
    opt_task.actions = axiom_from_action.keys()
    # TODO: maybe it would just be better to drop the negative throughout this process until this end
    with Verbose(False):
        model = build_model.compute_model(pddl_to_prolog.translate(opt_task))  # Changes based on init
    opt_task.axioms = original_axioms

    delta_state = (opt_task.init - real_state)  # Optimistic facts
    opt_facts = instantiate.get_fluent_facts(opt_task, model) | delta_state
    mock_fluent = MockSet(lambda item: (item.predicate in negative_from_name) or (item in opt_facts))
    instantiated_axioms = instantiate_necessary_axioms(model, real_state, mock_fluent, axiom_from_action)
    with Verbose(False):
        helpful_axioms, axiom_init, _ = axiom_rules.handle_axioms([action_instance], instantiated_axioms, [])
    axiom_from_atom = get_achieving_axioms(opt_task.init, helpful_axioms, axiom_init, negative_from_name)
    axiom_plan = []  # Could always add all conditions
    extract_axioms(axiom_from_atom, action_instance.precondition, axiom_plan)
    # TODO: test if no derived solution
    # TODO: add axiom init to reset state?
    return axiom_plan

def recover_stream_plan(evaluations, goal_expression, domain, stream_results, action_plan, negative, unit_costs):
    # TODO: toggle optimize more

    import pddl
    import instantiate
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects

    real_task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain, unit_costs))
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs))
    function_assignments = {fact.fluent: fact.expression for fact in opt_task.init  # init_facts
                            if isinstance(fact, pddl.f_expression.FunctionAssignment)}
    type_to_objects = instantiate.get_objects_by_type(opt_task.objects, opt_task.types)
    results_from_head = get_results_from_head(opt_evaluations)
    action_instances = instantiate_actions(opt_task, type_to_objects, function_assignments, action_plan)
    negative_from_name = {external.name: external for external in negative if isinstance(external, Predicate)}
    negative_from_name.update({external.blocked_predicate: external for external in negative
                               if isinstance(external, Stream) and external.is_negated()})

    axioms_from_name = get_derived_predicates(opt_task.axioms)
    opt_task.actions = []
    opt_state = set(opt_task.init)
    real_state = set(real_task.init)
    real_states = [real_state.copy()] # TODO: had old way of doing this (~July 2018)
    preimage_plan = []
    function_plan = set()
    for layer in action_instances:
        for pair, action_instance in layer:
            nonderived_preconditions = [l for l in action_instance.precondition
                                        if l.predicate not in axioms_from_name]
            if not conditions_hold(opt_state, nonderived_preconditions):
                continue
            axiom_plan = extract_axiom_plan(opt_task, real_state, opt_state, action_instance, negative_from_name)
            simplify_conditional_effects(real_state, opt_state, action_instance, axioms_from_name)
            preimage_plan.extend(axiom_plan + [action_instance])
            apply_action(opt_state, action_instance)
            apply_action(real_state, action_instance)
            real_states.append(real_state.copy())
            if not unit_costs and (pair is not None):
                function_result = extract_function_results(results_from_head, *pair)
                if function_result is not None:
                    function_plan.add(function_result)
            break
        else:
            raise RuntimeError('No action instances are applicable')

    # TODO: could instead just accumulate difference between real and opt
    full_preimage = plan_preimage(preimage_plan, [])
    stream_preimage = set(full_preimage) - set(real_task.init)
    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, stream_preimage))
    positive_preimage = stream_preimage - negative_preimage
    function_plan.update(convert_negative(negative_preimage, negative_from_name, full_preimage, real_states))

    step_from_fact = {fact_from_fd(l): full_preimage[l] for l in positive_preimage if not l.negated}
    target_facts = list(step_from_fact.keys())
    #stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_results)
    stream_plan = []
    node_from_atom = get_achieving_streams(evaluations, stream_results)
    extract_stream_plan(node_from_atom, target_facts, stream_plan)
    stream_plan = prune_stream_plan(evaluations, stream_plan, target_facts)

    steps_from_stream = {}
    for result in reversed(stream_plan):
        steps_from_stream[result] = set()
        for fact in result.get_certified():
            if (fact in step_from_fact) and (node_from_atom[fact].stream_result == result):
                steps_from_stream[result].update(step_from_fact[fact])
        for fact in result.instance.get_domain():
            step_from_fact[fact] = step_from_fact.get(fact, set()) | steps_from_stream[result]
    stream_plan = convert_fluent_streams(stream_plan, real_states, steps_from_stream)
    # visualize_constraints(map(fact_from_fd, stream_preimage))

    if DO_RESCHEDULE: # TODO: detect this based on unique or not
        # TODO: maybe test if partial order between two ways of achieving facts, if not prune
        new_stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_plan)
        if new_stream_plan is not None:
            stream_plan = new_stream_plan
    return stream_plan + list(function_plan)

##################################################

def relaxed_stream_plan(evaluations, goal_expression, domain, stream_results, negative, unit_costs, **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_results)
    problem = get_problem(opt_evaluations, goal_expression, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    #action_plan, action_cost = solve_from_task(task, **kwargs)
    #action_plan, action_cost = serialized_solve_from_task(task, **kwargs)
    action_plan, action_cost = abstrips_solve_from_task(task, **kwargs)
    #action_plan, action_cost = abstrips_solve_from_task_sequential(task, **kwargs)
    if action_plan is None:
        return None, action_cost
    # TODO: just use solve finite?

    stream_plan = recover_stream_plan(evaluations, goal_expression, domain,
                                      stream_results, action_plan, negative, unit_costs)
    combined_plan = stream_plan + obj_from_pddl_plan(action_plan)
    return combined_plan, action_cost
