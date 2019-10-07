from pddlstream.algorithms.downward import fd_from_fact, substitute_derived, is_applicable, apply_action, \
    fd_from_evaluation, task_from_domain_problem, get_problem, get_action_instances
from pddlstream.algorithms.reorder import separate_plan, get_stream_stats, dynamic_programming
from pddlstream.algorithms.scheduling.recover_axioms import extract_axioms
from pddlstream.algorithms.instantiate_task import get_achieving_axioms
from pddlstream.algorithms.scheduling.recover_streams import evaluations_from_stream_plan
from pddlstream.language.constants import get_prefix, EQ, is_plan, And
from pddlstream.language.conversion import evaluation_from_fact
from pddlstream.language.external import Result
from pddlstream.language.function import PredicateResult
from pddlstream.utils import Verbose, MockSet, neighbors_from_orders


def get_stream_instances(stream_plan):
    import pddl
    # TODO: something that inverts the negative items
    stream_instances = [] # TODO: could even apply these to the state directly
    for result in stream_plan:
        name = result.instance.external.name
        precondition = list(map(fd_from_fact, result.instance.get_domain()))
        effects = [([], fd_from_fact(fact)) for fact in result.get_certified() if get_prefix(fact) != EQ]
        cost = None # TODO: effort?
        instance = pddl.PropositionalAction(name, precondition, effects, cost)
        stream_instances.append(instance)
    return stream_instances

def instantiate_axioms(model, static_facts, fluent_facts, axiom_remap={}):
    import pddl
    instantiated_axioms = []
    for atom in model:
        if isinstance(atom.predicate, pddl.Axiom):
            axiom = axiom_remap.get(atom.predicate, atom.predicate)
            variable_mapping = dict([(par.name, arg)
                                     for par, arg in zip(axiom.parameters, atom.args)])
            inst_axiom = axiom.instantiate(variable_mapping, static_facts, fluent_facts)
            if inst_axiom:
                instantiated_axioms.append(inst_axiom)
    return instantiated_axioms

def replace_derived(task, negative_init, action_instances):
    import pddl_to_prolog
    import build_model
    import axiom_rules
    import pddl

    original_actions = task.actions
    original_init = task.init
    task.actions = []
    function_assignments = {f.fluent: f.expression for f in task.init
                            if isinstance(f, pddl.f_expression.FunctionAssignment)}
    task.init = (set(task.init) | {a.negate() for a in negative_init}) - set(function_assignments)
    for instance in action_instances:
        #axiom_plan = extract_axiom_plan(task, instance, negative_from_name={}) # TODO: refactor this

        # TODO: just instantiate task?
        with Verbose(False):
            model = build_model.compute_model(pddl_to_prolog.translate(task))  # Changes based on init
        # fluent_facts = instantiate.get_fluent_facts(task, model)
        fluent_facts = MockSet()
        instantiated_axioms = instantiate_axioms(model, task.init, fluent_facts)
        goal_list = [] # TODO: include the goal?
        with Verbose(False):  # TODO: helpful_axioms prunes axioms that are already true (e.g. not Unsafe)
            helpful_axioms, axiom_init, _ = axiom_rules.handle_axioms([instance], instantiated_axioms, goal_list)
        axiom_from_atom, _ = get_achieving_axioms(task.init | negative_init | set(axiom_init), helpful_axioms)
        # negated_from_name=negated_from_name)
        axiom_plan = []
        extract_axioms(axiom_from_atom, instance.precondition, axiom_plan)

        substitute_derived(axiom_plan, instance)
        assert(is_applicable(task.init, instance))
        apply_action(task.init, instance)
    task.actions = original_actions
    task.init = original_init

def get_combined_orders(evaluations, stream_plan, action_plan, domain):
    if not is_plan(action_plan):
        return action_plan
    # TODO: could just do this within relaxed
    # TODO: do I want to strip the fluents and just do the partial ordering?

    stream_instances = get_stream_instances(stream_plan)
    negative_results = filter(lambda r: isinstance(r, PredicateResult) and (r.value == False), stream_plan)
    negative_init = set(fd_from_evaluation(evaluation_from_fact(f))
                        for r in negative_results for f in r.get_certified())
    #negated_from_name = {r.instance.external.name for r in negative_results}
    opt_evaluations = evaluations_from_stream_plan(evaluations, stream_plan)
    goal_expression = And()
    task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain, unit_costs=True))

    action_instances = get_action_instances(task, action_plan)
    replace_derived(task, negative_init, action_instances)

    #combined_instances = stream_instances + action_instances
    orders = set()
    for i, a1 in enumerate(action_plan):
        for a2 in action_plan[i+1:]:
            orders.add((a1, a2))
    # TODO: just store first achiever here
    for i, instance1 in enumerate(stream_instances):
        for j in range(i+1, len(stream_instances)):
            effects = {e for _, e in  instance1.add_effects}
            if effects & set(stream_instances[j].precondition):
                orders.add((stream_plan[i], stream_plan[j]))
    for i, instance1 in enumerate(stream_instances):
        for j, instance2 in enumerate(action_instances):
            effects = {e for _, e in  instance1.add_effects} | \
                      {e.negate() for _, e in  instance1.del_effects}
            if effects & set(instance2.precondition):
                orders.add((stream_plan[i], action_plan[j]))
    return orders

##################################################

def reorder_combined_plan(evaluations, combined_plan, action_info, domain, **kwargs):
    if not is_plan(combined_plan):
        return combined_plan
    stream_plan, action_plan = separate_plan(combined_plan)
    orders = get_combined_orders(evaluations, stream_plan, action_plan, domain)
    _, out_orders = neighbors_from_orders(orders)
    valid_combine = lambda v, subset: out_orders[v] <= subset
    def stats_fn(operator):
        if isinstance(operator, Result):
            return get_stream_stats(operator)
        name, _ = operator
        info = action_info[name]
        return info.p_success, info.overhead
    return dynamic_programming(combined_plan, valid_combine, stats_fn, **kwargs)

##################################################

# def partial_ordered(plan):
#     # https://www.aaai.org/ocs/index.php/ICAPS/ICAPS10/paper/viewFile/1420/1539
#     # http://repository.cmu.edu/cgi/viewcontent.cgi?article=1349&context=compsci
#     # https://arxiv.org/pdf/1105.5441.pdf
#     # https://pdfs.semanticscholar.org/e057/e330249f447c2f065cf50db9dfaddad16aaa.pdf
#     # https://github.mit.edu/caelan/PAL/blob/master/src/search/post_processing.cc
#
#     instances = instantiate_plan(plan)
#     orders = set()
#     primary_effects = set() # TODO: start and goal operators here?
#     for i in reversed(xrange(len(instances))):
#         for pre in instances[i].preconditions:
#             for j in reversed(xrange(i)):
#                 #if pre in instances[j].effects:
#                 if any(eff == pre for eff in instances[j].effects):
#                     orders.add((j, i))
#                     primary_effects.add((j, pre))
#                     break
#         for eff in instances[i].effects:
#             for j in xrange(i):
#                 if any((pre.head == eff.head) and (pre.value != eff.value) for pre in instances[j].preconditions):
#                     orders.add((j, i))
#             if (i, eff) in primary_effects:
#                 for j in xrange(i):
#                     if any((eff2.head == eff.head) and (eff2.value != eff.value) for eff2 in instances[j].effects):
#                         orders.add((j, i))
#     # TODO: could remove transitive
#     # TODO: this isn't so helpful because it will choose arbitrary streams until an action is feasible (i.e. not intelligent ones)
#     for i, (action, args) in enumerate(plan):
#         print i, action, args #, instances[i].preconditions, instances[i].effects
#     print orders
#     print primary_effects
#     print topological_sort(range(len(plan)), orders, lambda v: hasattr(plan[v][0], 'stream'))
