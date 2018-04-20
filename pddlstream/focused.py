from pddlstream.conversion import get_pddl_problem, value_from_obj_plan, \
    obj_from_pddl_plan, substitute_expression, Head, get_prefix, get_args, Evaluation, \
    init_from_evaluations, evaluations_from_init, convert_expression, values_from_objects, objects_from_values
from pddlstream.fast_downward import solve_from_pddl, parse_domain
from pddlstream.instantiation import Instantiator
from pddlstream.object import Object
from pddlstream.stream import parse_stream, StreamInstance
from pddlstream.utils import INF, elapsed_time
import time

from pddlstream.incremental import parse_problem, solve_finite, revert_solution, process_stream_queue

# ('move', [<TypedObject ?q1: object>, <TypedObject ?q2: object>],
# <pddl.conditions.Conjunction object at 0x10c3369d0>,
# [<pddl.effects.Effect object at 0x10c336b50>, <pddl.effects.Effect object at 0x10c336bd0>],
# None)

def stream_action(stream_result):
    name = stream_result.stream_instance.stream.name
    from pddl.actions import Action
    from pddl.conditions import Conjunction, Truth, Atom
    from pddl.effects import Effect
    #from pddl_parser.parsing_functions import parse_action
    from pddl.f_expression import Increase, PrimitiveNumericExpression, ConstantNumericExpression

    print(stream_result)

    parameters = []
    preconditions = []
    for fact in stream_result.stream_instance.get_domain():
        predicate = get_prefix(fact)
        args = get_args(fact)
        atom = Atom(predicate, args)
        preconditions.append(atom)
    precondition = Conjunction(preconditions)
    print(precondition)
    effects = []
    for fact in stream_result.get_certified():
        predicate = get_prefix(fact)
        args = get_args(fact)
        atom = Atom(predicate, args)
        effect = Effect(parameters=[], condition=Truth(), literal=atom)
        effects.append(effect)

    fluent = PrimitiveNumericExpression(symbol='total-cost', args=[])
    expression = ConstantNumericExpression(1) # Integer
    increase = Increase(fluent=fluent, expression=expression) # Can also be None

    action = Action(name=name, parameters=parameters, num_external_parameters=len(parameters),
                    precondition=precondition, effects=effects, cost=increase)
    # TODO: previous problem seemed to be new predicates
    return


def solve_focused(problem, max_time=INF, **kwargs):
    start_time = time.time()
    num_iterations = 0
    best_plan = None; best_cost = INF
    domain_pddl = problem[2]
    evaluations, goal_expression, domain, streams = parse_problem(problem)
    for action in domain.actions:
        print(action.name, action.parameters, action.precondition, action.effects, action.cost)
        for effect in action.effects:
            print(effect.__dict__)
        print(action.precondition.parts)
        print(action.cost.__dict__)
    #return
    while elapsed_time(start_time) < max_time:
        # TODO: version that just calls one of the incremental algorithms
        num_iterations += 1
        print('Iteration: {} | Evaluations: {} | Cost: {} | Time: {:.3f}'.format(
            num_iterations, len(evaluations), best_cost, elapsed_time(start_time)))
        instantiator = Instantiator(evaluations, streams)
        opt_evaluations = set(evaluations)
        stream_results = []
        while instantiator.stream_queue and (elapsed_time(start_time) < max_time):
            stream_results += process_stream_queue(instantiator, opt_evaluations,
                                                   StreamInstance.next_optimistic, revisit=False, verbose=False)

        for stream_result in stream_results:
            stream_action(stream_result)

        opt_plan, opt_cost = solve_finite(opt_evaluations, goal_expression, domain, domain_pddl, **kwargs)
        print(opt_plan)

    plan, cost = solve_finite(evaluations, goal_expression, domain, domain_pddl, **kwargs)
    return revert_solution(plan, cost, evaluations)