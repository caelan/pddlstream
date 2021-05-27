#!/usr/bin/env python

from __future__ import print_function

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, print_solution, And, Output
from pddlstream.language.generator import from_gen
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo, add_opt_function
from pddlstream.language.object import UniqueOptValue, SharedOptValue
from pddlstream.utils import flatten, Profiler, SEPARATOR, inf_generator, INF

DOMAIN_PDDL = """
(define (domain cost)
  (:predicates 
    (Control ?x)
    (Goal)
  )
  ;(:functions ; FastDownawrd doesn't throw an error when not included
  ;  (Cost ?x)
  ;)
  (:action execute1
    :parameters (?x)
    :precondition (and (Control ?x)
                  )
    :effect (and ;(Goal)
                 (increase (total-cost) 2)
                 (increase (total-cost) (Cost ?x))
                 ;(increase (total-cost) 3) ; The last cost is the only one retained
            )
  )
  (:action execute2
    :parameters (?x1 ?x2)
    :precondition (and (Control ?x1) (Control ?x2)) ; TODO: make the costs parameters and add them in a generic fn
    :effect (and (Goal)
                 (increase (total-cost) (Cost ?x1))
                 (increase (total-cost) (Cost ?x2))
                 (increase (total-cost) (Sum ?x1 ?x2))
            )
  )
)
"""

STREAM_PDDL = """
(define (stream cost)
  (:stream sample-control
    :outputs (?x)
    :certified (Control ?x)
  )
  (:function (Cost ?x)
    (Control ?x)
  )
  (:function (Sum ?x1 ?x2)
    (and (Control ?x1) (Control ?x2))
  )
)
"""

##################################################

COST_CONSTANT = 1

def harmonic(x):
    return 1./(x + 1)

def cost_fn(x):
    cost = COST_CONSTANT
    if type(x) not in [UniqueOptValue, SharedOptValue]:
        cost += harmonic(x)
    # print(x, cost)
    # input()
    return cost

def main():
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    stream_map = {
        'sample-control': from_gen(Output(i) for i in inf_generator()),
        #'Cost': lambda x: COST_CONSTANT + harmonic(x),
        'Cost': cost_fn,
        'Sum': lambda *xs: sum(cost_fn(x) for x in xs),
    }
    stream_info = {
        'sample-control': StreamInfo(opt_gen_fn=None),
        #'Cost': FunctionInfo(opt_fn=lambda x: COST_CONSTANT),
        'Cost': FunctionInfo(opt_fn=cost_fn),
    }

    stream_map, stream_info = add_opt_function(name='Cost', base_fn=harmonic,
                                               stream_map=stream_map, stream_info=stream_info,
                                               constant=COST_CONSTANT, coefficient=1.)
    # TODO: cost fn that takes in a list of terms and sums them

    init = []
    goal = And(
        ('Goal',),
    )

    # TODO: parse multiple additive cost functions
    # TODO: generic additive/multiplicative cost function accumulators
    # TODO: store the cost procedure instead of just the function head
    # TODO: don't instantiate cost streams until the grounding actions
    constant_map = {}
    problem = PDDLProblem(DOMAIN_PDDL, constant_map, STREAM_PDDL, stream_map, init, goal)

    with Profiler():
        solution = solve(problem, stream_info=stream_info, algorithm=args.algorithm, unit_costs=args.unit, max_time=1,
                         #success_cost=0,
                         success_cost=INF,
                         planner='max-astar', debug=False)
    print_solution(solution)

if __name__ == '__main__':
    main()
