#!/usr/bin/env python

from __future__ import print_function

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import PDDLProblem, print_solution, And, Output
from pddlstream.language.generator import from_gen
from pddlstream.utils import flatten, Profiler, SEPARATOR, inf_generator

DOMAIN_PDDL = """
(define (domain cost)
  (:predicates 
    (Control ?x)
    (Goal)
  )
  ;(:functions ; FastDownawrd doesn't throw an error when not included
  ;  (Cost ?x)
  ;)
  (:action execute
    :parameters (?x)
    :precondition (and (Control ?x)
                  )
    :effect (and (Goal)
                 (increase (total-cost) 2)
                 (increase (total-cost) (Cost ?x))
                 (increase (total-cost) 3) ; The last cost is the only one retained
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
)
"""

##################################################

def main():
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    stream_map = {
        'sample-control': from_gen(Output(i) for i in inf_generator()),
        'Cost': lambda x: 1./(x + 1),
    }

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
        solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit,
                         max_time=2, success_cost=0,
                         planner='max-astar', debug=False)
    print_solution(solution)

if __name__ == '__main__':
    main()
