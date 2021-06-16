#!/usr/bin/env python

from __future__ import print_function

from pddlstream.algorithms.meta import solve, create_parser, analyze_goal
from pddlstream.language.constants import PDDLProblem, print_solution, And
from pddlstream.language.stream import DEBUG
from pddlstream.utils import flatten, Profiler, SEPARATOR

# Kitchen Storage

# https://zhuyifengzju.github.io/projects/hierarchical-scene-graph/
# https://arxiv.org/abs/2012.07277

DOMAIN_PDDL = """
(define (domain storage)
  (:predicates 
    (Region ?r)
    (Prepush ?r1 ?r2)
    (Stackable ?t ?b)
    (Movable ?o)
    (Cylinder ?o)
    (Box ?o)
    (Thing ?o)

    (HandEmpty)
    (Holding ?o)
    (On ?t ?b)
    (In ?t ?b)
    (Clear ?b)
    (Supporting ?b)
  )
  (:action pick
    :parameters (?t ?b)
    :precondition (and (Movable ?t) (Thing ?b) (not (= ?t ?b))
                       (HandEmpty) (On ?t ?b)
                       (Clear ?t)
                       ;(not (Supporting ?t))
                  )
    :effect (and (Holding ?t)
                 (when (Movable ?b) (Clear ?b))
                 (not (HandEmpty)) (not (On ?t ?b)))
  )
  (:action place
    :parameters (?t ?b)
    :precondition (and (Movable ?t) (Thing ?b) (not (= ?t ?b))
                       (Holding ?t)
                       ;(or (Region ?b) (Stackable ?t ?b))
                       (or (Region ?b) (Clear ?b))
                       ;(imply (Supporting ?b) (Region ?b))
                  )
    :effect (and (HandEmpty) (On ?t ?b)
                 (when (Movable ?b) (not (Clear ?b)))
                 (not (Holding ?t)))
  )
  (:action push
    :parameters (?o ?r1 ?r2)
    :precondition (and (Movable ?o) (Prepush ?r1 ?r2) 
                       (HandEmpty) (On ?o ?r1)
                       (Clear ?o)
                       ;(not (Supporting ?o))
                       )
    :effect (and (In ?o ?r2) 
                 (not (On ?o ?r1)))
  )
  ;(:derived (Clear ?b)
  ;  (exists (?t) (and (Movable ?t) (Thing ?b) (not (= ?t ?b))
  ;                    (not (On ?t ?b))
  ;                    
  ;                    )))
  ;(:derived (Supporting ?b)
  ;  (exists (?t) (and (Movable ?t) (Thing ?b) (not (= ?t ?b))
  ;                    (On ?t ?b))))
)
"""

STREAM_PDDL = None

##################################################

def get_problem1(n_regions=0, n_cylinders=3, n_boxes=3):
    constant_map = {}
    stream_map = DEBUG

    initial = 'initial'
    shelfA = 'shelfA'
    prepushA = 'prepushA'
    shelfB = 'shelfB'

    regions = [initial, shelfA, prepushA, shelfB] + \
              ['region{}'.format(i) for i in range(n_regions)]
    cylinders = ['cylinder{}'.format(i) for i in range(n_cylinders)]
    boxes = ['box{}'.format(i) for i in range(n_boxes)]

    init = [
        ('HandEmpty',),
        ('Prepush', prepushA, shelfA),
        #('Stackable', cylinders[2], cylinders[1]),
    ]
    init += [('Region', region) for region in regions]
    init += [('Cylinder', cylinder) for cylinder in cylinders]
    init += [('Box', box) for box in boxes]
    init.extend(flatten([('Movable', movable), ('On', movable, initial), ('Clear', movable)]
                        for movable in (cylinders + boxes)))
    init += [('Thing', thing) for thing in (regions + cylinders + boxes)]

    goal = And(
        ('In', cylinders[0], shelfA),
        ('On', cylinders[2], cylinders[1]),
    )

    return PDDLProblem(DOMAIN_PDDL, constant_map, STREAM_PDDL, stream_map, init, goal)

##################################################

def main():
    parser = create_parser()
    parser.add_argument('-n', '--n_boxes', default=3, type=int, help='The number of boxes')
    args = parser.parse_args()
    print('Arguments:', args)

    problem = get_problem1(n_boxes=args.n_boxes)
    print('Init:', sorted(problem.init))
    print('Goal:', problem.goal)

    print(analyze_goal(problem, debug=True, use_actions=True, blocked_predicates=[
        # TODO: make sure to use lowercase
        'handempty',
        'clear',
        # These are conditions that you can always reachieve?
    ]))
    print(SEPARATOR)

    with Profiler():
        solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit,
                         planner='ff-wastar1', debug=True)
    print_solution(solution)

if __name__ == '__main__':
    main()
