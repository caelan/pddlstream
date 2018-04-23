#!/usr/bin/env python

from __future__ import print_function

from pddlstream.conversion import AND, EQ
from pddlstream.fast_downward import TOTAL_COST
from pddlstream.incremental import solve_exhaustive, solve_incremental
from pddlstream.committed import solve_committed
from pddlstream.focused import solve_focused
from pddlstream.stream import from_gen_fn, from_fn, from_test
from pddlstream.utils import print_solution
import numpy as np

DOMAIN_PDDL = """
(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates 
    (Conf ?q)
    (Block ?b)
    (Pose ?p)
    (Kin ?q ?p)
    (AtPose ?p ?q)
    (AtConf ?q)
    (Holding ?b)
    (HandEmpty)
    (CFree ?p1 ?p2)
    (Unsafe ?p)
  )
  (:action move
    :parameters (?q1 ?q2)
    :precondition (and (Conf ?q1) (Conf ?q2) 
                       (AtConf ?q1))
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1))
             (increase (total-cost) 1))
  )
  (:action pick
    :parameters (?b ?p ?q)
    :precondition (and (Block ?b) (Kin ?q ?p)
                       (AtConf ?q) (AtPose ?b ?p) (HandEmpty))
    :effect (and (Holding ?b)
                 (not (AtPose ?b ?p)) (not (HandEmpty)))
  )
  (:action place
    :parameters (?b ?p ?q)
    :precondition (and (Block ?b) (Kin ?q ?p) 
                       (AtConf ?q) (Holding ?b) (not (Unsafe ?p)))
    :effect (and (AtPose ?b ?p) (HandEmpty)
                 (not (Holding ?b)))
  )
  (:derived (Unsafe ?p1) 
    (exists (?b ?p2) (and (Pose ?p1) (Block ?b) (Pose ?p2) (not (CFree ?p1 ?p2)) 
                            (AtPose ?b ?p2)))
  )                   
)
"""

STREAM_PDDL = """
(define (stream pick-and-place)
  (:stream sample-pose
    :inputs ()
    :domain ()
    :outputs (?p)
    :certified (and (Pose ?p))
  )
  (:stream inverse-kinematics
    :inputs (?p)
    :domain (Pose ?p)
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?q ?p))
  )
  (:stream collision-free
    :inputs (?p1 ?p2)
    :domain (and (Pose ?p1) (Pose ?p2))
    :outputs ()
    :certified (CFree ?p1 ?p2)
  )
)
"""

def get_problem1(n_blocks=2, n_poses=5):
    assert(n_blocks <= n_poses)
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_blocks)]
    conf = np.array([0, 5])
    grasp = np.array([0, 1])

    #objects = []
    init = [
        ('Conf', conf),
        ('AtConf', conf),
        ('HandEmpty',),
        (EQ, (TOTAL_COST,), 0)] + \
           [('Block', b) for b in blocks] + \
           [('Pose', p) for p in poses] + \
           [('AtPose', b, p) for b, p in zip(blocks, poses)]

    goal = (AND,
            ('AtPose', blocks[0], poses[1]),
            ('AtConf', conf))

    domain_pddl = DOMAIN_PDDL
    stream_pddl = STREAM_PDDL
    stream_map = {
        'sample-pose': from_gen_fn(lambda: ((np.array([x, 0]),) for x in range(len(poses), n_poses))),
        'inverse-kinematics':  from_fn(lambda p: (p + grasp,)),
        'collision-free': from_test(lambda p1, p2: 1e-1 < np.linalg.norm(p2-p1)),
    }
    constant_map = {}

    return init, goal, domain_pddl, stream_pddl, stream_map, constant_map

def main():
    problem = get_problem1()
    #solution = solve_exhaustive(problem)
    #solution = solve_incremental(problem)
    solution = solve_focused(problem)
    #solution = solve_committed(problem) # TODO: make sure this is right
    print_solution(solution)

if __name__ == '__main__':
    main()
