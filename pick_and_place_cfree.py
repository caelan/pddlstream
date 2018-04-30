#!/usr/bin/env python

from __future__ import print_function
from collections import namedtuple

from pddlstream.conversion import AND, EQ, And, Equal
from pddlstream.fast_downward import TOTAL_COST
from pddlstream.incremental import solve_exhaustive, solve_incremental
from pddlstream.committed import solve_committed
from pddlstream.focused import solve_focused
from pddlstream.stream import from_gen_fn, from_fn, from_test
from pddlstream.utils import print_solution, user_input
from discrete_tamp_viewer import DiscreteTAMPViewer, COLORS
import numpy as np
import math

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
  (:functions
    (Distance ?q1 ?q2)
  )
  (:action move
    :parameters (?q1 ?q2)
    :precondition (and (Conf ?q1) (Conf ?q2) 
                       (AtConf ?q1))
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1))
             (increase (total-cost) (Distance ?q1 ?q2)))
  )
  (:action pick
    :parameters (?b ?p ?q)
    :precondition (and (Block ?b) (Kin ?q ?p)
                       (AtConf ?q) (AtPose ?b ?p) (HandEmpty))
    :effect (and (Holding ?b)
                 (not (AtPose ?b ?p)) (not (HandEmpty)) 
                 (increase (total-cost) 1))
  )
  (:action place
    :parameters (?b ?p ?q)
    :precondition (and (Block ?b) (Kin ?q ?p) 
                       (AtConf ?q) (Holding ?b) (not (Unsafe ?p)))
    :effect (and (AtPose ?b ?p) (HandEmpty)
                 (not (Holding ?b))
                 (increase (total-cost) 1))
  )
  (:derived (Unsafe ?p1) 
    (exists (?b ?p2) (and (Pose ?p1) (Block ?b) (Pose ?p2) (not (CFree ?p1 ?p2)) 
                            (AtPose ?b ?p2)))
  )                   
)
"""

# Can infer domain from usage or from specification
STREAM_PDDL = """
(define (stream pick-and-place)
  (:rule 
    :parameters (?q ?p)
    :domain (Kin ?q ?p)
    :certified (and (Conf ?q) (Pose ?p))
  )
  (:rule 
    :parameters (?p1 ?p2)
    :domain (AtPose ?b ?p)
    :certified (and (Block ?b) (Pose ?p))
  )

  (:function (Distance ?q1 ?q2)
    (and (Conf ?q1) (Conf ?q2))
  )

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

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    known_poses = list(initial.block_poses.values()) + \
                  list(tamp_problem.goal_poses.values())

    #objects = []
    init = [
        ('Conf', initial.conf),
        ('AtConf', initial.conf),
        ('HandEmpty',),
        Equal((TOTAL_COST,), 0)] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Pose', p) for p in known_poses] + \
           [('AtPose', b, p) for b, p in initial.block_poses.items()]
    # [('Pose', p) for p in known_poses + tamp_problem.poses] + \

    goal = And(*[
        ('AtPose', b, p) for b, p in tamp_problem.goal_poses.items()
    ])

    domain_pddl = DOMAIN_PDDL
    stream_pddl = STREAM_PDDL

    def collision_test(p1, p2):
        return  np.linalg.norm(p2-p1) <= 1e-1

    def distance_fn(q1, q2):
        ord = 1 # 1 | 2
        return int(math.ceil(np.linalg.norm(q2 - q1, ord=ord)))

    # TODO: convert to lower case
    stream_map = {
        #'sample-pose': from_gen_fn(lambda: ((np.array([x, 0]),) for x in range(len(poses), n_poses))),
        'sample-pose': from_gen_fn(lambda: ((p,) for p in tamp_problem.poses)),
        'inverse-kinematics':  from_fn(lambda p: (p + GRASP,)),
        'collision-free': from_test(lambda *args: not collision_test(*args)),
        'distance': distance_fn,
    }
    constant_map = {}

    return init, goal, domain_pddl, stream_pddl, stream_map, constant_map


DiscreteTAMPState = namedtuple('DiscreteTAMPState', ['conf', 'holding', 'block_poses'])
DiscreteTAMPProblem = namedtuple('DiscreteTAMPProblem', ['initial', 'poses', 'goal_poses'])
GRASP = np.array([0, 0])

def get_shift_one_problem(n_blocks=2, n_poses=9):
    assert(2 <= n_blocks <= n_poses)
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]
    conf = np.array([0, -5])

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(conf, None, block_poses)
    goal_poses = {blocks[0]: poses[1]}

    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)
    #return DiscreteTAMPProblem(initial, poses, goal_poses)

def get_shift_all_problem(n_blocks=2, n_poses=9):
    assert(n_blocks + 1 <= n_poses)
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]
    conf = np.array([0, -5])

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(conf, None, block_poses)
    goal_poses = dict(zip(blocks, poses[1:]))

    return DiscreteTAMPProblem(initial, poses[n_blocks+1:], goal_poses)

def draw_state(viewer, state, colors):
    viewer.clear()
    viewer.draw_environment()
    viewer.draw_robot(*state.conf[::-1])
    for block, pose in state.block_poses.items():
        r, c = pose[::-1]
        viewer.draw_block(r, c, color=colors[block])
    if state.holding is not None:
        pose = state.conf - GRASP
        r, c = pose[::-1]
        viewer.draw_block(r, c, color=colors[state.holding])


def apply_action(state, action):
    conf, holding, block_poses = state
    # TODO: don't mutate block_poses?
    name = action[0]
    if name == 'move':
        _, conf = action[1:]
    elif name == 'pick':
        holding, _, _ = action[1:]
        del block_poses[holding]
    elif name == 'place':
        block, pose, _ = action[1:]
        holding = None
        block_poses[block] = pose
    else:
        raise ValueError(name)
    return DiscreteTAMPState(conf, holding, block_poses)

def main():
    problem_fn = get_shift_one_problem # get_shift_one_problem | get_shift_all_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    #solution = solve_exhaustive(pddlstream_problem, unit_costs=False)
    #solution = solve_incremental(pddlstream_problem, unit_costs=False)
    solution = solve_focused(pddlstream_problem, unit_costs=False, visualize=False)
    #solution = solve_committed(pddlstream_problem, unit_costs=True)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return
    print(evaluations)

    colors = dict(zip(tamp_problem.initial.block_poses, COLORS))
    viewer = DiscreteTAMPViewer(1, len(tamp_problem.poses), title='Initial')
    state = tamp_problem.initial
    print(state)
    draw_state(viewer, state, colors)
    for action in plan:
        user_input('Continue?')
        state = apply_action(state, action)
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')


if __name__ == '__main__':
    main()
