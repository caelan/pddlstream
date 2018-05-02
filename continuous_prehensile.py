#!/usr/bin/env python

from __future__ import print_function
from collections import namedtuple

from pddlstream.conversion import AND, EQ, And, Equal
from pddlstream.fast_downward import TOTAL_COST
from pddlstream.incremental import solve_exhaustive, solve_incremental
from pddlstream.committed import solve_committed
from pddlstream.focused import solve_focused
from pddlstream.stream import from_gen_fn, from_fn, from_test, Generator
from pddlstream.utils import print_solution, user_input
from continuous_tamp_viewer import ContinuousTMPViewer, GROUND, SUCTION_HEIGHT
from discrete_tamp_viewer import COLORS
from pddlstream.utils import read
import numpy as np
import math

BLOCK_WIDTH = 2
BLOCK_HEIGHT = BLOCK_WIDTH
GRASP = np.array([0, BLOCK_HEIGHT + SUCTION_HEIGHT/2]) # TODO: side grasps

class PoseGenerator(Generator):
    def __init__(self, *inputs):
        super(PoseGenerator, self).__init__()
        self.p, = inputs
    def generate(self, context=None): # TODO: context
        raise NotImplementedError()

def collision_test(b1, p1, b2, p2):
    return np.linalg.norm(p2 - p1) <= BLOCK_WIDTH

def distance_fn(q1, q2):
    ord = 1  # 1 | 2
    return int(math.ceil(np.linalg.norm(q2 - q1, ord=ord)))

def inverse_kin_fn(b, p):
    return (p + GRASP,)

def sample_pose(region):
    x1, x2 = np.array(region, dtype=float) + np.array([BLOCK_WIDTH, -BLOCK_WIDTH])/2.
    if x2 < x1:
        return None
    x = np.random.uniform(x1, x2)
    return np.array([x, 0])

def get_pose_gen(regions):
    def gen_fn(b, r):
        while True:
            p = sample_pose(regions[r])
            if p is None:
                break
            yield (p,)
    return gen_fn

def get_constraint_solver(regions):
    def constraint_solver(constraints):
        #import cvxopt
        #import scipy.optimize.linprog
        #import mosek # https://www.mosek.com/
        import gurobipy

        for constraint in constraints:
            if constraint[0] == '=':
                continue
            if constraint[0] in ('pose', 'conf'):
                continue

            print(constraint)

        raw_input('awefaewf')
    return constraint_solver

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    domain_pddl = read('continuous_domain.pddl')
    stream_pddl = read('continuous_stream.pddl')
    #print(domain_pddl)
    #print(stream_pddl)
    constant_map = {}

    init = [
        ('CanMove',),
        ('Conf', initial.conf),
        ('AtConf', initial.conf),
        ('HandEmpty',),
        Equal((TOTAL_COST,), 0)] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Pose', b, p) for b, p in initial.block_poses.items()] + \
           [('Region', r) for r in tamp_problem.regions.keys()] + \
           [('AtPose', b, p) for b, p in initial.block_poses.items()]

    goal_literals = [('In', b, r) for b, r in tamp_problem.goal_regions.items()]
    if tamp_problem.goal_conf is not None:
        goal_literals += [('AtConf', tamp_problem.goal_conf)]
    goal = And(*goal_literals)

    stream_map = {
        'sample-pose': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        'inverse-kinematics':  from_fn(inverse_kin_fn),
        #': from_fn(inverse_kin_fn),
        'collision-free': from_test(lambda *args: not collision_test(*args)),
        #'constraint-solver': get_constraint_solver(tamp_problem.regions),
        'distance': distance_fn,
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal


TAMPState = namedtuple('TAMPState', ['conf', 'holding', 'block_poses'])
TAMPProblem = namedtuple('TAMPProblem', ['initial', 'regions', 'goal_conf', 'goal_regions'])

def get_red_problem(n_blocks=2):
    regions = {
        GROUND: (-15, 15),
        'red': (5, 10)
    }

    conf = np.array([0, 5])
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    #poses = [np.array([(BLOCK_WIDTH + 1)*x, 0]) for x in range(n_blocks)]
    poses = [np.array([-(BLOCK_WIDTH + 1)*x, 0]) for x in range(n_blocks)]
    #poses = [sample_pose(regions[GROUND]) for _ in range(n_blocks)]

    initial = TAMPState(conf, None, dict(zip(blocks, poses)))
    goal_regions = {block: 'red' for block in blocks}

    return TAMPProblem(initial, regions, conf, goal_regions)

def draw_state(viewer, state, colors):
    viewer.clear_state()
    viewer.draw_environment()
    viewer.draw_robot(*state.conf)
    for block, pose in state.block_poses.items():
        viewer.draw_block(pose[0], BLOCK_WIDTH, BLOCK_HEIGHT, color=colors[block])
    if state.holding is not None:
        viewer.draw_block(state.conf[0], BLOCK_WIDTH, BLOCK_HEIGHT, color=colors[state.holding])


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
    return TAMPState(conf, holding, block_poses)

def main():
    problem_fn = get_red_problem # get_shift_one_problem | get_shift_all_problem
    tamp_problem = problem_fn()
    print(tamp_problem)

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    #solution = solve_incremental(pddlstream_problem, unit_costs=True)
    solution = solve_focused(pddlstream_problem, unit_costs=True, visualize=False)
    #solution = solve_committed(pddlstream_problem, unit_costs=True) # TODO: stream plan is None?
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return

    colors = dict(zip(tamp_problem.initial.block_poses, COLORS))
    viewer = ContinuousTMPViewer(tamp_problem.regions, title='Continuous TAMP')
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
