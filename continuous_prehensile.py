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

"""
['CUTOFF', 'UNDEFINED', 'ERROR_INDEX_OUT_OF_RANGE', 'CONTINUOUS', 'Param', 'SUBOPTIMAL', 'SEMICONT', 
    'ERROR_QCP_EQUALITY_CONSTRAINT', '__dict__', 'LOADED', 'INPROGRESS', 'ERROR_Q_NOT_PSD', '__weakref__', 
    'Status', 'ERROR_NO_LICENSE', 'GREATER_EQUAL', 'UNBOUNDED', 'NODE_LIMIT', 'param', 'ERROR_CALLBACK', 
    'SUPERBASIC', 'ERROR_DATA_NOT_AVAILABLE', 'ERROR_NOT_SUPPORTED', '__doc__', 'TIME_LIMIT', 
    'ERROR_IIS_NOT_INFEASIBLE', 'ERROR_FILE_READ', 'Callback', 'ERROR_JOB_REJECTED', 'MAXIMIZE', 'Error', 
    'LESS_EQUAL', 'OPTIMAL', 'ERROR_NETWORK', 'BINARY', 'ERROR_NULL_ARGUMENT', 'attr', 'NONBASIC_UPPER', 
    '__qualname__', 'ERROR_NOT_IN_MODEL', 'error', 'DEFAULT_CS_PORT', 'ERROR_OUT_OF_MEMORY', 'ERROR_NUMERIC', 
    '__module__', 'EQUAL', 'SOLUTION_LIMIT', 'ERROR_INTERNAL', 'ERROR_FILE_WRITE', 'NONBASIC_LOWER', 
    'ERROR_UNKNOWN_PARAMETER', 'BASIC', 'ERROR_NODEFILE', 'SEMIINT', 'MINIMIZE', 'ERROR_EXCEED_2B_NONZEROS', 
    'ITERATION_LIMIT', 'ERROR_INVALID_ARGUMENT', 'ERROR_UNKNOWN_ATTRIBUTE', 'INTEGER', 'status', 'INFINITY', 
    'Attr', 'INTERRUPTED', 'INF_OR_UNBD', 'ERROR_FAILED_TO_CREATE_MODEL', 'SOS_TYPE1', 'ERROR_VALUE_OUT_OF_RANGE', 
    'SOS_TYPE2', 'ERROR_INVALID_PIECEWISE_OBJ', 'ERROR_NOT_FOR_MIP', 'ERROR_SIZE_LIMIT_EXCEEDED', 
    'ERROR_OPTIMIZATION_IN_PROGRESS', 'NUMERIC', 'ERROR_DUPLICATES', 'callback', 'INFEASIBLE']
"""
# ['_env', '_Model__sos', '_Model__constrs', '_cmodel', 'Params', '_Model__vars', 'params', '_Model__qconstrs']

def get_constraint_solver(regions, max_time=5, verbose=False):
    # import cvxopt
    # import scipy.optimize.linprog
    # import mosek # https://www.mosek.com/
    from gurobipy import Model, GRB, quicksum
    def constraint_solver(facts):
        m = Model(name='TAMP')
        m.setParam(GRB.Param.OutputFlag, verbose)
        m.setParam(GRB.Param.TimeLimit, max_time)

        variable_from_id = {}
        param_from_id = {}
        for fact in facts:
            name = fact[0]
            args = fact[1:]
            if name == 'conf':
                param, = args
                if param not in variable_from_id:
                    index = len(variable_from_id)
                    x = m.addVar(name='{}{}-x'.format(name, index), vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=GRB.INFINITY)
                    y = m.addVar(name='{}{}-y'.format(name, index), vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=GRB.INFINITY)
                    variable_from_id[id(param)] = (x, y)
                    param_from_id[id(param)] = param
            elif name == 'pose':
                _, param = args
                if param not in variable_from_id:
                    index = len(variable_from_id)
                    x = m.addVar(name='{}{}-x'.format(name, index), vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=GRB.INFINITY)
                    y = m.addVar(name='{}{}-y'.format(name, index), vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=GRB.INFINITY)
                    variable_from_id[id(param)] = (x, y)
                    param_from_id[id(param)] = param

        objective_terms = []
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name == 'kin':
                _, q, p = args
                qx, qy = variable_from_id.get(id(q), q)
                px, py = variable_from_id.get(id(p), p)
                gx, gy = -GRASP
                m.addConstr(qx + gx == px)
                m.addConstr(qy + gy == py)
            elif name == 'contained':
                _, p, r = args
                px, py = variable_from_id.get(id(p), p)
                x1, x2 = regions[r]
                m.addConstr(x1 <= px - BLOCK_WIDTH/2)
                m.addConstr(px + BLOCK_WIDTH/2 <= x2)
                m.addConstr(py == 0)
            elif name == 'cfree':
                p1, p2 = args
                p1x, _ = variable_from_id.get(id(p1), p1)
                p2x, _ = variable_from_id.get(id(p2), p2)
                dist = m.addVar(lb=-GRB.INFINITY)
                abs_dist = m.addVar(lb=-GRB.INFINITY)
                m.addConstr(dist == p2x - p1x)
                m.addGenConstrAbs(abs_dist, dist) # abs_
                m.addConstr(BLOCK_WIDTH <= abs_dist)
            elif name == '=':
                fact = args[0]
                name, args = fact[0], fact[1:]
                if name == 'distance':
                    q1, q2 = args
                    q1x, q1y = variable_from_id.get(id(q1), q1)
                    q2x, q2y = variable_from_id.get(id(q2), q2)
                    dx = (q2x - q1x)
                    dy = (q2y - q1y)
                    objective_terms.append(dx*dx + dy*dy)

        m.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE)
        m.optimize()
        if m.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD):
            # https://www.gurobi.com/documentation/7.5/refman/optimization_status_codes.html
            return []
        value_from_id = {key: np.array([v.X for v in vars]) for key, vars in variable_from_id.items()}
        #print('Objective:', m.objVal)

        #for key, param in param_from_id.items():
        #    print(param, value_from_id[key])
        atoms = []
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name in ('conf', 'pose', 'kin', 'contained', 'cfree'):
                new_fact = (name,) + tuple(value_from_id.get(id(a), a) for a in args)
                atoms.append(new_fact)
        # TODO: evaluate functions here?
        return atoms
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
        'constraint-solver': get_constraint_solver(tamp_problem.regions),
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
