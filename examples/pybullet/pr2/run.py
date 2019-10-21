#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats
import argparse

from examples.pybullet.utils.pybullet_tools.pr2_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_stable_gen, get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State
from examples.pybullet.utils.pybullet_tools.pr2_problems import cleaning_problem, cooking_problem
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, point_from_pose, \
    disconnect, user_input, get_joint_positions, enable_gravity, save_state, restore_state, HideOutput, \
    get_distance, LockRenderer, get_min_limit, get_max_limit
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen
from pddlstream.language.constants import Equal, AND, print_solution
from pddlstream.utils import read, INF, get_file_path, find_unique
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, PartialInputs
from pddlstream.language.object import SharedOptValue
from collections import namedtuple

BASE_CONSTANT = 1
BASE_VELOCITY = 0.5

def place_movable(certified):
    for literal in certified:
        if literal[0] != 'not':
            continue
        fact = literal[1]
        if fact[0] == 'trajposecollision':
            _, b, p = fact[1:]
            p.assign()
        if fact[0] == 'trajarmcollision':
            _, a, q = fact[1:]
            q.assign()
        if fact[0] == 'trajgraspcollision':
            _, a, o, g = fact[1:]
            # TODO: finish this

# def get_base_motion_synth(problem, teleport=False):
#     # TODO: could factor the safety checks if desired (but no real point)
#     #fixed = get_fixed(robot, movable)
#     def fn(outputs, certified):
#         assert(len(outputs) == 1)
#         q0, _, q1 = find_unique(lambda f: f[0] == 'basemotion', certified)[1:]
#         place_movable(certified)
#         free_motion_fn = get_motion_gen(problem, teleport)
#         return free_motion_fn(q0, q1)
#     return fn

def move_cost_fn(c):
    [t] = c.commands
    distance = t.distance(distance_fn=lambda q1, q2: get_distance(q1[:2], q2[:2]))
    #return BASE_CONSTANT + distance / BASE_VELOCITY
    return 1

#######################################################

def extract_point2d(v):
    if isinstance(v, Conf):
        return v.values[:2]
    if isinstance(v, Pose):
        return point_from_pose(v.value)[:2]
    if isinstance(v, SharedOptValue):
        if v.stream == 'sample-pose':
            r, = v.values
            return point_from_pose(get_pose(r))[:2]
        if v.stream == 'inverse-kinematics':
            p, = v.values
            return extract_point2d(p)
    if isinstance(v, CustomValue):
        if v.stream == 'p-sp':
            r, = v.values
            return point_from_pose(get_pose(r))[:2]
        if v.stream == 'q-ik':
            p, = v.values
            return extract_point2d(p)
    raise ValueError(v.stream)

def opt_move_cost_fn(t):
    q1, q2 = t.values
    distance = get_distance(extract_point2d(q1), extract_point2d(q2))
    #return BASE_CONSTANT + distance / BASE_VELOCITY
    return 1

#######################################################

CustomValue = namedtuple('OptValue', ['stream', 'values'])

def opt_pose_fn(o, r):
    p = CustomValue('p-sp', (r,))
    return p,

def opt_ik_fn(a, o, p, g):
    q = CustomValue('q-ik', (p,))
    t = CustomValue('t-ik', tuple())
    return q, t

def opt_motion_fn(q1, q2):
    t = CustomValue('t-pbm', (q1, q2))
    return t,

#######################################################

def pddlstream_from_problem(problem, teleport=False):
    robot = problem.robot

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    #initial_bq = Pose(robot, get_pose(robot))
    initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))
    init = [
        ('CanMove',),
        ('BConf', initial_bq),
        ('AtBConf', initial_bq),
        Equal(('PickCost',), 1),
        Equal(('PlaceCost',), 1),
    ] + [('Sink', s) for s in problem.sinks] + \
           [('Stove', s) for s in problem.stoves] + \
           [('Connected', b, d) for b, d in problem.buttons] + \
           [('Button', b) for b, _ in problem.buttons]
    for arm in ARM_NAMES:
    #for arm in problem.arms:
        joints = get_arm_joints(robot, arm)
        conf = Conf(robot, joints, get_joint_positions(robot, joints))
        init += [('Arm', arm), ('AConf', arm, conf), ('HandEmpty', arm), ('AtAConf', arm, conf)]
        if arm in problem.arms:
            init += [('Controllable', arm)]
    for body in problem.movable:
        pose = Pose(body, get_pose(body))
        init += [('Graspable', body), ('Pose', body, pose),
                 ('AtPose', body, pose)]
        for surface in problem.surfaces:
            init += [('Stackable', body, surface)]
            if is_placement(body, surface):
                init += [('Supported', body, pose, surface)]

    goal = [AND]
    if problem.goal_conf is not None:
        goal_conf = Pose(robot, problem.goal_conf)
        init += [('BConf', goal_conf)]
        goal += [('AtBConf', goal_conf)]
    goal += [('Holding', a, b) for a, b in problem.goal_holding] + \
                     [('On', b, s) for b, s in problem.goal_on] + \
                     [('Cleaned', b)  for b in problem.goal_cleaned] + \
                     [('Cooked', b)  for b in problem.goal_cooked]

    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(problem)),
        'sample-grasp': from_list_fn(get_grasp_gen(problem)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(problem, teleport=teleport)),
        'plan-base-motion': from_fn(get_motion_gen(problem, teleport=teleport)),
        'MoveCost': move_cost_fn,
        'TrajPoseCollision': fn_from_constant(False),
        'TrajArmCollision': fn_from_constant(False),
        'TrajGraspCollision': fn_from_constant(False),
    }
    # get_press_gen(problem, teleport=teleport)

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

#######################################################

# TODO: avoid copying this?

def post_process(problem, plan, teleport=False):
    if plan is None:
        return None
    commands = []
    for i, (name, args) in enumerate(plan):
        if name == 'move_base':
            c = args[-1]
            new_commands = c.commands
        elif name == 'pick':
            a, b, p, g, _, c = args
            [t] = c.commands
            close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=teleport)
            attach = Attach(problem.robot, a, g, b)
            new_commands = [t, close_gripper, attach, t.reverse()]
        elif name == 'place':
            a, b, p, g, _, c = args
            [t] = c.commands
            gripper_joint = get_gripper_joints(problem.robot, a)[0]
            position = get_max_limit(problem.robot, gripper_joint)
            open_gripper = GripperCommand(problem.robot, a, position, teleport=teleport)
            detach = Detach(problem.robot, a, b)
            new_commands = [t, detach, open_gripper, t.reverse()]
        elif name == 'clean': # TODO: add text or change color?
            body, sink = args
            new_commands = [Clean(body)]
        elif name == 'cook':
            body, stove = args
            new_commands = [Cook(body)]
        elif name == 'press_clean':
            body, sink, arm, button, bq, c = args
            [t] = c.commands
            new_commands = [t, Clean(body), t.reverse()]
        elif name == 'press_cook':
            body, sink, arm, button, bq, c = args
            [t] = c.commands
            new_commands = [t, Cook(body), t.reverse()]
        else:
            raise ValueError(name)
        print(i, name, args, new_commands)
        commands += new_commands
    return commands

#######################################################

def main(display=True, teleport=False, partial=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    parser.add_argument('-viewer', action='store_true', help='enable the viewer while planning')
    #parser.add_argument('-display', action='store_true', help='displays the solution')
    args = parser.parse_args()

    connect(use_gui=args.viewer)
    problem_fn = cooking_problem
    # holding_problem | stacking_problem | cleaning_problem | cooking_problem
    # cleaning_button_problem | cooking_button_problem
    with HideOutput():
        problem = problem_fn()
    state_id = save_state()
    #saved_world = WorldSaver()
    #dump_world()

    pddlstream_problem = pddlstream_from_problem(problem, teleport=teleport)

    stream_info = {
        'sample-pose': StreamInfo(PartialInputs('?r')),
        'inverse-kinematics': StreamInfo(PartialInputs('?p')),
        'plan-base-motion': StreamInfo(PartialInputs('?q1 ?q2'), defer=True),
        'MoveCost': FunctionInfo(opt_move_cost_fn),
    } if partial else {
        'sample-pose': StreamInfo(from_fn(opt_pose_fn)),
        'inverse-kinematics': StreamInfo(from_fn(opt_ik_fn)),
        'plan-base-motion': StreamInfo(from_fn(opt_motion_fn)),
        'MoveCost': FunctionInfo(opt_move_cost_fn),
    }
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', stream_map.keys())

    pr = cProfile.Profile()
    pr.enable()
    with LockRenderer():
        #solution = solve_incremental(pddlstream_problem, debug=True)
        solution = solve_focused(pddlstream_problem, stream_info=stream_info, success_cost=INF, debug=True)
    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    if plan is None:
        return
    if (not display) or (plan is None):
        disconnect()
        return

    with LockRenderer():
        commands = post_process(problem, plan)
    if args.viewer:
        restore_state(state_id)
    else:
        disconnect()
        connect(use_gui=True)
        with HideOutput():
            problem_fn() # TODO: way of doing this without reloading?

    if args.simulate:
        control_commands(commands)
    else:
        apply_commands(State(), commands, time_step=0.01)
    user_input('Finish?')
    disconnect()
    # TODO: need to wrap circular joints

if __name__ == '__main__':
    main()