#!/usr/bin/env python

from __future__ import print_function

try:
    import pybullet as p
except ImportError:
    raise ImportError('This example requires PyBullet (https://pypi.org/project/pybullet/)')

import cProfile
import pstats

from examples.pybullet.utils.utils import connect, dump_world, get_pose, Pose, is_placement, \
    disconnect, user_input, get_joint_positions, enable_gravity, set_pose

from examples.pybullet.utils.pr2_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, get_stable_gen, \
    get_grasp_gen, get_press_gen, Attach, Detach, Clean, Cook, Trajectory, control_commands, step_commands
from examples.pybullet.utils.pr2_utils import get_arm_joints, ARM_NAMES
from examples.pybullet.utils.pr2_problems import holding_problem, stacking_problem, cleaning_problem, cooking_problem, \
    cleaning_button_problem, cooking_button_problem

from pddlstream.focused import solve_focused
from pddlstream.stream import from_fn, StreamInfo, from_gen_fn, empty_gen, from_list_fn, fn_from_constant
from pddlstream.synthesizer import StreamSynthesizer
from pddlstream.utils import print_solution, read, INF, get_file_path, find_unique


def place_movable(certified):
    for literal in certified:
        if literal[0] != 'not':
            continue
        fact = literal[1]
        if fact[0] == 'trajposecollision':
            _, b, p = fact[1:]
            p.step()
        if fact[0] == 'trajarmcollision':
            _, a, q = fact[1:]
            q.step()
        if fact[0] == 'trajgraspcollision':
            _, a, o, g = fact[1:]
            # TODO: finish this

def get_base_motion_synth(problem, teleport=False):
    # TODO: could factor the safety checks if desired (but no real point)
    #fixed = get_fixed(robot, movable)
    def fn(outputs, certified):
        assert(len(outputs) == 1)
        q0, _, q1 = find_unique(lambda f: f[0] == 'basemotion', certified)[1:]
        place_movable(certified)
        free_motion_fn = get_motion_gen(problem, teleport)
        return free_motion_fn(q0, q1)
    return fn

#######################################################

def pddlstream_from_problem(problem, teleport=False, movable_collisions=False):
    robot = problem.robot

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    initial_bq = Pose(robot, get_pose(robot))
    init = [
        ('CanMove',),
        ('BConf', initial_bq),
        ('AtBConf', initial_bq),
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

    goal = ['and']
    if problem.goal_conf is not None:
        goal_conf = Pose(robot, problem.goal_conf)
        init += [('BConf', goal_conf)]
        goal += [('AtBConf', goal_conf)]
    goal += [('Holding', a, b) for a, b in problem.goal_holding] + \
                     [('On', b, s) for b, s in problem.goal_on] + \
                     [('Cleaned', b)  for b in problem.goal_cleaned] + \
                     [('Cooked', b)  for b in problem.goal_cooked]

    stream_map = {
        'sample-pose': get_stable_gen(problem),
        'sample-grasp': from_list_fn(get_grasp_gen(problem)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(problem, teleport=teleport)),

        #'plan-base-motion': from_fn(get_motion_gen(problem, teleport=teleport)),
        'plan-base-motion': empty_gen(),
        'TrajPoseCollision': fn_from_constant(False),
        'TrajArmCollision': fn_from_constant(False),
        'TrajGraspCollision': fn_from_constant(False),
    }
    # get_press_gen(problem, teleport=teleport)

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

#######################################################

# TODO: avoid copying this?

def post_process(problem, plan):
    if plan is None:
        return None
    commands = []
    for i, (name, args) in enumerate(plan):
        print(i, name, args)
        if name == 'move_base':
            t = args[-1]
            new_commands = [t]
        elif name == 'pick':
            a, b, p, g, _, t = args
            attach = Attach(problem.robot, a, g, b)
            new_commands = [t, attach, t.reverse()]
        elif name == 'place':
            a, b, p, g, _, t = args
            detach = Detach(problem.robot, a, b)
            new_commands = [t, detach, t.reverse()]
        elif name == 'clean': # TODO: add text or change color?
            body, sink = args
            new_commands = [Clean(body)]
        elif name == 'cook':
            body, stove = args
            new_commands = [Cook(body)]
        elif name == 'press_clean':
            body, sink, arm, button, bq, t = args
            new_commands = [t, Clean(body), t.reverse()]
        elif name == 'press_cook':
            body, sink, arm, button, bq, t = args
            new_commands = [t, Cook(body), t.reverse()]
        else:
            raise ValueError(name)
        commands += new_commands
    return commands

#######################################################

def main(viewer=False, display=True, simulate=False, teleport=False):
    # TODO: fix argparse & FastDownward
    #parser = argparse.ArgumentParser()  # Automatically includes help
    #parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    #parser.add_argument('-display', action='store_true', help='enable viewer.')
    #args = parser.parse_args()

    connect(use_gui=viewer)
    problem_fn = cooking_problem
    # holding_problem | stacking_problem | cleaning_problem | cooking_problem
    # cleaning_button_problem | cooking_button_problem
    problem = problem_fn()
    state_id = p.saveState()
    #saved_world = WorldSaver()
    dump_world()

    pddlstream_problem = pddlstream_from_problem(problem, teleport=teleport)
    _, _, _, stream_map, init, goal = pddlstream_problem
    synthesizers = [
        StreamSynthesizer('safe-base-motion', {'plan-base-motion': 1,
                                               'TrajPoseCollision': 0, 'TrajGraspCollision': 0, 'TrajArmCollision': 0},
                          from_fn(get_base_motion_synth(problem, teleport))),
    ]
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', stream_map.keys())
    print('Synthesizers:', synthesizers)

    pr = cProfile.Profile()
    pr.enable()
    solution = solve_focused(pddlstream_problem, synthesizers=synthesizers, max_cost=INF)
    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    if plan is None:
        return
    if (not display) or (plan is None):
        p.disconnect()
        return

    if viewer:
        p.restoreState(state_id)
    else:
        disconnect()
        connect(use_gui=True)
        problem = problem_fn()  # TODO: way of doing this without reloading?

    commands = post_process(problem, plan)
    if simulate:
        enable_gravity()
        control_commands(commands)
    else:
        step_commands(commands, time_step=0.01)
    user_input('Finish?')
    disconnect()

if __name__ == '__main__':
    main()