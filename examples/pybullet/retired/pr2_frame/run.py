#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats

from examples.pybullet.utils.pybullet_tools.pr2_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_stable_gen, get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, State, apply_commands
from examples.pybullet.utils.pybullet_tools.pr2_problems import holding_problem
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_arm_joints, get_group_joints, get_group_conf
from examples.pybullet.utils.pybullet_tools.utils import connect, dump_world, get_pose, is_placement, \
    disconnect, user_input, get_joint_positions, enable_gravity, save_state, restore_state
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant
from pddlstream.utils import read, INF, get_file_path
from pddlstream.language.constants import print_solution

from examples.pybullet.pr2.run import post_process


# TODO: make a fixed body for each object (or one environment body)
# TODO: make no grasp a type of grasp for simplicity
# TODO: make the pose just be relative to a frame (world or arms). Then can just test if while moving two collide.
# TODO: check all moving with all
# TODO: always arm geometry wrt arm frame
# For base: bt, (arm1, q1, o1, p1), (frame2, q2, o2, p2) # TODO: separate out arm?
# For arm: bq, (arm1, at, o1, p1), (frame2, q2, o2, p2)
# TODO: when attempting to generalize, cannot put full kinematic chain in predicates
# Two strategies:
# 1) Conditional effects for directly updating world pose of changed variables (still depends on chains)
# 2) Axioms to update world poses for everything (common frame)
# TODO: from world pose can rederive relative transform for subcomponents
# Don't need world pose for holding? But then need pose

#######################################################

def pddlstream_from_problem(problem, teleport=False, movable_collisions=False):
    robot = problem.robot

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {
        'world': 'world',
    }

    world = 'world'
    #initial_bq = Pose(robot, get_pose(robot))
    initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))
    init = [
        ('CanMove',),
        ('BConf', initial_bq), # TODO: could make pose as well...
        ('AtBConf', initial_bq),
        ('AtAConf', world, None),
        ('AtPose', world, world, None),
    ] + [('Sink', s) for s in problem.sinks] + \
           [('Stove', s) for s in problem.stoves] + \
           [('Connected', b, d) for b, d in problem.buttons] + \
           [('Button', b) for b, _ in problem.buttons]
    #for arm in ARM_JOINT_NAMES:
    for arm in problem.arms:
        joints = get_arm_joints(robot, arm)
        conf = Conf(robot, joints, get_joint_positions(robot, joints))
        init += [('Arm', arm), ('AConf', arm, conf),
                 ('HandEmpty', arm), ('AtAConf', arm, conf), ('AtPose', arm, arm, None)]
        if arm in problem.arms:
            init += [('Controllable', arm)]
    for body in problem.movable:
        pose = Pose(body, get_pose(body))
        init += [('Graspable', body), ('Pose', body, pose),
                 ('AtPose', world, body, pose)]
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

        'plan-base-motion': from_fn(get_motion_gen(problem, teleport=teleport)),
        #'plan-base-motion': empty_gen(),
        'BTrajCollision': fn_from_constant(False),
    }
    # get_press_gen(problem, teleport=teleport)

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

#######################################################

def main(viewer=False, display=True, simulate=False, teleport=False):
    # TODO: fix argparse & FastDownward
    #parser = argparse.ArgumentParser()  # Automatically includes help
    #parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    #parser.add_argument('-display', action='store_true', help='enable viewer.')
    #args = parser.parse_args()

    connect(use_gui=viewer)
    problem_fn = holding_problem
    # holding_problem | stacking_problem | cleaning_problem | cooking_problem
    # cleaning_button_problem | cooking_button_problem
    problem = problem_fn()
    state_id = save_state()
    #saved_world = WorldSaver()
    dump_world()

    pddlstream_problem = pddlstream_from_problem(problem, teleport=teleport)
    _, _, _, stream_map, init, goal = pddlstream_problem
    #synthesizers = [
    #    StreamSynthesizer('safe-base-motion', {'plan-base-motion': 1,
    #                                           'TrajPoseCollision': 0, 'TrajGraspCollision': 0, 'TrajArmCollision': 0},
    #                      from_fn(get_base_motion_synth(problem, teleport))),
    #]
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', stream_map.keys())
    #print('Synthesizers:', synthesizers)

    pr = cProfile.Profile()
    pr.enable()
    solution = solve_focused(pddlstream_problem, success_cost=INF)
    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    if plan is None:
        return
    if (not display) or (plan is None):
        disconnect()
        return

    if viewer:
        restore_state(state_id)
    else:
        disconnect()
        connect(use_gui=True)
        problem = problem_fn()  # TODO: way of doing this without reloading?

    user_input('Execute?')
    commands = post_process(problem, plan)
    if simulate:
        control_commands(commands)
    else:
        apply_commands(State(), commands, time_step=0.01)
    user_input('Finish?')
    disconnect()

if __name__ == '__main__':
    main()