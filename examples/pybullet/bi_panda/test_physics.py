from __future__ import print_function

import numpy as np
import math

from examples.pybullet.utils.pybullet_tools.bi_panda_problems import create_bi_panda, create_short_table, Problem, create_table
from examples.pybullet.utils.pybullet_tools.panda_utils import get_other_arm, get_carry_conf, set_arm_conf, open_arm, get_gripper_joints, get_max_limit,\
    arm_conf, REST_LEFT_ARM, close_arm, set_group_conf, STRAIGHT_LEFT_ARM, get_extended_conf, PLATE_GRASP_LEFT_ARM, TOP_HOLDING_LEFT_ARM_CENTERED, BI_PANDA_GROUPS, arm_from_arm, \
    set_joint_positions_torque, get_gripper_link, get_arm_joints
from examples.pybullet.utils.pybullet_tools.utils import get_bodies, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, joint_from_name, get_point, wait_for_user,\
    RED, GREEN, BLUE, BLACK, WHITE, BROWN, TAN, GREY, create_cylinder, enable_gravity, link_from_name, get_link_pose, \
    Pose, set_joint_position, TRAY_URDF, set_pose, COKE_URDF, get_max_force, body_from_name, TARGET, is_pose_close, compute_jacobian, euler_from_quat,\
    wait_for_duration
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, disconnect, \
    get_joint_positions, HideOutput, LockRenderer, wait_for_user, get_max_limit, enable_gravity, get_link_pose, set_joint_positions_torque
import pybullet as p
from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import set_joint_force_limits, Attach, control_commands, Grasp, GripperCommand
from examples.pybullet.utils.pybullet_tools.ikfast.franka_panda.ik import is_ik_compiled, ikfast_inverse_kinematics, PANDA_LEFT_INFO, bi_panda_inverse_kinematics

def sample_placements(body_surfaces, obstacles=None, min_distances={}):
    if obstacles is None:
        obstacles = [body for body in get_bodies() if body not in body_surfaces]
    obstacles = list(obstacles)
    # TODO: max attempts here
    for body, surface in body_surfaces.items():
        min_distance = min_distances.get(body, 0.01)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                return False
            if not any(pairwise_collision(body, obst, max_distance=min_distance)
                       for obst in obstacles if obst not in [body, surface]):
                obstacles.append(body)
                break
    return True

def arm_strain(arm='left', grasp_type='top', num=2):
    # TODO: packing problem where you have to place in one direction
    print('in arm strain')
    base_extent = 5.0
    enable_gravity()
    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.04
    block_height = 0.1
    #block_height = 2*block_width
    block_area = block_width*block_width

    #plate_width = 2*math.sqrt(num*block_area)
    plate_width = 0.2
    #plate_width = 0.28
    #plate_width = 0.3
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.6)
    plate_height = 0.01

    other_arm = get_other_arm(arm)
    initial_conf = get_extended_conf(arm)

    add_data_path()
    floor = load_pybullet("plane.urdf")

    bi_panda = create_bi_panda()
    set_joint_force_limits(bi_panda, arm)

    set_arm_conf(bi_panda, arm, initial_conf)
    open_arm(bi_panda, arm)
    set_arm_conf(bi_panda, other_arm, arm_conf(other_arm, STRAIGHT_LEFT_ARM))
    close_arm(bi_panda, other_arm)
    Attach(bi_panda, arm, grasp, plate)
    l_hand_link = link_from_name(bi_panda, 'l_panda_hand')
    hand_pose = get_link_pose(bi_panda, l_hand_link)
    print(hand_pose)
    plate = create_box(0.09, plate_width, plate_height, color=GREEN, pose=Pose(point=(hand_pose[0][0]+.055, hand_pose[0][1], hand_pose[0][2] + 0.04)), mass=896)

    # plate_z = stable_z(plate, floor)
    # set_point(plate, Point(z=plate_z))
    surfaces = [floor, plate]

    # blocks = [create_cylinder(block_width/2, block_height, color=BLUE) for _ in range(num)]
    # initial_surfaces = {block: plate for block in blocks}

    # min_distances = {block: 0.05 for block in blocks}
    # sample_placements(initial_surfaces, min_distances=min_distances)
    # enable_gravity()

    return []

def bi_manual_forceful(arm='left', grasp_type='top', num=2):
    print('in bi_manual_place')
    base_extent = 5.0

    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.04
    block_height = 0.1
    #block_height = 2*block_width
    block_area = block_width*block_width

    tray_width = 0.27
    tray_height = 0.03

    other_arm = get_other_arm(arm)
    initial_conf = PLATE_GRASP_LEFT_ARM
    add_data_path()
    floor = load_pybullet("plane.urdf")
    bi_panda = create_bi_panda()
    set_joint_force_limits(bi_panda, arm)
    set_joint_force_limits(bi_panda, other_arm)
    set_arm_conf(bi_panda, arm, PLATE_GRASP_LEFT_ARM)
    open_arm(bi_panda, arm)
    set_arm_conf(bi_panda, other_arm, arm_conf(other_arm, TOP_HOLDING_LEFT_ARM_CENTERED))
    joints = [joint_from_name(bi_panda, name) for name in BI_PANDA_GROUPS[arm_from_arm(arm)]]
    set_joint_positions_torque(bi_panda, joints, PLATE_GRASP_LEFT_ARM)
    open_arm(bi_panda, other_arm)

    table = create_table(length=0.4, height=0.35, width = 0.3)
    set_point(table, point=Point(0.45,-.65, 0))
    table2 = create_table(length=0.4, height=0.35, width = 0.3)
    set_point(table2, point=Point(0, 1.8, 0))
    l_hand_link = link_from_name(bi_panda, 'l_panda_hand')
    # #left finger joint
    # l_left_finger_joint = joint_from_name(bi_panda, 'l_panda_finger_joint1')
    # #right finger joint
    # l_right_finger_joint = joint_from_name(bi_panda, 'l_panda_finger_joint2')
    # set_joint_position(bi_panda, l_right_finger_joint,(tray_height/2))
    # set_joint_position(bi_panda, l_left_finger_joint, (tray_height/2))
    # open_arm(bi_panda, arm)
    #left finger joint
    r_left_finger_joint = joint_from_name(bi_panda, 'r_panda_finger_joint1')
    #right finger joint
    r_right_finger_joint = joint_from_name(bi_panda, 'r_panda_finger_joint2')
    set_joint_position(bi_panda, r_right_finger_joint,block_width)
    set_joint_position(bi_panda, r_left_finger_joint, block_width)
    hand_pose = get_link_pose(bi_panda, l_hand_link)
    tray = load_pybullet(TRAY_URDF)
    pose=Pose(point=(hand_pose[0][0], hand_pose[0][1] - (tray_width / 2) - 0.062, hand_pose[0][2]), euler=(0,0,math.pi/2))
    set_pose(tray, pose)
    grasp = Grasp('top', tray, pose, [], [])
    attach = Attach(bi_panda, arm, grasp, tray)#Attach(bi_panda, arm, grasp, tray)
    gripper_link = get_gripper_link(bi_panda, arm)
    gripper_pose = get_link_pose(bi_panda, gripper_link)
    control_commands([attach])
    surfaces = [table]

    blocks = [load_pybullet(COKE_URDF) for _ in range(num)]
    initial_surfaces = {block: table for block in blocks}

    min_distances = {block: 0.05 for block in blocks}
    sample_placements(initial_surfaces, min_distances=min_distances)

    enable_gravity()
    return Problem(robot=bi_panda, movable=blocks, arms=[other_arm], holding_arm=arm, grasp_types=[grasp_type], surfaces=surfaces,
                    goal_on=[(block, tray) for block in blocks], base_limits=base_limits, holding_grasp=grasp, target_width=tray_width, post_goal = table2, gripper_ori=gripper_pose[1])

def compute_joint_velocities(target_ee_velocity, Ja, Jl):
    Jl_inv = np.linalg.pinv(Jl)
    Ja_inv = np.linalg.pinv(Ja)
    linear_vel = np.matmul(target_ee_velocity[:3], Jl_inv)
    angular_vel = np.matmul(target_ee_velocity[3:], Ja_inv)
    return linear_vel + angular_vel

def velocity_move(robot, arm, target_pose):
    print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
    print(target_pose, len(target_pose))
    # target_joints = target_pose[1]
    # target_pose = target_pose[0]
    gripper_link = get_gripper_link(robot, arm)
    joints = get_arm_joints(robot, arm)

    max_forces = [get_max_force(robot, joint) for joint in joints]
    # cur_pose = get_link_pose(int(robot), int(gripper_link))
    cur_pose = get_link_pose(robot, gripper_link)
    print(cur_pose)
    # jointPoses = [target_joints[i] for i in range(len(target_joints))]
    while not is_pose_close(cur_pose, target_pose):
        # print(len(target_joints))
        print(len(joints))
        Jl, Ja =  compute_jacobian(robot, gripper_link)
        Jl = np.array(Jl)[:7]
        Ja = np.array(Ja)[:7]

        diff = [target_pose[0][i] - cur_pose[0][i] for i in range(len(target_pose[0]))]
        ta = euler_from_quat(target_pose[1])
        ca = euler_from_quat(cur_pose[1])
        diffA = [ta[i] - ca[i] for i in range(len(ca))]
        div = 10
        vels = [0, 0.1, 0, 0, 0, 0]
        joint_velocities = compute_joint_velocities(vels, Ja, Jl)
        print(joint_velocities)
        p.setJointMotorControlArray(robot, joints, p.VELOCITY_CONTROL, targetVelocities=joint_velocities, forces=max_forces)
        cur_pose = get_link_pose(robot, gripper_link)



def velocity_control(problem):
    robot = problem.robot
    arm = 'left'
    gripper_link = link_from_name(robot, 'l_panda_link8')
    pose = list(get_link_pose(robot, gripper_link))
    step = .15 / 50
    pose = [[pose[0][0], pose[0][1] + step, pose[0][2]], pose[1]]
    for _ in range(50):

        velocity_move(robot, arm, pose)

def sample_joint_poses(problem):
    robot = problem.robot
    jointNums = get_arm_joints(robot, problem.holding_arm)
    jointPoses = get_joint_positions(robot, jointNums)
    gripper_link = get_gripper_link(robot, problem.holding_arm)
    pose = get_link_pose(robot, gripper_link)
    newJointPoses = None
    repeat_count = 0
    count = 0
    custom_limits = {}
    custom_limits[jointNums[0]] = (PLATE_GRASP_LEFT_ARM[0]-(math.pi/4), PLATE_GRASP_LEFT_ARM[0] + (math.pi/2))
    custom_limits[jointNums[-1]] = (PLATE_GRASP_LEFT_ARM[-1] - (math.pi/2), math.pi/2 - .1)
    step = .2
    pose = [[pose[0][0], pose[0][1] + step, pose[0][2]], pose[1]]
    joints = [jointPoses]
    for _ in range(70):
        set_joint_positions_torque(robot, jointNums, PLATE_GRASP_LEFT_ARM)
        while newJointPoses is None and not is_pose_close(get_link_pose(robot, gripper_link), pose):
            newJointPoses = bi_panda_inverse_kinematics(robot, problem.holding_arm, gripper_link, pose, custom_limits = custom_limits)
        set_joint_positions_torque(robot, jointNums, newJointPoses)
        pose = [[pose[0][0], pose[0][1] + step, pose[0][2]], pose[1]]
        joints.append(newJointPoses)
    # print(get_joint_positions(robot, jointNums))
    print()
    print()
    print()
    print(joints)

def main():
  connect(use_gui=True)
  enable_gravity()
  print('connected to gui')
  with HideOutput():
    problem = bi_manual_forceful()
  p.setRealTimeSimulation(1)
  sample_joint_poses(problem)
  while True:
    # p.stepSimulation()
    # wait_for_user()
    pass

if __name__ == '__main__':
  main()