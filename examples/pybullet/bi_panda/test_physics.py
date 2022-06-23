from __future__ import print_function

import numpy as np
import math

from examples.pybullet.utils.pybullet_tools.bi_panda_problems import create_bi_panda, create_short_table, Problem, create_table
from examples.pybullet.utils.pybullet_tools.panda_utils import get_other_arm, get_carry_conf, set_arm_conf, open_arm, get_gripper_joints, get_max_limit,\
    arm_conf, REST_LEFT_ARM, close_arm, set_group_conf, STRAIGHT_LEFT_ARM, get_extended_conf, PLATE_GRASP_LEFT_ARM, TOP_HOLDING_LEFT_ARM_CENTERED
from examples.pybullet.utils.pybullet_tools.utils import get_bodies, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, joint_from_name, get_point, wait_for_user,\
    RED, GREEN, BLUE, BLACK, WHITE, BROWN, TAN, GREY, create_cylinder, enable_gravity, link_from_name, get_link_pose, \
    Pose, set_joint_position, TRAY_URDF, set_pose, COKE_URDF
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, disconnect, \
    get_joint_positions, HideOutput, LockRenderer, wait_for_user, get_max_limit, enable_gravity
import pybullet as p
from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import set_joint_force_limits, Attach, control_commands, Grasp, GripperCommand

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
    # TODO: packing problem where you have to place in one direction
    print('in bi_manual_place')
    base_extent = 5.0

    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.04
    block_height = 0.1
    #block_height = 2*block_width
    block_area = block_width*block_width

    #plate_width = 2*math.sqrt(num*block_area)
    plate_width = 0.3
    #plate_width = 0.28
    #plate_width = 0.3
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.5)
    plate_height = 0.01

    other_arm = get_other_arm(arm)
    initial_conf = PLATE_GRASP_LEFT_ARM
    add_data_path()
    floor = load_pybullet("plane.urdf")
    bi_panda = create_bi_panda()
    set_joint_force_limits(bi_panda, arm)
    set_arm_conf(bi_panda, arm, PLATE_GRASP_LEFT_ARM)
    open_arm(bi_panda, arm)
    set_arm_conf(bi_panda, other_arm, arm_conf(other_arm, TOP_HOLDING_LEFT_ARM_CENTERED))
    close_arm(bi_panda, other_arm)

    table = create_table(length=0.3, height=0.35, width=0.4)
    set_point(table, point=Point(0.5,-.5, 0))
    l_hand_link = link_from_name(bi_panda, 'l_panda_hand')
    #left finger joint
    l_left_finger_joint = joint_from_name(bi_panda, 'l_panda_finger_joint1')
    #right finger joint
    l_right_finger_joint = joint_from_name(bi_panda, 'l_panda_finger_joint2')
    set_joint_position(bi_panda, l_right_finger_joint,plate_height+.07)
    set_joint_position(bi_panda, l_left_finger_joint, plate_height+.07)
    #left finger joint
    r_left_finger_joint = joint_from_name(bi_panda, 'r_panda_finger_joint1')
    #right finger joint
    r_right_finger_joint = joint_from_name(bi_panda, 'r_panda_finger_joint2')
    set_joint_position(bi_panda, r_right_finger_joint,block_width)
    set_joint_position(bi_panda, r_left_finger_joint, block_width)
    hand_pose = get_link_pose(bi_panda, l_hand_link)
    plate = load_pybullet(TRAY_URDF)
    pose=Pose(point=(hand_pose[0][0] + ((plate_width / 2 + 0.07)*math.cos(initial_conf[0])), hand_pose[0][1] + ((plate_width / 2 + 0.07) *math.sin(initial_conf[0])), hand_pose[0][2]), euler=(0,0,initial_conf[0]))
    set_pose(plate, pose)
    grasp = Grasp('side', plate, pose, [], [])
    attach = Attach(bi_panda, arm, grasp, plate)

    control_commands([attach,])
    surfaces = [table, plate]

    blocks = [load_pybullet(COKE_URDF) for _ in range(num)]
    initial_surfaces = {block: plate for block in blocks}

    min_distances = {block: 0.05 for block in blocks}
    sample_placements(initial_surfaces, min_distances=min_distances)

def main():
  connect(use_gui=True)
  enable_gravity()
  print('connected to gui')
  with HideOutput():
    problem = bi_manual_forceful()
  p.setRealTimeSimulation(1)
  while True:
    # p.stepSimulation()
    # wait_for_user()
    pass

if __name__ == '__main__':
  main()