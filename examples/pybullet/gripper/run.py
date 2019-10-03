#!/usr/bin/env python

from __future__ import print_function

import argparse

from examples.pybullet.utils.pybullet_tools.pr2_problems import create_floor, create_table
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_top_grasps
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, set_pose, Point, disconnect, HideOutput, \
    wait_for_user, load_pybullet, WSG_50_URDF, get_model_path, draw_pose, \
    link_from_name, get_max_limit, get_movable_joints, set_joint_position, unit_pose, create_box, RED, set_point, \
    stable_z, set_camera_pose, LockRenderer, add_line, multiply, invert, get_relative_pose

# https://www.generationrobots.com/en/403318-fe-gripper-for-panda-robotic-arm.html
# /usr/local/lib/python2.7/site-packages/pybullet_data/gripper/
# drake/manipulation/models/ycb/sdf/005_tomato_soup_can.sdf

def close_gripper(robot):
    for joint in get_movable_joints(robot):
        set_joint_position(robot, joint, get_max_limit(robot, joint))

def open_gripper(robot):
    for joint in get_movable_joints(robot):
        set_joint_position(robot, joint, get_max_limit(robot, joint))

BLOCK_SIDE = 0.07
TABLE_WIDTH = 1.0
EPSILON = 1e-3

# TODO: NAMO and side grasps

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    connect(use_gui=True)
    with LockRenderer():
        draw_pose(unit_pose(), length=1)
        floor = create_floor()
        set_point(floor, Point(z=-EPSILON))
        table = create_table(width=TABLE_WIDTH, length=TABLE_WIDTH/2, height=TABLE_WIDTH/2)
        #set_euler(table, Euler(yaw=np.pi/2))
        with HideOutput():
            # data_path = add_data_path()
            # robot_path = os.path.join(data_path, WSG_GRIPPER)
            robot_path = get_model_path(WSG_50_URDF)  # WSG_50_URDF | PANDA_HAND_URDF
            robot = load_pybullet(robot_path, fixed_base=True)
            #dump_body(robot)

        block = create_box(w=BLOCK_SIDE, l=BLOCK_SIDE, h=BLOCK_SIDE, color=RED)
        block_z = stable_z(block, table)
        set_point(block, Point(z=block_z))

        add_line(Point(x=-TABLE_WIDTH/2, z=block_z - BLOCK_SIDE/2 + EPSILON),
                 Point(x=+TABLE_WIDTH/2, z=block_z - BLOCK_SIDE/2 + EPSILON), color=RED)

        set_camera_pose(camera_point=Point(y=-1, z=block_z+1), target_point=Point(z=block_z))

    block_pose = get_pose(block)
    open_gripper(robot)
    tool_link = link_from_name(robot, 'tool_link')
    base_from_tool = get_relative_pose(robot, tool_link)
    #draw_pose(unit_pose(), parent=robot, parent_link=tool_link)
    grasps = get_top_grasps(block, tool_pose=unit_pose(), under=False)[1:2]
    for grasp in grasps:
        gripper_pose = multiply(block_pose, invert(grasp))
        set_pose(robot, multiply(gripper_pose, invert(base_from_tool)))
        wait_for_user()

    wait_for_user('Finish?')
    disconnect()

if __name__ == '__main__':
    main()