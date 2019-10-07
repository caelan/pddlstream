#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import numpy as np

from examples.pybullet.utils.pybullet_tools.pr2_problems import create_floor, create_table
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_top_grasps, get_side_grasps
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, set_pose, Point, disconnect, HideOutput, \
    wait_for_user, load_pybullet, WSG_50_URDF, get_model_path, draw_pose, \
    link_from_name, get_max_limit, get_movable_joints, set_joint_position, unit_pose, create_box, RED, set_point, \
    stable_z, set_camera_pose, LockRenderer, add_line, multiply, invert, get_relative_pose, GREEN, BLUE, TAN, Euler, Pose

from examples.pybullet.gripper.run_top import EPSILON, TABLE_WIDTH, BLOCK_SIDE, open_gripper

# TODO: NAMO

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    connect(use_gui=True)

    with LockRenderer():
        draw_pose(unit_pose(), length=1, width=1)
        floor = create_floor()
        set_point(floor, Point(z=-EPSILON))

        table1 = create_table(width=TABLE_WIDTH, length=TABLE_WIDTH/2, height=TABLE_WIDTH/2,
                              top_color=TAN, cylinder=False)
        set_point(table1, Point(y=+0.5))

        table2 = create_table(width=TABLE_WIDTH, length=TABLE_WIDTH/2, height=TABLE_WIDTH/2,
                              top_color=TAN, cylinder=False)
        set_point(table2, Point(y=-0.5))

        tables = [table1, table2]

        #set_euler(table1, Euler(yaw=np.pi/2))
        with HideOutput():
            # data_path = add_data_path()
            # robot_path = os.path.join(data_path, WSG_GRIPPER)
            robot_path = get_model_path(WSG_50_URDF)  # WSG_50_URDF | PANDA_HAND_URDF
            robot = load_pybullet(robot_path, fixed_base=True)
            #dump_body(robot)

        block1 = create_box(w=BLOCK_SIDE, l=BLOCK_SIDE, h=BLOCK_SIDE, color=RED)
        block_z = stable_z(block1, table1)
        set_point(block1, Point(y=-0.5, z=block_z))

        block2 = create_box(w=BLOCK_SIDE, l=BLOCK_SIDE, h=BLOCK_SIDE, color=GREEN)
        set_point(block2, Point(x=-0.25, y=-0.5, z=block_z))

        block3 = create_box(w=BLOCK_SIDE, l=BLOCK_SIDE, h=BLOCK_SIDE, color=BLUE)
        set_point(block3, Point(x=-0.15, y=+0.5, z=block_z))

        blocks = [block1, block2, block3]

        set_camera_pose(camera_point=Point(x=-1, z=block_z+1), target_point=Point(z=block_z))

    block_pose = get_pose(block1)
    open_gripper(robot)
    tool_link = link_from_name(robot, 'tool_link')
    base_from_tool = get_relative_pose(robot, tool_link)
    #draw_pose(unit_pose(), parent=robot, parent_link=tool_link)
    grasps = get_side_grasps(block1, tool_pose=Pose(euler=Euler(yaw=np.pi/2)),
                             top_offset=0.02, grasp_length=0.03, under=False)[1:2]
    for grasp in grasps:
        gripper_pose = multiply(block_pose, invert(grasp))
        set_pose(robot, multiply(gripper_pose, invert(base_from_tool)))
        wait_for_user()

    wait_for_user('Finish?')
    disconnect()

if __name__ == '__main__':
    main()