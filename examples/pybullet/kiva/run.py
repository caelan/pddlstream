#!/usr/bin/env python

from __future__ import print_function

import argparse
import os

from examples.pybullet.utils.pybullet_tools.pr2_problems import create_floor
from examples.pybullet.utils.pybullet_tools.utils import connect, disconnect, wait_for_user, LockRenderer, stable_z, \
    load_model, ROOMBA_URDF, \
    HideOutput, set_point, Point, load_pybullet, get_model_path, set_color, get_all_links, \
    KIVA_SHELF_SDF, add_data_path, draw_pose, unit_pose, dump_body

# TODO: Kiva robot
# TODO: stow, pick, and recharge stations

ORANGE = (1, 0.5, 0, 1)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-viewer', action='store_true', help='enable the viewer while planning')
    args = parser.parse_args()
    print(args)

    connect(use_gui=True)
    with LockRenderer():
        draw_pose(unit_pose(), length=1)
        floor = create_floor()
        with HideOutput():
            robot = load_pybullet(get_model_path(ROOMBA_URDF), fixed_base=True, scale=2.0)
            for link in get_all_links(robot):
                set_color(robot, link=link, color=ORANGE)
            robot_z = stable_z(robot, floor)
            set_point(robot, Point(z=robot_z))
        #set_base_conf(robot, rover_confs[i])

        data_path = add_data_path()
        shelf, = load_model(os.path.join(data_path, KIVA_SHELF_SDF), fixed_base=True) # TODO: returns a tuple
        dump_body(shelf)
        #draw_aabb(get_aabb(shelf))

    wait_for_user()
    disconnect()

if __name__ == '__main__':
    main()
