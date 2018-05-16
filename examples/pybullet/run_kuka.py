#!/usr/bin/env python

from __future__ import print_function

import argparse
import time
import pybullet as p # TODO: try/catch

from examples.pybullet.utils.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, get_stable_gen, get_ik_fn, get_free_motion_gen, \
    get_holding_motion_gen, get_movable_collision_test
from examples.pybullet.utils.utils import WorldSaver, connect, dump_world, get_pose, set_pose, Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, wait_for_interrupt, is_placement, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, get_bodies, input

#######################################################

def load_world():
    # TODO: store internal world info here to be reloaded
    robot = load_model(DRAKE_IIWA_URDF)
    # robot = load_model(KUKA_IIWA_URDF)
    floor = load_model('models/short_floor.urdf')
    sink = load_model(SINK_URDF, pose=Pose(Point(x=-0.5)))
    stove = load_model(STOVE_URDF, pose=Pose(Point(x=+0.5)))
    block = load_model(BLOCK_URDF, fixed_base=False)
    #cup = load_model('models/dinnerware/cup/cup_small.urdf', Pose(Point(x=+0.5, y=+0.5, z=0.5)), fixed_base=False)

    set_pose(block, Pose(Point(y=0.5, z=stable_z(block, floor))))
    set_default_camera()

    return robot, block

#

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    parser.add_argument('-display', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    connect(use_gui=args.viewer)
    robot, block = load_world()

    #robot2 = clone_body(robot)
    #block2 = clone_body(block)
    #dump_world()

    saved_world = WorldSaver()
    dump_world()

    #ss_problem = ss_from_problem(robot, movable=[block], teleport=False, movable_collisions=True)
    #ss_problem = ss_problem.debug_problem()
    #print(ss_problem)

    input('Execute?')
    return


    t0 = time.time()
    plan, evaluations = dual_focused(ss_problem, verbose=True)
    # plan, evaluations = incremental(ss_problem, verbose=True)
    print_plan(plan, evaluations)
    print(time.time() - t0)
    if (not args.display) or (plan is None):
        p.disconnect()
        return

    paths = []
    for action, params in plan:
        if action.name == 'place':
            paths += params[-1].reverse().body_paths
        elif action.name in ['move_free', 'move_holding', 'pick']:
            paths += params[-1].body_paths
    print(paths)
    command = Command(paths)

    if not args.viewer: # TODO: how to reenable the viewer
        disconnect()
        connect(use_gui=True)
        load_world()
    saved_world.restore()

    input('Execute?')

    command.control()
    #command.refine(num_steps=10).execute(time_step=0.005)
    #command.step()

    #dt = 1. / 240 # Bullet default
    #p.setTimeStep(dt)

    wait_for_interrupt()
    disconnect()

if __name__ == '__main__':
    main()