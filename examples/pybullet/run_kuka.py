#!/usr/bin/env python

from __future__ import print_function

import argparse
import time
import pybullet as p # TODO: try/catch
import numpy as np
import cProfile
import pstats

from examples.pybullet.utils.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, get_stable_gen, get_ik_fn, get_free_motion_gen, \
    get_holding_motion_gen, get_movable_collision_test
from examples.pybullet.utils.utils import WorldSaver, connect, dump_world, get_pose, set_pose, Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, wait_for_interrupt, is_placement, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, get_bodies, input

from pddlstream.utils import read, get_file_path
from pddlstream.conversion import And, Equal
from pddlstream.focused import solve_focused
from pddlstream.stream import from_fn, from_test, StreamInfo, from_gen_fn
from pddlstream.utils import print_solution, user_input, read, INF, get_file_path


def pddlstream_from_problem(robot, movable=[],
                    teleport=False, movable_collisions=False, grasp_name='top'):
    #assert (not are_colliding(tree, kin_cache))

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    print('Robot:', robot)
    conf = BodyConf(robot, get_configuration(robot))
    init = [('CanMove',),
            ('Conf', conf),
            ('AtConf', conf),
            ('HandEmpty',)]

    rigid = [body for body in get_bodies() if body != robot]
    fixed = [body for body in rigid if body not in movable]
    print('Movable:', movable)
    print('Fixed:', fixed)
    for body in movable:
        pose = BodyPose(body, get_pose(body))
        init += [('Graspable', body),
                 ('Pose', body, pose),
                 ('AtPose', body, pose)]
        for surface in fixed:
            init += [('Stackable', body, surface)]
            if is_placement(body, surface):
                init += [('Supported', body, pose, surface)]

    for body in fixed:
        name = get_body_name(body)
        if 'sink' in name:
            init += [('Sink', body)]
        if 'stove' in name:
            init += [('Stove', body)]

    body = movable[0]
    goal = ('and',
            ('AtConf', conf),
            #('Holding', body),
            #('On', body, fixed[1]),
            #('On', body, fixed[2]),
            ('Cleaned', body),
            #('Cooked', body),
    )

    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(fixed)),
        'sample-grasp': from_gen_fn(get_grasp_gen(robot, grasp_name)),
        'inverse-kinematics': from_fn(get_ik_fn(robot, fixed, teleport)),
        'plan-free-motion': from_fn(get_free_motion_gen(robot, fixed, teleport)),
        'plan-holding-motion': from_fn(get_holding_motion_gen(robot, fixed, teleport)),
        'TrajCollision': get_movable_collision_test(),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal


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

#######################################################

def main(viewer=False, display=True, simulate=False):
    # TODO: fix argparse & FastDownward
    #parser = argparse.ArgumentParser()  # Automatically includes help
    #parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    #parser.add_argument('-display', action='store_true', help='enable viewer.')
    #args = parser.parse_args()

    connect(use_gui=viewer)
    robot, block = load_world()
    saved_world = WorldSaver()
    dump_world()

    pddlstream_problem = pddlstream_from_problem(robot, movable=[block], teleport=False, movable_collisions=True)

    pr = cProfile.Profile()
    pr.enable()
    solution = solve_focused(pddlstream_problem, synthesizers=[], max_cost=INF)
    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    if plan is None:
        return

    if (not display) or (plan is None):
        p.disconnect()
        return

    paths = []
    for name, args in plan:
        if name == 'place':
            paths += args[-1].reverse().body_paths
        elif name in ['move', 'move_free', 'move_holding', 'pick']:
            paths += args[-1].body_paths
    print(paths)
    command = Command(paths)

    if not viewer: # TODO: how to reenable the viewer
        disconnect()
        connect(use_gui=True)
        load_world()
    saved_world.restore()

    input('Execute?')
    if simulate:
        command.control()
    else:
        #command.step()
        command.refine(num_steps=10).execute(time_step=0.005)

    #wait_for_interrupt()
    input('Finish?')
    disconnect()

if __name__ == '__main__':
    main()