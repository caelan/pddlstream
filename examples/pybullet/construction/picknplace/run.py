from __future__ import print_function

import argparse
import cProfile
import json
import os
import pstats
import random
import time
from collections import namedtuple

import numpy as np

from examples.pybullet.construction.extrusion.run import MotionTrajectory
from examples.pybullet.construction.extrusion.utils import get_disabled_collisions, parse_point, \
    parse_transform, get_custom_limits
from examples.pybullet.utils.pybullet_tools.ikfast.kuka_kr6r900.ik import sample_tool_ik
from examples.pybullet.utils.pybullet_tools.utils import get_movable_joints, link_from_name, set_pose, \
    multiply, invert, inverse_kinematics, plan_direct_joint_motion, Attachment, set_joint_positions, plan_joint_motion, \
    get_configuration, wait_for_interrupt, point_from_pose, HideOutput, load_pybullet, draw_pose, unit_quat, create_obj, \
    add_body_name, get_pose, pose_from_tform, connect, WorldSaver, get_sample_fn, \
    wait_for_duration, enable_gravity, enable_real_time, trajectory_controller, simulate_controller, \
    add_fixed_constraint, remove_fixed_constraint, Pose, Euler, get_collision_fn
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, print_solution
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.utils import read, get_file_path

#PICKNPLACE_DIRECTORY = 'picknplace/'
PICKNPLACE_DIRECTORY = ''
PICKNPLACE_FILENAMES = {
    #'choreo_brick_demo': 'choreo_brick_demo.json',
    'choreo_brick_demo': 'json/brick_demo.json',
    'choreo_eth-trees_demo': 'choreo_eth-trees_demo.json',
}
GRASP_NAMES = ['pick_grasp_approach_plane', 'pick_grasp_plane', 'pick_grasp_retreat_plane']
TOOL_NAME = 'eef_tcp_frame' # robot_tool0 | eef_base_link | eef_tcp_frame
SELF_COLLISIONS = False
MILLIMETER = 0.001
IK_FAST = True

##################################################

Brick = namedtuple('Brick', ['index', 'body', 'initial_pose', 'goal_pose',
                             'grasps', 'goal_supports'])

class WorldPose(object):
    def __init__(self, index, value):
        self.index = index
        self.value = value
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.index,
                                  str(np.array(point_from_pose(self.value))))

class Grasp(object):
    def __init__(self, index, num, approach, attach, retreat):
        self.index = index
        self.num = num
        self.approach = approach
        self.attach = attach
        self.retreat = retreat
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.index, self.num)

def strip_extension(path):
    root, ext = os.path.splitext(path)
    return root


##################################################

def load_pick_and_place(extrusion_name, scale=MILLIMETER, max_bricks=6):
    assert extrusion_name == 'choreo_brick_demo'
    root_directory = os.path.dirname(os.path.abspath(__file__))
    bricks_directory = os.path.join(root_directory, PICKNPLACE_DIRECTORY, 'bricks')
    print('Name: {}'.format(extrusion_name))
    with open(os.path.join(bricks_directory, PICKNPLACE_FILENAMES[extrusion_name]), 'r') as f:
        json_data = json.loads(f.read())

    kuka_urdf = '../models/framefab_kr6_r900_support/urdf/kr6_r900_mit_suction_gripper.urdf'
    obj_directory = os.path.join(bricks_directory, 'meshes', 'collision')
    with HideOutput():
        #world = load_pybullet(os.path.join(bricks_directory, 'urdf', 'brick_demo.urdf'))
        robot = load_pybullet(os.path.join(root_directory, kuka_urdf), fixed_base=True)
    #set_point(robot, (0.14, 0, 0))
    #dump_body(robot)

    pick_base_point = parse_point(json_data['pick_base_center_point'])
    draw_pose((pick_base_point, unit_quat()))
    place_base_point = parse_point(json_data['place_base_center_point'])
    draw_pose((place_base_point, unit_quat()))

    # workspace_geo_place_support_object_11 = pick_left_over_bricks_11
    obstacle_from_name = {}
    for filename in json_data['pick_support_surface_file_names']:
        obstacle_from_name[strip_extension(filename)] = \
            create_obj(os.path.join(obj_directory, filename), scale=scale, color=(0.5, 0.5, 0.5, 1))
    for filename in json_data['place_support_surface_file_names']:
        obstacle_from_name[strip_extension(filename)] = \
            create_obj(os.path.join(obj_directory, filename), scale=scale, color=(1., 0., 0., 1))

    brick_from_index = {}
    for json_element in json_data['sequenced_elements']:
        index = json_element['order_id']
        pick_body = create_obj(os.path.join(obj_directory, json_element['pick_element_geometry_file_name']),
                               scale=scale, color=(0, 0, 1, 1))
        add_body_name(pick_body, index)
        #place_body = create_obj(os.path.join(obj_directory, json_element['place_element_geometry_file_name']),
        #                        scale=scale, color=(0, 1, 0, 1))
        #add_body_name(place_body, name)
        world_from_obj_pick = get_pose(pick_body)

        # [u'pick_grasp_plane', u'pick_grasp_retreat_plane', u'place_grasp_retreat_plane',
        #  u'pick_grasp_approach_plane', u'place_grasp_approach_plane', u'place_grasp_plane']
        ee_grasp_poses = [{name: pose_from_tform(parse_transform(approach)) for name, approach in json_grasp.items()}
                           for json_grasp in json_element['grasps']]

        # pick_grasp_plane is at the top of the object with z facing downwards
        grasp_index = 0
        world_from_ee_pick = ee_grasp_poses[grasp_index]['pick_grasp_plane']
        world_from_ee_place = ee_grasp_poses[grasp_index]['place_grasp_plane']
        #draw_pose(world_from_ee_pick, length=0.04)
        #draw_pose(world_from_ee_place, length=0.04)

        ee_from_obj = multiply(invert(world_from_ee_pick), world_from_obj_pick) # Using pick frame
        world_from_obj_place = multiply(world_from_ee_place, ee_from_obj)
        #set_pose(pick_body, world_from_obj_place)

        grasps = [Grasp(index, num, *[multiply(invert(world_from_ee_pick[name]), world_from_obj_pick)
                                      for name in GRASP_NAMES])
                  for num, world_from_ee_pick in enumerate(ee_grasp_poses)]
        brick_from_index[index] = Brick(index=index, body=pick_body,
                                        initial_pose=WorldPose(index, world_from_obj_pick),
                                        goal_pose=WorldPose(index, world_from_obj_place),
                                        grasps=grasps,
                                        goal_supports=json_element.get('place_contact_ngh_ids', []))
        # pick_contact_ngh_ids are movable element contact partial orders
        # pick_support_surface_file_names are fixed element contact partial orders

    return robot, brick_from_index, obstacle_from_name

##################################################

def get_grasp_gen_fn(brick_from_index):
    def gen_fn(index):
        brick = brick_from_index[index]
        while True:
            original_grasp = random.choice(brick.grasps)
            theta = random.uniform(-np.pi, +np.pi)
            rotation = Pose(euler=Euler(yaw=theta))
            new_attach = multiply(rotation, original_grasp.attach)
            grasp = Grasp(None, None, None, new_attach, None)
            yield grasp,
    return gen_fn


def get_ik_gen_fn(robot, brick_from_index, obstacle_from_name, max_attempts=25):
    movable_joints = get_movable_joints(robot)
    tool_link = link_from_name(robot, TOOL_NAME)
    disabled_collisions = get_disabled_collisions(robot)
    sample_fn = get_sample_fn(robot, movable_joints)
    approach_distance = 0.1
    #approach_distance = 0.0
    approach_vector = approach_distance*np.array([0, 0, -1])

    def gen_fn(index, pose, grasp):
        body = brick_from_index[index].body
        set_pose(body, pose.value)

        obstacles = list(obstacle_from_name.values()) # + [body]
        collision_fn = get_collision_fn(robot, movable_joints, obstacles=obstacles, attachments=[],
                                        self_collisions=SELF_COLLISIONS,
                                        disabled_collisions=disabled_collisions,
                                        custom_limits=get_custom_limits(robot))
        attach_pose = multiply(pose.value, invert(grasp.attach))
        approach_pose = multiply(attach_pose, (approach_vector, unit_quat()))
        # approach_pose = multiply(pose.value, invert(grasp.approach))
        for _ in range(max_attempts):
            if IK_FAST:
                attach_conf = sample_tool_ik(robot, attach_pose)
            else:
                set_joint_positions(robot, movable_joints, sample_fn())  # Random seed
                attach_conf = inverse_kinematics(robot, tool_link, attach_pose)
            if (attach_conf is None) or collision_fn(attach_conf):
                continue
            set_joint_positions(robot, movable_joints, attach_conf)
            if IK_FAST:
                approach_conf = sample_tool_ik(robot, approach_pose, nearby_conf=attach_conf)
            else:
                approach_conf = inverse_kinematics(robot, tool_link, approach_pose)
            if (approach_conf is None) or collision_fn(approach_conf):
                continue
            set_joint_positions(robot, movable_joints, approach_conf)
            path = plan_direct_joint_motion(robot, movable_joints, attach_conf,
                                            obstacles=obstacles,
                                            self_collisions=SELF_COLLISIONS,
                                            disabled_collisions=disabled_collisions)
            if path is None: # TODO: retreat
                continue
            #path = [approach_conf, attach_conf]
            attachment = Attachment(robot, tool_link, grasp.attach, body)
            traj = MotionTrajectory(robot, movable_joints, path, attachments=[attachment])
            yield approach_conf, traj
            break
    return gen_fn


def parse_fluents(robot, brick_from_index, fluents, obstacles):
    tool_link = link_from_name(robot, TOOL_NAME)
    attachments = []
    for fact in fluents:
        if fact[0] == 'atpose':
            index, pose = fact[1:]
            body = brick_from_index[index].body
            set_pose(body, pose.value)
            obstacles.append(body)
        elif fact[0] == 'atgrasp':
            index, grasp = fact[1:]
            body = brick_from_index[index].body
            attachments.append(Attachment(robot, tool_link, grasp.attach, body))
        else:
            raise NotImplementedError(fact[0])
    return attachments


def get_motion_fn(robot, brick_from_index, obstacle_from_name, teleport=False):
    movable_joints = get_movable_joints(robot)
    disabled_collisions = get_disabled_collisions(robot)

    def fn(conf1, conf2, fluents=[]):
        if teleport is True:
            path = [conf1, conf2]
            traj = MotionTrajectory(robot, movable_joints, path)
            return traj,
        obstacles = list(obstacle_from_name.values())
        attachments = parse_fluents(robot, brick_from_index, fluents, obstacles)
        set_joint_positions(robot, movable_joints, conf1)
        path = plan_joint_motion(robot, movable_joints, conf2,
                                 obstacles=obstacles, attachments=attachments,
                                 self_collisions=SELF_COLLISIONS, disabled_collisions=disabled_collisions,
                                 #weights=weights, resolutions=resolutions,
                                 restarts=5, iterations=50, smooth=100)
        if path is None:
            return None
        traj = MotionTrajectory(robot, movable_joints, path)
        return traj,
    return fn


def get_collision_test(robot, brick_from_index, collisions=True):
    def test(*args):
        if not collisions:
            return False
        # TODO: finish this
        return False
    return test

##################################################

def get_pddlstream(robot, brick_from_index, obstacle_from_name, collisions=True, teleport=False):
    domain_pddl = read(get_file_path(__file__, os.path.join(PICKNPLACE_DIRECTORY, 'domain.pddl')))
    stream_pddl = read(get_file_path(__file__, os.path.join(PICKNPLACE_DIRECTORY, 'stream.pddl')))
    constant_map = {}

    conf = np.array(get_configuration(robot))
    init = [
        ('CanMove',),
        ('Conf', conf),
        ('AtConf', conf),
        ('HandEmpty',),
    ]

    goal_literals = [
        ('AtConf', conf),
        #('HandEmpty',),
    ]

    #indices = brick_from_index.keys()
    #indices = range(2, 5)
    indices = [4]
    for index in list(indices):
        indices.append(index+6)

    for index in indices:
        brick = brick_from_index[index]
        init += [
            ('Graspable', index),
            ('Pose', index, brick.initial_pose),
            ('AtPose', index, brick.initial_pose),
            ('Pose', index, brick.goal_pose),
        ]
        goal_literals += [
            #('Holding', index),
            ('AtPose', index, brick.goal_pose),
        ]
        for index2 in brick.goal_supports:
            brick2 = brick_from_index[index2]
            init += [
                ('Supported', index, brick.goal_pose, index2, brick2.goal_pose),
            ]
    goal = And(*goal_literals)

    if not collisions:
        obstacle_from_name = {}
    stream_map = {
        'sample-grasp': from_gen_fn(get_grasp_gen_fn(brick_from_index)),
        'inverse-kinematics': from_gen_fn(get_ik_gen_fn(robot, brick_from_index, obstacle_from_name)),
        'plan-motion': from_fn(get_motion_fn(robot, brick_from_index, obstacle_from_name, teleport=teleport)),
        'TrajCollision': get_collision_test(robot, brick_from_index, collisions=collisions),
    }
    #stream_map = 'debug'

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def step_trajectory(trajectory, attachments={}, time_step=np.inf):
    for _ in trajectory.iterate():
        for attachment in attachments.values():
            attachment.assign()
        if time_step == np.inf:
            wait_for_interrupt(time_step)
        else:
            wait_for_duration(time_step)

def step_plan(plan, **kwargs):
    wait_for_interrupt()
    attachments = {}
    for action, args in plan:
        trajectory = args[-1]
        if action == 'move':
            step_trajectory(trajectory, attachments, **kwargs)
        elif action == 'pick':
            attachment = trajectory.attachments.pop()
            step_trajectory(trajectory, attachments, **kwargs)
            attachments[attachment.child] = attachment
            step_trajectory(trajectory.reverse(), attachments, **kwargs)
        elif action == 'place':
            attachment = trajectory.attachments.pop()
            step_trajectory(trajectory, attachments, **kwargs)
            del attachments[attachment.child]
            step_trajectory(trajectory.reverse(), attachments, **kwargs)
        else:
            raise NotImplementedError(action)
    wait_for_interrupt()

##################################################

def simulate_trajectory(trajectory, time_step=0.0):
    start_time = time.time()
    for sim_time in simulate_controller(trajectory_controller(trajectory.robot, trajectory.joints, trajectory.path)):
        if time_step:
            time.sleep(time_step)
        #print(sim_time, elapsed_time(start_time))

def simulate_plan(plan, time_step=0.0, real_time=False): #time_step=np.inf
    wait_for_interrupt()
    enable_gravity()
    if real_time:
        enable_real_time()
    for action, args in plan:
        trajectory = args[-1]
        if action == 'move':
            simulate_trajectory(trajectory, time_step)
        elif action == 'pick':
            attachment = trajectory.attachments.pop()
            simulate_trajectory(trajectory, time_step)
            add_fixed_constraint(attachment.child, attachment.parent, attachment.parent_link)
            simulate_trajectory(trajectory.reverse(), time_step)
        elif action == 'place':
            attachment = trajectory.attachments.pop()
            simulate_trajectory(trajectory, time_step)
            remove_fixed_constraint(attachment.child, attachment.parent, attachment.parent_link)
            simulate_trajectory(trajectory.reverse(), time_step)
        else:
            raise NotImplementedError(action)
    wait_for_interrupt()

##################################################

def main():
    parser = argparse.ArgumentParser()
    # choreo_brick_demo | choreo_eth-trees_demo
    parser.add_argument('-p', '--problem', default='choreo_brick_demo', help='The name of the problem to solve')
    parser.add_argument('-c', '--cfree', action='store_true', help='Disables collisions with obstacles')
    parser.add_argument('-t', '--teleport', action='store_true', help='Teleports instead of computing motions')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning (slow!)')
    args = parser.parse_args()
    print('Arguments:', args)

    connect(use_gui=args.viewer)
    robot, brick_from_index, obstacle_from_name = load_pick_and_place(args.problem)

    np.set_printoptions(precision=2)
    pr = cProfile.Profile()
    pr.enable()
    with WorldSaver():
        pddlstream_problem = get_pddlstream(robot, brick_from_index, obstacle_from_name,
                                            collisions=not args.cfree, teleport=args.teleport)
        solution = solve_focused(pddlstream_problem, planner='ff-wastar1', max_time=30)
    pr.disable()
    pstats.Stats(pr).sort_stats('cumtime').print_stats(10)
    print_solution(solution)
    plan, _, _ = solution
    if plan is None:
        return
    step_plan(plan, time_step=(np.inf if args.teleport else 0.1))
    #simulate_plan(plan)


if __name__ == '__main__':
    main()