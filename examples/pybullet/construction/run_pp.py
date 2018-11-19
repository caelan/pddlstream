import json
import os
import cProfile
import pstats
import numpy as np
import time

from collections import namedtuple

from examples.pybullet.construction.run import MotionTrajectory
from examples.pybullet.construction.utils import DISABLED_COLLISIONS, parse_point, parse_transform
from examples.pybullet.utils.pybullet_tools.utils import get_movable_joints, link_from_name, has_link, set_pose, \
    multiply, invert, inverse_kinematics, plan_waypoints_joint_motion, Attachment, set_joint_positions, unit_quat, \
    plan_joint_motion, get_configuration, wait_for_interrupt, point_from_pose, HideOutput, load_pybullet, set_point, \
    draw_pose, unit_quat, create_obj, add_body_name, get_pose, pose_from_tform, connect, WorldSaver, get_sample_fn, \
    wait_for_duration, enable_gravity, enable_real_time, trajectory_controller, simulate_controller, \
    add_fixed_constraint, remove_fixed_constraint, elapsed_time, dump_body
from pddlstream.language.constants import And
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.utils import read, get_file_path, print_solution
from pddlstream.algorithms.focused import solve_focused


PICKNPLACE_DIRECTORY = 'picknplace/'
PICKNPLACE_FILENAMES = {
    #'choreo_brick_demo': 'choreo_brick_demo.json',
    'choreo_brick_demo': 'json/brick_demo.json',
    'choreo_eth-trees_demo': 'choreo_eth-trees_demo.json',
}
GRASP_NAMES = ['pick_grasp_approach_plane', 'pick_grasp_plane', 'pick_grasp_retreat_plane']
TOOL_NAME = 'eef_tcp_frame' # robot_tool0 | eef_base_link | eef_tcp_frame
SELF_COLLISIONS = False


##################################################

Brick = namedtuple('Brick', ['index', 'body', 'initial_pose', 'goal_pose',
                             'grasps', 'goal_supports'])

class WorldPose(object):
    def __init__(self, index, value):
        self.index = index
        self.value = value
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.index, str(np.array(point_from_pose(self.value))))

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

def load_pick_and_place(extrusion_name, scale=0.001, max_bricks=6):
    assert extrusion_name == 'choreo_brick_demo'
    root_directory = os.path.dirname(os.path.abspath(__file__))
    bricks_directory = os.path.join(root_directory, PICKNPLACE_DIRECTORY, 'bricks')
    print('Name: {}'.format(extrusion_name))
    with open(os.path.join(bricks_directory, PICKNPLACE_FILENAMES[extrusion_name]), 'r') as f:
        json_data = json.loads(f.read())

    #kuka_urdf = 'framefab_kr6_r900_support/urdf/kr6_r900_wo_ee.urdf'
    kuka_urdf = 'framefab_kr6_r900_support/urdf/kr6_r900_mit_suction_gripper.urdf'
    obj_directory = os.path.join(bricks_directory, 'meshes', 'collision')
    with HideOutput():
        #world = load_pybullet(os.path.join(bricks_directory, 'urdf', 'brick_demo.urdf'))
        robot = load_pybullet(os.path.join(root_directory, kuka_urdf), fixed_base=True)
    #set_point(robot, (0.14, 0, 0))
    dump_body(robot)

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
        for grasp in brick.grasps:
            yield grasp,
    return gen_fn


def get_ik_gen_fn(robot, brick_from_index, obstacle_from_name, max_attempts=25):
    movable_joints = get_movable_joints(robot)
    tool_link = link_from_name(robot, TOOL_NAME)
    disabled_collisions = {tuple(link_from_name(robot, link) for link in pair if has_link(robot, link))
                           for pair in DISABLED_COLLISIONS}
    sample_fn = get_sample_fn(robot, movable_joints)

    def gen_fn(index, pose, grasp):
        body = brick_from_index[index].body
        #world_pose = get_link_pose(robot, tool_link)
        #draw_pose(world_pose, length=0.04)
        #set_pose(body, multiply(world_pose, grasp.attach))
        #draw_pose(multiply(pose.value, invert(grasp.attach)), length=0.04)
        #wait_for_interrupt()
        set_pose(body, pose.value)
        for _ in range(max_attempts):
            set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
            attach_pose = multiply(pose.value, invert(grasp.attach))
            attach_conf = inverse_kinematics(robot, tool_link, attach_pose)
            if attach_conf is None:
                continue
            approach_pose = multiply(attach_pose, ([0, 0, -0.1], unit_quat()))
            #approach_pose = multiply(pose.value, invert(grasp.approach))
            approach_conf = inverse_kinematics(robot, tool_link, approach_pose)
            if approach_conf is None:
                continue
            # TODO: retreat
            path = plan_waypoints_joint_motion(robot, movable_joints, [approach_conf, attach_conf],
                                               obstacles=obstacle_from_name.values(),
                                               self_collisions=SELF_COLLISIONS, disabled_collisions=disabled_collisions)
            if path is None:
                continue
            #path = [approach_conf, attach_conf]
            attachment = Attachment(robot, tool_link, grasp.attach, body)
            traj = MotionTrajectory(robot, movable_joints, path, attachments=[attachment])
            yield approach_conf, traj
            break
    return gen_fn


def get_motion_fn(robot, brick_from_index, obstacle_from_name):
    movable_joints = get_movable_joints(robot)
    tool_link = link_from_name(robot, TOOL_NAME)
    disabled_collisions = {tuple(link_from_name(robot, link) for link in pair if has_link(robot, link))
                           for pair in DISABLED_COLLISIONS}

    def fn(conf1, conf2, fluents=[]):
        #path = [conf1, conf2]
        obstacles = list(obstacle_from_name.values())
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

        set_joint_positions(robot, movable_joints, conf1)
        path = plan_joint_motion(robot, movable_joints, conf2, obstacles=obstacles, attachments=attachments,
                                 self_collisions=True, disabled_collisions=disabled_collisions,
                                 #weights=weights, resolutions=resolutions,
                                 restarts=5, iterations=50, smooth=100)
        if path is None:
            return None
        traj = MotionTrajectory(robot, movable_joints, path)
        return traj,
    return fn


def get_collision_test(robot, brick_from_index):
    def test(*args):
        return False
    return test

##################################################

def get_pddlstream(robot, brick_from_index, obstacle_from_name):
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
    indices = range(2, 5)
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

    stream_map = {
        'sample-grasp': from_gen_fn(get_grasp_gen_fn(brick_from_index)),
        'inverse-kinematics': from_gen_fn(get_ik_gen_fn(robot, brick_from_index, obstacle_from_name)),
        'plan-motion': from_fn(get_motion_fn(robot, brick_from_index, obstacle_from_name)),
        'TrajCollision': get_collision_test(robot, brick_from_index),
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

def step_plan(plan, time_step=0.02): #time_step=np.inf
    wait_for_interrupt()
    attachments = {}
    for action, args in plan:
        trajectory = args[-1]
        if action == 'move':
            step_trajectory(trajectory, attachments, time_step)
        elif action == 'pick':
            attachment = trajectory.attachments.pop()
            step_trajectory(trajectory, attachments, time_step)
            attachments[attachment.child] = attachment
            step_trajectory(trajectory.reverse(), attachments, time_step)
        elif action == 'place':
            attachment = trajectory.attachments.pop()
            step_trajectory(trajectory, attachments, time_step)
            del attachments[attachment.child]
            step_trajectory(trajectory.reverse(), attachments, time_step)
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

def main(viewer=True, collisions=False):
    connect(use_gui=viewer)
    robot, brick_from_index, obstacle_from_name = load_pick_and_place('choreo_brick_demo') # choreo_brick_demo | choreo_eth-trees_demo
    if not collisions:
        obstacle_from_name = {}

    np.set_printoptions(precision=2)
    pr = cProfile.Profile()
    pr.enable()
    with WorldSaver():
        pddlstream_problem = get_pddlstream(robot, brick_from_index, obstacle_from_name)
        solution = solve_focused(pddlstream_problem, planner='ff-wastar1', max_time=30)
    pr.disable()
    pstats.Stats(pr).sort_stats('cumtime').print_stats(10)
    print_solution(solution)
    plan, _, _ = solution
    if plan is None:
        return
    step_plan(plan)
    #simulate_plan(plan)


if __name__ == '__main__':
    main()