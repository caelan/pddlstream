from __future__ import print_function

user_input = raw_input

import os
import time
import numpy as np
import random
import argparse
import pydrake

from itertools import product

from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph, DispatchLoadMessage)
from pydrake.lcm import DrakeLcm # Required else "ConnectDrakeVisualizer(): incompatible function arguments."
from pydrake.multibody.multibody_tree import (WeldJoint, PrismaticJoint, JointIndex, FrameIndex)
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import SignalLogger
from pydrake.trajectories import PiecewisePolynomial
from pydrake.systems.analysis import Simulator

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.utils import print_solution, read, INF, get_file_path

from examples.pybullet.utils.motion.motion_planners.rrt_connect import birrt, direct_path
from examples.drake.utils import BoundingBox, create_transform, get_model_bodies, get_model_joints, \
    get_joint_limits, set_min_joint_positions, set_max_joint_positions, get_relative_transform, \
    get_world_pose, set_world_pose, set_joint_position, sample_aabb_placement, fix_input_ports, get_movable_joints, \
    get_base_body, solve_inverse_kinematics, prune_fixed_joints, weld_to_world, get_configuration, get_model_name, \
    get_random_positions, get_aabb_z_placement, get_bodies, get_model_indices, set_joint_positions, get_joint_positions

from examples.drake.kuka_multibody_controllers import (KukaMultibodyController, HandController, ManipStateMachine)

# https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_tree.html
# wget -q https://registry.hub.docker.com/v1/repositories/mit6881/drake-course/tags -O -  | sed -e 's/[][]//g' -e 's/"//g' -e 's/ //g' | tr '}' '\n'  | awk -F: '{print $3}'
# https://stackoverflow.com/questions/28320134/how-to-list-all-tags-for-a-docker-image-on-a-remote-registry
# docker rmi $(docker images -q mit6881/drake-course)

# https://github.com/RobotLocomotion/drake/blob/a54513f9d0e746a810da15b5b63b097b917845f0/bindings/pydrake/multibody/test/multibody_tree_test.py
# ~/Programs/Classes/6811/drake_docker_utility_scripts/docker_run_bash_mac.sh drake-20180906 .
# http://127.0.0.1:7000/static/

IIWA_SDF_PATH = os.path.join(pydrake.getDrakePath(),
    "manipulation", "models", "iiwa_description", "sdf",
    #"iiwa14_no_collision_floating.sdf")
    "iiwa14_no_collision.sdf")

WSG50_SDF_PATH = os.path.join(pydrake.getDrakePath(),
    "manipulation", "models", "wsg_50_description", "sdf",
    "schunk_wsg_50.sdf")

TABLE_SDF_PATH = os.path.join(pydrake.getDrakePath(),
    "examples", "kuka_iiwa_arm", "models", "table",
    "extra_heavy_duty_table_surface_only_collision.sdf")

MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
SINK_PATH = os.path.join(MODELS_DIR, "sink.sdf")
STOVE_PATH = os.path.join(MODELS_DIR, "stove.sdf")
BROCCOLI_PATH = os.path.join(MODELS_DIR, "broccoli.sdf")

AABBs = {
    'table': BoundingBox(np.array([0.0, 0.0, 0.736]), np.array([0.7122, 0.762, 0.057]) / 2),
    'table2': BoundingBox(np.array([0.0, 0.0, 0.736]), np.array([0.7122, 0.762, 0.057]) / 2),
    'broccoli': BoundingBox(np.array([0.0, 0.0, 0.05]), np.array([0.025, 0.025, 0.05])),
    'sink': BoundingBox(np.array([0.0, 0.0, 0.025]), np.array([0.025, 0.025, 0.05]) / 2),
    'stove': BoundingBox(np.array([0.0, 0.0, 0.025]), np.array([0.025, 0.025, 0.05]) / 2),
}


def get_origin_aabb(model_index):
    raise NotImplementedError() # TODO: if AABBs name starts with type

# TODO: could compute bounding boxes using a rigid body tree

##################################################


def weld_gripper(mbp, robot_index, gripper_index):
    X_EeGripper = create_transform([0, 0, 0.081], [np.pi / 2, 0, np.pi / 2])
    robot_body = get_model_bodies(mbp, robot_index)[-1]
    gripper_body = get_model_bodies(mbp, gripper_index)[0]
    mbp.AddJoint(WeldJoint(name="weld_gripper_to_robot_ee",
                           parent_frame_P=robot_body.body_frame(),
                           child_frame_C=gripper_body.body_frame(),
                           X_PC=X_EeGripper))


##################################################


def load_meshcat():
    import meshcat
    vis = meshcat.Visualizer()
    #print(dir(vis)) # set_object
    return vis


def add_meshcat_visualizer(scene_graph, builder):
    from underactuated.meshcat_visualizer import MeshcatVisualizer
    viz = MeshcatVisualizer(scene_graph)
    builder.AddSystem(viz)
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    viz.get_input_port(0))
    viz.load()
    return viz


def add_drake_visualizer(scene_graph, lcm, builder):
    ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph, lcm=lcm)
    DispatchLoadMessage(scene_graph, lcm) # TODO: only update viewer after a plan is found


def add_logger(mbp, builder):
    state_log = builder.AddSystem(SignalLogger(mbp.get_continuous_state_output_port().size()))
    state_log._DeclarePeriodicPublish(0.02)
    builder.Connect(mbp.get_continuous_state_output_port(), state_log.get_input_port(0))
    return state_log


def connect_collisions(mbp, scene_graph, builder):
    # Connect scene_graph to MBP for collision detection.
    builder.Connect(
        mbp.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(mbp.get_source_id()))
    builder.Connect(
        scene_graph.get_query_output_port(),
        mbp.get_geometry_query_input_port())


def connect_controllers(builder, mbp, robot, gripper, print_period=1.0):
    iiwa_controller = KukaMultibodyController(plant=mbp,
                                              kuka_model_instance=robot,
                                              print_period=print_period)
    builder.AddSystem(iiwa_controller)
    builder.Connect(iiwa_controller.get_output_port(0),
                    mbp.get_input_port(0))
    builder.Connect(mbp.get_continuous_state_output_port(),
                    iiwa_controller.robot_state_input_port)

    hand_controller = HandController(plant=mbp,
                                     model_instance=gripper)
    builder.AddSystem(hand_controller)
    builder.Connect(hand_controller.get_output_port(0),
                    mbp.get_input_port(1))
    builder.Connect(mbp.get_continuous_state_output_port(),
                    hand_controller.robot_state_input_port)

    state_machine = ManipStateMachine(mbp)
    builder.AddSystem(state_machine)
    builder.Connect(mbp.get_continuous_state_output_port(),
                    state_machine.robot_state_input_port)
    builder.Connect(state_machine.kuka_plan_output_port,
                    iiwa_controller.plan_input_port)
    builder.Connect(state_machine.hand_setpoint_output_port,
                    hand_controller.setpoint_input_port)
    return state_machine


def build_diagram(mbp, scene_graph, lcm):
    builder = DiagramBuilder()
    builder.AddSystem(scene_graph)
    builder.AddSystem(mbp)
    connect_collisions(mbp, scene_graph, builder)
    #add_meshcat_visualizer(scene_graph, builder)
    add_drake_visualizer(scene_graph, lcm, builder)
    add_logger(mbp, builder)
    #return builder.Build()
    return builder


##################################################


WSG50_LEFT_FINGER = 'left_finger_sliding_joint'
WSG50_RIGHT_FINGER = 'right_finger_sliding_joint'

def get_close_wsg50_positions(mbp, model_index):
    left_joint = mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)
    right_joint = mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)
    return [left_joint.upper_limits()[0], right_joint.lower_limits()[0]]

def get_open_wsg50_positions(mbp, model_index):
    left_joint = mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)
    right_joint = mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)
    return [left_joint.lower_limits()[0], right_joint.upper_limits()[0]]

def close_wsg50_gripper(mbp, context, model_index): # 0.05
    set_max_joint_positions(context, [mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)])
    set_min_joint_positions(context, [mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)])

def open_wsg50_gripper(mbp, context, model_index):
    set_min_joint_positions(context, [mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)])
    set_max_joint_positions(context, [mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)])


##################################################

DEFAULT_WEIGHT = 1.0
DEFAULT_RESOLUTION = 0.005


def get_sample_fn(joints):
    return lambda: get_random_positions(joints)


def get_difference_fn(joints):

    def fn(q2, q1):
        assert len(joints) == len(q2)
        assert len(joints) == len(q1)
        # TODO: circular joints
        difference = []
        for joint, value2, value1 in zip(joints, q2, q1):
            #difference.append((value2 - value1) if is_circular(body, joint)
            #                  else circular_difference(value2, value1))
            difference.append(value2 - value1)
        return np.array(difference)
    return fn


def get_distance_fn(joints, weights=None):
    # TODO: custom weights and step sizes
    if weights is None:
        weights = DEFAULT_WEIGHT*np.ones(len(joints))
    difference_fn = get_difference_fn(joints)

    def fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))
    return fn


def get_refine_fn(joints, num_steps=0):
    difference_fn = get_difference_fn(joints)
    num_steps = num_steps + 1

    def fn(q1, q2):
        q = q1
        for i in range(num_steps):
            q = (1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q
            yield q
            # TODO: wrap these values
    return fn


def get_extend_fn(joints, resolutions=None):
    if resolutions is None:
        resolutions = DEFAULT_RESOLUTION*np.pi*np.ones(len(joints))
    assert len(joints) == len(resolutions)
    difference_fn = get_difference_fn(joints)

    def fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        refine_fn = get_refine_fn(joints, num_steps=int(np.max(steps)))
        return refine_fn(q1, q2)
    return fn


def within_limits(joint, position):
    lower, upper = get_joint_limits(joint)
    return lower <= position <= upper


def get_collision_fn(mbp, context, joints, collision_pairs=set()):
    # TODO: collision bodies or collision models?

    def fn(q):
        if any(not within_limits(joint, position) for joint, position in zip(joints, q)):
            return True
        if not collision_pairs:
            return False
        set_joint_positions(joints, context, q)
        return collision_pairs & get_colliding_models(mbp, context)
    return fn


def plan_straight_line_motion(mbp, context, joints, end_positions, resolutions=None, collision_pairs=set()):
    assert len(joints) == len(end_positions)
    start_positions = get_joint_positions(joints, context)
    return direct_path(start_positions, end_positions,
                       extend=get_extend_fn(joints, resolutions=resolutions),
                       collision=get_collision_fn(mbp, context, joints, collision_pairs=collision_pairs))


def plan_joint_motion(mbp, context, joints, end_positions,
                      weights=None, resolutions=None, collision_pairs=set(), **kwargs):
    assert len(joints) == len(end_positions)
    start_positions = get_joint_positions(joints, context)
    return birrt(start_positions, end_positions,
                 distance=get_distance_fn(joints, weights=weights),
                 sample=get_sample_fn(joints),
                 extend=get_extend_fn(joints, resolutions=resolutions),
                 collision=get_collision_fn(mbp, context, joints, collision_pairs=collision_pairs), **kwargs)

##################################################


def get_unit_vector(vec):
    norm = np.linalg.norm(vec)
    if norm == 0.:
        return vec
    return np.array(vec) / norm


def waypoints_from_path(joints, path):
    if len(path) < 2:
        return path
    difference_fn = get_difference_fn(joints)
    waypoints = [path[0]]
    last_conf = path[1]
    last_difference = get_unit_vector(difference_fn(last_conf, waypoints[-1]))
    for conf in path[2:]:
        difference = get_unit_vector(difference_fn(conf, waypoints[-1]))
        if not np.allclose(last_difference, difference, atol=1e-3, rtol=0):
            waypoints.append(last_conf)
            difference = get_unit_vector(difference_fn(conf, waypoints[-1]))
        last_conf = conf
        last_difference = difference
    waypoints.append(last_conf)
    return waypoints

##################################################


def get_top_cylinder_grasps(aabb, max_width=np.inf, grasp_length=0): # y is out of gripper
    tool = create_transform(translation=[0, 0, -0.07])
    center, extent = aabb
    w, l, h = 2*extent
    reflect_z = create_transform(rotation=[np.pi / 2, 0, 0])
    translate_z = create_transform(translation=[0, 0, -h / 2 + grasp_length])
    aabb_from_body = create_transform(translation=center).inverse()
    diameter = (w + l) / 2 # TODO: check that these are close
    if max_width < diameter:
        return
    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = create_transform(rotation=[0, 0, theta])
        yield reflect_z.multiply(rotate_z).multiply(translate_z).multiply(tool).multiply(aabb_from_body)

##################################################


class RelPose(object):
    def __init__(self, mbp, parent, child, transform): # TODO: operate on bodies
        self.mbp = mbp
        self.parent = parent
        self.child = child
        self.transform = transform
    def assign(self, context):
        parent_pose = get_relative_transform(self.mbp, context, self.parent)
        child_pose = parent_pose.multiply(self.transform)
        set_world_pose(self.mbp, context, self.child, child_pose)
    def __repr__(self):
        return '{}()'.format(self.__class__.__name__)


class Config(object):
    def __init__(self, joints, positions):
        assert len(joints) == len(positions)
        self.joints = joints
        self.positions = tuple(positions)
    def assign(self, context):
        for joint, position in zip(self.joints, self.positions):
            set_joint_position(joint, context, position) 
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, len(self.joints))


class Trajectory(object):
    def __init__(self, path, attachments=[]):
        self.path = tuple(path)
        self.attachments = attachments
    def iterate(self, context):
        for conf in self.path[1:]:
            conf.assign(context)
            for attach in self.attachments: # TODO: topological sort
                attach.assign(context)
            yield
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, len(self.path))

##################################################


def get_stable_gen(mbp, context, fixed=[]):
    world = mbp.world_frame

    def gen(obj_name, surface_name):
        object_aabb = AABBs[obj_name]
        obj = mbp.GetModelInstanceByName(obj_name)
        surface_aabb = AABBs[surface_name]
        surface = mbp.GetModelInstanceByName(surface_name)
        surface_pose = get_world_pose(mbp, context, surface)
        for local_pose in sample_aabb_placement(object_aabb, surface_aabb):
            world_pose = surface_pose.multiply(local_pose)
            set_world_pose(mbp, context, obj, world_pose)
            if not is_model_colliding(mbp, context, obj, obstacles=(fixed + [surface])):
                pose = RelPose(mbp, world, obj, world_pose)
                yield pose,
    return gen


def get_grasp_gen(mbp, gripper):
    gripper_frame = get_base_body(mbp, gripper).body_frame()

    def gen(obj_name):
        obj = mbp.GetModelInstanceByName(obj_name)
        #obj_frame = get_base_body(mbp, obj).body_frame()
        aabb = AABBs[obj_name]
        for transform in get_top_cylinder_grasps(aabb):
            grasp = RelPose(mbp, gripper_frame, obj, transform)
            yield grasp,
    return gen


def get_ik_fn(mbp, context, robot, gripper, fixed=[], distance=0.1, step_size=0.01):
    direction = np.array([0, -1, 0])
    gripper_frame = get_base_body(mbp, gripper).body_frame()
    joints = get_movable_joints(mbp, robot)
    collision_pairs = set(product([robot, gripper], fixed))
    collision_fn = get_collision_fn(mbp, context, joints, collision_pairs=collision_pairs)

    def fn(obj_name, pose, grasp):
        grasp_pose = pose.transform.multiply(grasp.transform.inverse())
        solution = None
        path = []
        for t in list(np.arange(0, distance, step_size)) + [distance]:
            current_vector = t * direction / np.linalg.norm(direction)
            current_pose = grasp_pose.multiply(create_transform(translation=current_vector))
            solution = solve_inverse_kinematics(mbp, gripper_frame, current_pose, initial_guess=solution)
            if solution is None:
                return None
            positions = [solution[j.position_start()] for j in joints]
            # TODO: holding
            if collision_fn(positions):
                return None
            path.append(Config(joints, positions))
        traj = Trajectory(path)
        return path[-1], traj
    return fn


def get_free_motion_fn(mbp, context, robot, gripper, fixed=[]):
    joints = get_movable_joints(mbp, robot)
    collision_pairs = set(product([robot, gripper], fixed))

    def fn(q1, q2):
        set_joint_positions(joints, context, q1.positions)
        #path = plan_straight_line_motion(mbp, context, joints, q2.positions, collision_pairs=collision_pairs)
        path = plan_joint_motion(mbp, context, joints, q2.positions, collision_pairs=collision_pairs)
        if path is None:
            return None
        traj = Trajectory([Config(joints, q) for q in path])
        #traj = Trajectory([q1, q2])
        return traj,
    return fn


def get_holding_motion_fn(mbp, context, robot, gripper, fixed=[]):
    joints = get_movable_joints(mbp, robot)
    collision_pairs = set(product([robot, gripper], fixed))

    def fn(q1, q2, o, g): # TODO: holding
        set_joint_positions(joints, context, q1.positions)
        #path = plan_straight_line_motion(mbp, context, joints, q2.positions, collision_pairs=collision_pairs)
        path = plan_joint_motion(mbp, context, joints, q2.positions, collision_pairs=collision_pairs)
        if path is None:
            return None
        traj = Trajectory([Config(joints, q) for q in path], attachments=[g])
        #traj = Trajectory([q1, q2], attachments=[g])
        return traj,
    return fn


##################################################


def get_pddlstream_problem(mbp, context, robot, gripper, movable=[], surfaces=[], fixed=[]):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    #world = mbp.world_body()
    world = mbp.world_frame()
    robot_joints = prune_fixed_joints(get_model_joints(mbp, robot))
    conf = Config(robot_joints, get_configuration(mbp, context, robot))
    init = [
        ('CanMove',),
        ('Conf', conf),
        ('AtConf', conf),
        ('HandEmpty',)
    ]

    #print('Movable:', movable)
    #print('Surfaces:', fixed)
    for obj in movable:
        obj_name = get_model_name(mbp, obj)
        #obj_frame = get_base_body(mbp, obj).body_frame()
        obj_pose = RelPose(mbp, world, obj, get_world_pose(mbp, context, obj))
        init += [('Graspable', obj_name),
                 ('Pose', obj_name, obj_pose),
                 ('AtPose', obj_name, obj_pose)]
        for surface in surfaces:
            surface_name = get_model_name(mbp, surface)
            init += [('Stackable', obj_name, surface_name)]
            #if is_placement(body, surface):
            #    init += [('Supported', body, pose, surface)]

    for surface in surfaces:
        surface_name = get_model_name(mbp, surface)
        if 'sink' in surface_name:
            init += [('Sink', surface_name)]
        if 'stove' in surface_name:
            init += [('Stove', surface_name)]

    obj_name = get_model_name(mbp, movable[0])
    goal = ('and',
            ('AtConf', conf),
            #('Holding', obj_name),
            #('On', obj_name, fixed[1]),
            #('On', obj_name, fixed[2]),
            #('Cleaned', obj_name),
            ('Cooked', obj_name),
    )

    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(mbp, context, fixed=fixed)),
        'sample-grasp': from_gen_fn(get_grasp_gen(mbp, gripper)),
        'inverse-kinematics': from_fn(get_ik_fn(mbp, context, robot, gripper, fixed=fixed)),
        'plan-free-motion': from_fn(get_free_motion_fn(mbp, context, robot, gripper, fixed=fixed)),
        'plan-holding-motion': from_fn(get_holding_motion_fn(mbp, context, robot, gripper, fixed=fixed)),
        #'TrajCollision': get_movable_collision_test(),
    }
    #stream_map = 'debug'

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################


def postprocess_plan(mbp, gripper, plan):
    trajectories = []
    if plan is None:
        return trajectories

    gripper_joints = prune_fixed_joints(get_model_joints(mbp, gripper)) 
    gripper_extend_fn = get_extend_fn(gripper_joints)
    gripper_closed_conf = get_close_wsg50_positions(mbp, gripper)
    gripper_path = list(gripper_extend_fn(gripper_closed_conf, get_open_wsg50_positions(mbp, gripper)))
    gripper_path.insert(0, gripper_closed_conf)
    open_traj = Trajectory(Config(gripper_joints, q) for q in gripper_path)
    close_traj = Trajectory(reversed(open_traj.path))
    # TODO: ceiling & orientation constraints
    # TODO: sampler chooses configurations that are far apart

    for name, args in plan:
        if name in ['clean', 'cook']:
            continue
        if name == 'pick':
            o, p, g, q, t = args
            trajectories.extend([
                Trajectory(reversed(t.path)),
                close_traj,
                Trajectory(t.path, attachments=[g]),
            ])
        elif name == 'place':
            o, p, g, q, t = args
            trajectories.extend([
                Trajectory(reversed(t.path), attachments=[g]),
                open_traj,
                Trajectory(t.path),
            ])
        else:
            trajectories.append(args[-1])

    return trajectories

def step_trajectories(diagram, diagram_context, context, trajectories, time_step=0.01):
    diagram.Publish(diagram_context)
    user_input('Start?')
    for traj in trajectories:
        for _ in traj.iterate(context):
            diagram.Publish(diagram_context)
            if time_step is None:
                user_input('Continue?')
            else:
                time.sleep(0.01)
    user_input('Finish?')

def simulate_splines(diagram, diagram_context, sim_duration, real_time_rate=1.0):
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(real_time_rate)
    simulator.Initialize()

    diagram.Publish(diagram_context)
    user_input('Start?')
    simulator.StepTo(sim_duration)
    user_input('Finish?')

##################################################


def compute_duration(splines):
    sim_duration = 0.
    for spline in splines:
        sim_duration += spline.end_time() + 0.5
    sim_duration += 5.0
    return sim_duration


RADIANS_PER_SECOND = np.pi / 2

def convert_splines(mbp, robot, gripper, context, trajectories):
    # TODO: move to trajectory class
    print()
    splines, gripper_setpoints = [], []
    for i, traj in enumerate(trajectories):
        traj.path[-1].assign(context)
        joints = traj.path[0].joints
        if len(joints) == 2:
            q_knots_kuka = np.zeros((2, 7))
            q_knots_kuka[0] = get_configuration(mbp, context, robot) # Second is velocity
            splines.append(PiecewisePolynomial.ZeroOrderHold([0, 1], q_knots_kuka.T))
        elif len(joints) == 7:
            # TODO: adjust timing based on distance & velocities
            # TODO: adjust number of waypoints
            distance_fn = get_distance_fn(joints)
            #path = [traj.path[0].positions, traj.path[-1].positions]
            path = [q.positions for q in traj.path]
            path = waypoints_from_path(joints, path) # TODO: increase time for pick/place & hold
            q_knots_kuka = np.vstack(path).T
            distances = [0.] + [distance_fn(q1, q2) for q1, q2 in zip(path, path[1:])]
            t_knots = np.cumsum(distances) / RADIANS_PER_SECOND # TODO: this should be a max
            d, n = q_knots_kuka.shape
            print('{}) d={}, n={}, duration={:.3f}'.format(i, d, n, t_knots[-1]))
            splines.append(PiecewisePolynomial.Cubic(
                breaks=t_knots, 
                knots=q_knots_kuka,
                knot_dot_start=np.zeros(d), 
                knot_dot_end=np.zeros(d)))
        else:
            raise ValueError(joints)
        _, gripper_setpoint = get_configuration(mbp, context, gripper)
        gripper_setpoints.append(gripper_setpoint)
    return splines, gripper_setpoints


##################################################


def get_colliding_bodies(mbp, context, min_penetration=0.0):
    # TODO: set collision geometries pairs to check
    # TODO: check collisions with a buffer (e.g. ComputeSignedDistancePairwiseClosestPoints())
    body_from_geometry_id = {}
    for body in get_bodies(mbp):
        for geometry_id in mbp.GetCollisionGeometriesForBody(body):
            body_from_geometry_id[geometry_id.get_value()] = body
    colliding_bodies = set()
    for penetration in mbp.CalcPointPairPenetrations(context):
        if penetration.depth < min_penetration:
            continue
        body1 = body_from_geometry_id[penetration.id_A.get_value()]
        body2 = body_from_geometry_id[penetration.id_B.get_value()]
        colliding_bodies.update([(body1, body2), (body2, body1)])
    return colliding_bodies


def get_colliding_models(mbp, context, **kwargs):
    colliding_models = set()
    for body1, body2 in get_colliding_bodies(mbp, context, **kwargs):
        colliding_models.add((body1.model_instance(), body2.model_instance()))
    return colliding_models


def is_model_colliding(mbp, context, model, obstacles=None):
    if obstacles is None:
        obstacles = get_model_indices(mbp) # All models
    if not obstacles:
        return False
    for model1, model2 in get_colliding_models(mbp, context):
        if (model1 == model) and (model2 in obstacles):
            return True
    return False


##################################################


def main():
    # TODO: GeometryInstance, InternalGeometry, & GeometryContext to get the shape of objects
    # TODO: cost-sensitive planning to avoid large kuka moves

    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--simulate', action='store_true', help='Simulate')
    parser.add_argument('-c', '--cfree', action='store_true', help='Disables collisions')
    args = parser.parse_args()

    time_step = 0.0002 # TODO: context.get_continuous_state_vector() fails
    #time_step = 0
    mbp = MultibodyPlant(time_step=time_step)
    scene_graph = SceneGraph() # Geometry
    lcm = DrakeLcm()

    robot = AddModelFromSdfFile(file_name=IIWA_SDF_PATH, model_name='robot',
                                scene_graph=scene_graph, plant=mbp)
    gripper = AddModelFromSdfFile(file_name=WSG50_SDF_PATH, model_name='gripper',
                                  scene_graph=scene_graph, plant=mbp)
    table = AddModelFromSdfFile(file_name=TABLE_SDF_PATH, model_name='table',
                                scene_graph=scene_graph, plant=mbp)
    table2 = AddModelFromSdfFile(file_name=TABLE_SDF_PATH, model_name='table2',
                                 scene_graph=scene_graph, plant=mbp)
    sink = AddModelFromSdfFile(file_name=SINK_PATH, model_name='sink',
                               scene_graph=scene_graph, plant=mbp)
    stove = AddModelFromSdfFile(file_name=STOVE_PATH, model_name='stove',
                                scene_graph=scene_graph, plant=mbp)
    broccoli = AddModelFromSdfFile(file_name=BROCCOLI_PATH, model_name='broccoli',
                                   scene_graph=scene_graph, plant=mbp)

    table2_x = 0.75
    table_top_z = get_aabb_z_placement(AABBs['sink'], AABBs['table'])
    weld_gripper(mbp, robot, gripper)
    weld_to_world(mbp, robot, create_transform(translation=[0, 0, table_top_z]))
    weld_to_world(mbp, table, create_transform())
    weld_to_world(mbp, table2, create_transform(translation=[table2_x, 0, 0]))
    weld_to_world(mbp, sink, create_transform(translation=[table2_x, 0.25, table_top_z]))
    weld_to_world(mbp, stove, create_transform(translation=[table2_x, -0.25, table_top_z]))
    mbp.Finalize(scene_graph)

    #dump_plant(mbp)
    #dump_models(mbp)

    ##################################################

    builder = build_diagram(mbp, scene_graph, lcm)
    state_machine = connect_controllers(builder, mbp, robot, gripper)
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    context = diagram.GetMutableSubsystemContext(mbp, diagram_context)
    #context = mbp.CreateDefaultContext()

    set_world_pose(mbp, context, broccoli, create_transform(translation=[table2_x, 0, table_top_z]))
    open_wsg50_gripper(mbp, context, gripper)
    #close_wsg50_gripper(mbp, context, gripper)
    #set_configuration(mbp, context, gripper, [-0.05, 0.05])

    diagram.Publish(diagram_context)
    #initial_state = context.get_continuous_state_vector().get_value() # CopyToVector
    initial_state = mbp.tree().get_multibody_state_vector(context).copy()
    #print(context.get_continuous_state().get_vector().get_value())
    #print(context.get_discrete_state_vector())
    #print(context.get_abstract_state())
    #print(mbp.num_positions())
    #print(mbp.num_velocities())

    #set_world_pose(mbp, context, broccoli, create_transform(translation=[table2_x, 0, table_top_z-0.1]))
    #colliding_pairs = get_colliding_bodies(mbp, context, min_penetration=0.0)
    #for body1, body2 in colliding_pairs:
    #    print(body1.get_parent_tree(), body2.model_instance())
    #return

    ##################################################

    fixed = [] if args.cfree else [table, table2, sink, stove]
    problem = get_pddlstream_problem(mbp, context, robot, gripper,
                                     movable=[broccoli],
                                     surfaces=[sink, stove],
                                     fixed=fixed)
    solution = solve_focused(problem, planner='ff-astar', max_cost=INF)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return
    trajectories = postprocess_plan(mbp, gripper, plan)
    splines, gripper_setpoints = convert_splines(mbp, robot, gripper, context, trajectories)
    sim_duration = compute_duration(splines)
    print('Splines: {}\nDuration: {:.3f} seconds'.format(len(splines), sim_duration))

    ##################################################

    #context.get_mutable_continuous_state_vector().SetFromVector(initial_state)
    mbp.tree().get_mutable_multibody_state_vector(context)[:] = initial_state
    #if not args.simulate:
    #    fix_input_ports(mbp, context)
    #sub_context = diagram.GetMutableSubsystemContext(mbp, diagram_context)
    #print(context == sub_context) # True

    if args.simulate:
        state_machine.Load(splines, gripper_setpoints)
        simulate_splines(diagram, diagram_context, sim_duration)
    else:
        step_trajectories(diagram, diagram_context, context, trajectories)


if __name__ == '__main__':
    main()
