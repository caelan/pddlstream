from __future__ import print_function

import numpy as np
import pydrake
import os
import sys

from collections import namedtuple

from pydrake.common import AddResourceSearchPath, FindResourceOrThrow
from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph, DispatchLoadMessage)
from pydrake.lcm import DrakeLcm # Required else "ConnectDrakeVisualizer(): incompatible function arguments."
from pydrake.multibody.multibody_tree import (ModelInstanceIndex, UniformGravityFieldElement, 
    WeldJoint, RevoluteJoint, PrismaticJoint, BodyIndex, JointIndex, FrameIndex)
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.util.eigen_geometry import Isometry3
from pydrake.math import RollPitchYaw
from pydrake.multibody import inverse_kinematics
from pydrake.solvers.mathematicalprogram import SolutionResult
from pydrake.trajectories import (
    PiecewisePolynomial
)
from pydrake.util.eigen_geometry import Isometry3
from pydrake.math import RollPitchYaw, RotationMatrix
from pydrake.systems.primitives import SignalLogger

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_fn, empty_gen
from pddlstream.language.synthesizer import StreamSynthesizer
from pddlstream.utils import print_solution, read, INF, get_file_path, find_unique, Verbose

# ~/Programs/Classes/6811/drake_docker_utility_scripts/docker_run_bash_mac.sh drake-20180906 ~/Programs/LIS/git/ss-drake/
# ~/Programs/Classes/6811/drake_docker_utility_scripts/docker_run_bash_mac.sh drake-20180906 .
# http://127.0.0.1:7000/static/

# http://drake.mit.edu/from_binary.html#binary-installation

iiwa_sdf_path = os.path.join(pydrake.getDrakePath(),
    "manipulation", "models", "iiwa_description", "sdf",
    #"iiwa14_no_collision_floating.sdf")
    "iiwa14_no_collision.sdf")

wsg50_sdf_path = os.path.join(pydrake.getDrakePath(),
    "manipulation", "models", "wsg_50_description", "sdf",
    "schunk_wsg_50.sdf")

table_sdf_path = os.path.join(pydrake.getDrakePath(),
    "examples", "kuka_iiwa_arm", "models", "table",
    "extra_heavy_duty_table_surface_only_collision.sdf")

apple_sdf_path = os.path.join(pydrake.getDrakePath(),
    "examples", "kuka_iiwa_arm", "models", "objects",
    "apple.sdf")

models_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")

bleach_path = os.path.join(models_path, "bleach_bottle.sdf")
sink_path = os.path.join(models_path, "sink.sdf")
stove_path = os.path.join(models_path, "stove.sdf")
broccoli_path = os.path.join(models_path, "broccoli.sdf")

BoundingBox = namedtuple('BoundingBox', ['center', 'extent'])

AABBs = {
    'table2': BoundingBox(np.array([0.0, 0.0, 0.736]), np.array([0.7122, 0.762, 0.057])),
    'broccoli': BoundingBox(np.array([0.0, 0.0, 0.05]), np.array([0.025, 0.025, 0.05])),
    'sink': BoundingBox(np.array([0.0, 0.0, 0.05]), np.array([0.025, 0.025, 0.05]) / 2),
    'stove': BoundingBox(np.array([0.0, 0.0, 0.05]), np.array([0.025, 0.025, 0.05]) / 2),
}

# TODO: could compute bounding boxes using a rigid body tree

IIWA_JOINT_NAMES = ["iiwa_joint_%d"%(i+1) for i in range(7)]
IIWA_JOINT_LIMITS = zip(len(IIWA_JOINT_NAMES)*[-2], len(IIWA_JOINT_NAMES)*[2])

user_input = raw_input

BASE_LINK = "iiwa_link_0"


##################################################

def get_aabb_top_grasps(aabb, under=False, max_width=np.inf, grasp_length=0.0):
    # TODO: combine with get_top_grasps
    center, extent = aabb
    w, l, h = 2*np.array(extent)
    reflect_z = Pose(euler=Euler(pitch=np.pi))
    translate = Pose(point=Point(z=(h / 2 - grasp_length)) + center)
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=Euler(yaw=(np.pi / 2 + i * np.pi)))
            grasps += [multiply_poses(translate, rotate_z, reflect_z)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=Euler(yaw=(i*np.pi)))
            grasps += [multiply_poses(translate, rotate_z, reflect_z)]
    return grasps

def get_aabb_lower(aabb):
    return np.array(aabb.center) - np.array(aabb.extent)

def get_aabb_upper(aabb):
    return np.array(aabb.center) + np.array(aabb.extent)

def get_aabb_z_placement(object_aabb, surface_aabb, z_epsilon=1e-3):
    z = (get_aabb_upper(surface_aabb) + object_aabb.extent - object_aabb.center)[2]
    return z + z_epsilon

def sample_aabb_placement(object_aabb, surface_aabb, **kwargs):
    z = get_aabb_z_placement(object_aabb, surface_aabb, **kwargs)
    while True:
        yaw = np.random.uniform(-np.pi, np.pi)
        lower = get_aabb_lower(surface_aabb)[:2]
        upper = get_aabb_upper(surface_aabb)[:2]
        [x, y] = np.random.uniform(lower, upper) - object_aabb.center[:2]
        yield Pose(np.array([x, y, z]), np.array([0, 0, yaw]))

def matrix_from_euler(euler):
    roll, pitch, yaw = euler
    return RollPitchYaw(roll, pitch, yaw).ToRotationMatrix().matrix()

def Pose(translation=None, rotation=None):
    pose = Isometry3.Identity()
    if translation is not None:
        pose.set_translation(translation)
    if rotation is not None:
        pose.set_rotation(matrix_from_euler(rotation))
    return pose

##################################################

def weld_gripper(mbp, robot_index, gripper_index):
    X_EeGripper = Pose([0, 0, 0.081], [np.pi/2, 0, np.pi/2])
    robot_body = get_model_bodies(mbp, robot_index)[-1]
    gripper_body = get_model_bodies(mbp, gripper_index)[0]
    mbp.AddJoint(WeldJoint(name="weld_gripper_to_robot_ee",
                           parent_frame_P=robot_body.body_frame(),
                           child_frame_C=gripper_body.body_frame(),
                           X_PC=X_EeGripper))

def weld_to_world(mbp, model_index, world_pose):
    mbp.AddJoint(
        WeldJoint(name="weld_to_world",
        parent_frame_P=mbp.world_body().body_frame(),
        child_frame_C=get_base_body(mbp, model_index).body_frame(),
        X_PC=world_pose))

##################################################

def add_meshcat_visualizer(scene_graph, builder):
    from underactuated.meshcat_visualizer import MeshcatVisualizer
    viz = MeshcatVisualizer(scene_graph)
    builder.AddSystem(viz)
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    viz.get_input_port(0))
    viz.load()
    return viz

def add_visualizer(scene_graph, lcm, builder):
    ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph, lcm=lcm)
    DispatchLoadMessage(scene_graph, lcm)

def add_logger(mbp, builder):
    state_log = builder.AddSystem(SignalLogger(mbp.get_continuous_state_output_port().size()))
    state_log._DeclarePeriodicPublish(0.02)
    builder.Connect(mbp.get_continuous_state_output_port(), state_log.get_input_port(0))
    return state_log

def connect_collisions(mbp, scene_graph, builder):
    builder.Connect(
        mbp.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(mbp.get_source_id()))
    builder.Connect(
        scene_graph.get_query_output_port(),
        mbp.get_geometry_query_input_port())

def build_diagram(mbp, scene_graph, lcm):
    mbp.Finalize(scene_graph)
    # Drake diagram
    builder = DiagramBuilder()
    builder.AddSystem(scene_graph)
    builder.AddSystem(mbp)
    # Connect scene_graph to MBP for collision detection.
    connect_collisions(mbp, scene_graph, builder)

    # Add meshcat visualizer if not in test mode
    #add_meshcat_visualizer(scene_graph, builder)
    add_visualizer(scene_graph, lcm, builder)

    # Add logger
    #add_logger(mbp, builder)

    # Build diagram.
    diagram = builder.Build()

    return diagram

import random

table_top_z = 0.736 + 0.057 / 2

##################################################

def joints_from_names(mbp, joint_names):
    return [mbp.GetJointByName(joint_name) for joint_name in joint_names]

def get_joint_angles(mbp, context, joint_names):
    return [joint.get_angle(context) for joint in joints_from_names(mbp, joint_names)]

def set_joint_angles(mbp, context, joint_names, joint_angles):
    assert len(joint_names) == len(joint_angles)
    return [joint.set_angle(context, angle)
            for joint, angle in zip(joints_from_names(mbp, joint_names), joint_angles)]

def get_joint_limits(joint):
    assert joint.num_positions() == 1
    [lower] = joint.lower_limits()
    [upper] = joint.upper_limits()
    return lower, upper

def randomize_positions(mbp, context):
    print(get_joint_angles(mbp, context, IIWA_JOINT_NAMES))
    for joint_name, joint_range in zip(IIWA_JOINT_NAMES, IIWA_JOINT_LIMITS):
        iiwa_joint = mbp.GetJointByName(joint_name)
        joint_angle = random.uniform(*joint_range)
        print(get_joint_limits(iiwa_joint))
        iiwa_joint.set_angle(context=context, angle=joint_angle)

##################################################

def get_joint_position(joint, context):
    if isinstance(joint, PrismaticJoint):
        return joint.get_translation(context)
    elif isinstance(joint, RevoluteJoint):
        return joint.get_angle(context)
    elif isinstance(joint, WeldJoint):
        raise RuntimeError(joint)
    else:
        raise NotImplementedError(joint)

def set_joint_position(joint, context, position):
    if isinstance(joint, PrismaticJoint):
        joint.set_translation(context, position)
    elif isinstance(joint, RevoluteJoint):
        joint.set_angle(context, position)
    elif isinstance(joint, WeldJoint):
        raise RuntimeError(joint)
    else:
        raise NotImplementedError(joint)

def get_configuration(mbp, context, model_index):
    joints = prune_fixed_joints(get_model_joints(mbp, model_index))
    return [get_joint_position(joint, context) for joint in joints]

def set_configuration(mbp, context, model_index, config):
    joints = prune_fixed_joints(get_model_joints(mbp, model_index))
    assert len(joints) == len(config)
    return [set_joint_position(joint, context, position) for joint, position in zip(joints, config)]

def set_min_joint_positions(mbp, context, joints):
    for joint in prune_fixed_joints(joints):
        [position] = joint.lower_limits()
        set_joint_position(joint, context, position)

def set_max_joint_positions(mbp, context, joints):
    for joint in prune_fixed_joints(joints):
        [position] = joint.upper_limits()
        set_joint_position(joint, context, position)

def get_relative_transform(mbp, context, frame2, frame1=None): # frame1 -> frame2
    if frame1 is None:
        frame1 = mbp.world_frame()
    return mbp.tree().CalcRelativeTransform(context, frame1, frame2)

def get_body_pose(mbp, context, body):
    return mbp.tree().EvalBodyPoseInWorld(context, body)

def get_world_pose(mbp, context, model_index):
    # CalcAllBodyPosesInWorld
    body = get_base_body(mbp, model_index)
    return mbp.tree().EvalBodyPoseInWorld(context, body)

def set_world_pose(mbp, context, model_index, world_pose):
    body = get_base_body(mbp, model_index)
    mbp.tree().SetFreeBodyPoseOrThrow(body, world_pose, context)

def context_stuff(diagram, mbp):
    # Fix multibodyplant actuation input port to 0.
    diagram_context = diagram.CreateDefaultContext()
    mbp_context = diagram.GetMutableSubsystemContext(
        mbp, diagram_context)

    for i in range(mbp.get_num_input_ports()):
        model_index = mbp.get_input_port(i)
        mbp_context.FixInputPort(model_index.get_index(), np.zeros(model_index.size()))

    # set initial pose for the apple.
    #X_WApple = Isometry3.Identity()
    #X_WApple.set_translation(apple_initial_position_in_world_frame)
    #mbt = mbp.tree()
    #mbt.SetFreeBodyPoseOrThrow(
    #    mbp.GetBodyByName("base_link_apple", apple_model), X_WApple, mbp_context)

    #X_WApple = Isometry3.Identity()
    #X_WApple.set_translation([0, 0, table_top_z])
    #mbp.tree().SetFreeBodyPoseOrThrow(
    #    mbp.GetBodyByName("iiwa_link_0"), X_WApple, mbp_context)
    return diagram_context

##################################################

# https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_tree.html
# wget -q https://registry.hub.docker.com/v1/repositories/mit6881/drake-course/tags -O -  | sed -e 's/[][]//g' -e 's/"//g' -e 's/ //g' | tr '}' '\n'  | awk -F: '{print $3}'
# https://stackoverflow.com/questions/28320134/how-to-list-all-tags-for-a-docker-image-on-a-remote-registry
# docker rmi $(docker images -q mit6881/drake-course)

#def get_joint_limits(joint):
#    joint = mbp.GetJointByName(joint_name)
#    joint.lower_limits()
#    joint.upper_limits()

#def GetEndEffectorWorldAlignedFrame():
#    X_EEa = Isometry3.Identity()
#    X_EEa.set_rotation(np.array([[0., 1., 0,],
#                                 [0, 0, 1],
#                                 [1, 0, 0]]))
#    return X_EEa

# https://github.com/RobotLocomotion/drake/blob/b30e4982bfa04e5de69e8641e7c6580b4c1eb42c/bindings/pydrake/solvers/mathematicalprogram_py.cc#L494


def solve_inverse_kinematics(mbp, target_frame, target_pose, 
        max_position_error=0.001, theta_bound=0.01*np.pi, initial_guess=None):
    if initial_guess is None:
        initial_guess = np.zeros(mbp.num_positions())
        for joint in prune_fixed_joints(get_joints(mbp)):
            initial_guess[joint.position_start()] = random.uniform(*get_joint_limits(joint))

    ik_scene = inverse_kinematics.InverseKinematics(mbp)
    world_frame = mbp.world_frame()

    ik_scene.AddOrientationConstraint(
        frameAbar=target_frame, R_AbarA=RotationMatrix.Identity(),
        frameBbar=world_frame, R_BbarB=RotationMatrix(target_pose.rotation()),
        theta_bound=theta_bound)

    lower = target_pose.translation() - max_position_error
    upper = target_pose.translation() + max_position_error
    ik_scene.AddPositionConstraint(
        frameB=target_frame, p_BQ=np.zeros(3),
        frameA=world_frame, p_AQ_lower=lower, p_AQ_upper=upper)

    prog = ik_scene.prog()
    prog.SetInitialGuess(ik_scene.q(), initial_guess)
    result = prog.Solve()
    if result != SolutionResult.kSolutionFound:
        return None
    return prog.GetSolution(ik_scene.q())

##################################################

def get_model_name(mbp, model_index):
    return mbp.tree().GetModelInstanceName(model_index)

def get_model_indices(mbp):
    return [ModelInstanceIndex(i) for i in range(mbp.num_model_instances())]

def get_model_names(mbp):
    return [get_model_name(mbp, index) for index in get_model_indices(mbp)]

def get_bodies(mbp):
    return [mbp.tree().get_body(BodyIndex(i)) for i in range(mbp.num_bodies())]

def get_joints(mbp):
    return [mbp.tree().get_joint(JointIndex(i)) for i in range(mbp.num_joints())]

def get_frames(mbp):
    return [mbp.tree().get_frame(FrameIndex(i)) for i in range(mbp.tree().num_frames())]

def get_model_bodies(mbp, model_index):
    body_names = []
    for body in get_bodies(mbp):
        if body.name() not in body_names:
            body_names.append(body.name())
    return [mbp.GetBodyByName(name, model_index) 
            for name in body_names if mbp.HasBodyNamed(name, model_index)]

def get_base_body(mbp, model_index):
    # TODO: make this less of a hack
    return get_model_bodies(mbp, model_index)[0]

def get_model_joints(mbp, model_index):
    joint_names = []
    for joint in get_joints(mbp):
        if joint.name() not in joint_names:
            joint_names.append(joint.name())
    return [mbp.GetJointByName(name, model_index) 
            for name in joint_names if mbp.HasJointNamed(name, model_index)]

#def get_joint_indices(mbp, model_index=None):
#    if model_index is None:
#        joint_range = range(mbp.num_joints())
#    else:
#        num_joints_models = [mbp.num_joints(index) for index in get_model_indices(mbp)]
#        int(model_index)
#        print()
#    return [JointIndex(i) for i in joint_range]

def dump_plant(mbp):
    print('\nModels:')
    for i, name in enumerate(get_model_names(mbp)):
        print("{}) {}".format(i, name))
    print('\nBodies:')
    for i, body in enumerate(get_bodies(mbp)):
        print("{}) {} {}: {}".format(i, body.__class__.__name__, body.name(), body.body_frame().name()))
    print('\nJoints:')
    for i, joint in enumerate(get_joints(mbp)):
        print("{}) {} {}: {}, {}".format(i, body.__class__.__name__, joint.name(), 
            joint.lower_limits(), joint.upper_limits()))
    print('\nFrames:')
    for i, frame in enumerate(get_frames(mbp)):
        print("{}) {}: {}".format(i, frame.name(), frame.body().name()))

def dump_model(mbp, model_index):
    print('\nModel {}: {}'.format(int(model_index), mbp.tree().GetModelInstanceName(model_index)))
    bodies = get_model_bodies(mbp, model_index)
    if bodies:
        print('Bodies:')
        for i, body in enumerate(bodies):
            print("{}) {} {}: {}".format(i, body.__class__.__name__, body.name(), body.body_frame().name()))
    joints = get_model_joints(mbp, model_index)
    if joints:
        print('Joints:')
        for i, joint in enumerate(joints):
            print("{}) {} {}: {}, {}".format(i, joint.__class__.__name__, joint.name(), 
                joint.lower_limits(), joint.upper_limits()))
    #print('\nFrames:')
    #for i, frame in enumerate(get_frames(mbp)):
    #    print("{}) {}: {}".format(i, frame.name(), frame.body().name()))

def dump_models(mbp):
    for model_index in get_model_indices(mbp):
        dump_model(mbp, model_index)

def is_fixed_joints(joint):
    return joint.num_positions() == 0

def prune_fixed_joints(joints):
    return list(filter(lambda j: not is_fixed_joints(j), joints))

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
    set_max_joint_positions(mbp, context, [mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)])
    set_min_joint_positions(mbp, context, [mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)])

def open_wsg50_gripper(mbp, context, model_index):
    set_min_joint_positions(mbp, context, [mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)])
    set_max_joint_positions(mbp, context, [mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)])

##################################################

def get_rest_positions(joints):
    return np.zeros(len(joints))

def get_random_positions(joints):
    return np.array([np.random.uniform(*get_joint_limits(joint)) for joint in joints])

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
        resolutions = 0.01*np.pi*np.ones(len(joints))
    assert len(joints) == len(resolutions)
    difference_fn = get_difference_fn(joints)
    def fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        refine_fn = get_refine_fn(joints, num_steps=int(np.max(steps)))
        return refine_fn(q1, q2)
    return fn

##################################################

def get_top_cylinder_grasps(aabb, max_width=np.inf, grasp_length=0): # y is out of gripper
    tool = Pose(translation=[0, 0, -0.1])
    center, extent = aabb
    w, l, h = 2*extent
    reflect_z = Pose(rotation=[np.pi/2, 0, 0])
    translate_z = Pose(translation=[0, 0, -h / 2 + grasp_length])
    aabb_from_body = Pose(translation=center).inverse()
    diameter = (w + l) / 2 # TODO: check that these are close
    if max_width < diameter:
        return
    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = Pose(rotation=[0, 0, theta])
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

def get_stable_gen(mbp, context):
    world = mbp.world_frame
    def gen(obj_name, surface_name):
        object_aabb = AABBs[obj_name]
        obj = mbp.GetModelInstanceByName(surface_name)
        surface_aabb = AABBs[surface_name]
        surface = mbp.GetModelInstanceByName(surface_name)
        surface_pose = get_world_pose(mbp, context, surface)
        for local_pose in sample_aabb_placement(object_aabb, surface_aabb):
            world_pose = surface_pose.multiply(local_pose)
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

def get_ik_fn(mbp, robot, gripper, distance=0.1, step_size=0.01):
    direction = np.array([0, -1, 0])
    gripper_frame = get_base_body(mbp, gripper).body_frame()
    joints = prune_fixed_joints(get_model_joints(mbp, robot))
    def fn(obj_name, pose, grasp):
        grasp_pose = pose.transform.multiply(grasp.transform.inverse())
        solution = None
        path = []
        for t in list(np.arange(0, distance, step_size)) + [distance]:
            current_vector = t * direction / np.linalg.norm(direction)
            current_pose = grasp_pose.multiply(Pose(translation=current_vector))
            solution = solve_inverse_kinematics(mbp, gripper_frame, current_pose, initial_guess=solution)
            if solution is None:
                return None
            path.append(Config(joints, [solution[j.position_start()] for j in joints]))
        traj = Trajectory(path)
        return path[-1], traj
    return fn

def get_free_motion_fn(mbp, robot):
    joints = prune_fixed_joints(get_model_joints(mbp, robot))
    extend_fn = get_extend_fn(joints)
    def fn(q1, q2):
        path = list(extend_fn(q1.positions, q2.positions))
        if path is None:
            return None
        traj = Trajectory([Config(joints, q) for q in path])
        #traj = Trajectory([q1, q2])
        return traj,
    return fn

def get_holding_motion_fn(mbp, robot):
    joints = prune_fixed_joints(get_model_joints(mbp, robot))
    extend_fn = get_extend_fn(joints)
    def fn(q1, q2, o, g):
        path = list(extend_fn(q1.positions, q2.positions))
        if path is None:
            return None
        traj = Trajectory([Config(joints, q) for q in path], attachments=[g])
        #traj = Trajectory([q1, q2], attachments=[g])
        return traj,
    return fn

##################################################

def pddlstream_stuff(mbp, context, robot, gripper, movable, surfaces):
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
        'sample-pose': from_gen_fn(get_stable_gen(mbp, context)),
        'sample-grasp': from_gen_fn(get_grasp_gen(mbp, gripper)),
        'inverse-kinematics': from_fn(get_ik_fn(mbp, robot, gripper)),
        'plan-free-motion': from_fn(get_free_motion_fn(mbp, robot)),
        'plan-holding-motion': from_fn(get_holding_motion_fn(mbp, robot)),
        #'TrajCollision': get_movable_collision_test(),
    }
    #stream_map = 'debug'

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

def test_stuff(mbp, robot, gripper):
    gripper_frame = mbp.GetFrameByName("body", gripper)
    print(gripper_frame.name())
    gripper_frame = get_base_body(mbp, gripper).body_frame()
    print(gripper_frame.name())

    #gripper_pose = get_body_pose(mbp, context, mbp.GetBodyByName('body', gripper))
    gripper_pose = get_world_pose(mbp, context, gripper)
    print(gripper_pose)
    print(get_relative_transform(mbp, context, gripper_frame))

    aabb = BoundingBox(np.array([0, 0, 0.05]), np.array([0.025, 0.025, 0.05])) # wrt body frame
    grasp = next(get_top_cylinder_grasps(aabb))
    #set_world_pose(mbp, context, broccoli, gripper_pose.multiply(grasp))

    diagram.Publish(diagram_context)
    user_input('Continue?')

    target_pose = get_world_pose(mbp, context, broccoli).multiply(grasp.inverse())
    print(target_pose)

    conf = solve_inverse_kinematics(mbp, gripper_frame, target_pose)
    if conf is None:
        return
    iiwa_conf = mbp.tree().get_positions_from_array(iiwa, conf)
    set_joint_angles(mbp, context, IIWA_JOINT_NAMES, conf)
    print(get_world_pose(mbp, context, gripper))
    print(get_relative_transform(mbp, context, gripper_frame))

    diagram.Publish(diagram_context)
    user_input('Next?')

    for i in range(10):
        randomize_positions(mbp, context)
        diagram.Publish(diagram_context)
        user_input('Continue?')
    user_input('Finish?')

def main():
    #import meshcat
    #vis = meshcat.Visualizer()
    #print(dir(vis)) # set_object

    mbp = MultibodyPlant(time_step=0)
    scene_graph = SceneGraph() # Geometry
    lcm = DrakeLcm()

    robot = AddModelFromSdfFile(file_name=iiwa_sdf_path, model_name='robot',
                               scene_graph=scene_graph, plant=mbp)
    gripper = AddModelFromSdfFile(file_name=wsg50_sdf_path, model_name='gripper',
                                  scene_graph=scene_graph, plant=mbp)
    table = AddModelFromSdfFile(file_name=table_sdf_path, model_name='table',
                                scene_graph=scene_graph, plant=mbp)
    table2 = AddModelFromSdfFile(file_name=table_sdf_path, model_name='table2',
                                 scene_graph=scene_graph, plant=mbp)
    #apple = AddModelFromSdfFile(file_name=apple_sdf_path, model_name='apple',
    #                                  scene_graph=scene_graph, plant=mbp)
    #bleach = AddModelFromSdfFile(file_name=bleach_path, model_name='bleach',
    #                             scene_graph=scene_graph, plant=mbp)
    sink = AddModelFromSdfFile(file_name=sink_path, model_name='sink',
                                scene_graph=scene_graph, plant=mbp)
    stove = AddModelFromSdfFile(file_name=stove_path, model_name='stove',
                                scene_graph=scene_graph, plant=mbp)
    broccoli = AddModelFromSdfFile(file_name=broccoli_path, model_name='broccoli',
                                scene_graph=scene_graph, plant=mbp)

    weld_gripper(mbp, robot, gripper)
    weld_to_world(mbp, robot, Pose(translation=[0, 0, table_top_z]))
    weld_to_world(mbp, table, Pose())
    weld_to_world(mbp, table2, Pose(translation=[0.75, 0, 0]))
    diagram = build_diagram(mbp, scene_graph, lcm)
    
    #dump_plant(mbp)
    dump_models(mbp)

    diagram_context = context_stuff(diagram, mbp)
    context = diagram.GetMutableSubsystemContext(mbp, diagram_context)
    table2_x = 0.75
    set_world_pose(mbp, context, sink, Pose(translation=[table2_x, 0.25, table_top_z]))
    set_world_pose(mbp, context, stove, Pose(translation=[table2_x, -0.25, table_top_z]))
    set_world_pose(mbp, context, broccoli, Pose(translation=[table2_x, 0, table_top_z]))
    open_wsg50_gripper(mbp, context, gripper)
    #close_wsg50_gripper(mbp, context, gripper)
    #set_configuration(mbp, context, gripper, [-0.05, 0.05])
    diagram.Publish(diagram_context)


    problem = pddlstream_stuff(mbp, context, robot, gripper, movable=[broccoli], surfaces=[sink, stove])
    solution = solve_focused(problem, planner='ff-astar', max_cost=INF)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return

    gripper_joints = prune_fixed_joints(get_model_joints(mbp, gripper))
    closed_conf = Config(gripper_joints, get_close_wsg50_positions(mbp, gripper))
    open_conf = Config(gripper_joints, get_open_wsg50_positions(mbp, gripper))

    # TODO: hold trajectories
    diagram.Publish(diagram_context)
    user_input('Start?')
    for name, args in plan:
        if name in ['clean', 'cook']:
            continue
        if name == 'pick':
            o, p, g, q, t = args
            trajectories = [
                Trajectory(reversed(t.path)),
                Trajectory([open_conf, closed_conf]),
                Trajectory(t.path, attachments=[g]),
            ]
        elif name == 'place':
            o, p, g, q, t = args
            trajectories = [
                Trajectory(reversed(t.path), attachments=[g]),
                Trajectory([closed_conf, open_conf]),
                Trajectory(t.path),
            ]
        else:
            trajectories = [args[-1]]
        for traj in trajectories:
            for _ in traj.iterate(context):
                diagram.Publish(diagram_context)
                user_input('Continue?')


# https://github.com/RobotLocomotion/drake/blob/a54513f9d0e746a810da15b5b63b097b917845f0/bindings/pydrake/multibody/test/multibody_tree_test.py

if __name__ == '__main__':
    main()
