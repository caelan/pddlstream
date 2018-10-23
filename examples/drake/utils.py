from __future__ import print_function

import random
import numpy as np

from collections import namedtuple

from pydrake.multibody.multibody_tree import (ModelInstanceIndex, UniformGravityFieldElement,
    WeldJoint, RevoluteJoint, PrismaticJoint, BodyIndex, JointIndex, FrameIndex)
from pydrake.multibody import inverse_kinematics
from pydrake.solvers.mathematicalprogram import SolutionResult
from pydrake.math import RollPitchYaw, RotationMatrix
from pydrake.util.eigen_geometry import Isometry3

BoundingBox = namedtuple('BoundingBox', ['center', 'extent'])


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
        yield create_transform(np.array([x, y, z]), np.array([0, 0, yaw]))

##################################################

def matrix_from_euler(euler):
    roll, pitch, yaw = euler
    return RollPitchYaw(roll, pitch, yaw).ToRotationMatrix().matrix()


def create_transform(translation=None, rotation=None):
    pose = Isometry3.Identity()
    if translation is not None:
        pose.set_translation(translation)
    if rotation is not None:
        pose.set_rotation(matrix_from_euler(rotation))
    return pose

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
    return [body for body in get_bodies(mbp) if body.model_instance() == model_index]
    #body_names = []
    #for body in get_bodies(mbp):
    #    if body.name() not in body_names:
    #        body_names.append(body.name())
    #return [mbp.GetBodyByName(name, model_index)
    #        for name in body_names if mbp.HasBodyNamed(name, model_index)]


def get_base_body(mbp, model_index):
    # TODO: make this less of a hack
    return get_model_bodies(mbp, model_index)[0]


def get_model_joints(mbp, model_index):
    return [joint for joint in get_joints(mbp) if joint.model_instance() == model_index]
    #joint_names = []
    #for joint in get_joints(mbp):
    #    if joint.name() not in joint_names:
    #        joint_names.append(joint.name())
    #return [mbp.GetJointByName(name, model_index)
    #        for name in joint_names if mbp.HasJointNamed(name, model_index)]

def is_fixed_joints(joint):
    return joint.num_positions() == 0


def prune_fixed_joints(joints):
    return list(filter(lambda j: not is_fixed_joints(j), joints))


def get_movable_joints(mbp, model_index):
    return prune_fixed_joints(get_model_joints(mbp, model_index))


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


def get_body_pose(context, body):
    #mbt = mbp.tree()
    mbt = body.get_parent_tree()
    return mbt.EvalBodyPoseInWorld(context, body)


def get_world_pose(mbp, context, model_index):
    # CalcAllBodyPosesInWorld
    body = get_base_body(mbp, model_index)
    return mbp.tree().EvalBodyPoseInWorld(context, body)


def set_world_pose(mbp, context, model_index, world_pose):
    body = get_base_body(mbp, model_index)
    mbp.tree().SetFreeBodyPoseOrThrow(body, world_pose, context)


def get_rest_positions(joints):
    return np.zeros(len(joints))


def get_random_positions(joints):
    return np.array([np.random.uniform(*get_joint_limits(joint)) for joint in joints])

##################################################

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


def weld_to_world(mbp, model_index, world_pose):
    mbp.AddJoint(
        WeldJoint(name="weld_to_world",
                  parent_frame_P=mbp.world_body().body_frame(),
                  child_frame_C=get_base_body(mbp, model_index).body_frame(),
                  X_PC=world_pose))
    

def fix_input_ports(mbp, context):
    for i in range(mbp.get_num_input_ports()):
        model_index = mbp.get_input_port(i)
        context.FixInputPort(model_index.get_index(), np.zeros(model_index.size()))


##################################################

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
