from __future__ import print_function

import random
import numpy as np
from collections import namedtuple
from itertools import product

from drake import lcmt_viewer_load_robot
from pydrake.all import (Quaternion, RigidTransform, RotationMatrix)
from pydrake.geometry import DispatchLoadMessage
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RollPitchYaw
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.multibody_tree import (ModelInstanceIndex, WeldJoint, RevoluteJoint, PrismaticJoint, BodyIndex,
                                              JointIndex, JointActuatorIndex, FrameIndex)
from pydrake.solvers.mathematicalprogram import SolutionResult
from pydrake.util.eigen_geometry import Isometry3

user_input = raw_input

BoundingBox = namedtuple('BoundingBox', ['center', 'extent'])


def get_aabb_lower(aabb):
    return np.array(aabb.center) - np.array(aabb.extent)


def get_aabb_upper(aabb):
    return np.array(aabb.center) + np.array(aabb.extent)


def vertices_from_aabb(aabb):
    center, extent = aabb
    return [center + np.multiply(extent, np.array(signs))
            for signs in product([-1, 1], repeat=len(extent))]


def aabb_from_points(points):
    lower = np.min(points, axis=0)
    upper = np.max(points, axis=0)
    center = (np.array(lower) + np.array(upper)) / 2.
    extent = (np.array(upper) - np.array(lower)) / 2.
    return BoundingBox(center, extent)


def aabb_contains_point(point, aabb):
    lower = get_aabb_lower(aabb)
    upper = get_aabb_upper(aabb)
    return np.greater_equal(point, lower).all() and \
           np.greater_equal(upper, point).all()


def get_model_aabb(mbp, context, box_from_geom, model_index):
    # TODO: world frame or wrt base?
    points = []
    body_names = {body.name() for body in get_model_bodies(mbp, model_index)}
    for (model_int, body_name, _), (aabb, body_from_geom, _) in box_from_geom.items():
        if (int(model_index) == model_int) and (body_name in body_names):
            body = mbp.GetBodyByName(body_name, model_index)
            world_from_body = get_body_pose(context, body)
            points.extend(world_from_body.multiply(body_from_geom).multiply(vertex)
                          for vertex in vertices_from_aabb(aabb))
    return aabb_from_points(points)

##################################################

def get_aabb_z_placement(object_aabb, surface_aabb, z_epsilon=1e-3):
    z = (get_aabb_upper(surface_aabb) + object_aabb.extent - object_aabb.center)[2]
    return z + z_epsilon


def sample_aabb_placement(object_aabb, surface_aabb, shrink=0.01, **kwargs):
    z = get_aabb_z_placement(object_aabb, surface_aabb, **kwargs)
    while True:
        yaw = np.random.uniform(-np.pi, np.pi)
        lower = get_aabb_lower(surface_aabb)[:2] + shrink*np.ones(2)
        upper = get_aabb_upper(surface_aabb)[:2] - shrink*np.ones(2)
        if np.greater(lower, upper).any():
            break
        [x, y] = np.random.uniform(lower, upper) - object_aabb.center[:2]
        yield create_transform(np.array([x, y, z]), np.array([0, 0, yaw]))

##################################################


def get_unit_vector(vec):
    norm = np.linalg.norm(vec)
    if norm == 0.:
        return vec
    return np.array(vec) / norm


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
    return str(mbp.tree().GetModelInstanceName(model_index))


def get_model_indices(mbp):
    return [ModelInstanceIndex(i) for i in range(mbp.num_model_instances())]


def get_model_names(mbp):
    return [get_model_name(mbp, index) for index in get_model_indices(mbp)]


def get_bodies(mbp):
    return [mbp.tree().get_body(BodyIndex(i)) for i in range(mbp.num_bodies())]


def get_joints(mbp):
    return [mbp.tree().get_joint(JointIndex(i)) for i in range(mbp.num_joints())]


def get_joint_actuators(mbp):
    return [mbp.tree().get_joint_actuator(JointActuatorIndex(i)) for i in range(mbp.num_actuators())]


def get_frames(mbp):
    return [mbp.tree().get_frame(FrameIndex(i)) for i in range(mbp.tree().num_frames())]


def get_model_bodies(mbp, model_index):
    return [body for body in get_bodies(mbp) if body.model_instance() == model_index]


def get_base_body(mbp, model_index):
    # TODO: make this less of a hack
    return get_model_bodies(mbp, model_index)[0]


def get_model_joints(mbp, model_index):
    return [joint for joint in get_joints(mbp) if joint.model_instance() == model_index]


def get_model_actuators(mbp, model_index):
    return [actuator for actuator in get_joint_actuators(mbp) if actuator.model_instance() == model_index]


def is_fixed_joint(joint):
    return joint.num_positions() == 0


def prune_fixed_joints(joints):
    return list(filter(lambda j: not is_fixed_joint(j), joints))


def get_movable_joints(mbp, model_index):
    return prune_fixed_joints(get_model_joints(mbp, model_index))


def get_parent_joints(mbp, body):
    # Really should just be none or one
    return [joint for joint in get_movable_joints(mbp, body.model_instance())
            if joint.child_body() == body]


def bodies_from_models(plant, models):
    return {body for model in models for body in get_model_bodies(plant, model)}

##################################################


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


def get_joint_positions(joints, context):
    return [get_joint_position(joint, context) for joint in joints]


def set_joint_positions(joints, context, positions):
    assert len(joints) == len(positions)
    return [set_joint_position(joint, context, position) for joint, position in zip(joints, positions)]


def get_configuration(mbp, context, model_index):
    return get_joint_positions(get_movable_joints(mbp, model_index), context)


def set_configuration(mbp, context, model_index, config):
    return set_joint_positions(get_movable_joints(mbp, model_index), context, config)


def get_rest_positions(joints):
    # TODO: get min/max positions
    return np.zeros(len(joints))


def get_random_positions(joints):
    return np.array([np.random.uniform(*get_joint_limits(joint)) for joint in joints])


##################################################


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


##################################################


def get_state(mbp, context):
    #q = mbp.tree().get_multibody_state_vector(context)[:mbp.num_positions()]
    #print(mbp.tree().get_positions_from_array(task.movable[0], q))
    #return context.get_continuous_state_vector().get_value() # CopyToVector
    return mbp.tree().get_multibody_state_vector(context).copy()


def set_state(mbp, context, state):
    #context.get_mutable_continuous_state_vector().SetFromVector(initial_state)
    mbp.tree().get_mutable_multibody_state_vector(context)[:] = state


def get_positions(mbp, context):
    return get_state(mbp, context)[:mbp.num_positions()]

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


#def fix_input_ports(mbp, context):
#    for i in range(mbp.get_num_input_ports()):
#        model_index = mbp.get_input_port(i)
#        context.FixInputPort(model_index.get_index(), np.zeros(model_index.size()))


##################################################

def solve_inverse_kinematics(mbp, target_frame, target_pose,
        max_position_error=0.005, theta_bound=0.01*np.pi, initial_guess=None):
    if initial_guess is None:
        # TODO: provide initial guess for some joints (like iiwa joint0)
        initial_guess = np.zeros(mbp.num_positions())
        for joint in prune_fixed_joints(get_joints(mbp)):
            lower, upper = get_joint_limits(joint)
            if -np.inf < lower < upper < np.inf:
                initial_guess[joint.position_start()] = random.uniform(lower, upper)
    assert mbp.num_positions() == len(initial_guess)

    ik_scene = InverseKinematics(mbp)
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
    # AddAngleBetweenVectorsConstraint
    # AddGazeTargetConstraint

    prog = ik_scene.prog()
    prog.SetInitialGuess(ik_scene.q(), initial_guess)
    result = prog.Solve()
    if result != SolutionResult.kSolutionFound:
        return None
    return prog.GetSolution(ik_scene.q())

##################################################


def get_colliding_bodies(diagram, diagram_context, plant, scene_graph, min_penetration=0.0):
    # TODO: set collision geometries pairs to check
    # TODO: check collisions with a buffer (e.g. ComputeSignedDistancePairwiseClosestPoints())
    # WARNING: indices have equality defined but not a hash function
    sg_context = diagram.GetMutableSubsystemContext(scene_graph, diagram_context)
    query_object = scene_graph.get_query_output_port().Eval(sg_context)
    inspector = query_object.inspector()
    colliding_bodies = set()
    for penetration in query_object.ComputePointPairPenetration():
        if min_penetration <= penetration.depth:
            body1, body2 = [plant.GetBodyFromFrameId(inspector.GetFrameId(geometry_id))
                  for geometry_id in [penetration.id_A, penetration.id_B]]
            colliding_bodies.update([(body1, body2), (body2, body1)])
    return colliding_bodies


def exists_colliding_pair(diagram, diagram_context, plant, scene_graph, body_pairs, **kwargs):
    if not body_pairs:
        return False
    intersection = get_colliding_bodies(diagram, diagram_context, plant, scene_graph, **kwargs) & body_pairs
    #if intersection:
    #    print([(one.name(), two.name()) for one, two in intersection])
    #    diagram.Publish(diagram_context)
    #    raw_input('Continue?')
    return bool(intersection)

##################################################

def get_geom_name(geom):
    name_from_type = {
        geom.BOX: 'box',
        geom.CYLINDER: 'cylinder',
        geom.SPHERE: 'sphere',
        geom.MESH: 'mesh',
    }
    return name_from_type[geom.type]

def get_box_from_geom(scene_graph, visual_only=True):
    # TODO: GeometryInstance, InternalGeometry, & GeometryContext to get the shape of objects
    # TODO: get_contact_results_output_port
    # https://github.com/RussTedrake/underactuated/blob/master/src/underactuated/meshcat_visualizer.py
    # https://github.com/RobotLocomotion/drake/blob/master/lcmtypes/lcmt_viewer_draw.lcm
    mock_lcm = DrakeMockLcm()
    DispatchLoadMessage(scene_graph, mock_lcm)
    load_robot_msg = lcmt_viewer_load_robot.decode(
        mock_lcm.get_last_published_message("DRAKE_VIEWER_LOAD_ROBOT"))
    # 'link', 'num_links'
    #builder.Connect(scene_graph.get_pose_bundle_output_port(),
    #                viz.get_input_port(0))

    # TODO: hash bodies instead
    box_from_geom = {}
    for body_index in range(load_robot_msg.num_links):
        # 'geom', 'name', 'num_geom', 'robot_num'
        link = load_robot_msg.link[body_index]
        [source_name, frame_name] = link.name.split("::")
        # source_name = 'Source_1'
        model_index = link.robot_num

        visual_index = 0
        for geom in sorted(link.geom, key=lambda g: g.position[::-1]): # sort by z, y, x
            # 'color', 'float_data', 'num_float_data', 'position', 'quaternion', 'string_data', 'type'
            # string_data is empty...
            if visual_only and (geom.color[3] == 0):
                continue

            # TODO: sort by lowest point on the bounding box?
            # TODO: maybe just return the set of bodies in order and let the user decide what to with them
            visual_index += 1 # TODO: affected by transparent visual
            if geom.type == geom.BOX:
                assert geom.num_float_data == 3
                [width, length, height] = geom.float_data
                extent = np.array([width, length, height]) / 2.
            elif geom.type == geom.SPHERE:
                assert geom.num_float_data == 1
                [radius] = geom.float_data
                extent = np.array([radius, radius, radius])
            elif geom.type == geom.CYLINDER:
                assert geom.num_float_data == 2
                [radius, height] = geom.float_data
                extent = np.array([radius, radius, height/2.])
                #meshcat_geom = meshcat.geometry.Cylinder(
                #    geom.float_data[1], geom.float_data[0])
                # In Drake, cylinders are along +z
            #elif geom.type == geom.MESH:
            #    meshcat_geom = meshcat.geometry.ObjMeshGeometry.from_file(
            #            geom.string_data[0:-3] + "obj")
            else:
                #print("Robot {}, link {}, geometry {}: UNSUPPORTED GEOMETRY TYPE {} WAS IGNORED".format(
                #    model_index, frame_name, visual_index-1, geom.type))
                continue
            link_from_box = RigidTransform(
                RotationMatrix(Quaternion(geom.quaternion)), geom.position).GetAsIsometry3() #.GetAsMatrix4()
            box_from_geom[model_index, frame_name, visual_index-1] = \
                (BoundingBox(np.zeros(3), extent), link_from_box, get_geom_name(geom))
    return box_from_geom
