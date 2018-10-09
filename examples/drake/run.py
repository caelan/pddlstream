from __future__ import print_function

user_input = raw_input

import os
import time
import pydrake
import numpy as np
import random

from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph, DispatchLoadMessage)
from pydrake.lcm import DrakeLcm # Required else "ConnectDrakeVisualizer(): incompatible function arguments."
from pydrake.multibody.multibody_tree import (WeldJoint, PrismaticJoint, JointIndex, FrameIndex)
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import SignalLogger

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.utils import print_solution, read, INF, get_file_path, find_unique

from examples.drake.utils import BoundingBox, Pose, get_model_bodies, get_model_joints, \
    get_joint_angles, get_joint_limits, set_min_joint_positions, set_max_joint_positions, get_relative_transform, \
    get_world_pose, set_world_pose, set_joint_position, sample_aabb_placement, \
    get_base_body, solve_inverse_kinematics, prune_fixed_joints, weld_to_world, get_configuration, get_model_name, \
    set_joint_angles, create_context, get_random_positions, get_aabb_z_placement, set_configuration

from kuka_multibody_controllers import (KukaMultibodyController,
                                        HandController,
                                        ManipStateMachine)
from pydrake.trajectories import PiecewisePolynomial
from pydrake.systems.analysis import Simulator

# https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_tree.html
# wget -q https://registry.hub.docker.com/v1/repositories/mit6881/drake-course/tags -O -  | sed -e 's/[][]//g' -e 's/"//g' -e 's/ //g' | tr '}' '\n'  | awk -F: '{print $3}'
# https://stackoverflow.com/questions/28320134/how-to-list-all-tags-for-a-docker-image-on-a-remote-registry
# docker rmi $(docker images -q mit6881/drake-course)

# https://github.com/RobotLocomotion/drake/blob/a54513f9d0e746a810da15b5b63b097b917845f0/bindings/pydrake/multibody/test/multibody_tree_test.py
# ~/Programs/Classes/6811/drake_docker_utility_scripts/docker_run_bash_mac.sh drake-20180906 .
# http://127.0.0.1:7000/static/

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

#apple_sdf_path = os.path.join(pydrake.getDrakePath(),
#    "examples", "kuka_iiwa_arm", "models", "objects",
#    "apple.sdf")

models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
bleach_path = os.path.join(models_dir, "bleach_bottle.sdf")
sink_path = os.path.join(models_dir, "sink.sdf")
stove_path = os.path.join(models_dir, "stove.sdf")
broccoli_path = os.path.join(models_dir, "broccoli.sdf")

AABBs = {
    'table': BoundingBox(np.array([0.0, 0.0, 0.736]), np.array([0.7122, 0.762, 0.057]) / 2),
    'table2': BoundingBox(np.array([0.0, 0.0, 0.736]), np.array([0.7122, 0.762, 0.057]) / 2),
    'broccoli': BoundingBox(np.array([0.0, 0.0, 0.05]), np.array([0.025, 0.025, 0.05])),
    'sink': BoundingBox(np.array([0.0, 0.0, 0.025]), np.array([0.025, 0.025, 0.05]) / 2),
    'stove': BoundingBox(np.array([0.0, 0.0, 0.025]), np.array([0.025, 0.025, 0.05]) / 2),
}

# TODO: could compute bounding boxes using a rigid body tree

##################################################

def weld_gripper(mbp, robot_index, gripper_index):
    X_EeGripper = Pose([0, 0, 0.081], [np.pi / 2, 0, np.pi / 2])
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
    DispatchLoadMessage(scene_graph, lcm)

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

def connect_controllers(builder, plant, iiwa_model, gripper_model, q_traj_list, gripper_setpoint_list):
    assert len(q_traj_list) == len(gripper_setpoint_list)

    # Add iiwa controller
    print_period = 0.5
    iiwa_controller = KukaMultibodyController(plant=plant,
                                              kuka_model_instance=iiwa_model,
                                              print_period=print_period)
    builder.AddSystem(iiwa_controller)
    builder.Connect(iiwa_controller.get_output_port(0),
                    plant.get_input_port(0))
    builder.Connect(plant.get_continuous_state_output_port(),
                    iiwa_controller.robot_state_input_port)

    # Add hand controller
    hand_controller = HandController(plant=plant,
                                     model_instance=gripper_model)
    builder.AddSystem(hand_controller)
    builder.Connect(hand_controller.get_output_port(0),
                    plant.get_input_port(1))
    builder.Connect(plant.get_continuous_state_output_port(),
                    hand_controller.robot_state_input_port)

    # Add state machine
    state_machine = ManipStateMachine(plant, q_traj_list, gripper_setpoint_list)
    builder.AddSystem(state_machine)
    builder.Connect(plant.get_continuous_state_output_port(),
                    state_machine.robot_state_input_port)
    builder.Connect(state_machine.kuka_plan_output_port,
                    iiwa_controller.plan_input_port)
    builder.Connect(state_machine.hand_setpoint_output_port,
                    hand_controller.setpoint_input_port)

    # Add logger
    state_log = builder.AddSystem(SignalLogger(plant.get_continuous_state_output_port().size()))
    state_log._DeclarePeriodicPublish(0.02)
    builder.Connect(plant.get_continuous_state_output_port(), state_log.get_input_port(0))

    # Build diagram.
    #return builder.Build()

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
    set_max_joint_positions(mbp, context, [mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)])
    set_min_joint_positions(mbp, context, [mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)])

def open_wsg50_gripper(mbp, context, model_index):
    set_min_joint_positions(mbp, context, [mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)])
    set_max_joint_positions(mbp, context, [mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)])

##################################################

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
        resolutions = 0.005*np.pi*np.ones(len(joints))
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
    reflect_z = Pose(rotation=[np.pi / 2, 0, 0])
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

def get_pddlstream_problem(mbp, context, robot, gripper, movable, surfaces):
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

def test_inverse_kinematics(mbp, diagram, diagram_context, context, robot, gripper, broccoli):
    IIWA_JOINT_NAMES = ["iiwa_joint_%d" % (i + 1) for i in range(7)]
    IIWA_JOINT_LIMITS = zip(len(IIWA_JOINT_NAMES) * [-2], len(IIWA_JOINT_NAMES) * [2])

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
    iiwa_conf = mbp.tree().get_positions_from_array(robot, conf)
    set_joint_angles(mbp, context, IIWA_JOINT_NAMES, conf)
    print(get_world_pose(mbp, context, gripper))
    print(get_relative_transform(mbp, context, gripper_frame))

    diagram.Publish(diagram_context)
    user_input('Next?')

    for i in range(10):
        print(get_joint_angles(mbp, context, IIWA_JOINT_NAMES))
        for joint_name, joint_range in zip(IIWA_JOINT_NAMES, IIWA_JOINT_LIMITS):
            iiwa_joint = mbp.GetJointByName(joint_name)
            joint_angle = random.uniform(*joint_range)
            print(get_joint_limits(iiwa_joint))
            iiwa_joint.set_angle(context=context, angle=joint_angle)
        diagram.Publish(diagram_context)
        user_input('Continue?')
    user_input('Finish?')

##################################################

def postprocess_plan(mbp, gripper, plan):
    trajectories = []
    if plan is None:
        return trajectories

    gripper_joints = prune_fixed_joints(get_model_joints(mbp, gripper)) 
    gripper_extend_fn = get_extend_fn(gripper_joints)
    open_traj = Trajectory(Config(gripper_joints, q) for q in gripper_extend_fn(
        get_close_wsg50_positions(mbp, gripper), get_open_wsg50_positions(mbp, gripper)))
    close_traj = Trajectory(reversed(open_traj.path)) # TODO: omits the start point
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

def step_trajectories(diagram, diagram_context, context, trajectories):
    diagram.Publish(diagram_context)
    user_input('Start?')
    for traj in trajectories:
        for _ in traj.iterate(context):
            diagram.Publish(diagram_context)
            time.sleep(0.01)
        #user_input('Continue?')
    user_input('Finish?')

def simulate_splines(diagram, diagram_context, sim_duration):
    real_time_rate = 1.0
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(real_time_rate)
    simulator.Initialize()

    diagram.Publish(diagram_context)
    user_input('Start?')
    simulator.StepTo(sim_duration)
    user_input('Finish?')

##################################################

def compute_duration(q_traj_list):
    sim_duration = 0.
    for q_traj in q_traj_list:
        sim_duration += q_traj.end_time() + 0.5
    sim_duration += 5.0
    return sim_duration

def convert_splines(mbp, robot, gripper, context, trajectories):
    # TODO: move to trajectory class
    q_traj_list, gripper_setpoint_list = [], []
    for traj in trajectories:
        traj.path[-1].assign(context)
        joints = traj.path[0].joints
        if len(joints) == 2:
            q_knots_kuka = np.zeros((2, 7))
            q_knots_kuka[0] = get_configuration(mbp, context, robot) # Second is velocity
            q_traj_list.append(PiecewisePolynomial.ZeroOrderHold(
                [0, 1], q_knots_kuka.T))
        elif len(joints) == 7:
            # TODO: adjust timing based on distance & velocities
            # TODO: adjust number of waypoints
            path = [traj.path[0], traj.path[-1]]
            q_knots_kuka = np.vstack([q.positions for q in path])
            n, d = q_knots_kuka.shape
            t_knots = np.array([0., 1.])
            q_traj_list.append(PiecewisePolynomial.Cubic(
                breaks=t_knots, 
                knots=q_knots_kuka.T,
                knot_dot_start=np.zeros(d), 
                knot_dot_end=np.zeros(d)))
        else:
            raise ValueError(joints)
        _, gripper_setpoint = get_configuration(mbp, context, gripper)
        gripper_setpoint_list.append(gripper_setpoint)
    return q_traj_list, gripper_setpoint_list

##################################################

def main():
    # TODO: argparse for step vs simulate

    # TODO: this doesn't update unless it's set to zero
    timestep = 0 #.0002 # 0
    mbp = MultibodyPlant(time_step=timestep)
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

    table_top_z = get_aabb_z_placement(AABBs['sink'], AABBs['table'])
    weld_gripper(mbp, robot, gripper)
    weld_to_world(mbp, robot, Pose(translation=[0, 0, table_top_z]))
    weld_to_world(mbp, table, Pose())
    weld_to_world(mbp, table2, Pose(translation=[0.75, 0, 0]))
    mbp.Finalize(scene_graph)

    ##################################################
    
    #dump_plant(mbp)
    #dump_models(mbp)

    context = mbp.CreateDefaultContext()
    table2_x = 0.75
    # TODO: weld sink & stove
    set_world_pose(mbp, context, sink, Pose(translation=[table2_x, 0.25, table_top_z]))
    set_world_pose(mbp, context, stove, Pose(translation=[table2_x, -0.25, table_top_z]))
    set_world_pose(mbp, context, broccoli, Pose(translation=[table2_x, 0, table_top_z]))
    open_wsg50_gripper(mbp, context, gripper)
    #close_wsg50_gripper(mbp, context, gripper)
    #set_configuration(mbp, context, gripper, [-0.05, 0.05])
    
    initial_state = context.get_continuous_state_vector().get_value() # CopyToVector
    #print(context.get_continuous_state().get_vector())
    #print(context.get_continuous_state_vector())
    #print(context.get_discrete_state_vector())
    #print(context.get_abstract_state())

    ##################################################

    problem = get_pddlstream_problem(mbp, context, robot, gripper, movable=[broccoli], surfaces=[sink, stove])
    solution = solve_focused(problem, planner='ff-astar', max_cost=INF)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return
    trajectories = postprocess_plan(mbp, gripper, plan)
    q_traj_list, gripper_setpoint_list = convert_splines(mbp, robot, gripper, context, trajectories)
    sim_duration = compute_duration(q_traj_list)
    print('Splines: {}\nDuration: {} seconds'.format(len(q_traj_list), sim_duration))

    ##################################################

    builder = build_diagram(mbp, scene_graph, lcm)
    connect_controllers(builder, mbp, robot, gripper, q_traj_list, gripper_setpoint_list)
    diagram = builder.Build()
    diagram_context = create_context(diagram, mbp)
    context = diagram.GetMutableSubsystemContext(mbp, diagram_context)
    context.get_mutable_continuous_state_vector().SetFromVector(initial_state)

    step_trajectories(diagram, diagram_context, context, trajectories)
    #simulate_splines(diagram, diagram_context, sim_duration)

if __name__ == '__main__':
    main()
