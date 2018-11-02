from __future__ import print_function

import time
import numpy as np
import argparse
import cProfile
import pstats
import random
from itertools import product

from examples.drake.generators import Pose, Conf, Trajectory, get_stable_gen, get_grasp_gen, get_ik_fn, \
    get_motion_fn, get_pull_fn, get_collision_test
from examples.drake.iiwa_utils import get_close_wsg50_positions, get_open_wsg50_positions, \
    open_wsg50_gripper, get_open_positions, get_closed_positions
from examples.drake.motion import get_distance_fn, get_extend_fn, waypoints_from_path
from examples.drake.problems import load_manipulation, load_station, load_tables
from examples.drake.utils import get_model_joints, get_world_pose, set_world_pose, set_joint_position, \
    prune_fixed_joints, get_configuration, get_model_name, dump_models, user_input, \
    get_model_indices, exists_colliding_pair, get_joint_positions, get_parent_joints, get_base_body, get_body_pose

from pydrake.geometry import (ConnectDrakeVisualizer, DispatchLoadMessage)
from pydrake.lcm import DrakeLcm # Required else "ConnectDrakeVisualizer(): incompatible function arguments."
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import SignalLogger
from pydrake.trajectories import PiecewisePolynomial
from pydrake.systems.analysis import Simulator

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.language.constants import And
from pddlstream.utils import print_solution, read, INF, get_file_path

# https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_tree.html
# wget -q https://registry.hub.docker.com/v1/repositories/mit6881/drake-course/tags -O -  | sed -e 's/[][]//g' -e 's/"//g' -e 's/ //g' | tr '}' '\n'  | awk -F: '{print $3}'
# https://stackoverflow.com/questions/28320134/how-to-list-all-tags-for-a-docker-image-on-a-remote-registry
# docker rmi $(docker images -q mit6881/drake-course)

# https://github.com/RobotLocomotion/drake/blob/a54513f9d0e746a810da15b5b63b097b917845f0/bindings/pydrake/multibody/test/multibody_tree_test.py
# ~/Programs/LIS/git/pddlstream$ ~/Programs/Classes/6811/drake_docker_utility_scripts/docker_run_bash_mac.sh drake-20181012 .
# http://127.0.0.1:7000/static/

# gz sdf -p ../urdf/iiwa14_polytope_collision.urdf > /iiwa14_polytope_collision.sdf

##################################################


def load_meshcat():
    import meshcat
    return meshcat.Visualizer()  # vis.set_object


def add_meshcat_visualizer(scene_graph, builder):
    # https://github.com/rdeits/meshcat-python
    # https://github.com/RussTedrake/underactuated/blob/master/src/underactuated/meshcat_visualizer.py
    from underactuated.meshcat_visualizer import MeshcatVisualizer
    viz = MeshcatVisualizer(scene_graph, draw_timestep=0.033333)
    builder.AddSystem(viz)
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    viz.get_input_port(0))
    viz.load()
    return viz


def add_drake_visualizer(scene_graph, builder):
    lcm = DrakeLcm()
    ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph, lcm=lcm)
    DispatchLoadMessage(scene_graph, lcm) # TODO: only update viewer after a plan is found
    return lcm # Important that variable is saved

##################################################


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
    from examples.drake.kuka_multibody_controllers import (KukaMultibodyController, HandController, ManipStateMachine)

    iiwa_controller = KukaMultibodyController(plant=mbp,
                                              kuka_model_instance=robot,
                                              print_period=print_period)
    builder.AddSystem(iiwa_controller)
    builder.Connect(iiwa_controller.get_output_port(0),
                    mbp.get_input_port(0)) # RuntimeError: Input port is already wired
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

def build_diagram(mbp, scene_graph, meshcat=False):
    builder = DiagramBuilder()
    builder.AddSystem(scene_graph)
    builder.AddSystem(mbp)
    connect_collisions(mbp, scene_graph, builder)
    if meshcat:
        vis = add_meshcat_visualizer(scene_graph, builder)
    else:
        vis = add_drake_visualizer(scene_graph, builder)
    add_logger(mbp, builder)
    #return builder.Build()
    return builder, vis

##################################################

def get_pddlstream_problem(mbp, context, scene_graph, task, collisions=True):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    robot = task.robot
    #gripper = task.gripper
    robot_name = get_model_name(mbp, robot)

    world = mbp.world_frame() # mbp.world_body()
    robot_joints = prune_fixed_joints(get_model_joints(mbp, robot))
    robot_conf = Conf(robot_joints, get_configuration(mbp, context, robot))
    init = [
        ('Robot', robot_name),
        ('CanMove', robot_name),
        ('Conf', robot_name, robot_conf),
        ('AtConf', robot_name, robot_conf),
        ('HandEmpty', robot_name),
    ]
    goal_literals = [
        ('AtConf', robot_name, robot_conf),
        #('Holding', robot_name, get_model_name(mbp, task.movable[0])),
    ]

    for obj in task.movable:
        obj_name = get_model_name(mbp, obj)
        #obj_frame = get_base_body(mbp, obj).body_frame()
        obj_pose = Pose(mbp, world, obj, get_world_pose(mbp, context, obj)) # get_relative_transform
        init += [('Graspable', obj_name),
                 ('Pose', obj_name, obj_pose),
                 ('AtPose', obj_name, obj_pose)]
        for surface in task.surfaces:
            init += [('Stackable', obj_name, surface)]
            #if is_placement(body, surface):
            #    init += [('Supported', body, pose, surface)]

    for surface in task.surfaces:
        surface_name = get_model_name(mbp, surface.model_index)
        if 'sink' in surface_name:
            init += [('Sink', surface)]
        if 'stove' in surface_name:
            init += [('Stove', surface)]

    for door in task.doors:
        door_body = mbp.tree().get_body(door)
        door_name = door_body.name()
        door_joints = get_parent_joints(mbp, door_body)
        door_conf = Conf(door_joints, get_joint_positions(door_joints, context))
        init += [
            ('Door', door_name),
            ('Conf', door_name, door_conf),
            ('AtConf', door_name, door_conf),
        ]
        for positions in [get_open_positions(door_body)]: #, get_closed_positions(door_body)]:
            conf = Conf(door_joints, positions)
            init += [('Conf', door_name, conf)]
            goal_literals += [('AtConf', door_name, conf)]

    for obj, surface in task.goal_on:
        obj_name = get_model_name(mbp, obj)
        goal_literals.append(('On', obj_name, surface))
    for obj in task.goal_cooked:
        obj_name = get_model_name(mbp, obj)
        goal_literals.append(('Cooked', obj_name))

    goal = And(*goal_literals)
    print('Initial:', init)
    print('Goal:', goal)

    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(task, context, collisions=collisions)),
        'sample-grasp': from_gen_fn(get_grasp_gen(task)),
        'inverse-kinematics': from_fn(get_ik_fn(task, context, collisions=collisions)),
        'plan-motion': from_fn(get_motion_fn(task, context, collisions=collisions)),
        'plan-pull': from_fn(get_pull_fn(task, context, collisions=collisions)),
        'TrajCollision': get_collision_test(task, context, collisions=collisions),
    }
    #stream_map = 'debug'

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def get_open_trajectory(mbp, gripper):
    gripper_joints = prune_fixed_joints(get_model_joints(mbp, gripper))
    gripper_extend_fn = get_extend_fn(gripper_joints)
    gripper_closed_conf = get_close_wsg50_positions(mbp, gripper)
    gripper_path = list(gripper_extend_fn(gripper_closed_conf, get_open_wsg50_positions(mbp, gripper)))
    gripper_path.insert(0, gripper_closed_conf)
    return Trajectory(Conf(gripper_joints, q) for q in gripper_path)

def postprocess_plan(mbp, gripper, plan):
    trajectories = []
    if plan is None:
        return trajectories

    open_traj = get_open_trajectory(mbp, gripper)
    close_traj = Trajectory(reversed(open_traj.path))
    # TODO: ceiling & orientation constraints
    # TODO: sampler chooses configurations that are far apart

    # TODO: maybe just specify the position sequence
    attachments = {}
    for name, args in plan:
        if name in ['clean', 'cook']:
            continue
        if name == 'pick':
            r, o, p, g, q, t = args
            trajectories.extend([Trajectory(reversed(t.path), attachments=attachments.values()), close_traj])
            attachments[o] = g
            trajectories.append(Trajectory(t.path, attachments=attachments.values()))
        elif name == 'place':
            r, o, p, g, q, t = args
            trajectories.extend([Trajectory(reversed(t.path), attachments=attachments.values()), open_traj])
            del attachments[o]
            trajectories.append(Trajectory(t.path, attachments=attachments.values()))
        elif name == 'pull':
            t = args[-1]
            trajectories.extend([close_traj, Trajectory(t.path, attachments=attachments.values()), open_traj])
        else:
            t = args[-1]
            trajectories.append(Trajectory(t.path, attachments=attachments.values()))

    return trajectories

def step_trajectories(diagram, diagram_context, context, trajectories, time_step=0.001, teleport=False):
    diagram.Publish(diagram_context)
    user_input('Step?')
    for traj in trajectories:
        if teleport:
            traj.path = traj.path[::len(traj.path)-1]
        for _ in traj.iterate(context):
            diagram.Publish(diagram_context)
            if time_step is None:
                user_input('Continue?')
            else:
                time.sleep(time_step)
    user_input('Finish?')

def simulate_splines(diagram, diagram_context, sim_duration, real_time_rate=1.0):
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(real_time_rate)
    simulator.Initialize()

    diagram.Publish(diagram_context)
    user_input('Simulate?')
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
        if len(joints) == 8: # TODO: fix this
            joints = joints[:7]

        if len(joints) == 2:
            q_knots_kuka = np.zeros((2, 7))
            q_knots_kuka[0] = get_configuration(mbp, context, robot) # Second is velocity
            splines.append(PiecewisePolynomial.ZeroOrderHold([0, 1], q_knots_kuka.T))
        elif len(joints) == 7:
            # TODO: adjust timing based on distance & velocities
            # TODO: adjust number of waypoints
            distance_fn = get_distance_fn(joints)
            #path = [traj.path[0].positions, traj.path[-1].positions]
            path = [q.positions[:len(joints)] for q in traj.path]
            path = waypoints_from_path(joints, path) # TODO: increase time for pick/place & hold
            q_knots_kuka = np.vstack(path).T
            distances = [0.] + [distance_fn(q1, q2) for q1, q2 in zip(path, path[1:])]
            t_knots = np.cumsum(distances) / RADIANS_PER_SECOND # TODO: this should be a max
            d, n = q_knots_kuka.shape
            print('{}) d={}, n={}, duration={:.3f}'.format(i, d, n, t_knots[-1]))
            print(t_knots)
            splines.append(PiecewisePolynomial.Cubic(
                breaks=t_knots, 
                knots=q_knots_kuka,
                knot_dot_start=np.zeros(d), 
                knot_dot_end=np.zeros(d)))
            # RuntimeError: times must be in increasing order.
        else:
            raise ValueError(joints)
        _, gripper_setpoint = get_configuration(mbp, context, gripper)
        gripper_setpoints.append(gripper_setpoint)
    return splines, gripper_setpoints


##################################################

def test_manipulation(plan_list, gripper_setpoint_list):
    from pydrake.common import FindResourceOrThrow
    from .lab_1.manipulation_station_simulator import ManipulationStationSimulator

    is_hardware = False
    object_file_path = FindResourceOrThrow(
            #"drake/examples/manipulation_station/models/061_foam_brick.sdf")
            "drake/external/models_robotlocomotion/ycb_objects/061_foam_brick.sdf")

    manip_station_sim = ManipulationStationSimulator(
        time_step=1e-3,
        object_file_path=object_file_path,
        object_base_link_name="base_link",
        is_hardware=is_hardware)

    q0 = [0, 0.6-np.pi/6, 0, -1.75, 0, 1.0, 0]

    if is_hardware:
        iiwa_position_command_log = manip_station_sim.RunRealRobot(plan_list, gripper_setpoint_list)
    else:
        #q0[1] += np.pi/6
        iiwa_position_command_log = manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                        extra_time=2.0, q0_kuka=q0)
    return iiwa_position_command_log

##################################################

def test_generators(task, diagram, diagram_context):
    mbp = task.mbp
    context = diagram.GetMutableSubsystemContext(mbp, diagram_context)

    # Test grasps
    print(get_base_body(mbp, task.gripper).name())
    print(get_body_pose(context, mbp.GetBodyByName('left_finger', task.gripper)).matrix() -
          get_body_pose(context, get_base_body(mbp, task.gripper)).matrix())
    user_input('Start')
    model = task.movable[0]
    grasp_gen_fn = get_grasp_gen(task)
    for grasp, in grasp_gen_fn(get_model_name(mbp, model)):
        grasp.assign(context)
        diagram.Publish(diagram_context)
        user_input('Continue')

    # Test placements
    user_input('Start')
    pose_gen_fn = get_stable_gen(task, context)
    model = task.movable[0]
    for pose, in pose_gen_fn(get_model_name(mbp, model), task.surfaces[0]):
       pose.assign(context)
       diagram.Publish(diagram_context)
       user_input('Continue')

##################################################

def main(deterministic=True):
    # TODO: GeometryInstance, InternalGeometry, & GeometryContext to get the shape of objects
    # TODO: cost-sensitive planning to avoid large kuka moves
    # get_contact_results_output_port
    # TODO: gripper closing via collision information

    if deterministic:
        random.seed(0)
        np.random.seed(0)

    parser = argparse.ArgumentParser()
    #parser.add_argument('-p', '--problem')
    parser.add_argument('-c', '--cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-m', '--meshcat', action='store_true', help='Use the meshcat viewer')
    parser.add_argument('-s', '--simulate', action='store_true', help='Simulate')
    args = parser.parse_args()

    time_step = 0.0002 # TODO: context.get_continuous_state_vector() fails
    problem_fn = load_manipulation # load_tables | load_manipulation | load_station

    meshcat_vis = None
    if args.meshcat:
        meshcat_vis = load_meshcat()  # Important that variable is saved
        # http://127.0.0.1:7000/static/

    mbp, scene_graph, task = problem_fn(time_step=time_step)
    #station, mbp, scene_graph = load_station(time_step=time_step)
    #builder.AddSystem(station)
    #dump_plant(mbp)
    #dump_models(mbp)
    print(task)
    #print(sorted(body.name() for body in task.movable_bodies()))
    #print(sorted(body.name() for body in task.fixed_bodies()))

    ##################################################

    builder, _ = build_diagram(mbp, scene_graph, args.meshcat)
    if args.simulate:
        # TODO: RuntimeError: Input port is already wired
        state_machine = connect_controllers(builder, mbp, task.robot, task.gripper)
    else:
        state_machine = None
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    context = diagram.GetMutableSubsystemContext(mbp, diagram_context)
    task.diagram = diagram
    task.diagram_context = diagram_context

    #context = mbp.CreateDefaultContext()
    #context = scene_graph.CreateDefaultContext()
    for joint, position in task.initial_positions.items():
        set_joint_position(joint, context, position)
    for model, pose in task.initial_poses.items():
        set_world_pose(mbp, context, model, pose)
    open_wsg50_gripper(mbp, context, task.gripper)
    #close_wsg50_gripper(mbp, context, task.gripper)
    #set_configuration(mbp, context, task.gripper, [-0.05, 0.05])

    # from underactuated.meshcat_visualizer import MeshcatVisualizer
    # #add_meshcat_visualizer(scene_graph)
    # viz = MeshcatVisualizer(scene_graph, draw_timestep=0.033333)
    # viz.load()
    # viz.draw(context)

    diagram.Publish(diagram_context)
    #initial_state = context.get_continuous_state_vector().get_value() # CopyToVector
    initial_state = mbp.tree().get_multibody_state_vector(context).copy()
    #print(exists_colliding_pair(mbp, context, product(get_model_indices(mbp), repeat=2)))
    #point_pair = scene_graph.get_query_output_port().Eval(builder.CreateDefaultContext())

    # get_mutable_multibody_state_vector
    #q = mbp.tree().get_multibody_state_vector(context)[:mbp.num_positions()]
    #print(mbp.tree().get_positions_from_array(task.movable[0], q))

    ##################################################

    problem = get_pddlstream_problem(mbp, context, scene_graph, task, collisions=not args.cfree)
    pr = cProfile.Profile()
    pr.enable()
    solution = solve_focused(problem, planner='ff-wastar2', max_cost=INF, max_time=120, debug=False)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        return
    trajectories = postprocess_plan(mbp, task.gripper, plan)

    ##################################################

    #context.get_mutable_continuous_state_vector().SetFromVector(initial_state)
    mbp.tree().get_mutable_multibody_state_vector(context)[:] = initial_state
    #if not args.simulate:
    #    fix_input_ports(mbp, context)
    #sub_context = diagram.GetMutableSubsystemContext(mbp, diagram_context)
    #print(context == sub_context) # True

    if args.simulate:
        splines, gripper_setpoints = convert_splines(mbp, task.robot, task.gripper, context, trajectories)
        sim_duration = compute_duration(splines)
        print('Splines: {}\nDuration: {:.3f} seconds'.format(len(splines), sim_duration))
        mbp.tree().get_mutable_multibody_state_vector(context)[:] = initial_state

        if True:
            state_machine.Load(splines, gripper_setpoints)
            simulate_splines(diagram, diagram_context, sim_duration)
        else:
            # NOTE: there is a plan that moves home initially for 15 seconds
            from .lab_1.robot_plans import JointSpacePlan
            plan_list = [JointSpacePlan(spline) for spline in splines]
            #meshcat_vis.delete()
            user_input('Simulate?')
            test_manipulation(plan_list, gripper_setpoints)
    else:
        #time_step = None
        #time_step = 0.001
        time_step = 0.01
        step_trajectories(diagram, diagram_context, context, trajectories, time_step=time_step) #, teleport=True)


if __name__ == '__main__':
    main()

# python2 -m examples.drake.run
