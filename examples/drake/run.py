from __future__ import print_function

import argparse
import cProfile
import pstats
import random

import numpy as np

from examples.drake.generators import Pose, Conf, get_grasp_gen_fn, get_ik_gen_fn, \
    get_motion_fn, get_pull_fn, get_collision_test, get_reachable_pose_gen_fn, get_open_trajectory, Trajectory
from examples.drake.iiwa_utils import get_door_positions, DOOR_OPEN
from examples.drake.simulation import compute_duration, convert_splines, step_trajectories, \
    simulate_splines
from examples.drake.problems import PROBLEMS
from examples.drake.systems import RenderSystemWithGraphviz
from examples.drake.utils import get_world_pose, get_configuration, \
    get_model_name, get_joint_positions, get_parent_joints, \
    get_state, set_state, get_movable_joints
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, print_solution
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.utils import read, INF, get_file_path


# Listing all available docker images
# https://stackoverflow.com/questions/28320134/how-to-list-all-tags-for-a-docker-image-on-a-remote-registry
# wget -q https://registry.hub.docker.com/v1/repositories/mit6881/drake-course/tags -O -  | sed -e 's/[][]//g' -e 's/"//g' -e 's/ //g' | tr '}' '\n'  | awk -F: '{print $3}'

# Removing old docker images
# docker rmi $(docker images -q mit6881/drake-course)

# Converting from URDF to SDF
# gz sdf -p ../urdf/iiwa14_polytope_collision.urdf > /iiwa14_polytope_collision.sdf

##################################################

def get_pddlstream_problem(task, context, collisions=True):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    plant = task.mbp
    robot = task.robot
    robot_name = get_model_name(plant, robot)

    world = plant.world_frame() # mbp.world_body()
    robot_joints = get_movable_joints(plant, robot)
    robot_conf = Conf(robot_joints, get_configuration(plant, context, robot))
    init = [
        ('Robot', robot_name),
        ('CanMove', robot_name),
        ('Conf', robot_name, robot_conf),
        ('AtConf', robot_name, robot_conf),
        ('HandEmpty', robot_name),
    ]
    goal_literals = []
    if task.reset_robot:
        goal_literals.append(('AtConf', robot_name, robot_conf),)

    for obj in task.movable:
        obj_name = get_model_name(plant, obj)
        #obj_frame = get_base_body(mbp, obj).body_frame()
        obj_pose = Pose(plant, world, obj, get_world_pose(plant, context, obj)) # get_relative_transform
        init += [('Graspable', obj_name),
                 ('Pose', obj_name, obj_pose),
                 #('InitPose', obj_name, obj_pose),
                 ('AtPose', obj_name, obj_pose)]
        for surface in task.surfaces:
            init += [('Stackable', obj_name, surface)]
            # TODO: detect already stacked

    for surface in task.surfaces:
        surface_name = get_model_name(plant, surface.model_index)
        if 'sink' in surface_name:
            init += [('Sink', surface)]
        if 'stove' in surface_name:
            init += [('Stove', surface)]

    for door in task.doors:
        door_body = plant.tree().get_body(door)
        door_name = door_body.name()
        door_joints = get_parent_joints(plant, door_body)
        door_conf = Conf(door_joints, get_joint_positions(door_joints, context))
        init += [
            ('Door', door_name),
            ('Conf', door_name, door_conf),
            ('AtConf', door_name, door_conf),
        ]
        for positions in [get_door_positions(door_body, DOOR_OPEN)]:
            conf = Conf(door_joints, positions)
            init += [('Conf', door_name, conf)]
        if task.reset_doors:
            goal_literals += [('AtConf', door_name, door_conf)]

    for obj, transform in task.goal_poses.items():
        obj_name = get_model_name(plant, obj)
        obj_pose = Pose(plant, world, obj, transform)
        init += [('Pose', obj_name, obj_pose)]
        goal_literals.append(('AtPose', obj_name, obj_pose))
    for obj in task.goal_holding:
        goal_literals.append(('Holding', robot_name, get_model_name(plant, obj)))
    for obj, surface in task.goal_on:
        goal_literals.append(('On', get_model_name(plant, obj), surface))
    for obj in task.goal_cooked:
        goal_literals.append(('Cooked', get_model_name(plant, obj)))

    goal = And(*goal_literals)
    print('Initial:', init)
    print('Goal:', goal)

    stream_map = {
        #'sample-pose': from_gen_fn(get_stable_gen(task, context, collisions=collisions)),
        'sample-reachable-pose': from_gen_fn(get_reachable_pose_gen_fn(task, context, collisions=collisions)),
        'sample-grasp': from_gen_fn(get_grasp_gen_fn(task)),
        'plan-ik': from_gen_fn(get_ik_gen_fn(task, context, collisions=collisions)),
        'plan-motion': from_fn(get_motion_fn(task, context, collisions=collisions)),
        'plan-pull': from_fn(get_pull_fn(task, context, collisions=collisions)),
        'TrajPoseCollision': get_collision_test(task, context, collisions=collisions),
        'TrajConfCollision': get_collision_test(task, context, collisions=collisions),
    }
    #stream_map = 'debug' # Runs PDDLStream with "placeholder streams" for debugging

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal


def postprocess_plan(mbp, gripper, plan):
    trajectories = []
    if plan is None:
        return trajectories
    open_traj = get_open_trajectory(mbp, gripper)
    close_traj = open_traj.reverse()

    for name, args in plan:
        if name in ['clean', 'cook']:
            continue
        traj = args[-1]
        if name == 'pick':
            trajectories.extend([Trajectory(reversed(traj.path)), close_traj, traj])
        elif name == 'place':
            trajectories.extend([traj.reverse(), open_traj, Trajectory(traj.path)])
        elif name == 'pull':
            trajectories.extend([close_traj, traj, open_traj])
        elif name == 'move':
            trajectories.append(traj)
        else:
            raise NotImplementedError(name)
    return trajectories


def plan_trajectories(task, context, collisions=True, max_time=180):
    stream_info = {
        'TrajPoseCollision': FunctionInfo(p_success=1e-3, eager=False),
        'TrajConfCollision': FunctionInfo(p_success=1e-3, eager=False),
    }
    problem = get_pddlstream_problem(task, context, collisions=collisions)
    pr = cProfile.Profile()
    pr.enable()
    solution = solve_focused(problem, stream_info=stream_info, planner='ff-wastar2',
                             success_cost=INF, max_time=max_time, debug=False,
                             unit_efforts=True, effort_weight=1, search_sample_ratio=0)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(5)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        print('Unable to find a solution in under {} seconds'.format(max_time))
        return None
    return postprocess_plan(task.mbp, task.gripper, plan)

##################################################

def main(deterministic=False):
    # TODO: cost-sensitive planning to avoid large kuka moves

    #time_step = 0.0002 # TODO: context.get_continuous_state_vector() fails
    time_step = 2e-3
    if deterministic:
        # TODO: still not fully deterministic
        random.seed(0)
        np.random.seed(0)

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--problem', default='load_manipulation', help='The name of the problem to solve')
    parser.add_argument('-c', '--cfree', action='store_true', help='Disables collisions when planning')
    parser.add_argument('-v', '--visualizer', action='store_true', help='Use the drake visualizer')
    parser.add_argument('-s', '--simulate', action='store_true', help='Simulates the system')
    args = parser.parse_args()

    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if args.problem not in problem_fn_from_name:
        raise ValueError(args.problem)
    print('Problem:', args.problem)
    problem_fn = problem_fn_from_name[args.problem]

    meshcat_vis = None
    if not args.visualizer:
        import meshcat
        # Important that variable is saved
        meshcat_vis = meshcat.Visualizer()  # vis.set_object
        # http://127.0.0.1:7000/static/

    task, diagram, state_machine = problem_fn(time_step=time_step, use_meshcat=not args.visualizer,
                                              use_controllers=args.simulate)
    print(task)

    ##################################################

    plant = task.mbp
    #dump_plant(plant)
    #dump_models(plant)
    diagram_context = task.diagram_context
    RenderSystemWithGraphviz(diagram) # Useful for getting port names
    context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    task.set_initial()
    task.publish()
    initial_state = get_state(plant, context)
    trajectories = plan_trajectories(task, context, collisions=not args.cfree)
    if trajectories is None:
        return

    ##################################################

    set_state(plant, context, initial_state)
    if args.simulate:
        from .manipulation_station.robot_plans import JointSpacePlan
        splines, gripper_setpoints = convert_splines(plant, task.robot, task.gripper, context, trajectories)
        sim_duration = compute_duration(splines)
        plan_list = [JointSpacePlan(spline) for spline in splines]
        print('Splines: {}\nDuration: {:.3f} seconds'.format(len(splines), sim_duration))
        set_state(plant, context, initial_state)
        #state_machine.Load(splines, gripper_setpoints)
        state_machine.Load(plan_list, gripper_setpoints)
        simulate_splines(diagram, diagram_context, sim_duration)
    else:
        #time_step = None
        time_step = 0.001 if meshcat else 0.02
        step_trajectories(diagram, diagram_context, context, trajectories, time_step=time_step) #, teleport=True)


if __name__ == '__main__':
    main()

# .../pddlstream$ python2 -m examples.drake.run
