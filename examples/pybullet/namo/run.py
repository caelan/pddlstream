#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import pstats
from collections import namedtuple
from itertools import combinations, product, islice

import numpy as np

from examples.pybullet.pr2_belief.problems import BeliefState
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Conf, Pose, control_commands, Attach, Detach, \
    create_trajectory, apply_commands, State, Command, Grasp
from examples.pybullet.utils.pybullet_tools.utils import connect, disconnect, draw_base_limits, WorldSaver, joint_from_name, \
    get_sample_fn, get_distance_fn, get_collision_fn, MAX_DISTANCE, get_extend_fn, wait_for_user, Point, remove_body, \
    LockRenderer, get_bodies, draw_point, add_line, set_point, create_box, stable_z, load_model, TURTLEBOT_URDF, \
    joints_from_names, create_cylinder, set_joint_positions, get_joint_positions, HideOutput, GREY, TAN, RED, \
    pairwise_collision, get_halton_sample_fn, get_distance, get_subtree_aabb, link_from_name, BodySaver, \
    approximate_as_cylinder, get_point, set_point, set_euler, draw_aabb, draw_pose, get_pose, \
    Point, Euler, remove_debug, quat_from_euler, get_link_pose, invert, multiply, set_pose, \
    base_values_from_pose, halton_generator, set_renderer, get_visual_data, BLUE
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.function import FunctionInfo
from pddlstream.language.constants import And, print_solution
from pddlstream.language.generator import from_test, from_fn, from_gen_fn
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, INF, get_file_path

# https://github.com/erwincoumans/pybullet_robots/tree/master/data/turtlebot
# https://github.com/erwincoumans/pybullet_robots/tree/master/data/f10_racecar
# https://github.com/erwincoumans/bullet3/tree/master/data/racecar
# Logistics ICAPS domain

BASE_LINK = 'base_link'
BASE_JOINTS = ['x', 'y', 'theta']
#BASE_RESOLUTIONS = 0.05*np.ones(3) # Default
BASE_RESOLUTIONS = np.array([0.05, 0.05, np.pi/16])

def get_base_joints(robot):
    return joints_from_names(robot, BASE_JOINTS)

def get_base_conf(robot):
    return get_joint_positions(robot, get_base_joints(robot))

def set_base_conf(robot, conf):
    set_joint_positions(robot, get_base_joints(robot), conf)

def get_custom_limits(robot, base_limits):
    x_limits, y_limits = zip(*base_limits)
    return {
        joint_from_name(robot, 'x'): x_limits,
        joint_from_name(robot, 'y'): y_limits,
    }

#######################################################

def point_from_conf(conf, z=0.01):
    x, y, theta = conf
    return (x, y, z)

def get_test_cfree_conf_pose(problem):
    def test(r, q, b2, p2):
        if not problem.collisions:
            return True
        q.assign()
        p2.assign()
        return not pairwise_collision(r, b2)
    return test

def get_test_cfree_traj_pose(problem):
    def test(r, t, b2, p2):
        if not problem.collisions:
            return True
        p2.assign()
        state = State()
        for _ in t.apply(state):
            state.assign()
            #for b1 in state.attachments:
            #    if pairwise_collision(b1, b2):
            #        return False
            if pairwise_collision(r, b2):
                set_renderer(True)
                color = get_visual_data(b2)[0].rgbaColor
                handles = add_line(point_from_conf(t.path[0].values, z=0.02),
                                   point_from_conf(t.path[-1].values, z=0.02), color=color)
                wait_for_user()
                set_renderer(False)
                return False
        return True
    return test

def get_grasp_generator(problem):
    def gen(robot, body):
        link = link_from_name(robot, BASE_LINK)
        with BodySaver(robot):
            set_base_conf(robot, np.zeros(3))
            robot_pose = get_link_pose(robot, link)
            robot_aabb = get_subtree_aabb(robot, link)
            #draw_aabb(robot_aabb)
        lower, upper = robot_aabb
        center, (diameter, height) = approximate_as_cylinder(body)
        _, _, z = get_point(body) # Assuming already placed stably
        position = Point(x=upper[0] + diameter / 2., z=z)

        for [scale] in halton_generator(d=1):
            yaw = scale*2*np.pi - np.pi
            quat = quat_from_euler(Euler(yaw=yaw))
            body_pose = (position, quat)
            robot_from_body = multiply(invert(robot_pose), body_pose)
            grasp = Pose(body, robot_from_body)
            yield (grasp,)
            #world_pose = multiply(get_link_pose(robot, link), robot_from_body)
            #set_pose(body, world_pose)
            #handles = draw_pose(get_pose(body), length=1)
            #wait_for_user()
            #for handle in handles:
            #    remove_debug(handle)
    return gen

def get_ik_fn(problem):
    def fn(robot, body, pose, grasp):
        joints = get_base_joints(robot)
        robot_pose = multiply(pose.value, invert(grasp.value))
        base_values = base_values_from_pose(robot_pose)
        conf = Conf(robot, joints, base_values)
        conf.assign()
        if any(pairwise_collision(robot, obst) for obst in problem.obstacles):
            return None
        return (conf,)
    return fn

def get_fk_fn(problem):
    # TODO: could sample placements in place or not
    def fn(robot, conf, body, grasp):
        conf.assign()
        link = link_from_name(robot, BASE_LINK)
        world_from_body = multiply(get_link_pose(robot, link), grasp.value)
        pose = Pose(body, world_from_body)
        pose.assign()
        if any(pairwise_collision(body, obst) for obst in problem.obstacles):
            return None
        return (pose,)
    return fn

def get_sample_gen_fn(problem):
    def gen_fn(robot):
        joints = get_base_joints(robot)
        sample_fn = get_halton_sample_fn(robot, joints, custom_limits=problem.custom_limits)
        collision_fn = get_collision_fn(robot, joints, problem.obstacles, attachments=[],
                                        self_collisions=False, disabled_collisions=set(),
                                        custom_limits=problem.custom_limits, max_distance=MAX_DISTANCE)
        handles = []
        while True:
            sample = sample_fn()
            if not collision_fn(sample):
                q = Conf(robot, joints, sample)
                handles.extend(draw_point(point_from_conf(sample), size=0.05))
                yield (q,)
    return gen_fn

def get_motion_fn(problem, max_distance=0.45):
    def fn(body, q1, q2):
        joints = get_base_joints(body)
        extend_fn = get_extend_fn(body, joints, resolutions=BASE_RESOLUTIONS)
        collision_fn = get_collision_fn(body, joints, problem.obstacles, attachments=[],
                                        self_collisions=False, disabled_collisions=set(),
                                        custom_limits=problem.custom_limits, max_distance=MAX_DISTANCE)
        # TODO: degree
        # distance = distance_fn(q1.values, q2.values)
        distance = get_distance(q1.values[:2], q2.values[:2])
        if max_distance < distance:
            return None
        path = [q1.values] + list(extend_fn(q1.values, q2.values))
        if any(map(collision_fn, path)):
            return None
        handles = add_line(point_from_conf(q1.values), point_from_conf(q2.values))
        t = create_trajectory(body, joints, path)
        return (t,)
    return fn

def get_cost_fn(problem):
    def fn(r, q1, q2):
        return get_distance(q1.values[:2], q2.values[:2])
    return fn

#######################################################

def create_vertices(problem, robot, samples, samples_per_ft2=8):
    lower, upper = problem.limits
    area = np.product(upper - lower)
    print('Area:', area)
    num_samples = int(samples_per_ft2 * area)

    gen_fn = get_sample_gen_fn(problem)
    init = []
    for (q,) in islice(gen_fn(robot), num_samples):
        samples.append(q)
        init += [('Conf', robot, q)]
    return init

def create_edges(problem, robot, samples):
    motion_fn = get_motion_fn(problem)

    edges = []
    init = []
    for q1, q2 in product(samples, repeat=2):
        if q1 == q2:
            continue
        result = motion_fn(robot, q1, q2)
        if result is None:
            continue
        (t,) = result
        edges.append((q1, q2))
        init += [('Motion', robot, q1, q2, t), ('Traj', robot, t)]
    return init

def pddlstream_from_problem(problem, teleport=False):
    # TODO: push and attach to movable objects

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    # TODO: action to generically connect to the roadmap
    # TODO: could check individual vertices first
    # TODO: dynamically generate the roadmap in interesting parts of the space
    # TODO: visibility graphs for sparse roadmaps
    # TODO: approximate robot with isotropic geometry
    # TODO: make the effort finite if applied to the roadmap vertex

    samples = []
    init = []
    for robot, conf in problem.initial_confs.items():
        samples.append(conf)
        init += [('Robot', robot), ('Conf', robot, conf), ('AtConf', robot, conf), ('Free', robot)]
    for body, pose in problem.initial_poses.items():
        init += [('Body', body), ('Pose', body, pose), ('AtPose', body, pose)]

    goal_literals = []
    goal_literals += [('Holding', robot, body) for robot, body in problem.goal_holding.items()]
    for robot, base_values in problem.goal_confs.items():
        q_goal = Conf(robot, get_base_joints(robot), base_values)
        samples.append(q_goal)
        init += [('Conf', robot, q_goal)]
        goal_literals += [('AtConf', robot, q_goal)]
    goal_formula = And(*goal_literals)

    # TODO: assuming holonomic for now
    [body] = problem.robots

    with LockRenderer():
        init += create_vertices(problem, body, samples)
        #init += create_edges(problem, body, samples)

    stream_map = {
        'test-cfree-conf-pose': from_test(get_test_cfree_conf_pose(problem)),
        'test-cfree-traj-pose': from_test(get_test_cfree_traj_pose(problem)),
        'sample-grasp': from_gen_fn(get_grasp_generator(problem)),
        'compute-ik': from_fn(get_ik_fn(problem)),
        'compute-motion': from_fn(get_motion_fn(problem)),
        'test-reachable': from_test(lambda *args: False),
        'Cost': get_cost_fn(problem),
    }
    #stream_map = 'debug'

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula

#######################################################

class NAMOProblem(object):
    def __init__(self, robots, limits, movable=[], collisions=True,
                 goal_holding={}, goal_confs={}):
        self.robots = tuple(robots)
        self.limits = limits
        self.movable = tuple(movable)
        self.collisions = collisions
        self.goal_holding = goal_holding
        self.goal_confs = goal_confs
        self.obstacles = tuple(body for body in get_bodies()
                          if body not in self.robots + self.movable)
        self.custom_limits = {}
        if self.limits is not None:
            for robot in self.robots:
                self.custom_limits.update(get_custom_limits(robot, self.limits))
        self.initial_confs = {robot: Conf(robot, get_base_joints(robot)) for robot in self.robots}
        self.initial_poses = {body: Pose(body) for body in self.movable}

def problem_fn(n_rovers=1, collisions=True):
    base_extent = 2.5
    base_limits = (-base_extent / 2. * np.ones(2), base_extent / 2. * np.ones(2))
    mound_height = 0.1

    floor = create_box(base_extent, base_extent, 0.001, color=TAN)  # TODO: two rooms
    set_point(floor, Point(z=-0.001 / 2.))

    wall1 = create_box(base_extent + mound_height, mound_height, mound_height, color=GREY)
    set_point(wall1, Point(y=base_extent / 2., z=mound_height / 2.))
    wall2 = create_box(base_extent + mound_height, mound_height, mound_height, color=GREY)
    set_point(wall2, Point(y=-base_extent / 2., z=mound_height / 2.))
    wall3 = create_box(mound_height, base_extent + mound_height, mound_height, color=GREY)
    set_point(wall3, Point(x=base_extent / 2., z=mound_height / 2.))
    wall4 = create_box(mound_height, base_extent + mound_height, mound_height, color=GREY)
    set_point(wall4, Point(x=-base_extent / 2., z=mound_height / 2.))

    wall5 = create_box(mound_height, (base_extent + mound_height)/ 2., mound_height, color=GREY)
    set_point(wall5, Point(y=base_extent / 4., z=mound_height / 2.))

    rover_confs = [(+1, 0, np.pi), (-1, 0, 0)]
    assert n_rovers <= len(rover_confs)

    robots = []
    for i in range(n_rovers):
        with HideOutput():
            rover = load_model(TURTLEBOT_URDF)
        robot_z = stable_z(rover, floor)
        set_point(rover, Point(z=robot_z))
        set_base_conf(rover, rover_confs[i])
        robots.append(rover)
    goal_confs = {robots[0]: rover_confs[-1]}
    #goal_confs = {}

    # TODO: make the objects smaller
    cylinder_radius = 0.25
    body1 = create_cylinder(cylinder_radius, mound_height, color=RED)
    set_point(body1, Point(y=-base_extent / 4., z=mound_height / 2.))
    body2 = create_cylinder(cylinder_radius, mound_height, color=BLUE)
    set_point(body2, Point(x=base_extent / 4., y=3*base_extent / 8., z=mound_height / 2.))
    movable = [body1, body2]
    #goal_holding = {robots[0]: body1}
    goal_holding = {}

    return NAMOProblem(robots, base_limits, movable, collisions=collisions,
                       goal_holding=goal_holding, goal_confs=goal_confs)

#######################################################

class Vaporize(Command):
    def __init__(self, body):
        self.body = body
    def apply(self, state, **kwargs):
        remove_body(self.body)
        yield
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

def post_process(problem, plan, teleport=False):
    if plan is None:
        return None
    commands = []
    attachments = {}
    for i, (name, args) in enumerate(plan):
        if name == 'vaporize':
            if len(args) == 2:
                b, p = args
            else:
                r, q, b, p, g = args
            new_commands = [Vaporize(b)]
        elif name == 'pick':
            r, q, b, p, g = args
            #attachments[r] = r
            new_commands = [Attach(r, arm=BASE_LINK, grasp=g, body=b)]
        elif name == 'place':
            # TODO: make a drop all rocks
            r, s = args
            new_commands = [Detach(r, arm=BASE_LINK, body=attachments[r])]
        elif name == 'move':
            r, q1, q2, t = args
            new_commands = [t]
        else:
            raise ValueError(name)
        print(i, name, args, new_commands)
        commands += new_commands
    return commands

#######################################################

def main(display=True, teleport=False):
    parser = argparse.ArgumentParser()
    #parser.add_argument('-problem', default='rovers1', help='The name of the problem to solve')
    parser.add_argument('-algorithm', default='focused', help='Specifies the algorithm')
    parser.add_argument('-cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-t', '--max_time', default=120, type=int, help='The max time')
    parser.add_argument('-unit', action='store_true', help='Uses unit costs')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    parser.add_argument('-viewer', action='store_true', help='enable the viewer while planning')
    args = parser.parse_args()
    print(args)

    #problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    #if args.problem not in problem_fn_from_name:
    #    raise ValueError(args.problem)
    #problem_fn = problem_fn_from_name[args.problem]
    connect(use_gui=args.viewer)
    with HideOutput():
        problem = problem_fn(collisions=not args.cfree)
    saver = WorldSaver()
    draw_base_limits(problem.limits, color=RED)

    pddlstream_problem = pddlstream_from_problem(problem, teleport=teleport)
    stream_info = {
        'test-cfree-conf-pose': StreamInfo(negate=True, p_success=1e-2),
        'test-cfree-traj-pose': StreamInfo(negate=True, p_success=1e-1),
        'compute-motion': StreamInfo(eager=True, p_success=0),
        'test-reachable': StreamInfo(eager=True),
        'Distance': FunctionInfo(eager=True),
    }
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)

    success_cost = 0 if args.optimal else INF
    planner = 'ff-wastar1'
    search_sample_ratio = 0
    max_planner_time = 10

    pr = cProfile.Profile()
    pr.enable()
    with LockRenderer(True):
        if args.algorithm == 'focused':
            solution = solve_focused(pddlstream_problem, stream_info=stream_info,
                                     planner=planner, max_planner_time=max_planner_time, debug=False,
                                     unit_costs=args.unit, success_cost=success_cost,
                                     max_time=args.max_time, verbose=True,
                                     unit_efforts=True, effort_weight=1,
                                     bind=True, max_skeletons=None,
                                     search_sample_ratio=search_sample_ratio)
        elif args.algorithm == 'incremental':
            solution = solve_incremental(pddlstream_problem,
                                         planner=planner, max_planner_time=max_planner_time,
                                         unit_costs=args.unit, success_cost=success_cost,
                                         max_time=args.max_time, verbose=True)
        else:
            raise ValueError(args.algorithm)

    print_solution(solution)
    plan, cost, evaluations = solution
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(25) # cumtime | tottime
    if plan is None:
        wait_for_user()
        return
    if (not display) or (plan is None):
        disconnect()
        return

    with LockRenderer():
        commands = post_process(problem, plan, teleport=teleport)
        saver.restore()  # Assumes bodies are ordered the same way
    if not args.viewer:
        disconnect()
        connect(use_gui=True)
        with LockRenderer():
            with HideOutput():
                problem_fn() # TODO: way of doing this without reloading?
            saver.restore() # Assumes bodies are ordered the same way

    wait_for_user()
    if args.simulate:
        control_commands(commands)
    else:
        time_step = None if teleport else 0.01
        apply_commands(BeliefState(problem), commands, time_step=time_step)
    wait_for_user()
    disconnect()

if __name__ == '__main__':
    main()
