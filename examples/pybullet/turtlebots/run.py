#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import pstats
import numpy as np

from examples.continuous_tamp.primitives import get_value_at_time
from pddlstream.language.temporal import get_end, compute_duration
from examples.pybullet.namo.stream import BASE_RESOLUTIONS, get_turtle_aabb, get_base_joints, set_base_conf, \
    get_custom_limits, point_from_conf, get_motion_fn, create_vertices
from examples.pybullet.pr2_belief.problems import BeliefState
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Conf, create_trajectory
from examples.pybullet.utils.pybullet_tools.utils import connect, disconnect, draw_base_limits, WorldSaver, \
    wait_for_user, LockRenderer, get_bodies, add_line, create_box, stable_z, load_model, TURTLEBOT_URDF, \
    HideOutput, GREY, TAN, RED, get_extend_fn, pairwise_collision, draw_point, VideoSaver, \
    set_point, Point, GREEN, BLUE, set_color, get_all_links, wait_for_duration, user_input, \
    aabb_union, draw_aabb, aabb_overlap, remove_all_debug, get_base_distance_fn, dump_body, \
    link_from_name, get_visual_data, COLOR_FROM_NAME, YELLOW

from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo
from pddlstream.language.constants import And, print_solution, PDDLProblem, Equal
from pddlstream.language.generator import from_test, from_fn, negate_test
from pddlstream.utils import read, INF, get_file_path, randomize, inclusive_range


TOP_LINK = 'plate_top_link'
WEIGHTS = np.array([1, 1, 0])  # 1 / BASE_RESOLUTIONS
DIST_PER_TIME = 0.25 # meters / sec
MAX_ENERGY = 100 # percent
CHARGE_PER_TIME = 20 # percent / sec
#BURN_PER_TIME = 25 # percent / sec
BURN_PER_TIME = 0
#INITIAL_ENERGY = 0
INITIAL_ENERGY = MAX_ENERGY

def set_body_color(body, color):
    for link in get_all_links(body):
        set_color(body, color, link)

def get_turtle_traj_aabb(traj):
    if not hasattr(traj, 'aabb'):
        if isinstance(traj, Conf):
            traj.assign()
            traj.aabb = get_turtle_aabb(traj.body)
        else:
            traj.aabb = aabb_union(map(get_turtle_traj_aabb, traj.iterate()))
    return traj.aabb

def linear_trajectory(conf1, conf2):
    robot = conf1.body
    joints = conf1.joints
    extend_fn = get_extend_fn(robot, joints, resolutions=BASE_RESOLUTIONS)
    path = [conf1.values] + list(extend_fn(conf1.values, conf2.values))
    return create_trajectory(robot, joints, path)

def get_test_cfree_traj_traj(problem):
    if len(problem.robots) == 0:
        robot1, robot2 = None, None
    elif len(problem.robots) == 1:
        robot1, robot2 = problem.get_body(problem.robots[0]), None
    else:
        robot1, robot2 = list(map(problem.get_body, problem.robots))

    def test(traj1, traj2):
        if not problem.collisions:
            return True
        if (robot1 is None) or (robot2 is None):
            return True
        if traj1 is traj2:
            return False
        if not aabb_overlap(get_turtle_traj_aabb(traj1), get_turtle_traj_aabb(traj2)):
            return True
        # TODO: could also use radius to prune
        # TODO: prune if start and end conf collide

        for conf1 in randomize(traj1.iterate()):
            if not aabb_overlap(get_turtle_traj_aabb(conf1), get_turtle_traj_aabb(traj2)):
                continue
            set_base_conf(robot1, conf1.values)
            for conf2 in randomize(traj2.iterate()):
                if not aabb_overlap(get_turtle_traj_aabb(conf1), get_turtle_traj_aabb(conf2)):
                    continue
                set_base_conf(robot2, conf2.values)
                if pairwise_collision(robot1, robot2):
                    return False
        return True
    return test

def get_motion_fn2(problem):
    robot = list(map(problem.get_body, problem.initial_confs))[0]
    motion_fn = get_motion_fn(problem, max_distance=0.45, weights=WEIGHTS, resolutions=BASE_RESOLUTIONS)
    def fn(q1, q2):
        return motion_fn(robot, q1, q2)
    return fn

def get_distance_fn():
    distance_fn = get_base_distance_fn(weights=WEIGHTS)
    # TODO: encode different joint speeds

    def fn(traj):
        #return 1
        return traj.distance(distance_fn)
    return fn

#######################################################

def draw_edges(edges):
    vertices = {v for edge in edges for v in edge}
    handles = []
    for conf in vertices:
        handles.extend(draw_point(point_from_conf(conf.values), size=0.05))
    for conf1, conf2 in edges:
        handles.append(add_line(point_from_conf(conf1.values),
                                point_from_conf(conf2.values)))
    return handles

def pddlstream_from_problem(problem, teleport=False):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {'{}'.format(name).lower(): name
                    for name in problem.initial_confs.keys()}

    edges = set()
    init = [
    ]
    for name, conf in problem.initial_confs.items():
        init += [
            ('Safe',),
            ('Robot', name),
            ('Conf', conf),
            ('AtConf', name, conf),
            Equal(('Speed', name), DIST_PER_TIME),
            Equal(('BatteryCapacity', name), MAX_ENERGY),
            Equal(('RechargeRate', name), CHARGE_PER_TIME),
            Equal(('ConsumptionRate', name), BURN_PER_TIME),
            Equal(('Energy', name), INITIAL_ENERGY),
        ]

    goal_literals = [
        #('Safe',),
        #('Unachievable',),
    ]

    for name, base_values in problem.goal_confs.items():
        body = problem.get_body(name)
        joints = get_base_joints(body)
        #extend_fn = get_extend_fn(body, joints, resolutions=5*BASE_RESOLUTIONS)
        #q_init = problem.initial_confs[name]
        #path = [q_init] + [Conf(body, joints, q) for q in extend_fn(q_init.values, base_values)]
        #edges.update(zip(path, path[1:]))
        #q_goal = path[-1]
        q_goal = Conf(body, joints, base_values)
        init += [('Conf', q_goal)]
        goal_literals += [('AtConf', name, q_goal)]
    goal_formula = And(*goal_literals)

    robot = list(map(problem.get_body, problem.initial_confs))[0]
    with LockRenderer():
        init += [('Conf', q) for _, n, q in create_vertices(problem, robot, [], samples_per_ft2=6)]

    #vertices = {v for edge in edges for v in edge}
    #handles = []
    #for vertex in vertices:
    #    handles.extend(draw_point(point_from_conf(vertex.values), size=0.05))

    #for conf1, conf2 in edges:
    #    traj = linear_trajectory(conf1, conf2)
    #    init += [
    #        ('Conf', conf1),
    #        ('Traj', traj),
    #        ('Conf', conf2),
    #        ('Motion', conf1, traj, conf2),
    #    ]
    #draw_edges(edges)

    cfree_traj_traj_test = get_test_cfree_traj_traj(problem)
    cfree_traj_traj_list_gen_fn = from_test(cfree_traj_traj_test)
    traj_traj_collision_test = negate_test(cfree_traj_traj_test)
    stream_map = {
        'test-cfree-conf-conf': cfree_traj_traj_list_gen_fn,
        'test-cfree-traj-conf': cfree_traj_traj_list_gen_fn,
        'test-cfree-traj-traj': cfree_traj_traj_list_gen_fn,
        'ConfConfCollision': traj_traj_collision_test,
        'TrajConfCollision': traj_traj_collision_test,
        'TrajTrajCollision': traj_traj_collision_test,
        'compute-motion': from_fn(get_motion_fn2(problem)),
        'TrajDistance': get_distance_fn(),
    }
    #stream_map = 'debug'

    problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)

    return problem, edges

#######################################################

class NAMOProblem(object):
    def __init__(self, body_from_name, robots, limits, collisions=True, goal_confs={}):
        self.body_from_name = dict(body_from_name)
        self.robots = tuple(robots)
        self.limits = limits
        self.collisions = collisions
        self.goal_confs = dict(goal_confs)

        robot_bodies = set(map(self.get_body, self.robots))
        self.obstacles = tuple(body for body in get_bodies()
                               if body not in robot_bodies)
        self.custom_limits = {}
        if self.limits is not None:
            for body in robot_bodies:
                self.custom_limits.update(get_custom_limits(body, self.limits, yaw_limit=(0, 0)))
        self.initial_confs = {name: Conf(self.get_body(name), get_base_joints(self.get_body(name)))
                              for name in self.robots}
    def get_body(self, name):
        return self.body_from_name[name]


def create_environment(base_extent, mound_height):
    # TODO: reuse this instead of making a new method
    floor = create_box(base_extent, base_extent, 0.001, color=TAN)  # TODO: two rooms
    set_point(floor, Point(z=-0.001 / 2.))
    #return floor, []

    wall1 = create_box(base_extent + mound_height, mound_height, mound_height, color=GREY)
    set_point(wall1, Point(y=base_extent / 2., z=mound_height / 2.))
    wall2 = create_box(base_extent + mound_height, mound_height, mound_height, color=GREY)
    set_point(wall2, Point(y=-base_extent / 2., z=mound_height / 2.))
    wall3 = create_box(mound_height, base_extent + mound_height, mound_height, color=GREY)
    set_point(wall3, Point(x=base_extent / 2., z=mound_height / 2.))
    wall4 = create_box(mound_height, base_extent + mound_height, mound_height, color=GREY)
    set_point(wall4, Point(x=-base_extent / 2., z=mound_height / 2.))
    return floor, [wall1, wall2, wall3, wall4]

def problem_fn(n_robots=2, collisions=True):
    base_extent = 2.0
    base_limits = (-base_extent / 2. * np.ones(2), base_extent / 2. * np.ones(2))
    mound_height = 0.1

    floor, walls = create_environment(base_extent, mound_height)
    width = base_extent / 2. - 4*mound_height
    wall5 = create_box(mound_height, width, mound_height, color=GREY)
    set_point(wall5, Point(y=-(base_extent/2 - width / 2.), z=mound_height / 2.))
    wall6 = create_box(mound_height, width, mound_height, color=GREY)
    set_point(wall6, Point(y=+(base_extent/2 - width / 2.), z=mound_height / 2.))

    distance = 0.5
    #initial_confs = [(-distance, -distance, 0),
    #                 (-distance, +distance, 0)]
    initial_confs = [(-distance, -distance, 0),
                     (+distance, +distance, 0)]
    assert n_robots <= len(initial_confs)

    body_from_name = {}
    #robots = ['green']
    robots = ['green', 'blue']
    with LockRenderer():
        for i, name in enumerate(robots):
            with HideOutput():
                body = load_model(TURTLEBOT_URDF) # TURTLEBOT_URDF | ROOMBA_URDF
            body_from_name[name] = body
            robot_z = stable_z(body, floor)
            set_point(body, Point(z=robot_z))
            set_base_conf(body, initial_confs[i])
            set_body_color(body, COLOR_FROM_NAME[name])

    goals = [(+distance, -distance, 0),
             (+distance, +distance, 0)]
    #goals = goals[::-1]
    goals = initial_confs[::-1]
    goal_confs = dict(zip(robots, goals))

    return NAMOProblem(body_from_name, robots, base_limits, collisions=collisions, goal_confs=goal_confs)

#######################################################

def display_plan(problem, state, plan, time_step=0.01, sec_per_step=0.005):
    duration = compute_duration(plan)
    real_time = None if sec_per_step is None else (duration * sec_per_step / time_step)
    print('Duration: {} | Step size: {} | Real time: {}'.format(duration, time_step, real_time))
    colors = {robot: COLOR_FROM_NAME[robot] for robot in problem.robots}
    for i, t in enumerate(inclusive_range(0, duration, time_step)):
        print('Step={} | t={}'.format(i, t))
        for action in plan:
            name, args, start, duration = action
            if not (action.start <= t <= get_end(action)):
                continue
            if name == 'move':
                robot, conf1, traj, conf2 = args
                traj = [conf1.values, conf2.values]
                fraction = (t - action.start) / action.duration
                conf = get_value_at_time(traj, fraction)
                body = problem.get_body(robot)
                color = COLOR_FROM_NAME[robot]
                if colors[robot] != color:
                    set_color(body, color, link_from_name(body, TOP_LINK))
                    colors[robot] = color
                set_base_conf(body, conf)
            elif name == 'recharge':
                robot, conf = args
                body = problem.get_body(robot)
                color = YELLOW
                if colors[robot] != color:
                    set_color(body, color, link_from_name(body, TOP_LINK))
                    colors[robot] = color
            else:
                raise ValueError(name)
        if sec_per_step is None:
            user_input('Continue?')
        else:
            wait_for_duration(sec_per_step)

#######################################################

def main(display=True, teleport=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-algorithm', default='incremental', help='Specifies the algorithm')
    parser.add_argument('-cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-t', '--max_time', default=5*60, type=int, help='The max time')
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

    pddlstream, edges = pddlstream_from_problem(problem, teleport=teleport)
    _, constant_map, _, stream_map, init, goal = pddlstream
    print('Constants:', constant_map)
    print('Init:', init)
    print('Goal:', goal)

    success_cost = 0 if args.optimal else INF
    max_planner_time = 10

    stream_info = {
        'compute-motion': StreamInfo(eager=True, p_success=0),
        'ConfConfCollision': FunctionInfo(p_success=1, overhead=0.1),
        'TrajConfCollision': FunctionInfo(p_success=1, overhead=1),
        'TrajTrajCollision': FunctionInfo(p_success=1, overhead=10),
        'TrajDistance': FunctionInfo(eager=True), # Need to eagerly evaluate otherwise 0 duration (failure)
    }

    pr = cProfile.Profile()
    pr.enable()
    with LockRenderer(False):
        if args.algorithm == 'incremental':
            solution = solve_incremental(pddlstream,
                                         max_planner_time=max_planner_time,
                                         success_cost=success_cost, max_time=args.max_time,
                                         start_complexity=INF,
                                         verbose=True, debug=True)
        elif args.algorithm == 'focused':
            solution = solve_focused(pddlstream, stream_info=stream_info,
                                      max_planner_time=max_planner_time,
                                      success_cost=success_cost, max_time=args.max_time,
                                      max_skeletons=None, bind=True, max_failures=INF,
                                      verbose=True, debug=True)
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

    if not args.viewer:
        disconnect()
        connect(use_gui=True)
        with LockRenderer():
            with HideOutput():
                problem_fn() # TODO: way of doing this without reloading?
    saver.restore() # Assumes bodies are ordered the same way
    draw_edges(edges)

    state = BeliefState(problem)
    wait_for_user()
    #time_step = None if teleport else 0.01
    #with VideoSaver('video.mp4'):
    display_plan(problem, state, plan)
    wait_for_user()
    disconnect()

if __name__ == '__main__':
    main()
