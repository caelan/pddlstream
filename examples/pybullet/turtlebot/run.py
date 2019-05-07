#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import pstats
import numpy as np

from examples.continuous_tamp.primitives import get_value_at_time
from examples.continuous_tamp.run import compute_duration, inclusive_range, get_end
from examples.pybullet.namo.run import get_base_joints, set_base_conf, get_custom_limits, \
    point_from_conf, BASE_RESOLUTIONS, get_turtle_aabb
from examples.pybullet.pr2_belief.problems import BeliefState
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Conf, create_trajectory
from examples.pybullet.utils.pybullet_tools.utils import connect, disconnect, draw_base_limits, WorldSaver, \
    wait_for_user, LockRenderer, get_bodies, add_line, create_box, stable_z, load_model, TURTLEBOT_URDF, \
    HideOutput, GREY, TAN, RED, get_extend_fn, pairwise_collision, draw_point, VideoSaver, wait_for_interrupt, \
    set_point, Point, GREEN, BLUE, set_color, get_all_links, wait_for_duration, user_input, \
    aabb_union, draw_aabb, aabb_overlap, remove_all_debug, get_base_distance_fn
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import And, print_solution, PDDLProblem
from pddlstream.utils import read, INF, get_file_path
from pddlstream.language.generator import from_test

# TODO: Kiva robot and Amazon shelves

def get_turtle_traj_aabb(traj):
    aabbs = []
    for conf in traj.iterate():
        conf.assign()
        aabbs.append(get_turtle_aabb(conf.body))
    return aabb_union(aabbs)

def linear_trajectory(conf1, conf2):
    robot = conf1.body
    joints = conf1.joints
    extend_fn = get_extend_fn(robot, joints, resolutions=BASE_RESOLUTIONS)
    path = [conf1.values] + list(extend_fn(conf1.values, conf2.values))
    return create_trajectory(robot, joints, path)

def get_test_cfree_traj_traj(problem, collisions=True):
    robot1, robot2 = list(map(problem.get_body, problem.initial_confs))
    aabb_from_traj = {}

    def test(traj1, traj2):
        if not collisions:
            return True
        if traj1 not in aabb_from_traj:
            aabb_from_traj[traj1] = get_turtle_traj_aabb(traj1)
        if traj2 not in aabb_from_traj:
            aabb_from_traj[traj2] = get_turtle_traj_aabb(traj2)
        if not aabb_overlap(aabb_from_traj[traj1], aabb_from_traj[traj2]):
            return True

        for q1 in traj1.iterate():
            set_base_conf(robot1, q1.values)
            for q2 in traj2.iterate():
                set_base_conf(robot2, q2.values)
                if pairwise_collision(robot1, robot2):
                    return False
        return True
    return test

def get_duration_fn(dist_per_time=0.25):
    weights = np.array([1, 1, 0]) # 1 / BASE_RESOLUTIONS
    distance_fn = get_base_distance_fn(weights=weights)

    def fn(traj):
        # TODO: encode different joint speeds
        #return 1
        distance = traj.distance(distance_fn)
        duration = distance / dist_per_time
        return duration
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
    init = []
    for name, conf in problem.initial_confs.items():
        init += [
            ('Safe',),
            ('Robot', name),
            ('Conf', conf),
            ('AtConf', name, conf)
        ]

    goal_literals = [
        #('Safe',),
        #('Unachievable',),
    ]

    for name, base_values in problem.goal_confs.items():
        body = problem.get_body(name)
        joints = get_base_joints(body)
        extend_fn = get_extend_fn(body, joints, resolutions=5*BASE_RESOLUTIONS)
        q_init = problem.initial_confs[name]
        path = [q_init] + [Conf(body, joints, q) for q in extend_fn(q_init.values, base_values)]
        edges.update(zip(path, path[1:]))
        q_goal = path[-1]
        init += [('Conf', q_goal)]
        goal_literals += [('AtConf', name, q_goal)]
    goal_formula = And(*goal_literals)

    vertices = {v for edge in edges for v in edge}
    handles = []
    for vertex in vertices:
        handles.extend(draw_point(point_from_conf(vertex.values), size=0.05))

    for conf1, conf2 in edges:
        traj = linear_trajectory(conf1, conf2)
        init += [
            ('Conf', conf1),
            ('Traj', traj),
            ('Conf', conf2),
            ('Motion', conf1, traj, conf2),
        ]
    draw_edges(edges)

    test_cfree_traj_traj = from_test(get_test_cfree_traj_traj(problem))
    stream_map = {
        'test-cfree-conf-conf': test_cfree_traj_traj,
        'test-cfree-traj-conf': test_cfree_traj_traj,
        'test-cfree-traj-traj': test_cfree_traj_traj,
        #'compute-motion': from_fn(get_motion_fn(problem)),
        'Duration': get_duration_fn(),
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
                self.custom_limits.update(get_custom_limits(body, self.limits))
        self.initial_confs = {name: Conf(self.get_body(name), get_base_joints(self.get_body(name)))
                              for name in self.robots}
    def get_body(self, name):
        return self.body_from_name[name]


def create_environment(base_extent, mound_height):
    # TODO: reuse this instead of making a new method
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
    return floor, [wall1, wall2, wall3, wall4]

def problem_fn(n_robots=2, collisions=True):
    base_extent = 2.5
    base_limits = (-base_extent / 2. * np.ones(2), base_extent / 2. * np.ones(2))
    mound_height = 0.1
    floor, walls = create_environment(base_extent, mound_height)

    distance = 0.75
    initial_confs = [(-distance, -distance, 0),
                     (-distance, +distance, 0)]
    assert n_robots <= len(initial_confs)

    colors = [GREEN, BLUE]

    body_from_name = {}
    robots = []
    with LockRenderer():
        for i in range(n_robots):
            name = 'r{}'.format(i)
            robots.append(name)
            with HideOutput():
                body = load_model(TURTLEBOT_URDF) # TURTLEBOT_URDF | ROOMBA_URDF
            body_from_name[name] = body
            robot_z = stable_z(body, floor)
            set_point(body, Point(z=robot_z))
            set_base_conf(body, initial_confs[i])
            for link in get_all_links(body):
                set_color(body, colors[i], link)

    goals = [(+distance, -distance, 0),
             (+distance, +distance, 0)]
    goals = goals[::-1]
    goal_confs = dict(zip(robots, goals))

    return NAMOProblem(body_from_name, robots, base_limits, collisions=collisions, goal_confs=goal_confs)

#######################################################

def display_plan(problem, state, plan, time_step=0.01, sec_per_step=0.005):
    duration = compute_duration(plan)
    real_time = None if sec_per_step is None else (duration * sec_per_step / time_step)
    print('Duration: {} | Step size: {} | Real time: {}'.format(duration, time_step, real_time))
    for t in inclusive_range(0, duration, time_step):
        for action in plan:
            name, args, start, duration = action
            if not (action.start <= t <= get_end(action)):
                continue
            if name == 'move':
                robot, conf1, traj, conf2 = args
                traj = [conf1.values, conf2.values]
                fraction = (t - action.start) / action.duration
                conf = get_value_at_time(traj, fraction)
                set_base_conf(problem.get_body(robot), conf)
            else:
                raise ValueError(name)
        if sec_per_step is None:
            user_input('Continue?')
        else:
            wait_for_duration(sec_per_step)

#######################################################

def main(display=True, teleport=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-t', '--max_time', default=120, type=int, help='The max time')
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

    pr = cProfile.Profile()
    pr.enable()
    with LockRenderer(False):
        solution = solve_incremental(pddlstream,
                                     max_planner_time=max_planner_time,
                                     success_cost=success_cost,
                                     #start_complexity=INF,
                                     max_time=args.max_time,
                                     verbose=True, debug=True)

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
