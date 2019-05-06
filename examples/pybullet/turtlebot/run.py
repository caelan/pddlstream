#!/usr/bin/env python

from __future__ import print_function

import argparse
import cProfile
import pstats
import numpy as np

from itertools import product

from examples.continuous_tamp.primitives import get_value_at_time
from examples.continuous_tamp.run import compute_duration, inclusive_range
from examples.pybullet.namo.run import get_base_joints, set_base_conf, get_custom_limits, \
    point_from_conf, BASE_RESOLUTIONS
from examples.pybullet.pr2_belief.problems import BeliefState
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Conf
from examples.pybullet.utils.pybullet_tools.utils import connect, disconnect, draw_base_limits, WorldSaver, \
    wait_for_user, LockRenderer, get_bodies, add_line, create_box, stable_z, load_model, TURTLEBOT_URDF, \
    HideOutput, GREY, TAN, RED, get_extend_fn, pairwise_collision, \
    set_point, Point, GREEN, BLUE, set_color, get_all_links, wait_for_duration, user_input
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import And, print_solution, PDDLProblem
from pddlstream.utils import read, INF, get_file_path
from pddlstream.language.generator import from_test

# TODO: Kiva robot and Amazon shelves

def get_test_cfree_traj_traj(problem):
    robot1, robot2 = list(problem.initial_confs)
    joints = get_base_joints(robot1)
    extend_fn = get_extend_fn(robot1, joints, resolutions=BASE_RESOLUTIONS)

    def test(traj1, traj2):
        start1, end1 = traj1
        path1 = [start1] + list(extend_fn(start1, end1))
        start2, end2 = traj2
        path2 = [start2] + list(extend_fn(start2, end2))
        for q1, q2 in product(path1, path2):
            set_base_conf(robot1, q1)
            set_base_conf(robot2, q2)
            if pairwise_collision(robot1, robot2):
                return False
        return True
    return test

#######################################################

def pddlstream_from_problem(problem, teleport=False):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {'{}'.format(name).lower(): name
                    for name in problem.initial_confs.keys()}
    edges = []
    init = []
    for robot, conf in problem.initial_confs.items():
        init += [
            ('Safe',),
            ('Robot', robot),
            ('Conf', conf),
            ('AtConf', robot, conf)
        ]

    goal_literals = [
        ('Safe',),
        #('Unachievable',),
    ]
    for robot, base_values in problem.goal_confs.items():
        q_goal = Conf(robot, get_base_joints(robot), base_values)
        edges.append((problem.initial_confs[robot], q_goal))
        init += [('Conf', robot, q_goal)]
        goal_literals += [('AtConf', robot, q_goal)]
    goal_formula = And(*goal_literals)

    handles = []
    for conf1, conf2 in edges:
        traj = [conf1.values, conf2.values]
        init += [
            ('Conf', conf1),
            ('Traj', traj),
            ('Conf', conf2),
            ('Motion', conf1, traj, conf2),
        ]
        handles.append(add_line(point_from_conf(conf1.values),
                                point_from_conf(conf2.values)))

    stream_map = {
        'test-cfree-traj-traj': from_test(get_test_cfree_traj_traj(problem)),
        #'test-cfree-traj-pose': from_test(get_test_cfree_traj_pose(problem)),
        #'compute-motion': from_fn(get_motion_fn(problem)),
        #'Cost': get_cost_fn(problem),
    }
    #stream_map = 'debug'

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)

#######################################################

class NAMOProblem(object):
    def __init__(self, robots, limits, collisions=True, goal_confs={}):
        self.robots = tuple(robots)
        self.limits = limits
        self.collisions = collisions
        self.goal_confs = goal_confs
        self.obstacles = tuple(body for body in get_bodies() if body not in self.robots)
        self.custom_limits = {}
        if self.limits is not None:
            for robot in self.robots:
                self.custom_limits.update(get_custom_limits(robot, self.limits))
        self.initial_confs = {robot: Conf(robot, get_base_joints(robot)) for robot in self.robots}


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
    initial_confs = [(-distance, -distance, 0), (-distance, +distance, 0)]
    assert n_robots <= len(initial_confs)

    colors = [GREEN, BLUE]

    robots = []
    with LockRenderer():
        for i in range(n_robots):
            with HideOutput():
                rover = load_model(TURTLEBOT_URDF) # TURTLEBOT_URDF | ROOMBA_URDF
            robot_z = stable_z(rover, floor)
            set_point(rover, Point(z=robot_z))
            set_base_conf(rover, initial_confs[i])
            robots.append(rover)
            for link in get_all_links(rover):
                set_color(rover, colors[i], link)

    goals = [(+distance, -distance, 0), (+distance, +distance, 0)]
    goals = goals[::-1]
    goal_confs = dict(zip(robots, goals))

    return NAMOProblem(robots, base_limits, collisions=collisions, goal_confs=goal_confs)

#######################################################

def display_plan(state, plan, time_step=0.01, sec_per_step=0.005):
    duration = compute_duration(plan)
    real_time = None if sec_per_step is None else (duration * sec_per_step / time_step)
    print('Duration: {} | Step size: {} | Real time: {}'.format(duration, time_step, real_time))
    for t in inclusive_range(0, duration, time_step):
        for action in plan:
            name, args, start, duration = action
            if name == 'move':
                robot, conf1, traj, conf2 = args
                traj = [conf1.values, conf2.values]
                fraction = (t - action.start) / action.duration
                conf = get_value_at_time(traj, fraction)
                set_base_conf(robot, conf)
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
    parser.add_argument('-unit', action='store_true', help='Uses unit costs')
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
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)

    success_cost = 0 if args.optimal else INF
    max_planner_time = 10

    pr = cProfile.Profile()
    pr.enable()
    with LockRenderer(True):
        solution = solve_incremental(pddlstream_problem,
                                     max_planner_time=max_planner_time,
                                     unit_costs=args.unit, success_cost=success_cost,
                                     start_complexity=INF,
                                     max_time=args.max_time, verbose=True, debug=True)

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

    wait_for_user()
    #time_step = None if teleport else 0.01
    state = BeliefState(problem)
    display_plan(state, plan)
    wait_for_user()
    disconnect()

if __name__ == '__main__':
    main()
