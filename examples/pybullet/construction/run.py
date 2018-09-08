from __future__ import print_function

import cProfile
import json
import os
import pstats

import numpy as np

from examples.pybullet.construction.debug import get_test_cfree
from examples.pybullet.construction.utils import parse_elements, parse_node_points, parse_ground_nodes, create_elements, \
    JSON_FILENAME, TOOL_NAME, disabled_collisions, check_trajectory_collision, get_grasp_pose, load_world, \
    prune_dominated, get_element_neighbors
from examples.pybullet.utils.pybullet_tools.utils import connect, dump_body, disconnect, wait_for_interrupt, \
    get_movable_joints, get_sample_fn, set_joint_positions, link_from_name, add_line, inverse_kinematics, \
    get_link_pose, multiply, wait_for_duration, \
    get_pose, invert, point_from_pose, get_distance, get_joint_positions, wrap_angle, get_collision_fn, \
    set_color, has_gui
from pddlstream.algorithms.incremental import solve_exhaustive
from pddlstream.language.constants import PDDLProblem, And
from pddlstream.language.generator import from_test
from pddlstream.utils import read, get_file_path, print_solution, user_input


class PrintTrajectory(object):
    def __init__(self, robot, trajectory, element, colliding=set()):
        self.robot = robot
        self.trajectory = trajectory
        self.element = element
        self.colliding = colliding
    def __repr__(self):
        return 't{}'.format(self.element)


def get_pddlstream(trajectories, element_bodies, ground_nodes):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}

    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    stream_map = {
        'test-cfree': from_test(get_test_cfree(element_bodies)),
    }

    init = []
    for n in ground_nodes:
        init.append(('Connected', n))
    for t in trajectories:
        e = t.element
        n1, n2 = e
        init.extend([
            ('Node', n1),
            ('Node', n2),
            ('Element', e),
            ('Traj', t),
            ('Connection', n1, e, t, n2),
            ('Connection', n2, e, t, n1),
        ])

    goal_literals = [('Printed', e) for e in element_bodies]
    goal = And(*goal_literals)
    # TODO: weight or order these in some way

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


def plan_sequence(trajectories, element_bodies, ground_nodes):
    if trajectories is None:
        return None
    # randomize_successors=True
    pr = cProfile.Profile()
    pr.enable()
    pddlstream_problem = get_pddlstream(trajectories, element_bodies, ground_nodes)
    solution = solve_exhaustive(pddlstream_problem, planner='goal-lazy', max_time=300, debug=True)
    #solution = solve_exhaustive(pddlstream_problem, planner='ff-lazy', max_time=300, debug=True)
    # Reachability heuristics good for detecting dead-ends
    # Infeasibility from the start means disconnected or

    # Random restart based strategy here
    print_solution(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    plan, _, _ = solution
    if plan is None:
        return None
    return [t for _, (n1, e, t, n2) in plan]


def optimize_angle(robot, link, element_pose, translation, direction, initial_angles, collision_fn, max_error=1e-2):
    movable_joints = get_movable_joints(robot)
    initial_conf = get_joint_positions(robot, movable_joints)
    best_error, best_angle, best_conf = max_error, None, None
    for i, angle in enumerate(initial_angles):
        grasp_pose = get_grasp_pose(translation, direction, angle)
        target_pose = multiply(element_pose, invert(grasp_pose))
        conf = inverse_kinematics(robot, link, target_pose)
        # if conf is None:
        #    continue
        #if pairwise_collision(robot, robot):
        conf = get_joint_positions(robot, movable_joints)
        if not collision_fn(conf):
            link_pose = get_link_pose(robot, link)
            error = get_distance(point_from_pose(target_pose), point_from_pose(link_pose))
            if error < best_error:  # TODO: error a function of direction as well
                best_error, best_angle, best_conf = error, angle, conf
            # wait_for_interrupt()
        if i != len(initial_angles)-1:
            set_joint_positions(robot, movable_joints, initial_conf)
    #print(best_error, translation, direction, best_angle)
    if best_conf is not None:
        set_joint_positions(robot, movable_joints, best_conf)
        #wait_for_interrupt()
    return best_angle, best_conf


def compute_direction_path(robot, p1, p2, element_body, direction, collision_fn):
    step_size = 0.0025 # 0.005
    #angle_step_size = np.pi / 128
    angle_step_size = np.math.radians(0.25)
    angle_deltas = [-angle_step_size, 0, angle_step_size]
    #num_initial = 12
    num_initial = 1

    length = np.linalg.norm(p2 - p1) # 5cm
    start, end = -length / 2, length / 2
    steps = np.append(np.arange(start, end, step_size), [end])
    #print('Length: {} | Steps: {}'.format(length, len(steps)))

    #initial_angles = [wrap_angle(angle) for angle in np.linspace(0, 2*np.pi, num_initial, endpoint=False)]
    initial_angles = [wrap_angle(angle) for angle in np.random.uniform(0, 2*np.pi, num_initial)]
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)
    set_joint_positions(robot, movable_joints, sample_fn())
    link = link_from_name(robot, TOOL_NAME)
    element_pose = get_pose(element_body)
    current_angle, current_conf = optimize_angle(robot, link, element_pose,
                                                 steps[0], direction, initial_angles, collision_fn)
    if current_conf is None:
        return None
    # TODO: constrain maximum conf displacement
    # TODO: alternating minimization for just position and also orientation
    trajectory = [current_conf]
    for translation in steps[1:]:
        #set_joint_positions(robot, movable_joints, current_conf)
        initial_angles = [wrap_angle(current_angle + delta) for delta in angle_deltas]
        current_angle, current_conf = optimize_angle(robot, link, element_pose,
                                                     translation, direction, initial_angles, collision_fn)
        if current_conf is None:
            return None
        trajectory.append(current_conf)
    return trajectory


def sample_print_path(robot, node_points, element, element_body, collision_fn):
    #max_directions = 10
    #max_directions = 16
    max_directions = 1
    #for direction in np.linspace(0, 2*np.pi, 10, endpoint=False):
    for direction in np.random.uniform(0, 2*np.pi, max_directions):
        n1, n2 = element
        trajectory = compute_direction_path(robot, node_points[n1], node_points[n2],
                                            element_body, direction, collision_fn)
        if trajectory is not None:
            return trajectory
    return None


def sample_trajectories(robot, obstacles, node_points, element_bodies, ground_nodes, num_trajs=100):
    disabled = {tuple(link_from_name(robot, link) for link in pair) for pair in disabled_collisions}
    collision_fn = get_collision_fn(robot, get_movable_joints(robot), obstacles, [],
                                    self_collisions=True, disabled_collisions=disabled, use_limits=False)

    # TODO: can cache which elements collide and prune dominated
    element_neighbors = get_element_neighbors(element_bodies)
    elements_order = list(element_bodies.keys())
    bodies_order = [element_bodies[e] for e in elements_order]
    #max_trajs = 5
    max_trajs = np.inf

    all_trajectories = []
    for index, (element, element_body) in enumerate(element_bodies.items()):
        # TODO: prune elements that collide with all their neighbors
        trajectories = []
        for i in range(num_trajs):
            trajectory = sample_print_path(robot, node_points, element, element_body, collision_fn)
            if trajectory is None:
                continue
            collisions = check_trajectory_collision(robot, trajectory, bodies_order)
            colliding = {e for k, e in enumerate(elements_order) if (element != e) and collisions[k]}
            #colliding = {e for e, body in element_bodies.items()
            #             if check_trajectory_collision(robot, trajectory, [body])}
            if (element_neighbors[element] <= colliding) and not any(n in ground_nodes for n in element):
                continue
            traj = PrintTrajectory(robot, trajectory, element, colliding)
            trajectories.append(traj) # TODO: make more if many collisions in particular
            #prune_dominated(trajectories)

            if has_gui():
                for e, body in element_bodies.items():
                    if e == element:
                        set_color(body, (0, 1, 0, 1))
                    elif e in colliding:
                        set_color(body, (1, 0, 0, 1))
                    else:
                        set_color(body, (1, 0, 0, 0))
                wait_for_interrupt()
            if not colliding or (max_trajs <= len(trajectories)):
                break

        prune_dominated(trajectories)
        print(index, len(trajectories), sorted([len(t.colliding) for t in trajectories]))
        all_trajectories.extend(trajectories)
        if not trajectories:
            if has_gui():
                for e, body in element_bodies.items():
                    if e == element:
                        set_color(body, (0, 1, 0, 1))
                    elif e in element_neighbors[e]:
                        set_color(body, (1, 0, 0, 1))
                    else:
                        set_color(body, (1, 0, 0, 0))
                wait_for_interrupt()
            return None
    return all_trajectories

def display(trajectories):
    connect(use_gui=True)
    floor, robot = load_world()
    wait_for_interrupt()
    movable_joints = get_movable_joints(robot)
    #element_bodies = dict(zip(elements, create_elements(node_points, elements)))
    #for body in element_bodies.values():
    #    set_color(body, (1, 0, 0, 0))
    for trajectory in trajectories:
        print(trajectory.element, len(trajectory.trajectory))
        #wait_for_interrupt()
        #set_color(element_bodies[element], (1, 0, 0, 1))
        last_point = None
        for conf in trajectory.trajectory:
            set_joint_positions(robot, movable_joints, conf)
            current_point = point_from_pose(get_link_pose(robot, link_from_name(robot, TOOL_NAME)))
            if last_point is not None:
                handle = add_line(last_point, current_point, color=(1, 0, 0))
            last_point = current_point
            wait_for_duration(0.025)
    #user_input('Finish?')
    wait_for_interrupt()
    disconnect()


def main(viewer=False):
    root_directory = os.path.dirname(os.path.abspath(__file__))
    #read_minizinc_data(os.path.join(root_directory, 'print_data.txt'))
    #return

    with open(os.path.join(root_directory, JSON_FILENAME), 'r') as f:
        json_data = json.loads(f.read())

    elements = parse_elements(json_data)
    node_points = parse_node_points(json_data)
    ground_nodes = parse_ground_nodes(json_data)

    node_order = list(range(len(node_points)))
    #np.random.shuffle(node_order)
    node_order = sorted(node_order, key=lambda n: node_points[n][2])
    elements = sorted(elements, key=lambda e: min(node_points[n][2] for n in e))

    #node_order = node_order[:100]
    ground_nodes = [n for n in ground_nodes if n in node_order]
    elements = [element for element in elements if all(n in node_order for n in element)]
    elements = elements[:25]
    #elements = elements[:50]
    #elements = elements[:100]
    #elements = elements[:150]
    #elements = elements[150:]

    print('Nodes: {} | Ground: {} | Elements: {}'.format(
        len(node_points), len(ground_nodes), len(elements)))
    #print(json_data['assembly_type']) # extrusion
    #print(json_data['model_type']) # spatial_frame
    #print(json_data['unit']) # millimeter
    #plan = plan_sequence_test(node_points, elements, ground_nodes)

    connect(use_gui=viewer)
    floor, robot = load_world()
    dump_body(robot)

    #for element in elements:
    #    draw_element(node_points, element)
    #    #user_input('Continue?')
    #for name, args in plan:
    #    n1, e, n2 = args
    #    draw_element(node_points, e)
    #    user_input('Continue?')
    #test_ik(robot, node_order, node_points)

    element_bodies = dict(zip(elements, create_elements(node_points, elements)))
    pr = cProfile.Profile()
    pr.enable()
    trajectories = sample_trajectories(robot, [floor], node_points, element_bodies, ground_nodes)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    user_input('Continue?')

    plan = plan_sequence(trajectories, element_bodies, ground_nodes)
    disconnect()

    #display(trajectories)
    if plan is not None:
        display(plan)

    # Collisions at the end points?


if __name__ == '__main__':
    main()