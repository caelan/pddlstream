from __future__ import print_function

import cProfile
import pstats
import numpy as np

from examples.pybullet.construction.debug import get_test_cfree, test_grasps, test_print
from examples.pybullet.construction.utils import parse_elements, parse_node_points, parse_ground_nodes, create_elements, \
    load_extrusion, TOOL_NAME, disabled_collisions, check_trajectory_collision, get_grasp_pose, load_world, \
    prune_dominated, get_element_neighbors, sample_direction, draw_element
from examples.pybullet.utils.pybullet_tools.utils import connect, dump_body, disconnect, wait_for_interrupt, \
    get_movable_joints, get_sample_fn, set_joint_positions, link_from_name, add_line, inverse_kinematics, \
    get_link_pose, multiply, wait_for_duration, set_color, has_gui, \
    get_pose, invert, point_from_pose, get_distance, get_joint_positions, wrap_angle, get_collision_fn
from pddlstream.algorithms.incremental import solve_exhaustive
from pddlstream.language.constants import PDDLProblem, And
from pddlstream.language.generator import from_test
from pddlstream.utils import read, get_file_path, print_solution, user_input


class PrintTrajectory(object):
    def __init__(self, robot, trajectory, element, reverse, colliding=set()):
        self.robot = robot
        self.trajectory = trajectory
        self.n1, self.n2 = reversed(element) if reverse else element
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
        init.extend([
            ('Node', t.n1),
            ('Node', t.n2),
            ('Element', t.element),
            ('Traj', t),
            ('Connection', t.n1, t.element, t, t.n2),
        ])

    goal_literals = [('Printed', e) for e in element_bodies]
    goal = And(*goal_literals)
    # TODO: weight or order these in some way
    # TODO: instantiation slowness is due to condition effects. Change!

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


def plan_sequence(trajectories, element_bodies, ground_nodes):
    if trajectories is None:
        return None
    # http://www.fast-downward.org/Doc/Heuristic#h.5Em_heuristic
    # randomize_successors=True
    pr = cProfile.Profile()
    pr.enable()
    pddlstream_problem = get_pddlstream(trajectories, element_bodies, ground_nodes)
    #solution = solve_exhaustive(pddlstream_problem, planner='goal-lazy', max_time=300, debug=True)
    solution = solve_exhaustive(pddlstream_problem, planner='add-random-lazy', max_time=300,
                                max_planner_time=300, debug=True)
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


def optimize_angle(robot, link, element_pose, translation, direction, reverse, initial_angles,
                   collision_fn, max_error=1e-2):
    movable_joints = get_movable_joints(robot)
    initial_conf = get_joint_positions(robot, movable_joints)
    best_error, best_angle, best_conf = max_error, None, None
    for i, angle in enumerate(initial_angles):
        grasp_pose = get_grasp_pose(translation, direction, angle, reverse)
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


def compute_direction_path(robot, length, reverse, element_body, direction, collision_fn):
    step_size = 0.0025 # 0.005
    #angle_step_size = np.pi / 128
    angle_step_size = np.math.radians(0.25)
    angle_deltas = [-angle_step_size, 0, angle_step_size]
    #num_initial = 12
    num_initial = 1

    steps = np.append(np.arange(-length / 2, length / 2, step_size), [length / 2])
    #print('Length: {} | Steps: {}'.format(length, len(steps)))

    #initial_angles = [wrap_angle(angle) for angle in np.linspace(0, 2*np.pi, num_initial, endpoint=False)]
    initial_angles = [wrap_angle(angle) for angle in np.random.uniform(0, 2*np.pi, num_initial)]
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)
    set_joint_positions(robot, movable_joints, sample_fn())
    link = link_from_name(robot, TOOL_NAME)
    element_pose = get_pose(element_body)
    current_angle, current_conf = optimize_angle(robot, link, element_pose,
                                                 steps[0], direction, reverse, initial_angles, collision_fn)
    if current_conf is None:
        return None
    # TODO: constrain maximum conf displacement
    # TODO: alternating minimization for just position and also orientation
    trajectory = [current_conf]
    for translation in steps[1:]:
        #set_joint_positions(robot, movable_joints, current_conf)
        initial_angles = [wrap_angle(current_angle + delta) for delta in angle_deltas]
        current_angle, current_conf = optimize_angle(robot, link, element_pose,
                                                     translation, direction, reverse, initial_angles, collision_fn)
        if current_conf is None:
            return None
        trajectory.append(current_conf)
    return trajectory


def sample_print_path(robot, length, reverse, element_body, collision_fn):
    #max_directions = 10
    #max_directions = 16
    max_directions = 1
    #for direction in np.linspace(0, 2*np.pi, 10, endpoint=False):
    directions = [sample_direction() for _ in range(max_directions)]
    #directions = np.linspace(0, 2*np.pi, 10, endpoint=False)
    #directions = np.random.uniform(0, 2*np.pi, max_directions)
    for direction in directions:
        trajectory = compute_direction_path(robot, length, reverse, element_body, direction, collision_fn)
        if trajectory is not None:
            return trajectory
    return None


def sample_trajectories(robot, obstacles, node_points, element_bodies, ground_nodes):
    disabled = {tuple(link_from_name(robot, link) for link in pair) for pair in disabled_collisions}
    collision_fn = get_collision_fn(robot, get_movable_joints(robot), obstacles, [],
                                    self_collisions=True, disabled_collisions=disabled, use_limits=False)

    # TODO: can cache which elements collide and prune dominated
    element_neighbors = get_element_neighbors(element_bodies)
    elements_order = list(element_bodies.keys())
    bodies_order = [element_bodies[e] for e in elements_order]
    #max_trajs = 4
    max_trajs = np.inf
    num_trajs = 100
    # 50 doesn't seem to be enough

    all_trajectories = []
    for index, (element, element_body) in enumerate(element_bodies.items()):
        # TODO: prune elements that collide with all their neighbors
        n1, n2 = element
        length = np.linalg.norm(node_points[n2] - node_points[n1])  # 5cm
        for reverse in [False, True]:
            trajectories = []
            for i in range(num_trajs):
                trajectory = sample_print_path(robot, length, reverse, element_body, collision_fn)
                if trajectory is None:
                    continue
                collisions = check_trajectory_collision(robot, trajectory, bodies_order)
                colliding = {e for k, e in enumerate(elements_order) if (element != e) and collisions[k]}
                #colliding = {e for e, body in element_bodies.items()
                #             if check_trajectory_collision(robot, trajectory, [body])}
                if (element_neighbors[element] <= colliding) and not any(n in ground_nodes for n in element):
                    # Only need to consider neighbors on one side
                    continue
                trajectories.append(PrintTrajectory(robot, trajectory, element, reverse, colliding))
                prune_dominated(trajectories)

                if has_gui():
                    for e, body in element_bodies.items():
                        if e == element:
                            set_color(body, (0, 1, 0, 1))
                        elif e in colliding:
                            set_color(body, (1, 0, 0, 1))
                        else:
                            set_color(body, (1, 0, 0, 0))
                    wait_for_interrupt()
                # TODO: make more if many collisions in particular
                if not colliding or (max_trajs <= len(trajectories)):
                    break

            prune_dominated(trajectories)
            print(index, reverse, len(trajectories), sorted(len(t.colliding) for t in trajectories))
            all_trajectories.extend(trajectories)
            if not trajectories:
                # TODO: only break if both directions infeasible
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

def display(ground_nodes, trajectories):
    connect(use_gui=True)
    floor, robot = load_world()
    wait_for_interrupt()
    movable_joints = get_movable_joints(robot)
    #element_bodies = dict(zip(elements, create_elements(node_points, elements)))
    #for body in element_bodies.values():
    #    set_color(body, (1, 0, 0, 0))
    connected = set(ground_nodes)
    for trajectory in trajectories:
        print(trajectory.element, trajectory.n1 in connected, len(trajectory.trajectory))
        #wait_for_interrupt()
        #set_color(element_bodies[element], (1, 0, 0, 1))
        last_point = None
        for conf in trajectory.trajectory:
            set_joint_positions(robot, movable_joints, conf)
            current_point = point_from_pose(get_link_pose(robot, link_from_name(robot, TOOL_NAME)))
            if last_point is not None:
                handle = add_line(last_point, current_point, color=(1, 0, 0))
            last_point = current_point
            #wait_for_duration(0.025)
            wait_for_duration(0.1)
        connected.add(trajectory.n2)
    #user_input('Finish?')
    wait_for_interrupt()
    disconnect()


def draw_model(elements, node_points, ground_nodes):
    handles = []
    for element in elements:
        is_ground = any(n in ground_nodes for n in element)
        color = (0, 0, 1, 1) if is_ground else (1, 0, 0, 1)
        handles.append(draw_element(node_points, element, color=color))
    return handles

def main(viewer=False):
    # https://github.mit.edu/yijiangh/assembly-instances
    #read_minizinc_data(os.path.join(root_directory, 'print_data.txt'))
    #return

    # djmm_test_block | mars_bubble | sig_artopt-bunny | topopt-100 | topopt-205 | topopt-310 | voronoi
    elements, node_points, ground_nodes = load_extrusion('topopt-100')

    node_order = list(range(len(node_points)))
    #np.random.shuffle(node_order)
    node_order = sorted(node_order, key=lambda n: node_points[n][2])
    elements = sorted(elements, key=lambda e: min(node_points[n][2] for n in e))

    #node_order = node_order[:100]
    ground_nodes = [n for n in ground_nodes if n in node_order]
    elements = [element for element in elements if all(n in node_order for n in element)]
    #elements = elements[:5]
    #elements = elements[:25]
    elements = elements[:50]
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
    #dump_body(robot)

    #test_grasps(robot, node_points, elements)
    #test_print(robot, node_points, elements)
    #return

    if has_gui():
        draw_model(elements, node_points, ground_nodes)
        wait_for_interrupt('Continue?')
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
        display(ground_nodes, plan)

    # Collisions at the end points?


if __name__ == '__main__':
    main()