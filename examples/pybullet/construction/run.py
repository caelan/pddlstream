from __future__ import print_function

import cProfile
import pstats
import numpy as np

from examples.pybullet.construction.debug import get_test_cfree, test_grasps, test_print
from examples.pybullet.construction.utils import parse_elements, parse_node_points, parse_ground_nodes, create_elements, \
    load_extrusion, TOOL_NAME, disabled_collisions, check_trajectory_collision, get_grasp_pose, load_world, \
    prune_dominated, get_element_neighbors, get_node_neighbors, sample_direction, draw_element
from examples.pybullet.utils.pybullet_tools.utils import connect, dump_body, disconnect, wait_for_interrupt, \
    get_movable_joints, get_sample_fn, set_joint_positions, link_from_name, add_line, inverse_kinematics, \
    get_link_pose, multiply, wait_for_duration, set_color, has_gui, add_text, \
    get_pose, invert, point_from_pose, get_distance, get_joint_positions, wrap_angle, get_collision_fn
from pddlstream.algorithms.incremental import solve_exhaustive, solve_incremental
from pddlstream.language.constants import PDDLProblem, And
from pddlstream.language.generator import from_test, from_gen_fn
from pddlstream.utils import read, get_file_path, print_solution, user_input, irange


class PrintTrajectory(object):
    def __init__(self, robot, trajectory, element, reverse, colliding=set()):
        self.robot = robot
        self.trajectory = trajectory
        self.n1, self.n2 = reversed(element) if reverse else element
        self.element = element
        self.colliding = colliding
    def __repr__(self):
        return '{}->{}'.format(self.n1, self.n2)


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

def get_pddlstream2(robot, obstacles, node_points, element_bodies, ground_nodes, trajectories=[]):
    domain_pddl = read(get_file_path(__file__, 'domain2.pddl'))
    constant_map = {}

    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    stream_map = {
        'test-cfree': from_test(get_test_cfree(element_bodies)),
        'sample-print': from_gen_fn(get_print_gen_fn(robot, obstacles, node_points, element_bodies, ground_nodes)),
    }

    init = []
    for n in ground_nodes:
        init.append(('Grounded', n))
    for e in element_bodies:
        n1, n2 = e
        init.extend([
            ('Element', e),
            ('Printed', e),
            ('Edge', n1, e, n2),
            ('Edge', n2, e, n1),
            ('StartNode', n1, e),
            ('StartNode', n2, e),
        ])
        #if is_ground(e, ground_nodes):
        #    init.append(('Grounded', e))
    #for e1, neighbors in get_element_neighbors(element_bodies).items():
    #    for e2 in neighbors:
    #        init.append(('Supports', e1, e2))
    for t in trajectories:
        init.extend([
            ('Traj', t),
            ('PrintAction', t.n1, t.element, t),
        ])

    goal = And(*[('Removed', e) for e in element_bodies])

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


def plan_sequence(robot, obstacles, node_points, element_bodies, ground_nodes, trajectories=[]):
    if trajectories is None:
        return None
    pddlstream_fn = get_pddlstream2 # get_pddlstream | get_pddlstream2

    # TODO: Iterated search
    # http://www.fast-downward.org/Doc/Heuristic#h.5Em_heuristic
    # randomize_successors=True
    pr = cProfile.Profile()
    pr.enable()
    pddlstream_problem = pddlstream_fn(robot, obstacles, node_points, element_bodies,
                                       ground_nodes, trajectories=trajectories)
    #solution = solve_exhaustive(pddlstream_problem, planner='goal-lazy', max_time=300, debug=True)
    solution = solve_incremental(pddlstream_problem, planner='add-random-lazy', max_time=300,
                                 max_planner_time=300, debug=False)
    # solve_exhaustive | solve_incremental
    # Reachability heuristics good for detecting dead-ends
    # Infeasibility from the start means disconnected or collision
    # Random restart based strategy here
    print_solution(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    plan, _, _ = solution

    if plan is None:
        return None
    if pddlstream_fn == get_pddlstream:
        return [t for _, (n1, e, t, n2) in plan]
    elif pddlstream_fn == get_pddlstream2:
        return [t for _, (n1, e, t) in reversed(plan)]
    else:
        raise ValueError(pddlstream_fn)


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

def get_print_gen_fn(robot, obstacles, node_points, element_bodies, ground_nodes):
    max_attempts = 50
    max_trajectories = 10
    check_collisions = False
    # 50 doesn't seem to be enough

    disabled = {tuple(link_from_name(robot, link) for link in pair) for pair in disabled_collisions}
    collision_fn = get_collision_fn(robot, get_movable_joints(robot), obstacles, [],
                                    self_collisions=True, disabled_collisions=disabled, use_limits=False)
    #element_neighbors = get_element_neighbors(element_bodies)
    node_neighbors = get_node_neighbors(element_bodies)
    elements_order = list(element_bodies.keys())
    bodies_order = [element_bodies[e] for e in elements_order]

    def gen_fn(node1, element):
        reverse = (node1 != element[0])
        element_body = element_bodies[element]
        n1, n2 = reversed(element) if reverse else element
        p1, p2 = node_points[n1], node_points[n2]
        # if (p2 - p1)[2] < 0:
        #    continue
        length = np.linalg.norm(p2 - p1)  # 5cm
        trajectories = []
        for num in irange(max_trajectories):
            for attempt in range(max_attempts):
                path = sample_print_path(robot, length, reverse, element_body, collision_fn)
                if path is None:
                    continue
                if check_collisions:
                    collisions = check_trajectory_collision(robot, path, bodies_order)
                    colliding = {e for k, e in enumerate(elements_order) if (element != e) and collisions[k]}
                else:
                    colliding = set()
                if (node_neighbors[n1] <= colliding) and not any(n in ground_nodes for n in element):
                    continue
                print_traj = PrintTrajectory(robot, path, element, reverse, colliding)
                trajectories.append(print_traj)
                if print_traj not in trajectories:
                    continue
                print('{}) {}->{} | {} | {} | {}'.format(num, n1, n2, attempt, len(trajectories),
                                                sorted(len(t.colliding) for t in trajectories)))
                yield print_traj,
                if not colliding:
                    return
            else:
                break
    return gen_fn

def sample_trajectories(robot, obstacles, node_points, element_bodies, ground_nodes):
    gen_fn = get_print_gen_fn(robot, obstacles, node_points, element_bodies, ground_nodes)
    all_trajectories = []
    for index, (element, element_body) in enumerate(element_bodies.items()):
        add_text(element[0], position=(0, 0, -0.05), parent=element_body)
        add_text(element[1], position=(0, 0, +0.05), parent=element_body)
        trajectories = []
        for node1 in element:
            for traj, in gen_fn(node1, element):
                trajectories.append(traj)
        all_trajectories.extend(trajectories)
        if not trajectories:
            #if has_gui():
            #    for e, body in element_bodies.items():
            #        if e == element:
            #            set_color(body, (0, 1, 0, 1))
            #        elif e in element_neighbors[e]:
            #            set_color(body, (1, 0, 0, 1))
            #        else:
            #            set_color(body, (1, 0, 0, 0))
            #    wait_for_interrupt()
            return None
    return all_trajectories

def display(ground_nodes, trajectories, time_step=0.05):
    connect(use_gui=True)
    floor, robot = load_world()
    wait_for_interrupt()
    movable_joints = get_movable_joints(robot)
    #element_bodies = dict(zip(elements, create_elements(node_points, elements)))
    #for body in element_bodies.values():
    #    set_color(body, (1, 0, 0, 0))
    connected = set(ground_nodes)
    for trajectory in trajectories:
        print(trajectory, trajectory.n1 in connected, trajectory.n2 in connected,
              is_ground(trajectory.element, ground_nodes), len(trajectory.trajectory))
        #wait_for_interrupt()
        #set_color(element_bodies[element], (1, 0, 0, 1))
        last_point = None
        handles = []
        for conf in trajectory.trajectory:
            set_joint_positions(robot, movable_joints, conf)
            current_point = point_from_pose(get_link_pose(robot, link_from_name(robot, TOOL_NAME)))
            if last_point is not None:
                color = (0, 0, 1) if is_ground(trajectory.element, ground_nodes) else (1, 0, 0)
                handles.append(add_line(last_point, current_point, color=color))
            last_point = current_point
            wait_for_duration(time_step)
        connected.add(trajectory.n2)
        #wait_for_interrupt()
    #user_input('Finish?')
    wait_for_interrupt()
    disconnect()

def is_ground(element, ground_nodes):
    return any(n in ground_nodes for n in element)

def draw_model(elements, node_points, ground_nodes):
    handles = []
    for element in elements:
        color = (0, 0, 1) if is_ground(element, ground_nodes) else (1, 0, 0)
        handles.append(draw_element(node_points, element, color=color))
    return handles

def main(viewer=False):
    # TODO: setCollisionFilterGroupMask
    # TODO: only produce collisions rather than collision-free
    # TODO: return collisions using wild-stream functionality
    # TODO: handle movements between selected edges
    # TODO: fail if wild stream produces unexpected facts
    # TODO: try search at different cost levels (i.e. w/ and w/o abstract)

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
    elements = elements[:10]
    #elements = elements[:25]
    #elements = elements[:50]
    #elements = elements[:100]
    #elements = elements[:150]
    #elements = elements[150:]

    print('Nodes: {} | Ground: {} | Elements: {}'.format(
        len(node_points), len(ground_nodes), len(elements)))
    #plan = plan_sequence_test(node_points, elements, ground_nodes)

    connect(use_gui=viewer)
    floor, robot = load_world()
    obstacles = [floor]
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

    precompute = False
    if precompute:
        pr = cProfile.Profile()
        pr.enable()
        trajectories = sample_trajectories(robot, obstacles, node_points, element_bodies, ground_nodes)
        pr.disable()
        pstats.Stats(pr).sort_stats('tottime').print_stats(10)
        user_input('Continue?')
    else:
        trajectories = []
    plan = plan_sequence(robot, obstacles, node_points, element_bodies, ground_nodes, trajectories=trajectories)
    disconnect()

    #display(trajectories)
    if plan is not None:
        display(ground_nodes, plan)

    # Collisions at the end points?


if __name__ == '__main__':
    main()