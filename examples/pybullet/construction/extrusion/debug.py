import cProfile
import pstats
import re

import numpy as np

from examples.pybullet.construction.extrusion.utils import TOOL_NAME, create_elements, get_grasp_pose, sample_direction
from examples.pybullet.utils.pybullet_tools.utils import get_movable_joints, get_joint_name, get_sample_fn, \
    set_joint_positions, wait_for_interrupt, link_from_name, inverse_kinematics, get_link_pose, Pose, Euler, Point, \
    multiply, set_pose, get_pose, invert, draw_pose
from pddlstream.algorithms.incremental import solve_exhaustive
from pddlstream.language.constants import And, PDDLProblem, print_solution
from pddlstream.language.generator import from_test
from pddlstream.utils import read, get_file_path, user_input


def test_confs(robot, num_samples=10):
    joints = get_movable_joints(robot)
    print('Joints', [get_joint_name(robot, joint) for joint in joints])
    sample_fn = get_sample_fn(robot, joints)
    for i in range(num_samples):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(robot, joints, conf)
        wait_for_interrupt()


def get_test_cfree(element_bodies):
    def test(traj, element):
        if traj.element == element:
            return True
        return element not in traj.colliding
        #collisions = check_trajectory_collision(traj.robot, traj.trajectory, [element_bodies[element]])
        #return not any(collisions)
    return test


def get_pddlstream_test(node_points, elements, ground_nodes):
    # stripstream/lis_scripts/run_print.py
    # stripstream/lis_scripts/print_data.txt

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    constant_map = {}

    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    #stream_pddl = None
    stream_map = {
        'test-cfree': from_test(get_test_cfree({})),
    }

    nodes = list(range(len(node_points))) # TODO: sort nodes by height?

    init = []
    for n in nodes:
        init.append(('Node', n))
    for n in ground_nodes:
        init.append(('Connected', n))
    for e in elements:
        init.append(('Element', e))
        n1, n2 = e
        t = None
        init.extend([
            ('Connection', n1, e, t, n2),
            ('Connection', n2, e, t, n1),
        ])
        #init.append(('Edge', n1, n2))

    goal_literals = [('Printed', e) for e in elements]
    goal = And(*goal_literals)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


def test_ik(robot, node_order, node_points):
    link = link_from_name(robot, TOOL_NAME)
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)
    for n in node_order:
        point = node_points[n]
        set_joint_positions(robot, movable_joints, sample_fn())
        conf = inverse_kinematics(robot, link, (point, None))
        if conf is not None:
            set_joint_positions(robot, movable_joints, conf)
            continue
        link_point, link_quat = get_link_pose(robot, link)
        #print(point, link_point)
        #user_input('Continue?')
        wait_for_interrupt()
    # TODO: draw the transforms


def test_grasps(robot, node_points, elements):
    element = elements[-1]
    [element_body] = create_elements(node_points, [element])

    phi = 0
    link = link_from_name(robot, TOOL_NAME)
    link_pose = get_link_pose(robot, link)
    draw_pose(link_pose) #, parent=robot, parent_link=link)
    wait_for_interrupt()

    n1, n2 = element
    p1 = node_points[n1]
    p2 = node_points[n2]
    length = np.linalg.norm(p2 - p1)
    # Bottom of cylinder is p1, top is p2
    print(element, p1, p2)
    for phi in np.linspace(0, np.pi, 10, endpoint=True):
        theta = np.pi/4
        for t in np.linspace(-length/2, length/2, 10):
            translation = Pose(point=Point(z=-t)) # Want to be moving backwards because +z is out end effector
            direction = Pose(euler=Euler(roll=np.pi / 2 + theta, pitch=phi))
            angle = Pose(euler=Euler(yaw=1.2))
            grasp_pose = multiply(angle, direction, translation)
            element_pose = multiply(link_pose, grasp_pose)
            set_pose(element_body, element_pose)
            wait_for_interrupt()
            #user_input('Continue?')

    #for theta in np.linspace(0, 2 * np.pi, 10, endpoint=False):
    #    n1, n2 = element
    #    length = np.linalg.norm(node_points[n2] - node_points[n1])
    #    for t in np.linspace(-length/2, length/2, 10):
    #        grasp_pose = get_grasp_pose(t, phi, theta)
    #        element_pose = multiply(link_pose, grasp_pose)
    #        set_pose(element_body, element_pose)
    #        wait_for_interrupt()
    #        #user_input('Continue?')


def test_print(robot, node_points, elements):
    #elements = [elements[0]]
    elements = [elements[-1]]
    [element_body] = create_elements(node_points, elements)
    wait_for_interrupt()

    phi = 0
    #grasp_rotations = [Pose(euler=Euler(roll=np.pi/2, pitch=phi, yaw=theta))
    #               for theta in np.linspace(0, 2*np.pi, 10, endpoint=False)]
    #grasp_rotations = [Pose(euler=Euler(roll=np.pi/2, pitch=theta, yaw=phi))
    #               for theta in np.linspace(0, 2*np.pi, 10, endpoint=False)]
    grasp_rotations = [sample_direction() for _ in range(25)]

    link = link_from_name(robot, TOOL_NAME)
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)
    for grasp_rotation in grasp_rotations:
        n1, n2 = elements[0]
        length = np.linalg.norm(node_points[n2] - node_points[n1])
        set_joint_positions(robot, movable_joints, sample_fn())
        for t in np.linspace(-length/2, length/2, 10):
            #element_translation = Pose(point=Point(z=-t))
            #grasp_pose = multiply(grasp_rotation, element_translation)
            #reverse = Pose(euler=Euler())
            reverse = Pose(euler=Euler(roll=np.pi))
            grasp_pose = get_grasp_pose(t, grasp_rotation, 0)
            grasp_pose = multiply(grasp_pose, reverse)

            element_pose = get_pose(element_body)
            link_pose = multiply(element_pose, invert(grasp_pose))
            conf = inverse_kinematics(robot, link, link_pose)
            wait_for_interrupt()
            #user_input('Continue?')


def plan_sequence_test(node_points, elements, ground_nodes):
    pr = cProfile.Profile()
    pr.enable()
    pddlstream_problem = get_pddlstream_test(node_points, elements, ground_nodes)
    #solution = solve_focused(pddlstream_problem, planner='goal-lazy', max_time=10, debug=False)
    solution = solve_exhaustive(pddlstream_problem, planner='goal-lazy', max_time=10, debug=False)
    print_solution(solution)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    plan, _, _ = solution
    return plan


def read_minizinc_data(path):
    # https://github.com/yijiangh/Choreo/blob/9481202566a4cff4d49591bec5294b1e02abcc57/framefab_task_sequence_planning/framefab_task_sequence_planner/minizinc/as_minizinc_data_layer_1.dzn

    data = read(path)

    n = int(re.findall(r'n = (\d+);', data)[0])
    m = int(re.findall(r'm = (\d+);', data)[0])
    # print re.search(r'n = (\d+);', data).group(0)
    g_data = np.array(re.findall(r'G_data = \[([01,]*)\];', data)[0].split(',')[:-1], dtype=int)
    a_data = np.array(re.findall(r'A_data = \[([01,]*)\];', data)[0].split(',')[:-1], dtype=int).reshape([n, n])
    t_data = np.array(re.findall(r'T_data = \[([01,]*)\];', data)[0].split(',')[:-1], dtype=int).reshape([n, n, m])

    print(n, m)
    print(g_data.shape, g_data.size, np.sum(g_data))  # 1 if edge(e) is grounded
    print(a_data.shape, a_data.size, np.sum(a_data))  # 1 if edge(e) and edge(j) share a node
    print(t_data.shape, t_data.size, np.sum(t_data))  # 1 if printing edge e with orientation a does not collide with edge j

    elements = list(range(n))
    orientations = list(range(m))
    #orientations = random.sample(orientations, 10)

    return g_data, a_data, t_data