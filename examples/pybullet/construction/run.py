import json
import os

import numpy as np

from examples.pybullet.utils.pybullet_tools.utils import connect, dump_body, disconnect, wait_for_interrupt, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, link_from_name, set_point, set_quat, \
    add_line, quat_from_euler, Euler, HideOutput, create_cylinder, load_pybullet, inverse_kinematics, get_link_pose

JSON_FILENAME = 'voronoi_S1.0_09-05-2018.json'

KUKA_PATH = 'framefab_kr6_r900_support/urdf/kr6_r900_workspace.urdf'

# [u'base_frame_in_rob_base', u'element_list', u'node_list', u'assembly_type', u'model_type', u'unit']

def parse_point(json_point):
    return np.array([json_point['X'], json_point['Y'], json_point['Z']]) / 1e3 # / 1e3

def parse_origin(json_data):
    return parse_point(json_data['base_frame_in_rob_base']['Origin'])

def parse_elements(json_data):
    return [tuple(json_element['end_node_ids'])
            for json_element in json_data['element_list']] # 'layer_id

def parse_node_points(json_data):
    origin = parse_origin(json_data)
    return [origin + parse_point(json_node['point']) for json_node in json_data['node_list']]

def parse_ground_nodes(json_data):
    return {i for i, json_node in enumerate(json_data['node_list']) if json_node['is_grounded'] == 1}

def sample_confs(robot, num_samples=10):
    joints = get_movable_joints(robot)
    print('Joints', [get_joint_name(robot, joint) for joint in joints])
    sample_fn = get_sample_fn(robot, joints)
    for i in range(num_samples):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(robot, joints, conf)
        wait_for_interrupt()

def draw_elements(node_points, elements):
    color = (1, 0, 0, 1)
    element_lines = []
    for (n1, n2) in elements:
        p1 = node_points[n1]
        p2 = node_points[n2]
        element_lines.append(add_line(p1, p2, color=color[:3]))
    return element_lines

def create_elements(node_points, elements, radius=0.0005, color=(1, 0, 0, 1)):
    element_bodies = []
    for (n1, n2) in elements:
        p1 = node_points[n1]
        p2 = node_points[n2]
        height = np.linalg.norm(p2 - p1)
        if height == 0:
            continue
        center = (p1 + p2) / 2
        # extents = (p2 - p1) / 2
        body = create_cylinder(radius, height, color=color)
        set_point(body, center)
        element_bodies.append(body)

        delta = p2 - center
        x, y, z = delta
        phi = np.math.atan2(y, x)
        theta = np.math.acos(z / np.linalg.norm(delta))

        euler = Euler(pitch=theta, yaw=phi)
        # euler = Euler() # Default is vertical
        quat = quat_from_euler(euler)
        set_quat(body, quat)
    return element_bodies


def main(viewer=True):
    root_directory = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(root_directory, JSON_FILENAME), 'r') as f:
        json_data = json.loads(f.read())

    elements = parse_elements(json_data)
    node_points = parse_node_points(json_data)
    ground_nodes = parse_ground_nodes(json_data)

    print('Nodes: {} | Ground: {} | Elements: {}'.format(
        len(node_points), len(ground_nodes), len(elements)))
    #print(json_data['assembly_type']) # extrusion
    #print(json_data['model_type']) # spatial_frame
    #print(json_data['unit']) # millimeter

    connect(use_gui=viewer)
    #floor = load_model('models/short_floor.urdf')
    with HideOutput():
        robot = load_pybullet(os.path.join(root_directory, KUKA_PATH))
    dump_body(robot)
    draw_elements(node_points, elements)

    tool_name = 'eef_tcp_frame'
    link = link_from_name(robot, tool_name)
    node_order = list(range(len(node_points)))
    np.random.shuffle(node_order)

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
        print(point, link_point)
        #user_input('Continue?')
        wait_for_interrupt()
    # TODO: draw the transforms

    #element_bodies = create_elements(origin, node_points, elements)
    #print('Bodies: {}'.format(len(elements)))

    #user_input('Finish?')
    wait_for_interrupt()
    disconnect()

if __name__ == '__main__':
    main()