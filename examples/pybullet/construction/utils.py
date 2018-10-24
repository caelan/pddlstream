import os
from collections import defaultdict

import numpy as np
import random
import json

from examples.pybullet.utils.pybullet_tools.utils import add_line, create_cylinder, set_point, Euler, quat_from_euler, \
    set_quat, get_movable_joints, set_joint_positions, pairwise_collision, Pose, multiply, Point, load_model, \
    HideOutput, load_pybullet, quat_from_vector_angle, unit_point


EXTRUSION_DIRECTORY = 'spatial_extrusion/'
EXTRUSION_FILENAMES = {
    'djmm_test_block': 'djmm_test_block_S1.0_09-17-2018.json',
    'mars_bubble': 'mars_bubble_S1.0_09-17-2018.json',
    'sig_artopt-bunny': 'sig_artopt-bunny_S1.0_09-17-2018.json',
    'topopt-100': 'topopt-100_S1.0_09-17-2018.json',
    'topopt-205': 'topopt-205_S0.7_09-17-2018.json',
    'topopt-310': 'topopt-310_S1.0_09-17-2018.json',
    'voronoi': 'voronoi_S1.0_09-05-2018.json',
}

KUKA_PATH = 'framefab_kr6_r900_support/urdf/kr6_r900.urdf'
#KUKA_PATH = 'framefab_kr6_r900_support/urdf/kr6_r900_workspace.urdf'
TOOL_NAME = 'eef_tcp_frame'
DISABLED_COLLISIONS = [
    # ('robot_link_1', 'workspace_objects'),
    # ('robot_link_2', 'workspace_objects'),
    # ('robot_link_3', 'workspace_objects'),
    # ('robot_link_4', 'workspace_objects'),
    ('robot_link_5', 'eef_base_link'),
]
# [u'base_frame_in_rob_base', u'element_list', u'node_list', u'assembly_type', u'model_type', u'unit']

##################################################

PICKNPLACE_DIRECTORY = 'picknplace/'
PICKNPLACE_FILENAMES = {
    'choreo_brick_demo': 'choreo_brick_demo.json',
    'choreo_eth-trees_demo': 'choreo_eth-trees_demo.json',
}

def load_pick_and_place(extrusion_name):
    root_directory = os.path.dirname(os.path.abspath(__file__))
    extrusion_path = os.path.join(root_directory, PICKNPLACE_DIRECTORY, PICKNPLACE_FILENAMES[extrusion_name])
    print('Name: {}'.format(extrusion_name))
    print('Path: {}'.format(extrusion_path))
    with open(extrusion_path, 'r') as f:
        json_data = json.loads(f.read())

    print(json_data.keys())

    parse_point(json_data['pick_base_center_point'])
    parse_point(json_data['place_base_center_point'])
    unstackable_bodies = json_data['pick_support_surface_file_names']
    stackable_bodies = json_data['place_support_surface_file_names']
    print(unstackable_bodies)
    print(stackable_bodies)

    pick_orders = set()
    place_orders = set()
    for json_element in json_data['sequenced_elements']:
        # [u'place_element_geometry_file_name', u'order_id', u'place_support_surface_file_names',
        # u'grasps', u'place_contact_ngh_ids', u'pick_support_surface_file_names',
        # u'pick_element_geometry_file_name', u'pick_contact_ngh_ids']
        index = json_element['order_id']
        pick_indices = json_element.get('pick_contact_ngh_ids', [])
        place_indices = json_element.get('place_contact_ngh_ids', [])
        pick_orders.update((index, above) for above in pick_indices)
        place_orders.update((index, above) for above in place_orders)


        #json_element['pick_element_geometry_file_name']
        #json_element['place_element_geometry_file_name']
        pickable = json_element.get('pick_support_surface_file_names', [])
        placable = json_element.get('place_support_surface_file_names', [])
        grasps = [{name: parse_transform(approach)
                   for name, approach in json_grasp.items()}
                  for json_grasp in json_element['grasps']]
        print(index, len(grasps), pick_indices, place_indices, pickable, placable)

##################################################

def load_extrusion(extrusion_name):
    root_directory = os.path.dirname(os.path.abspath(__file__))
    extrusion_path = os.path.join(root_directory, EXTRUSION_DIRECTORY, EXTRUSION_FILENAMES[extrusion_name])
    print('Name: {}'.format(extrusion_name))
    print('Path: {}'.format(extrusion_path))
    with open(extrusion_path, 'r') as f:
        json_data = json.loads(f.read())

    elements = parse_elements(json_data)
    node_points = parse_node_points(json_data)
    ground_nodes = parse_ground_nodes(json_data)
    print('Assembly: {} | Model: {} | Unit: {}'.format(
        json_data['assembly_type'], json_data['model_type'], json_data['unit'])) # extrusion, spatial_frame, millimeter
    print('Nodes: {} | Ground: {} | Elements: {}'.format(
        len(node_points), len(ground_nodes), len(elements)))
    return elements, node_points, ground_nodes

def parse_point(json_point, scale=1e-3):
    return scale * np.array([json_point['X'], json_point['Y'], json_point['Z']])

def parse_transform(json_transform):
    transform = np.eye(4)
    transform[:3, 3] = parse_point(json_transform['Origin']) # Normal
    transform[:3, :3] = np.vstack([parse_point(json_transform[axis], scale=1)
                                   for axis in ['XAxis', 'YAxis', 'ZAxis']])
    return transform


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

##################################################

def draw_element(node_points, element, color=(1, 0, 0)):
    n1, n2 = element
    p1 = node_points[n1]
    p2 = node_points[n2]
    return add_line(p1, p2, color=color[:3])


def create_elements(node_points, elements, radius=0.0005, color=(1, 0, 0, 1)):
    # TODO: just shrink the structure to prevent worrying about collisions at end-points
    #radius = 0.0001
    #radius = 0.00005
    #radius = 0.000001
    radius = 1e-6
    # TODO: seems to be a min radius

    shrink = 0.01
    #shrink = 0.
    element_bodies = []
    for (n1, n2) in elements:
        p1, p2 = node_points[n1], node_points[n2]
        height = max(np.linalg.norm(p2 - p1) - 2*shrink, 0)
        #if height == 0: # Cannot keep this here
        #    continue
        center = (p1 + p2) / 2
        # extents = (p2 - p1) / 2
        body = create_cylinder(radius, height, color=color)
        set_point(body, center)
        element_bodies.append(body)

        delta = p2 - p1
        x, y, z = delta
        phi = np.math.atan2(y, x)
        theta = np.math.acos(z / np.linalg.norm(delta))
        set_quat(body, quat_from_euler(Euler(pitch=theta, yaw=phi)))
        # p1 is z=-height/2, p2 is z=+height/2
    return element_bodies


def check_trajectory_collision(robot, trajectory, bodies):
    # TODO: each new addition makes collision checking more expensive
    #offset = 4
    movable_joints = get_movable_joints(robot)
    #for q in trajectory[offset:-offset]:
    collisions = [False for _ in range(len(bodies))] # TODO: batch collision detection
    for q in trajectory:
        set_joint_positions(robot, movable_joints, q)
        for i, body in enumerate(bodies):
            if not collisions[i]:
                collisions[i] |= pairwise_collision(robot, body)
    return collisions


#def get_grasp_rotation(direction, angle):
    #return Pose(euler=Euler(roll=np.pi / 2, pitch=direction, yaw=angle))
    #rot = Pose(euler=Euler(roll=np.pi / 2))
    #thing = (unit_point(), quat_from_vector_angle(direction, angle))
    #return multiply(thing, rot)

def sample_direction():
    ##roll = random.uniform(0, np.pi)
    #roll = np.pi/4
    #pitch = random.uniform(0, 2*np.pi)
    #return Pose(euler=Euler(roll=np.pi / 2 + roll, pitch=pitch))
    roll = random.uniform(-np.pi/2, np.pi/2)
    pitch = random.uniform(-np.pi/2, np.pi/2)
    return Pose(euler=Euler(roll=roll, pitch=pitch))


def get_grasp_pose(translation, direction, angle, reverse, offset=1e-3):
    #direction = Pose(euler=Euler(roll=np.pi / 2, pitch=direction))
    return multiply(Pose(point=Point(z=offset)),
                    Pose(euler=Euler(yaw=angle)),
                    direction,
                    Pose(point=Point(z=translation)),
                    Pose(euler=Euler(roll=(1-reverse) * np.pi)))


def load_world():
    root_directory = os.path.dirname(os.path.abspath(__file__))
    with HideOutput():
        floor = load_model('models/short_floor.urdf')
        robot = load_pybullet(os.path.join(root_directory, KUKA_PATH), fixed_base=True)
    set_point(floor, Point(z=-0.01))
    return floor, robot


def prune_dominated(trajectories):
    start_len = len(trajectories)
    for traj1 in list(trajectories):
        if any((traj1 != traj2) and (traj2.colliding <= traj1.colliding) for traj2 in trajectories):
            trajectories.remove(traj1)
    return len(trajectories) == start_len

##################################################

def get_node_neighbors(elements):
    node_neighbors = defaultdict(set)
    for e in elements:
        n1, n2 = e
        node_neighbors[n1].add(e)
        node_neighbors[n2].add(e)
    return node_neighbors


def get_element_neighbors(element_bodies):
    node_neighbors = get_node_neighbors(element_bodies)
    element_neighbors = defaultdict(set)
    for e in element_bodies:
        n1, n2 = e
        element_neighbors[e].update(node_neighbors[n1])
        element_neighbors[e].update(node_neighbors[n2])
        element_neighbors[e].remove(e)
    return element_neighbors