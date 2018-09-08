import os
from collections import defaultdict

import numpy as np

from examples.pybullet.utils.pybullet_tools.utils import add_line, create_cylinder, set_point, Euler, quat_from_euler, \
    set_quat, get_movable_joints, set_joint_positions, pairwise_collision, Pose, multiply, Point, load_model, \
    HideOutput, load_pybullet

JSON_FILENAME = 'voronoi_S1.0_09-05-2018.json'
KUKA_PATH = 'framefab_kr6_r900_support/urdf/kr6_r900.urdf'
#KUKA_PATH = 'framefab_kr6_r900_support/urdf/kr6_r900_workspace.urdf'
TOOL_NAME = 'eef_tcp_frame'
disabled_collisions = [
    # ('robot_link_1', 'workspace_objects'),
    # ('robot_link_2', 'workspace_objects'),
    # ('robot_link_3', 'workspace_objects'),
    # ('robot_link_4', 'workspace_objects'),
    ('robot_link_5', 'eef_base_link'),
]
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


def draw_element(node_points, element, color=(1, 0, 0, 1)):
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
    element_bodies = []
    for (n1, n2) in elements:
        p1 = node_points[n1]
        p2 = node_points[n2]
        height = max(np.linalg.norm(p2 - p1) - 2*shrink, 0)
        #if height == 0: # Cannot keep this here
        #    continue
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


def get_grasp_rotation(direction, angle):
    return Pose(euler=Euler(roll=np.pi / 2, pitch=direction, yaw=angle))


def get_grasp_pose(translation, direction, angle):
    offset = 1e-3
    return multiply(Pose(point=Point(z=offset)),
                    get_grasp_rotation(direction, angle),
                    Pose(point=Point(z=translation)))


def load_world():
    root_directory = os.path.dirname(os.path.abspath(__file__))
    floor = load_model('models/short_floor.urdf')
    with HideOutput():
        robot = load_pybullet(os.path.join(root_directory, KUKA_PATH))
    set_point(floor, Point(z=-0.01))
    return floor, robot


def prune_dominated(trajectories):
    for traj1 in list(trajectories):
        if any((traj1 != traj2) and (traj2.colliding <= traj1.colliding) for traj2 in trajectories):
            trajectories.remove(traj1)


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