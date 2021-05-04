import numpy as np
import random

from collections import OrderedDict
from examples.pybullet.utils.pybullet_tools.utils import add_data_path, load_pybullet, set_point, Point, create_box, \
    stable_z, HUSKY_URDF, load_model, TURTLEBOT_URDF, joints_from_names, \
    set_joint_positions, get_joint_positions, link_from_name, get_link_pose, draw_pose, wait_for_user, \
    get_center_extent, draw_aabb, get_aabb, HideOutput, GREY, BLACK, RED, BLUE, BROWN, TAN, GREEN, get_bodies
from examples.pybullet.tamp.problems import sample_placements

BASE_JOINTS = ['x', 'y', 'theta']

def get_base_joints(robot):
    return joints_from_names(robot, BASE_JOINTS)

def get_base_conf(robot):
    return get_joint_positions(robot, get_base_joints(robot))

def set_base_conf(robot, conf):
    set_joint_positions(robot, get_base_joints(robot), conf)

KINECT_FRAME = 'camera_rgb_optical_frame' # eyes
#KINECT_FRAME = 'eyes'

#######################################################

class RoversProblem(object):
    def __init__(self, rovers=[], landers=[], objectives=[], rocks=[], soils=[], stores=[], limits=[], body_types=[]):
        self.rovers = rovers
        self.landers = landers
        self.objectives = objectives
        self.rocks = rocks
        self.soils = soils
        self.stores = stores
        self.limits = limits
        no_collisions = self.rovers + self.rocks
        self.fixed = set(get_bodies()) - set(no_collisions)
        self.body_types = body_types
        self.costs = False

#######################################################

def rovers1(n_rovers=2, n_objectives=4, n_rocks=3, n_soil=3, n_stores=1, n_obstacles=8):
    base_extent = 5.0
    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    mount_width = 0.5
    mound_height = 0.1

    floor = create_box(base_extent, base_extent, 0.001, color=TAN) # TODO: two rooms
    set_point(floor, Point(z=-0.001/2.))

    wall1 = create_box(base_extent + mound_height, mound_height, mound_height, color=GREY)
    set_point(wall1, Point(y=base_extent/2., z=mound_height/2.))
    wall2 = create_box(base_extent + mound_height, mound_height, mound_height, color=GREY)
    set_point(wall2, Point(y=-base_extent/2., z=mound_height/2.))
    wall3 = create_box(mound_height, base_extent + mound_height, mound_height, color=GREY)
    set_point(wall3, Point(x=base_extent/2., z=mound_height/2.))
    wall4 = create_box(mound_height, base_extent + mound_height, mound_height, color=GREY)
    set_point(wall4, Point(x=-base_extent/2., z=mound_height/2.))
    # TODO: can add obstacles along the wall

    wall = create_box(mound_height, base_extent, mound_height, color=GREY)  # TODO: two rooms
    set_point(wall, Point(z=mound_height / 2.))

    add_data_path()
    with HideOutput():
        lander = load_pybullet(HUSKY_URDF, scale=1)
    lander_z = stable_z(lander, floor)
    set_point(lander, Point(-1.9, -2, lander_z))

    mound1 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound1, [+2, 2, mound_height/2.])
    mound2 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound2, [-2, 2, mound_height/2.])
    mound3 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound3, [+0.5, 2, mound_height/2.])
    mound4 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound4, [-0.5, 2, mound_height/2.])
    mounds = [mound1, mound2, mound3, mound4]
    random.shuffle(mounds)

    body_types = []
    initial_surfaces = OrderedDict()
    min_distances = {}
    for _ in range(n_obstacles):
        body = create_box(mound_height, mound_height, 4*mound_height, color=GREY)
        initial_surfaces[body] = floor

    rover_confs = [(+1, -1.75, np.pi), (-1, -1.75, 0)]
    assert n_rovers <= len(rover_confs)

    landers = [lander]
    stores = ['store{}'.format(i) for i in range(n_stores)]

    rovers = []
    for i in range(n_rovers):
        # camera_rgb_optical_frame
        with HideOutput():
            rover = load_model(TURTLEBOT_URDF)
        robot_z = stable_z(rover, floor)
        set_point(rover, Point(z=robot_z))
        #handles = draw_aabb(get_aabb(rover)) # Includes the origin
        #print(get_center_extent(rover))
        #wait_for_user()
        set_base_conf(rover, rover_confs[i])
        rovers.append(rover)
        #dump_body(rover)
        #draw_pose(get_link_pose(rover, link_from_name(rover, KINECT_FRAME)))

    obj_width = 0.07
    obj_height = 0.2

    objectives = []
    for i in range(n_objectives):
        body = create_box(obj_width, obj_width, obj_height, color=BLUE)
        objectives.append(body)
        #initial_surfaces[body] = random.choice(mounds)
        initial_surfaces[body] = mounds[i]
    min_distances.update({r: 0.05 for r in objectives})

    # TODO: it is moving to intermediate locations attempting to reach the rocks

    rocks = []
    for _ in range(n_rocks):
        body = create_box(0.075, 0.075, 0.01, color=BLACK)
        rocks.append(body)
        body_types.append((body, 'stone'))
    for _ in range(n_soil):
        body = create_box(0.1, 0.1, 0.005, color=BROWN)
        rocks.append(body)
        body_types.append((body, 'soil'))
    soils = [] # Treating soil as rocks for simplicity

    initial_surfaces.update({r: floor for r in rocks})
    min_distances.update({r: 0.2 for r in rocks})
    sample_placements(initial_surfaces, min_distances=min_distances)

    return RoversProblem(rovers, landers, objectives, rocks, soils, stores, base_limits, body_types)

PROBLEMS = [
    rovers1,
]
