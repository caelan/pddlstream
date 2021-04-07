import numpy as np

from examples.pybullet.utils.pybullet_tools.pr2_problems import create_pr2
from examples.pybullet.utils.pybullet_tools.pr2_utils import set_arm_conf, arm_conf, REST_LEFT_ARM, close_arm, \
    set_group_conf, ARM_NAMES
from examples.pybullet.utils.pybullet_tools.utils import add_data_path, load_pybullet, set_point, Point, create_box, \
    stable_z, HUSKY_URDF, dump_body, wait_for_user, GREY, BLACK, RED, BLUE, BROWN, TAN
from examples.pybullet.tamp.problems import sample_placements

from examples.pybullet.turtlebot_rovers.problems import RoversProblem


def problem1(n_rovers=1, n_objectives=1, n_rocks=2, n_soil=2, n_stores=1):
    base_extent = 5.0
    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))

    floor = create_box(base_extent, base_extent, 0.001, color=TAN) # TODO: two rooms
    set_point(floor, Point(z=-0.001/2.))

    add_data_path()
    lander = load_pybullet(HUSKY_URDF, scale=1)
    lander_z = stable_z(lander, floor)
    set_point(lander, Point(-2, -2, lander_z))
    #wait_for_user()

    mount_width = 0.5
    mound_height = mount_width
    mound1 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound1, [+2, 1.5, mound_height/2.])
    mound2 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound2, [-2, 1.5, mound_height/2.])

    initial_surfaces = {}

    rover_confs = [(+1, -1.75, np.pi), (-1, -1.75, 0)]
    assert n_rovers <= len(rover_confs)

    #body_names = map(get_name, env.GetBodies())
    landers = [lander]
    stores = ['store{}'.format(i) for i in range(n_stores)]

    #affine_limits = aabb_extrema(aabb_union([aabb_from_body(body) for body in env.GetBodies()])).T
    rovers = []
    for i in range(n_rovers):
        robot = create_pr2()
        dump_body(robot) # camera_rgb_optical_frame
        rovers.append(robot)
        set_group_conf(robot, 'base', rover_confs[i])
        for arm in ARM_NAMES:
            set_arm_conf(robot, arm, arm_conf(arm, REST_LEFT_ARM))
            close_arm(robot, arm)

    obj_width = 0.07
    obj_height = 0.2

    objectives = []
    for _ in range(n_objectives):
        body = create_box(obj_width, obj_width, obj_height, color=BLUE)
        objectives.append(body)
        initial_surfaces[body] = mound1
    for _ in range(n_objectives):
        body = create_box(obj_width, obj_width, obj_height, color=RED)
        initial_surfaces[body] = mound2

    # TODO: it is moving to intermediate locations attempting to reach the rocks
    rocks = []
    for _ in range(n_rocks):
        body = create_box(0.075, 0.075, 0.05, color=BLACK)
        rocks.append(body)
        initial_surfaces[body] = floor
    soils = []
    for _ in range(n_soil):
        body = create_box(0.1, 0.1, 0.025, color=BROWN)
        soils.append(body)
        initial_surfaces[body] = floor
    sample_placements(initial_surfaces)

    #for name in rocks + soils:
    #    env.GetKinBody(name).Enable(False)
    return RoversProblem(rovers, landers, objectives, rocks, soils, stores, base_limits)

PROBLEMS = [
    problem1,
]