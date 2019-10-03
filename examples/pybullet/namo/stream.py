from itertools import islice, product

import numpy as np

from examples.pybullet.utils.pybullet_tools.pr2_primitives import State, Pose, Conf, create_trajectory
from examples.pybullet.utils.pybullet_tools.utils import get_subtree_aabb, link_from_name, joints_from_names, \
    get_joint_positions, set_joint_positions, joint_from_name, pairwise_collision, set_renderer, get_visual_data, \
    add_line, wait_for_user, BodySaver, get_link_pose, approximate_as_cylinder, get_point, Point, halton_generator, \
    quat_from_euler, Euler, multiply, invert, base_values_from_pose, get_halton_sample_fn, get_collision_fn, \
    MAX_DISTANCE, draw_point, get_base_distance_fn, get_extend_fn, get_distance

BASE_LINK = 'base_link'
BASE_JOINTS = ['x', 'y', 'theta']
BASE_RESOLUTIONS = np.array([0.02, 0.02, np.pi/16])
#BASE_RESOLUTIONS = 0.05*np.ones(3) # Default

#######################################################

def get_turtle_aabb(robot):
    return get_subtree_aabb(robot, link_from_name(robot, BASE_LINK))


def get_base_joints(robot):
    return joints_from_names(robot, BASE_JOINTS)


def get_base_conf(robot):
    return get_joint_positions(robot, get_base_joints(robot))


def set_base_conf(robot, conf):
    set_joint_positions(robot, get_base_joints(robot), conf)


def get_custom_limits(robot, base_limits, yaw_limit=None):
    x_limits, y_limits = zip(*base_limits)
    custom_limits = {
        joint_from_name(robot, 'x'): x_limits,
        joint_from_name(robot, 'y'): y_limits,
    }
    if yaw_limit is not None:
        custom_limits.update({
            joint_from_name(robot, 'theta'): yaw_limit,
        })
    return custom_limits


def point_from_conf(conf, z=0.01):
    x, y, theta = conf
    return (x, y, z)

#######################################################

def get_test_cfree_conf_pose(problem):
    def test(r, q, b2, p2):
        if not problem.collisions:
            return True
        q.assign()
        p2.assign()
        return not pairwise_collision(r, b2)
    return test


def get_test_cfree_traj_pose(problem):
    def test(r, t, b2, p2):
        if not problem.collisions:
            return True
        p2.assign()
        state = State()
        for _ in t.apply(state):
            state.assign()
            #for b1 in state.attachments:
            #    if pairwise_collision(b1, b2):
            #        return False
            if pairwise_collision(r, b2):
                set_renderer(True)
                color = get_visual_data(b2)[0].rgbaColor
                handles = add_line(point_from_conf(t.path[0].values, z=0.02),
                                   point_from_conf(t.path[-1].values, z=0.02), color=color)
                wait_for_user()
                set_renderer(False)
                return False
        return True
    return test


def get_grasp_generator(problem):
    def gen(robot, body):
        link = link_from_name(robot, BASE_LINK)
        with BodySaver(robot):
            set_base_conf(robot, np.zeros(3))
            robot_pose = get_link_pose(robot, link)
            robot_aabb = get_turtle_aabb(robot)
            #draw_aabb(robot_aabb)
        lower, upper = robot_aabb
        center, (diameter, height) = approximate_as_cylinder(body)
        _, _, z = get_point(body) # Assuming already placed stably
        position = Point(x=upper[0] + diameter / 2., z=z)

        for [scale] in halton_generator(d=1):
            yaw = scale*2*np.pi - np.pi
            quat = quat_from_euler(Euler(yaw=yaw))
            body_pose = (position, quat)
            robot_from_body = multiply(invert(robot_pose), body_pose)
            grasp = Pose(body, robot_from_body)
            yield (grasp,)
            #world_pose = multiply(get_link_pose(robot, link), robot_from_body)
            #set_pose(body, world_pose)
            #handles = draw_pose(get_pose(body), length=1)
            #wait_for_user()
            #for handle in handles:
            #    remove_debug(handle)
    return gen


def get_ik_fn(problem):
    def fn(robot, body, pose, grasp):
        joints = get_base_joints(robot)
        robot_pose = multiply(pose.value, invert(grasp.value))
        base_values = base_values_from_pose(robot_pose)
        conf = Conf(robot, joints, base_values)
        conf.assign()
        if any(pairwise_collision(robot, obst) for obst in problem.obstacles):
            return None
        return (conf,)
    return fn


def get_fk_fn(problem):
    # TODO: could sample placements in place or not
    def fn(robot, conf, body, grasp):
        conf.assign()
        link = link_from_name(robot, BASE_LINK)
        world_from_body = multiply(get_link_pose(robot, link), grasp.value)
        pose = Pose(body, world_from_body)
        pose.assign()
        if any(pairwise_collision(body, obst) for obst in problem.obstacles):
            return None
        return (pose,)
    return fn


def get_sample_gen_fn(problem):
    def gen_fn(robot):
        joints = get_base_joints(robot)
        sample_fn = get_halton_sample_fn(robot, joints, custom_limits=problem.custom_limits)
        collision_fn = get_collision_fn(robot, joints, problem.obstacles, attachments=[],
                                        self_collisions=False, disabled_collisions=set(),
                                        custom_limits=problem.custom_limits, max_distance=MAX_DISTANCE)
        handles = []
        while True:
            sample = sample_fn()
            if not collision_fn(sample):
                q = Conf(robot, joints, sample)
                handles.extend(draw_point(point_from_conf(sample), size=0.05))
                yield (q,)
    return gen_fn


def get_motion_fn(problem, max_distance=0.45, weights=np.array([1, 1, 0]), resolutions=BASE_RESOLUTIONS):
    def fn(body, q1, q2):
        joints = get_base_joints(body)
        distance_fn = get_base_distance_fn(weights)
        extend_fn = get_extend_fn(body, joints, resolutions=resolutions)
        collision_fn = get_collision_fn(body, joints, problem.obstacles, attachments=[],
                                        self_collisions=False, disabled_collisions=set(),
                                        custom_limits=problem.custom_limits, max_distance=MAX_DISTANCE)
        # TODO: degree
        distance = distance_fn(q1.values, q2.values)
        if max_distance < distance:
            return None
        path = [q1.values] + list(extend_fn(q1.values, q2.values))
        if any(map(collision_fn, path)):
            return None
        handles = add_line(point_from_conf(q1.values), point_from_conf(q2.values))
        t = create_trajectory(body, joints, path)
        return (t,)
    return fn


def get_cost_fn(problem):
    def fn(r, q1, q2):
        return get_distance(q1.values[:2], q2.values[:2])
    return fn

#######################################################

def create_vertices(problem, robot, samples, samples_per_ft2=8, **kwargs):
    lower, upper = problem.limits
    area = np.product(upper - lower)
    print('Area:', area)
    num_samples = int(samples_per_ft2 * area)

    gen_fn = get_sample_gen_fn(problem)
    init = []
    for (q,) in islice(gen_fn(robot), num_samples):
        samples.append(q)
        init += [('Conf', robot, q)]
    return init


def create_edges(problem, robot, samples, **kwargs):
    motion_fn = get_motion_fn(problem, **kwargs)

    edges = []
    init = []
    for q1, q2 in product(samples, repeat=2):
        if q1 == q2:
            continue
        result = motion_fn(robot, q1, q2)
        if result is None:
            continue
        (t,) = result
        edges.append((q1, q2))
        init += [('Motion', robot, q1, q2, t), ('Traj', robot, t)]
    return init