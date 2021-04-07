import numpy as np

from examples.pybullet.utils.pybullet_tools.pr2_primitives import Conf, control_commands, Trajectory, create_trajectory
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_group_joints, \
    visible_base_generator, inverse_visibility
from examples.pybullet.utils.pybullet_tools.pr2_primitives import apply_commands, get_custom_limits
from examples.pybullet.utils.pybullet_tools.utils import pairwise_collision, draw_base_limits, WorldSaver, get_point, \
    all_between, get_bodies, plan_direct_joint_motion, plan_joint_motion

VIS_RANGE = 2.5
COM_RANGE = 5


def get_fixed_bodies(problems):
    return set(get_bodies()) - set(problems.rovers + problems.rocks + problems.soils)


def get_reachable_test(problem):
    def test(bq):
        return True
    return test


def get_inv_vis_gen(problem, max_range=VIS_RANGE, custom_limits={}):
    base_range = (0, max_range)
    robot = problem.rovers[0]
    fixed = get_fixed_bodies(problem)
    def gen(objective):
        base_joints = get_group_joints(robot, 'base')
        head_joints = get_group_joints(robot, 'head')
        target_point = get_point(objective)
        base_generator = visible_base_generator(robot, target_point, base_range)
        lower_limits, upper_limits = get_custom_limits(robot, base_joints, custom_limits)
        while True:
            base_conf = next(base_generator)
            if not all_between(lower_limits, base_conf, upper_limits):
                continue
            bq = Conf(robot, base_joints, base_conf)
            bq.assign()
            if any(pairwise_collision(robot, b) for b in fixed):
                continue
            head_conf = inverse_visibility(robot, target_point)
            if head_conf is None:  # TODO: test if visible
                continue
            hq = Conf(robot, head_joints, head_conf)
            y = None
            yield (bq, hq, y)
    return gen


def get_inv_com_gen(problem, **kwargs):
    return get_inv_vis_gen(problem, max_range=COM_RANGE, **kwargs)


def get_above_gen(problem, custom_limits={}):
    rover = problem.rovers[0]
    fixed = get_fixed_bodies(problem)
    def gen(rock):
        base_joints = get_group_joints(rover, 'base')
        x, y, _ = get_point(rock)
        lower_limits, upper_limits = get_custom_limits(rover, base_joints, custom_limits)
        while True:
            theta = np.random.uniform(-np.pi, np.pi)
            base_conf = [x, y, theta]
            if not all_between(lower_limits, base_conf, upper_limits):
                continue
            bq = Conf(rover, base_joints, base_conf)
            bq.assign()
            if any(pairwise_collision(rover, b) for b in fixed):
                continue
            yield (bq,)
    return gen


def get_base_motion_fn(problem, custom_limits={}, teleport=False):
    rover = problem.rovers[0]
    fixed = get_fixed_bodies(problem)
    def test(bq1, bq2):
        if teleport:
            ht = Trajectory([bq1, bq2])
        else:
            bq1.assign()
            path = plan_joint_motion(rover, bq1.joints, bq2.values, custom_limits=custom_limits,
                                     obstacles=fixed, self_collisions=False)
            if path is None:
                return None
            ht = create_trajectory(rover, bq2.joints, path)
        return (ht,)
    return test


def get_head_motion_fn(problem, teleport=False):
    rover = problem.rovers[0]
    def test(hq1, hq2):
        if teleport:
            ht = Trajectory([hq1, hq2])
        else:
            hq1.assign()
            path = plan_direct_joint_motion(rover, hq1.joints, hq2.values, self_collisions=False)
            if path is None:
                return None
            ht = create_trajectory(rover, hq1.joints, path)
        return (ht,)
    return test