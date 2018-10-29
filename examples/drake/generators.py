from itertools import product

import numpy as np

from examples.drake.iiwa_utils import get_top_cylinder_grasps, open_wsg50_gripper
from examples.drake.motion import get_collision_fn, plan_joint_motion
from examples.drake.utils import get_relative_transform, set_world_pose, set_joint_position, get_body_pose, \
    get_base_body, sample_aabb_placement, is_model_colliding, get_movable_joints, create_transform, \
    solve_inverse_kinematics, set_joint_positions, get_box_from_geom


class RelPose(object):
    def __init__(self, mbp, parent, child, transform): # TODO: operate on bodies
        self.mbp = mbp
        self.parent = parent
        self.child = child
        self.transform = transform

    def assign(self, context):
        parent_pose = get_relative_transform(self.mbp, context, self.parent)
        child_pose = parent_pose.multiply(self.transform)
        set_world_pose(self.mbp, context, self.child, child_pose)

    def __repr__(self):
        return '{}()'.format(self.__class__.__name__)


class Config(object):
    def __init__(self, joints, positions):
        assert len(joints) == len(positions)
        self.joints = joints
        self.positions = tuple(positions)

    def assign(self, context):
        for joint, position in zip(self.joints, self.positions):
            set_joint_position(joint, context, position)

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, len(self.joints))


class Trajectory(object):
    def __init__(self, path, attachments=[]):
        self.path = tuple(path)
        self.attachments = attachments

    def iterate(self, context):
        for conf in self.path[1:]:
            conf.assign(context)
            for attach in self.attachments: # TODO: topological sort
                attach.assign(context)
            yield

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, len(self.path))

##################################################

def get_stable_gen(task, context):
    mbp = task.mbp
    world = mbp.world_frame
    box_from_geom = get_box_from_geom(task.scene_graph)

    def gen(obj_name, surface_name, body_name):
        obj = mbp.GetModelInstanceByName(obj_name)
        surface = mbp.GetModelInstanceByName(surface_name)
        surface_body = mbp.GetBodyByName(body_name, surface)
        #surface_pose = get_world_pose(mbp, context, surface)
        surface_pose = get_body_pose(context, surface_body)

        #object_aabb = AABBs[obj_name]
        #surface_aabb = AABBs[surface_name]
        object_aabb = box_from_geom[int(obj), get_base_body(mbp, obj).name()]
        surface_aabb = box_from_geom[int(surface), surface_body.name()]
        for local_pose in sample_aabb_placement(object_aabb, surface_aabb):
            world_pose = surface_pose.multiply(local_pose)
            set_world_pose(mbp, context, obj, world_pose)
            if not is_model_colliding(mbp, context, obj, obstacles=task.fixed): #obstacles=(fixed + [surface])):
                pose = RelPose(mbp, world, obj, world_pose)
                yield pose,
    return gen


def get_grasp_gen(task):
    mbp = task.mbp
    gripper_frame = get_base_body(mbp, task.gripper).body_frame()
    box_from_geom = get_box_from_geom(task.scene_graph)

    def gen(obj_name):
        obj = mbp.GetModelInstanceByName(obj_name)
        #obj_frame = get_base_body(mbp, obj).body_frame()
        #aabb = AABBs[obj_name]
        aabb = box_from_geom[int(obj), get_base_body(mbp, obj).name()]

        for transform in get_top_cylinder_grasps(aabb):
            grasp = RelPose(mbp, gripper_frame, obj, transform)
            yield grasp,
    return gen


def get_ik_fn(mbp, context, robot, gripper, fixed=[], distance=0.1, step_size=0.01):
    direction = np.array([0, -1, 0])
    gripper_frame = get_base_body(mbp, gripper).body_frame()
    joints = get_movable_joints(mbp, robot)
    collision_pairs = set(product([robot, gripper], fixed))
    collision_fn = get_collision_fn(mbp, context, joints, collision_pairs=collision_pairs)

    def fn(obj_name, pose, grasp):
        grasp_pose = pose.transform.multiply(grasp.transform.inverse())
        solution = None
        path = []
        for t in list(np.arange(0, distance, step_size)) + [distance]:
            current_vector = t * direction / np.linalg.norm(direction)
            current_pose = grasp_pose.multiply(create_transform(translation=current_vector))
            solution = solve_inverse_kinematics(mbp, gripper_frame, current_pose, initial_guess=solution)
            if solution is None:
                return None
            positions = [solution[j.position_start()] for j in joints]
            # TODO: holding
            if collision_fn(positions):
                return None
            path.append(Config(joints, positions))
        traj = Trajectory(path)
        return path[-1], traj
    return fn

##################################################

def get_free_motion_fn(mbp, context, robot, gripper, fixed=[]):
    joints = get_movable_joints(mbp, robot)
    collision_pairs = set(product([robot, gripper], fixed))

    def fn(q1, q2):
        open_wsg50_gripper(mbp, context, gripper)
        set_joint_positions(joints, context, q1.positions)
        path = plan_joint_motion(mbp, context, joints, q2.positions, collision_pairs=collision_pairs)
        if path is None:
            return None
        traj = Trajectory([Config(joints, q) for q in path])
        #traj = Trajectory([q1, q2])
        return traj,
    return fn


def get_holding_motion_fn(mbp, context, robot, gripper, fixed=[]):
    joints = get_movable_joints(mbp, robot)
    collision_pairs = set(product([robot, gripper], fixed))

    def fn(q1, q2, o, g): # TODO: holding
        #close_wsg50_gripper(mbp, context, gripper)
        set_joint_positions(joints, context, q1.positions)
        path = plan_joint_motion(mbp, context, joints, q2.positions, collision_pairs=collision_pairs)
        if path is None:
            return None
        traj = Trajectory([Config(joints, q) for q in path], attachments=[g])
        #traj = Trajectory([q1, q2], attachments=[g])
        return traj,
    return fn