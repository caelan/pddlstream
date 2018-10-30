from itertools import product

import numpy as np

from examples.drake.iiwa_utils import get_top_cylinder_grasps, open_wsg50_gripper, get_box_grasps, WSG50_LEFT_FINGER
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
    world = mbp.world_frame()
    box_from_geom = get_box_from_geom(task.scene_graph)

    def gen(obj_name, surface):
        obj = mbp.GetModelInstanceByName(obj_name)
        surface_body = mbp.GetBodyByName(surface.body_name, surface.model_index)
        surface_pose = get_body_pose(context, surface_body)

        #object_aabb, object_local = AABBs[obj_name], Isometry3.Identity()
        #surface_aabb, surface_local = AABBs[surface_name], Isometry3.Identity()
        object_aabb, object_local = box_from_geom[int(obj), get_base_body(mbp, obj).name(), 0]
        surface_aabb, surface_local = box_from_geom[int(surface.model_index), surface.body_name, surface.visual_index]
        for surface_from_object in sample_aabb_placement(object_aabb, surface_aabb):
            world_pose = surface_pose.multiply(surface_local).multiply(
                surface_from_object).multiply(object_local.inverse())
            pose = RelPose(mbp, world, obj, world_pose)
            pose.assign(context)
            if not is_model_colliding(mbp, context, obj, obstacles=task.fixed): #obstacles=(fixed + [surface])):
                yield pose,
    return gen


def get_grasp_gen(task):
    mbp = task.mbp
    gripper_frame = get_base_body(mbp, task.gripper).body_frame()
    box_from_geom = get_box_from_geom(task.scene_graph)

    def gen(obj_name):
        obj = mbp.GetModelInstanceByName(obj_name)
        #obj_aabb, obj_from_box = AABBs[obj_name], Isometry3.Identity()
        obj_aabb, obj_from_box = box_from_geom[int(obj), get_base_body(mbp, obj).name(), 0]
        #finger_aabb, finger_from_box = box_from_geom[int(task.gripper), 'left_finger', 0]
        # TODO: union of bounding boxes

        #for gripper_from_box in get_top_cylinder_grasps(obj_aabb):
        for gripper_from_box in get_box_grasps(obj_aabb):
            gripper_from_obj = gripper_from_box.multiply(obj_from_box.inverse())
            grasp = RelPose(mbp, gripper_frame, obj, gripper_from_obj)
            yield grasp,
    return gen


def get_ik_fn(task, context, max_failures=1, distance=0.1, step_size=0.01):
    direction = np.array([0, -1, 0])
    gripper_frame = get_base_body(task.mbp, task.gripper).body_frame()
    joints = get_movable_joints(task.mbp, task.robot)
    collision_pairs = set(product([task.robot, task.gripper], task.fixed))
    collision_fn = get_collision_fn(task.mbp, context, joints, collision_pairs=collision_pairs)

    def fn(obj_name, pose, grasp):
        grasp_pose = pose.transform.multiply(grasp.transform.inverse())
        solution = None
        path = []
        attempts = 0
        last_success = 0
        while (attempts - last_success) < max_failures:
            attempts += 1
            for t in list(np.arange(0, distance, step_size)) + [distance]:
                current_vector = t * direction / np.linalg.norm(direction)
                current_pose = grasp_pose.multiply(create_transform(translation=current_vector))
                solution = solve_inverse_kinematics(task.mbp, gripper_frame, current_pose, initial_guess=solution)
                if solution is None:
                    return None
                positions = [solution[j.position_start()] for j in joints]
                # TODO: holding
                if collision_fn(positions):
                    return None
                path.append(Config(joints, positions))
            traj = Trajectory(path)
            last_success = attempts
            return path[-1], traj
    return fn

##################################################

def get_free_motion_fn(task, context):
    joints = get_movable_joints(task.mbp, task.robot)
    collision_pairs = set(product([task.robot, task.gripper], task.fixed))

    def fn(q1, q2):
        open_wsg50_gripper(task.mbp, context, task.gripper)
        set_joint_positions(joints, context, q1.positions)
        path = plan_joint_motion(task.mbp, context, joints, q2.positions, collision_pairs=collision_pairs)
        if path is None:
            return None
        traj = Trajectory([Config(joints, q) for q in path])
        #traj = Trajectory([q1, q2])
        return traj,
    return fn


def get_holding_motion_fn(task, context):
    joints = get_movable_joints(task.mbp, task.robot)

    def fn(q1, q2, o, g):
        collision_pairs = set(product([task.robot, task.gripper, o], task.fixed))
        #close_wsg50_gripper(mbp, context, gripper)
        set_joint_positions(joints, context, q1.positions)
        path = plan_joint_motion(task.mbp, context, joints, q2.positions,
                                 collision_pairs=collision_pairs, attachments=[g])
        if path is None:
            return None
        traj = Trajectory([Config(joints, q) for q in path], attachments=[g])
        #traj = Trajectory([q1, q2], attachments=[g])
        return traj,
    return fn