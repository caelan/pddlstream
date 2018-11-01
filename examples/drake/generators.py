from itertools import product

import numpy as np

from examples.drake.iiwa_utils import get_top_cylinder_grasps, open_wsg50_gripper, get_box_grasps, WSG50_LEFT_FINGER
from examples.drake.motion import get_collision_fn, plan_joint_motion, plan_waypoints_joint_motion, \
    refine_joint_path, get_extend_fn
from examples.drake.utils import get_relative_transform, set_world_pose, set_joint_position, get_body_pose, \
    get_base_body, sample_aabb_placement, is_model_colliding, get_movable_joints, create_transform, \
    solve_inverse_kinematics, set_joint_positions, get_box_from_geom, get_joint_positions, get_joint_limits


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
    #pitch_range = (0, 0) # Top grasps
    #pitch_range = (np.pi/3, np.pi/3)
    pitch_range = (2*np.pi/5, 2*np.pi/5)
    #pitch_range = (-np.pi/2, np.pi/2)

    def gen(obj_name):
        obj = mbp.GetModelInstanceByName(obj_name)
        #obj_aabb, obj_from_box = AABBs[obj_name], Isometry3.Identity()
        obj_aabb, obj_from_box = box_from_geom[int(obj), get_base_body(mbp, obj).name(), 0]
        #finger_aabb, finger_from_box = box_from_geom[int(task.gripper), 'left_finger', 0]
        # TODO: union of bounding boxes

        #for gripper_from_box in get_top_cylinder_grasps(obj_aabb):
        for gripper_from_box in get_box_grasps(obj_aabb, pitch_range=pitch_range):
            gripper_from_obj = gripper_from_box.multiply(obj_from_box.inverse())
            grasp = RelPose(mbp, gripper_frame, obj, gripper_from_obj)
            yield grasp,
    return gen

def interpolate_translation(transform, translation, step_size=0.01):
    distance = np.linalg.norm(translation)
    if distance == 0:
        yield transform
        return
    direction = np.array(translation) / distance
    for t in list(np.arange(0, distance, step_size)) + [distance]:
        yield transform.multiply(create_transform(translation=t * direction))

def plan_workspace_motion(mbp, context, joints, frame, frame_path, initial_guess=None, **kwargs):
    collision_fn = get_collision_fn(mbp, context, joints, **kwargs)
    solution = initial_guess
    waypoints = []
    for frame_pose in frame_path:
        solution = solve_inverse_kinematics(mbp, frame, frame_pose, initial_guess=solution)
        if solution is None:
            return None
        positions = [solution[j.position_start()] for j in joints]
        if collision_fn(positions):
            return None
        waypoints.append(positions)
    return waypoints

def get_ik_fn(task, context, max_failures=5, distance=0.15, step_size=0.04):
    #distance = 0.0
    approach_vector = distance*np.array([0, -1, 0])
    gripper_frame = get_base_body(task.mbp, task.gripper).body_frame()
    joints = get_movable_joints(task.mbp, task.robot)
    collision_pairs = set(product([task.robot, task.gripper], task.fixed))
    initial_guess = None
    #initial_guess = get_joint_positions(joints, context) # TODO: start with initial

    def fn(robot_name, obj_name, pose, grasp):
        # TODO: if gripper/block in collision, return
        grasp_pose = pose.transform.multiply(grasp.transform.inverse())
        gripper_path = list(interpolate_translation(grasp_pose, approach_vector))

        attempts = 0
        last_success = 0
        while (attempts - last_success) < max_failures:
            attempts += 1
            waypoints = plan_workspace_motion(task.mbp, context, joints, gripper_frame, gripper_path,
                                              initial_guess=initial_guess, collision_pairs=collision_pairs) # TODO: while holding
            if waypoints is None:
                continue
            set_joint_positions(joints, context, waypoints[0])
            path = plan_waypoints_joint_motion(task.mbp, context, joints, waypoints[1:], collision_pairs=collision_pairs)
            if path is None:
                continue
            #path = refine_joint_path(joints, path)
            traj = Trajectory([Config(joints, q) for q in path])
            #print(attempts - last_success)
            last_success = attempts
            return traj.path[-1], traj
    return fn

def get_door_fn(task, context, max_attempts=25, step_size=np.pi/16):
    cupboard = task.mbp.GetModelInstanceByName('cupboard')
    box_from_geom = get_box_from_geom(task.scene_graph)
    gripper_frame = get_base_body(task.mbp, task.gripper).body_frame()
    robot_joints = get_movable_joints(task.mbp, task.robot)

    distance = 0.01
    approach_vector = distance*np.array([0, -1, 0])
    # TODO: could also push the door

    def fn(body_name):
        body = task.mbp.GetBodyByName(body_name, cupboard)
        door_joints = [joint for joint in get_movable_joints(task.mbp, cupboard)
                       if joint.child_body().name() == body_name]

        limit = 0.497*np.pi # Seems to be the limit

        #print(get_joint_limits(door_joints[0])) # -inf, +inf
        start_positions = get_joint_positions(door_joints, context)
        #end_positions = [-np.pi/2]
        #end_positions = [-limit]
        end_positions = [-0.49*np.pi]

        extend_fn = get_extend_fn(door_joints, resolutions=step_size*np.ones(len(door_joints)))
        door_joint_path = [start_positions] + list(extend_fn(start_positions, end_positions)) # TODO: check for collisions
        door_pose_path = []
        for robot_conf in door_joint_path:
            set_joint_positions(door_joints, context, robot_conf)
            door_pose_path.append(get_body_pose(context, body))

        handle_aabb, handle_from_box = box_from_geom[int(cupboard), body_name, 1]
        grasps = list(get_box_grasps(handle_aabb, pitch_range=(np.pi/2, np.pi/2), grasp_length=0.01))
        gripper_from_box = grasps[1] # Second grasp is np.pi/2, corresponding to +y
        gripper_from_obj = gripper_from_box.multiply(handle_from_box.inverse())
        pull_path = [body_pose.multiply(gripper_from_obj.inverse()) for body_pose in door_pose_path]

        start_path = list(interpolate_translation(pull_path[0], approach_vector))
        end_path = list(interpolate_translation(pull_path[-1], approach_vector))
        gripper_path = start_path[::-1] + pull_path[1:] + end_path[1:]

        for _ in range(max_attempts):
            robot_path = plan_workspace_motion(task.mbp, context, robot_joints, gripper_frame, reversed(gripper_path))
            if robot_path is None:
                continue
            robot_path = robot_path[::-1]
            for door_conf, robot_conf in zip(door_joint_path, robot_path):
                set_joint_positions(door_joints, context, door_conf)
                set_joint_positions(robot_joints, context, robot_conf)
                yield robot_conf

            #grasp = RelPose(mbp, gripper_frame, obj, gripper_from_obj)
            #yield grasp,

    return fn

##################################################

def get_motion_fn(task, context, fluents=[]):
    joints = get_movable_joints(task.mbp, task.robot)
    collision_pairs = set(product([task.robot, task.gripper], task.fixed))

    def fn(robot_name, conf1, conf2):
        open_wsg50_gripper(task.mbp, context, task.gripper)
        set_joint_positions(joints, context, conf1.positions)
        path = plan_joint_motion(task.mbp, context, joints, conf2.positions,
                                 collision_pairs=collision_pairs,
                                 restarts=5, iterations=75, smooth=100)
        if path is None:
            return None
        #path = refine_joint_path(joints, path)
        traj = Trajectory([Config(joints, q) for q in path])
        return traj,
    return fn


def get_holding_motion_fn(task, context):
    joints = get_movable_joints(task.mbp, task.robot)

    def fn(robot_name, conf1, conf2, obj_name, grasp):
        #obj = task.mbp.GetModelInstanceByName(obj_name)
        collision_pairs = set(product([task.robot, task.gripper, grasp.child], task.fixed))
        #close_wsg50_gripper(mbp, context, gripper)
        set_joint_positions(joints, context, conf1.positions)
        path = plan_joint_motion(task.mbp, context, joints, conf2.positions,
                                 collision_pairs=collision_pairs, attachments=[grasp],
                                 restarts=5, iterations=75, smooth=100)
        if path is None:
            return None
        #path = refine_joint_path(joints, path)
        traj = Trajectory([Config(joints, q) for q in path], attachments=[grasp])
        return traj,
    return fn