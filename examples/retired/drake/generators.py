from itertools import product

import numpy as np

from examples.drake.iiwa_utils import open_wsg50_gripper, get_box_grasps, get_top_cylinder_grasps, \
    get_close_wsg50_positions, get_open_wsg50_positions
from examples.drake.motion import plan_joint_motion, plan_waypoints_joint_motion, \
    get_extend_fn, interpolate_translation, plan_workspace_motion, get_collision_fn, get_distance_fn
from examples.drake.utils import get_relative_transform, set_world_pose, set_joint_position, get_body_pose, \
    get_base_body, sample_aabb_placement, get_movable_joints, get_model_name, set_joint_positions, get_box_from_geom, \
    exists_colliding_pair, get_model_bodies, aabb_contains_point, bodies_from_models, get_model_aabb

from pydrake.trajectories import PiecewisePolynomial

RADIANS_PER_SECOND = np.pi / 4

class Pose(object):
    # TODO: unify Pose & Conf?
    def __init__(self, mbp, parent, child, transform):
        self.mbp = mbp
        self.parent = parent # body_frame
        self.child = child # model_index
        self.transform = transform

    @property
    def bodies(self):
        return get_model_bodies(self.mbp, self.child)

    def assign(self, context):
        parent_pose = get_relative_transform(self.mbp, context, self.parent)
        child_pose = parent_pose.multiply(self.transform)
        set_world_pose(self.mbp, context, self.child, child_pose)

    def __repr__(self):
        return '{}({}->{})'.format(self.__class__.__name__, get_model_name(self.mbp, self.child), self.parent.name())


class Conf(object):
    def __init__(self, joints, positions):
        assert len(joints) == len(positions)
        self.joints = joints
        self.positions = tuple(positions)

    @property
    def bodies(self): # TODO: descendants
        return {joint.child_body() for joint in self.joints}

    def assign(self, context):
        for joint, position in zip(self.joints, self.positions):
            set_joint_position(joint, context, position)

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, len(self.joints))


class Trajectory(object):
    def __init__(self, path, attachments=[], force_control=False):
        self.path = tuple(path)
        self.attachments = attachments
        self.force_control = force_control
        # TODO: store a common set of joints instead (take in joints and path and convert to confs)

    @property
    def joints(self):
        return self.path[0].joints

    @property
    def bodies(self):
        joint_bodies = {joint.child_body() for joint in self.joints}
        for attachment in self.attachments:
            joint_bodies.update(attachment.bodies)
        return joint_bodies

    def reverse(self):
        return self.__class__(self.path[::-1], self.attachments, self.force_control)

    def iterate(self, context):
        for conf in self.path[1:]:
            conf.assign(context)
            for attach in self.attachments: # TODO: topological sort
                attach.assign(context)
            yield

    def spline(self):
        distance_fn = get_distance_fn(self.joints)
        path = [q.positions[:len(self.joints)] for q in self.path]
        q_knots_kuka = np.vstack(path).T
        distances = [0.] + [distance_fn(q1, q2) for q1, q2 in zip(path, path[1:])]
        # TODO: increase time for pick/place & hold
        t_knots = np.cumsum(distances) / RADIANS_PER_SECOND  # TODO: this should be a max
        d, n = q_knots_kuka.shape
        return PiecewisePolynomial.Cubic(
            breaks=t_knots,
            knots=q_knots_kuka,
            knot_dot_start=np.zeros(d),
            knot_dot_end=np.zeros(d))

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, len(self.joints), len(self.path))


def get_open_trajectory(plant, gripper):
    gripper_joints = get_movable_joints(plant, gripper)
    gripper_extend_fn = get_extend_fn(gripper_joints)
    gripper_closed_conf = get_close_wsg50_positions(plant, gripper)
    gripper_path = list(gripper_extend_fn(gripper_closed_conf, get_open_wsg50_positions(plant, gripper)))
    gripper_path.insert(0, gripper_closed_conf)
    return Trajectory(Conf(gripper_joints, q) for q in gripper_path)

##################################################


def get_pose_gen(task, context, collisions=True, shrink=0.025):
    mbp = task.mbp
    world = mbp.world_frame()
    box_from_geom = get_box_from_geom(task.scene_graph)
    fixed = task.fixed_bodies() if collisions else []

    def gen(obj_name, surface):
        obj = mbp.GetModelInstanceByName(obj_name)
        obj_aabb, obj_from_box, _ = box_from_geom[int(obj), get_base_body(mbp, obj).name(), 0]
        collision_pairs = set(product(get_model_bodies(mbp, obj), fixed))
        #object_radius = min(object_aabb[:2])

        surface_body = mbp.GetBodyByName(surface.body_name, surface.model_index)
        surface_pose = get_body_pose(context, surface_body)
        surface_aabb, surface_from_box, _ = box_from_geom[int(surface.model_index), surface.body_name, surface.visual_index]

        for surface_box_from_obj_box in sample_aabb_placement(obj_aabb, surface_aabb, shrink=shrink):
            world_pose = surface_pose.multiply(surface_from_box).multiply(
                surface_box_from_obj_box).multiply(obj_from_box.inverse())
            pose = Pose(mbp, world, obj, world_pose)
            pose.assign(context)
            if not exists_colliding_pair(task.diagram, task.diagram_context, task.mbp, task.scene_graph, collision_pairs):
                yield (pose,)
    return gen


def get_grasp_gen_fn(task):
    mbp = task.mbp
    gripper_frame = get_base_body(mbp, task.gripper).body_frame()
    box_from_geom = get_box_from_geom(task.scene_graph)

    def gen(obj_name):
        obj = mbp.GetModelInstanceByName(obj_name)
        obj_aabb, obj_from_box, obj_shape = box_from_geom[int(obj), get_base_body(mbp, obj).name(), 0]
        #finger_aabb, finger_from_box, _ = box_from_geom[int(task.gripper), 'left_finger', 0]
        # TODO: union of bounding boxes

        if obj_shape == 'cylinder':
            grasp_gen = get_top_cylinder_grasps(obj_aabb)
        elif obj_shape == 'box':
            pitch = 2 * np.pi / 5 # 0 | np.pi/3 | 2 * np.pi / 5
            grasp_gen = get_box_grasps(obj_aabb, pitch_range=(pitch, pitch))
        else:
            raise NotImplementedError(obj_shape)

        for gripper_from_box in grasp_gen:
            gripper_from_obj = gripper_from_box.multiply(obj_from_box.inverse())
            grasp = Pose(mbp, gripper_frame, obj, gripper_from_obj)
            yield (grasp,)
    return gen

##################################################


def plan_frame_motion(plant, joints, frame, frame_path,
                      initial_guess=None, resolutions=None, collision_fn=lambda q: False):
    waypoints = plan_workspace_motion(plant, joints, frame, frame_path,
                                      initial_guess=initial_guess, collision_fn=collision_fn)
    if waypoints is None:
        return None
    return plan_waypoints_joint_motion(joints, waypoints, resolutions=resolutions, collision_fn=collision_fn)


def get_ik_gen_fn(task, context, collisions=True, max_failures=10, approach_distance=0.25, step_size=0.035):
    approach_vector = approach_distance * np.array([0, -1, 0])
    gripper_frame = get_base_body(task.mbp, task.gripper).body_frame()
    fixed = task.fixed_bodies() if collisions else []
    initial_guess = None
    #initial_guess = get_joint_positions(get_movable_joints(task.mbp, task.robot), context)

    def fn(robot_name, obj_name, obj_pose, obj_grasp):
        # TODO: if gripper/block in collision, return
        robot = task.mbp.GetModelInstanceByName(robot_name)
        joints = get_movable_joints(task.mbp, robot)
        collision_pairs = set(product(bodies_from_models(task.mbp, [robot, task.gripper]), fixed))
        collision_fn = get_collision_fn(task.diagram, task.diagram_context, task.mbp, task.scene_graph,
                                        joints, collision_pairs=collision_pairs) # TODO: while holding

        gripper_pose = obj_pose.transform.multiply(obj_grasp.transform.inverse())
        gripper_path = list(interpolate_translation(gripper_pose, approach_vector, step_size=step_size))

        attempts = 0
        last_success = 0
        while (attempts - last_success) < max_failures:
            attempts += 1
            obj_pose.assign(context)
            path = plan_frame_motion(task.plant, joints, gripper_frame, gripper_path,
                                     initial_guess=initial_guess, collision_fn=collision_fn)
            if path is None:
                continue
            print('IK attempts: {}'.format(attempts))
            traj = Trajectory([Conf(joints, q) for q in path], attachments=[obj_grasp])
            conf = traj.path[-1]
            yield (conf, traj)
            last_success = attempts
    return fn


def get_reachable_pose_gen_fn(task, context, collisions=True, **kwargs):
    pose_gen_fn = get_pose_gen(task, context, collisions=collisions)
    ik_gen_fn = get_ik_gen_fn(task, context, collisions=collisions, max_failures=1, **kwargs)
    def gen(r, o, g, s):
        for p, in pose_gen_fn(o, s):
            for (q, t) in ik_gen_fn(r, o, p, g):
                yield (p, q, t)
    return gen

##################################################

def get_body_path(body, context, joints, joint_path):
    body_path = []
    for conf in joint_path:
        set_joint_positions(joints, context, conf)
        body_path.append(get_body_pose(context, body))
    return body_path


def get_door_grasp(door_body, box_from_geom):
    pitch = np.pi/3 # np.pi/2
    grasp_length = 0.02
    target_shape, target_ori = 'cylinder', 1  # Second grasp is np.pi/2, corresponding to +y
    # target_shape, target_ori = 'box', 0 # left_door TODO: right_door
    for i in range(2):
        handle_aabb, handle_from_box, handle_shape = box_from_geom[int(door_body.model_instance()), door_body.name(), i]
        if handle_shape == target_shape:
            break
    else:
        raise RuntimeError(target_shape)
    [gripper_from_box] = list(get_box_grasps(handle_aabb, orientations=[target_ori],
                                             pitch_range=(pitch, pitch), grasp_length=grasp_length))
    return gripper_from_box.multiply(handle_from_box.inverse())


def get_pull_fn(task, context, collisions=True, max_attempts=25, step_size=np.pi / 16, approach_distance=0.05):
    # TODO: could also push the door either perpendicular or parallel
    # TODO: allow small rotation error perpendicular to handle
    # DoDifferentialInverseKinematics
    box_from_geom = get_box_from_geom(task.scene_graph)
    gripper_frame = get_base_body(task.mbp, task.gripper).body_frame()
    fixed = task.fixed_bodies() if collisions else []
    #approach_vector = approach_distance*np.array([0, -1, 0])

    def fn(robot_name, door_name, door_conf1, door_conf2):
        robot = task.mbp.GetModelInstanceByName(robot_name)
        robot_joints = get_movable_joints(task.mbp, robot)
        collision_pairs = set(product(bodies_from_models(task.mbp, [robot, task.gripper]), fixed))
        collision_fn = get_collision_fn(task.diagram, task.diagram_context, task.mbp, task.scene_graph,
                                        robot_joints, collision_pairs=collision_pairs)

        door_body = task.mbp.GetBodyByName(door_name)
        door_joints = door_conf1.joints
        gripper_from_door = get_door_grasp(door_body, box_from_geom)

        extend_fn = get_extend_fn(door_joints, resolutions=step_size*np.ones(len(door_joints)))
        door_joint_path = [door_conf1.positions] + list(extend_fn(door_conf1.positions, door_conf2.positions)) # TODO: check for collisions
        door_body_path = get_body_path(door_body, context, door_joints, door_joint_path)
        pull_cartesian_path = [door_pose.multiply(gripper_from_door.inverse()) for door_pose in door_body_path]

        #start_path = list(interpolate_translation(pull_cartesian_path[0], approach_vector))
        #end_path = list(interpolate_translation(pull_cartesian_path[-1], approach_vector))
        for _ in range(max_attempts):
            # TODO: could solve for kinematic solution of robot and door together
            pull_joint_waypoints = plan_workspace_motion(task.mbp, robot_joints, gripper_frame,
                                                         reversed(pull_cartesian_path), collision_fn=collision_fn)
            if pull_joint_waypoints is None:
                continue
            pull_joint_waypoints = pull_joint_waypoints[::-1]
            combined_joints = robot_joints + door_joints
            combined_waypoints = [list(rq) + list(dq) for rq, dq in zip(pull_joint_waypoints, door_joint_path)]
            pull_joint_path = plan_waypoints_joint_motion(combined_joints, combined_waypoints,
                                                          collision_fn=lambda q: False)
            if pull_joint_path is None:
                continue
            # TODO: approach & retreat path?
            robot_conf1 = Conf(robot_joints, pull_joint_waypoints[0])
            robot_conf2 = Conf(robot_joints, pull_joint_waypoints[-1])
            traj = Trajectory(Conf(combined_joints, combined_conf) for combined_conf in pull_joint_path)
            return (robot_conf1, robot_conf2, traj)
    return fn

##################################################

def get_aabb_sample_fn(joints, context, body, aabb, sample_fn):
    #sample_fn = get_sample_fn(joints)
    def fn():
        while True:
            q = sample_fn()
            set_joint_positions(joints, context, q)
            world_from_body = get_body_pose(context, body)
            point_world = world_from_body.translation()
            if aabb_contains_point(point_world, aabb):
                return q
    return fn

def parse_fluents(fluents, context, obstacles):
    attachments = []
    for fact in fluents:
        predicate = fact[0]
        if predicate == 'atconf':
            name, conf = fact[1:]
            conf.assign(context)
            obstacles.update(conf.bodies)
        elif predicate == 'atpose':
            name, pose = fact[1:]
            pose.assign(context)
            obstacles.update(pose.bodies)
        elif predicate == 'atgrasp':
            robot, name, grasp = fact[1:]
            attachments.append(grasp)
        else:
            raise ValueError(predicate)
    return attachments


def get_motion_fn(task, context, collisions=True, teleport=False):
    gripper = task.gripper

    def fn(robot_name, conf1, conf2, fluents=[]):
        robot = task.mbp.GetModelInstanceByName(robot_name)
        joints = get_movable_joints(task.mbp, robot)
        if teleport:
            traj = Trajectory([conf1, conf2])
            return (traj,)

        moving = bodies_from_models(task.mbp, [robot, gripper])
        obstacles = set(task.fixed_bodies())
        attachments = parse_fluents(fluents, context, obstacles)
        for grasp in attachments:
            moving.update(grasp.bodies)
        obstacles -= moving

        collision_pairs = set(product(moving, obstacles)) if collisions else set()
        collision_fn = get_collision_fn(task.diagram, task.diagram_context, task.mbp, task.scene_graph,
                                        joints, collision_pairs=collision_pairs, attachments=attachments)
        #weights = np.ones(len(joints))
        #weights = np.array([sum(weights[i:]) for i in range(len(weights))])
        distance_fn = None # TODO: adjust the resolutions

        open_wsg50_gripper(task.mbp, context, gripper)
        path = plan_joint_motion(joints, conf1.positions, conf2.positions,
                                 distance_fn=distance_fn, collision_fn=collision_fn,
                                 restarts=10, iterations=75, smooth=50)
        if path is None:
            return None
        traj = Trajectory([Conf(joints, q) for q in path], attachments=attachments)
        return (traj,)
    return fn

##################################################

def get_collision_test(task, context, collisions=True):
    # TODO: precompute and hash?
    def test(traj, obj_name, pose):
        if not collisions:
            return False
        moving = bodies_from_models(task.mbp, [task.robot, task.gripper])
        moving.update(traj.bodies)
        obstacles = set(pose.bodies) - moving
        collision_pairs = set(product(moving, obstacles))
        if not collision_pairs:
            return False
        pose.assign(context)
        for _ in traj.iterate(context):
            if exists_colliding_pair(task.diagram, task.diagram_context, task.mbp, task.scene_graph, collision_pairs):
                return True
        return False
    return test
