from __future__ import print_function

import numpy as np

from examples.discrete_belief.dist import DDist
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Command, Pose, Conf, Trajectory, \
    create_trajectory, Attach, Detach, get_target_path, SELF_COLLISIONS
from examples.pybullet.utils.pybullet_tools.pr2_problems import get_fixed_bodies
from examples.pybullet.utils.pybullet_tools.pr2_utils import HEAD_LINK_NAME, get_visual_detections, \
    visible_base_generator, inverse_visibility, get_kinect_registrations, get_detection_cone, get_viewcone, \
    MAX_KINECT_DISTANCE, plan_scan_path, get_group_joints, get_group_conf, set_group_conf
from examples.pybullet.utils.pybullet_tools.utils import link_from_name, create_mesh, set_pose, get_link_pose, \
    wait_for_duration, unit_pose, remove_body, is_center_stable, get_body_name, get_name, point_from_pose, \
    plan_waypoints_joint_motion, pairwise_collision, plan_direct_joint_motion, BodySaver, set_joint_positions, \
    INF, get_length, multiply, wait_for_user, LockRenderer, set_color, RED, GREEN, apply_alpha, dump_body, \
    get_link_subtree, child_link_from_joint, get_link_name

VIS_RANGE = (0.5, 1.5)
REG_RANGE = (0.5, 1.5)
ROOM_SCAN_TILT = np.pi / 6
P_LOOK_FP = 0
P_LOOK_FN = 0 # 1e-1

def get_in_range_test(task, range):
    # TODO: could just test different visibility w/o ranges
    def test(o, p, bq):
        if o in task.rooms:
            return True
        target_xy = point_from_pose(p.value)[:2]
        base_xy = bq.values[:2]
        return range[0] <= get_length(np.array(target_xy) - base_xy) <= range[1]
    return test

def get_vis_base_gen(task, base_range, collisions=False):
    robot = task.robot
    base_joints = get_group_joints(robot, 'base')
    obstacles = get_fixed_bodies(task) if collisions else []

    def gen(o, p):
        if o in task.rooms: # TODO: predicate instead
            return
        # default_conf = arm_conf(a, g.carry)
        # joints = get_arm_joints(robot, a)
        # TODO: check collisions with fixed links
        target_point = point_from_pose(p.value)
        base_generator = visible_base_generator(robot, target_point, base_range)
        while True:
            set_pose(o, p.value)  # p.assign()
            bq = Conf(robot, base_joints, next(base_generator))
            # bq = Pose(robot, get_pose(robot))
            bq.assign()
            if any(pairwise_collision(robot, b) for b in obstacles):
                yield None
            else:
                yield (bq,)
    # TODO: return list_fn & accelerated
    return gen

def get_inverse_visibility_fn(task, collisions=True):
    robot = task.robot
    head_joints = get_group_joints(robot, 'head')
    obstacles = get_fixed_bodies(task) if collisions else []

    def fn(o, p, bq):
        set_pose(o, p.value) # p.assign()
        bq.assign()
        if o in task.rooms:
            waypoints = plan_scan_path(task.robot, tilt=ROOM_SCAN_TILT)
            set_group_conf(robot, 'head', waypoints[0])
            path = plan_waypoints_joint_motion(robot, head_joints, waypoints[1:],
                                          obstacles=obstacles, self_collisions=SELF_COLLISIONS)
            if path is None:
                return None
            ht = create_trajectory(robot, head_joints, path)
            hq = ht.path[0]
        else:
            target_point = point_from_pose(p.value)
            head_conf = inverse_visibility(robot, target_point)
            if head_conf is None: # TODO: test if visible
                return None
            hq = Conf(robot, head_joints, head_conf)
            ht = Trajectory([hq])
        return (hq, ht)
    return fn

#######################################################

def plan_head_traj(task, head_conf):
    robot = task.robot
    obstacles = get_fixed_bodies(task) # TODO: movable objects
    # head_conf = get_joint_positions(robot, head_joints)
    # head_path = [head_conf, hq.values]
    head_joints = get_group_joints(robot, 'head')
    head_path = plan_direct_joint_motion(robot, head_joints, head_conf,
                                  obstacles=obstacles, self_collisions=SELF_COLLISIONS)
    assert(head_path is not None)
    return create_trajectory(robot, head_joints, head_path)

def inspect_trajectory(task, trajectory):
    if not trajectory.path:
        return
    robot = trajectory.path[0].body
    obstacles = get_fixed_bodies(task) # TODO: movable objects
    # TODO: minimum distance of some sort (to prevent from looking at the bottom)
    # TODO: custom lower limit as well
    head_waypoints = []
    for target_point in get_target_path(trajectory):
        head_conf = inverse_visibility(robot, target_point)
        # TODO: could also draw the sequence of inspected points as the head moves
        if head_conf is None:
            continue
        head_waypoints.append(head_conf)
    head_joints = get_group_joints(robot, 'head')
    #return create_trajectory(robot, head_joints, head_waypoints)
    head_path = plan_waypoints_joint_motion(robot, head_joints, head_waypoints,
                                            obstacles=obstacles, self_collisions=SELF_COLLISIONS)
    assert(head_path is not None)
    return create_trajectory(robot, head_joints, head_path)

def move_look_trajectory(task, trajectory, max_tilt=np.pi / 6):  # max_tilt=INF):
    # TODO: implement a minimum distance instead of max_tilt
    # TODO: pr2 movement restrictions
    #base_path = [pose.to_base_conf() for pose in trajectory.path]
    base_path = trajectory.path
    if not base_path:
        return trajectory
    obstacles = get_fixed_bodies(task) # TODO: movable objects
    robot = base_path[0].body
    target_path = get_target_path(trajectory)
    waypoints = []
    index = 0
    with BodySaver(robot):
        #current_conf = base_values_from_pose(get_pose(robot))
        for i, conf in enumerate(base_path): # TODO: just do two loops?
            conf.assign()
            while index < len(target_path):
                if i < index:
                    # Don't look at past or current conf
                    target_point = target_path[index]
                    head_conf = inverse_visibility(robot, target_point) # TODO: this is slightly slow
                    #print(index, target_point, head_conf)
                    if (head_conf is not None) and (head_conf[1] < max_tilt):
                        break
                index += 1
            else:
                head_conf = get_group_conf(robot, 'head')
            set_group_conf(robot, 'head', head_conf)
            #print(i, index, conf.values, head_conf) #, get_pose(robot))
            waypoints.append(np.concatenate([conf.values, head_conf]))
    joints = tuple(base_path[0].joints) + tuple(get_group_joints(robot, 'head'))
    #joints = get_group_joints(robot, 'base') + get_group_joints(robot, 'head')
    #set_pose(robot, unit_pose())
    #set_group_conf(robot, 'base', current_conf)
    path = plan_waypoints_joint_motion(robot, joints, waypoints,
                                       obstacles=obstacles, self_collisions=SELF_COLLISIONS)
    return create_trajectory(robot, joints, path)
    #Pose(robot, pose_from_base_values(q, bq1.value))
    #new_traj.path.append(Pose(...))

#######################################################

class AttachCone(Command): # TODO: make extend Attach?
    def __init__(self, robot):
        self.robot = robot
        self.group = 'head'
        self.cone = None
    def apply(self, state, **kwargs):
        with LockRenderer():
            self.cone = get_viewcone(color=apply_alpha(RED, 0.5))
            state.poses[self.cone] = None
            cone_pose = Pose(self.cone, unit_pose())
            attach = Attach(self.robot, self.group, cone_pose, self.cone)
            attach.assign()
            wait_for_duration(1e-2)
        for _ in attach.apply(state, **kwargs):
            yield
    def __repr__(self):
        return '{}()'.format(self.__class__.__name__)

class DetachCone(Command): # TODO: make extend Detach?
    def __init__(self, attach):
        self.attach = attach
    def apply(self, state, **kwargs):
        cone = self.attach.cone
        detach = Detach(self.attach.robot, self.attach.group, cone)
        for _ in detach.apply(state, **kwargs):
            yield
        del state.poses[cone]
        remove_body(cone)
        wait_for_duration(1e-2)
    def __repr__(self):
        return '{}()'.format(self.__class__.__name__)

def get_cone_commands(robot):
    attach = AttachCone(robot)
    detach = DetachCone(attach)
    return attach, detach

#######################################################

def get_observation_fn(surface, p_look_fp=P_LOOK_FP, p_look_fn=P_LOOK_FN):
    # TODO: clip probabilities so doesn't become zero
    def fn(s):
        # P(obs | s1=loc1, a=control_loc)
        if s == surface:
            return DDist({True: 1 - p_look_fn,
                          False: p_look_fn})
        return DDist({True: p_look_fp,
                      False: 1 - p_look_fp})
    return fn

# TODO: update whether localized on scene

class Scan(Command):
    _duration = 0.5

    def __init__(self, robot, surface, detect=True, camera_frame=HEAD_LINK_NAME):
        self.robot = robot
        self.surface = surface
        self.camera_frame = camera_frame
        self.link = link_from_name(robot, self.camera_frame)
        self.detect = detect

    def apply(self, state, **kwargs):
        # TODO: identify surface automatically
        with LockRenderer():
            cone = get_viewcone(color=apply_alpha(RED, 0.5))
            set_pose(cone, get_link_pose(self.robot, self.link))
            wait_for_duration(1e-2)
        wait_for_duration(self._duration) # TODO: don't sleep if no viewer?
        remove_body(cone)
        wait_for_duration(1e-2)

        if self.detect:
            # TODO: the collision geometries are being visualized
            # TODO: free the renderer
            head_joints = get_group_joints(self.robot, 'head')
            exclude_links = set(get_link_subtree(self.robot, child_link_from_joint(head_joints[-1])))
            detections = get_visual_detections(self.robot, camera_link=self.camera_frame, exclude_links=exclude_links,)
                                               #color=apply_alpha(RED, 0.5))
            print('Detections:', detections)
            for body, dist in state.b_on.items():
                obs = (body in detections) and (is_center_stable(body, self.surface))
                if obs or (self.surface not in state.task.rooms):
                    # TODO: make a command for scanning a room instead?
                    dist.obsUpdate(get_observation_fn(self.surface), obs)
            #state.localized.update(detections)
        # TODO: pose for each object that can be real or fake
        yield

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, get_body_name(self.surface))


class ScanRoom(Command):
    _tilt = np.pi / 6

    def __init__(self, robot, surface):
        self.robot = robot
        self.surface = surface

    def apply(self, state, **kwargs):
        assert(self.surface in state.task.rooms)
        obs_fn = get_observation_fn(self.surface)
        for body, dist in state.b_on.items():
            if 0 < dist.prob(self.surface):
                dist.obsUpdate(obs_fn, True)
        # detections = get_visual_detections(self.robot)
        #print('Detections:', detections)
        #for body, dist in state.b_on.items():
        #    obs = (body in detections) and (is_center_stable(body, self.surface))
        #    dist.obsUpdate(obs_fn, obs)
        yield

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, get_body_name(self.surface))

#######################################################

class Detect(Command):
    def __init__(self, robot, surface, body):
        self.robot = robot
        self.surface = surface
        self.body = body

    def apply(self, state, **kwargs):
        yield
        # TODO: need to be careful that we don't move in between this...
        # detections = get_visual_detections(self.robot)
        # print('Detections:', detections)
        # dist = state.b_on[self.body]
        # obs = (self.body in detections)
        # dist.obsUpdate(get_observation_fn(self.surface), obs)
        # if obs:
        #     state.localized.add(self.body)
        # TODO: pose for each object that can be real or fake

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, get_name(self.surface), get_name(self.body))


class Register(Command):
    _duration = 1.0

    def __init__(self, robot, body, camera_frame=HEAD_LINK_NAME, max_depth=MAX_KINECT_DISTANCE):
        self.robot = robot
        self.body = body
        self.camera_frame = camera_frame
        self.max_depth = max_depth
        self.link = link_from_name(robot, camera_frame)

    def control(self, **kwargs):
        # TODO: filter for target object and location?
        return get_kinect_registrations(self.robot)

    def apply(self, state, **kwargs):
        # TODO: check if actually can register
        mesh, _ = get_detection_cone(self.robot, self.body, camera_link=self.camera_frame, depth=self.max_depth)
        if mesh is None:
            wait_for_user()
        assert(mesh is not None)
        with LockRenderer():
            cone = create_mesh(mesh, color=apply_alpha(GREEN, 0.5))
            set_pose(cone, get_link_pose(self.robot, self.link))
            wait_for_duration(1e-2)
        wait_for_duration(self._duration)
        remove_body(cone)
        wait_for_duration(1e-2)
        state.registered.add(self.body)
        yield

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                  get_name(self.body))
