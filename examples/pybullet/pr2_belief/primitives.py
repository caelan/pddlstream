from __future__ import print_function

import numpy as np

from examples.discrete_belief.dist import DDist
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Command, Pose, Conf, Grasp, Trajectory, create_trajectory, Attach, Detach
from examples.pybullet.utils.pybullet_tools.pr2_utils import HEAD_LINK_NAME, get_visual_detections, \
    PR2_GROUPS, visible_base_generator, inverse_visibility, get_kinect_registrations, get_detection_cone, get_viewcone, \
    MAX_KINECT_DISTANCE, plan_scan_path, plan_pause_scan_path, get_group_joints, get_group_conf, set_group_conf
from examples.pybullet.utils.pybullet_tools.pr2_problems import get_fixed_bodies
from examples.pybullet.utils.pybullet_tools.utils import link_from_name, create_mesh, set_pose, get_link_pose, \
    wait_for_duration, unit_pose, remove_body, is_center_stable, get_body_name, get_name, joints_from_names, \
    point_from_pose, set_base_values, plan_waypoints_joint_motion, base_values_from_pose, \
    pairwise_collision, get_pose, plan_direct_joint_motion, BodySaver, get_center_extent


def get_vis_gen(problem, max_attempts=25, base_range=(0.5, 1.5)):
    robot = problem.robot
    fixed = get_fixed_bodies(problem)
    #base_joints = joints_from_names(robot, PR2_GROUPS['base'])
    head_joints = get_group_joints(robot, 'head')
    def gen(o, p):
        # default_conf = arm_conf(a, g.carry)
        # joints = get_arm_joints(robot, a)
        # TODO: check collisions with fixed links
        target_point = point_from_pose(p.value)
        base_generator = visible_base_generator(robot, target_point, base_range)
        while True:
            for _ in range(max_attempts):
                set_pose(o, p.value)
                base_conf = next(base_generator)
                set_base_values(robot, base_conf)  # TODO: use pose or joint?
                # set_joint_positions(robot, base_joints, base_conf)
                if any(pairwise_collision(robot, b) for b in fixed):
                    continue
                head_conf = inverse_visibility(robot, target_point)
                if head_conf is None:  # TODO: test if visible
                    continue
                bp = Pose(robot, get_pose(robot))
                hq = Conf(robot, head_joints, head_conf)
                yield (bp, hq)
                break
            else:
                yield None

    return gen

def get_scan_gen(state, max_attempts=25, base_range=(0.5, 1.5)):
    task = state.task
    robot = task.robot
    #base_joints = joints_from_names(robot, PR2_GROUPS['base'])
    head_joints = get_group_joints(robot, 'head')
    vis_gen = get_vis_gen(task, max_attempts=max_attempts, base_range=base_range)
    tilt = np.pi / 6
    def gen(o, p):
        if o in task.rooms:
            #bp = Pose(robot, unit_pose())
            bp = state.poses[robot]
            #hq = Conf(robot, head_joints, np.zeros(len(head_joints)))
            #ht = create_trajectory(robot, head_joints, plan_pause_scan_path(robot, tilt=tilt))
            waypoints = plan_scan_path(task.robot, tilt=tilt)
            set_group_conf(robot, 'head', waypoints[0])
            path = plan_waypoints_joint_motion(robot, head_joints, waypoints[1:],
                                          obstacles=None, self_collisions=False)
            ht = create_trajectory(robot, head_joints, path)
            yield bp, ht.path[0], ht
        else:
            for bp, hq in vis_gen(o, p):
                ht = Trajectory([hq])
                yield bp, ht.path[0], ht
    return gen

#######################################################

def plan_head_traj(robot, head_conf):
    # head_conf = get_joint_positions(robot, head_joints)
    # head_path = [head_conf, hq.values]
    head_joints = get_group_joints(robot, 'head')
    head_path = plan_direct_joint_motion(robot, head_joints, head_conf,
                                  obstacles=None, self_collisions=False)
    assert(head_path is not None)
    return create_trajectory(robot, head_joints, head_path)

def get_target_point(conf):
    # TODO: center of mass instead?
    # TODO: look such that cone bottom touches at bottom
    with BodySaver(conf.body):
        conf.step()
        center, _ = get_center_extent(conf.body)
        return center

def get_target_path(trajectory):
    # TODO: only do bounding boxes for moving links on the trajectory
    target_path = []
    for conf in trajectory.path:
        # TODO: could draw the target point path sequence
        target_path.append(get_target_point(conf))
    return target_path

def inspect_trajectory(trajectory):
    if not trajectory.path:
        return
    robot = trajectory.path[0].body
    # TODO: minimum distance of some sort (to prevent from looking at the bottom)
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
                                            obstacles=None, self_collisions=False)
    assert(head_path is not None)
    return create_trajectory(robot, head_joints, head_path)

def move_look_trajectory(trajectory):
    # TODO: pr2 movement restrictions
    base_path = [pose.to_base_conf() for pose in trajectory.path]
    if not base_path:
        return trajectory
    robot = base_path[0].body
    target_path = get_target_path(trajectory)
    waypoints = []
    index = 0
    with BodySaver(robot):
        current_conf = base_values_from_pose(get_pose(robot))
        for i, conf in enumerate(base_path): # TODO: just do two loops?
            conf.step()
            while index < len(target_path):
                if i < index:
                    # Don't look at past or current conf
                    target_point = target_path[index]
                    head_conf = inverse_visibility(robot, target_point)
                    if head_conf is not None:
                        break
                index += 1
            else:
                head_conf = get_group_conf(robot, 'head')
            #print(conf.values, head_conf)
            waypoints.append(np.concatenate([conf.values, head_conf]))
        #joints = tuple(base_path[0].joints) + tuple(get_group_joints(robot, 'head'))
        joints = get_group_joints(robot, 'base') + get_group_joints(robot, 'head')
        set_pose(robot, unit_pose())
        set_group_conf(robot, 'base', current_conf)
        path = plan_waypoints_joint_motion(robot, joints, waypoints,
                                           obstacles=None, self_collisions=False)
    new_traj = create_trajectory(robot, joints, path)
    #Pose(robot, pose_from_base_values(q, bq1.value))
    #new_traj.path.append(Pose(...))
    # TODO: need to convert add Pose commands for the switch from joints to pose...
    raise NotImplementedError()

#######################################################

class AttachCone(Command): # TODO: make extend Attach?
    def __init__(self, robot):
        self.robot = robot
        self.group = 'head'
    def apply(self, state, **kwargs):
        self.cone = get_viewcone(color=(1, 0, 0, 0.5))
        state.poses[self.cone] = None
        attach = Attach(self.robot, self.group, Pose(self.cone, unit_pose()), self.cone)
        attach.apply(state, **kwargs)
    def __repr__(self):
        return '{}()'.format(self.__class__.__name__)

class DetachCone(Command): # TODO: make extend Detach?
    def __init__(self, attach):
        self.attach = attach
    def apply(self, state, **kwargs):
        cone = self.attach.cone
        detach = Detach(self.attach.robot, self.attach.group, cone)
        detach.apply(state, **kwargs)
        del state.poses[cone]
        remove_body(cone)
    def __repr__(self):
        return '{}()'.format(self.__class__.__name__)

def get_cone_commands(robot):
    attach = AttachCone(robot)
    detach = DetachCone(attach)
    return attach, detach

#######################################################

def get_observation_fn(surface, p_look_fp=0, p_look_fn=0):
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

    def __init__(self, robot, surface):
        self.robot = robot
        self.surface = surface
        self.link = link_from_name(robot, HEAD_LINK_NAME)

    def apply(self, state, **kwargs):
        # TODO: identify surface automatically
        cone = get_viewcone(color=(1, 0, 0, 0.5))
        set_pose(cone, get_link_pose(self.robot, self.link))
        wait_for_duration(self._duration) # TODO: don't sleep if no viewer?
        remove_body(cone)

        detections = get_visual_detections(self.robot)
        print('Detections:', detections)
        for body, dist in state.b_on.items():
            obs = (body in detections) and (is_center_stable(body, self.surface))
            if obs or (self.surface not in state.task.rooms):
                # TODO: make a command for scanning a room instead?
                dist.obsUpdate(get_observation_fn(self.surface), obs)
        #state.localized.update(detections)
        # TODO: pose for each object that can be real or fake

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

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, get_body_name(self.surface))

#######################################################

class Detect(Command):
    def __init__(self, robot, surface, body):
        self.robot = robot
        self.surface = surface
        self.body = body

    def apply(self, state, **kwargs):
        return
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

    def __init__(self, robot, body):
        self.robot = robot
        self.body = body
        self.link = link_from_name(robot, HEAD_LINK_NAME)

    def step(self):
        # TODO: filter for target object and location?
        return get_kinect_registrations(self.robot)

    def apply(self, state, **kwargs):
        mesh, _ = get_detection_cone(self.robot, self.body, depth=MAX_KINECT_DISTANCE)
        assert(mesh is not None)
        cone = create_mesh(mesh, color=(0, 1, 0, 0.5))
        set_pose(cone, get_link_pose(self.robot, self.link))
        wait_for_duration(self._duration)
        # time.sleep(1.0)
        remove_body(cone)
        state.registered.add(self.body)

    control = step

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                  get_name(self.body))

