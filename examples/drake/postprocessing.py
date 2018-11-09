import time

import numpy as np
from pydrake.systems.analysis import Simulator

from examples.drake.generators import Trajectory, Conf
from examples.drake.iiwa_utils import get_close_wsg50_positions, get_open_wsg50_positions
from examples.drake.motion import get_extend_fn, get_distance_fn, waypoints_from_path
from examples.drake.utils import prune_fixed_joints, get_model_joints, get_configuration, user_input
from pydrake.trajectories import PiecewisePolynomial

def get_open_trajectory(plant, gripper):
    gripper_joints = prune_fixed_joints(get_model_joints(plant, gripper))
    gripper_extend_fn = get_extend_fn(gripper_joints)
    gripper_closed_conf = get_close_wsg50_positions(plant, gripper)
    gripper_path = list(gripper_extend_fn(gripper_closed_conf, get_open_wsg50_positions(plant, gripper)))
    gripper_path.insert(0, gripper_closed_conf)
    return Trajectory(Conf(gripper_joints, q) for q in gripper_path)


def postprocess_plan(mbp, gripper, plan):
    trajectories = []
    if plan is None:
        return trajectories

    open_traj = get_open_trajectory(mbp, gripper)
    close_traj = Trajectory(reversed(open_traj.path))
    # TODO: ceiling & orientation constraints
    # TODO: sampler chooses configurations that are far apart

    # TODO: maybe just specify the position sequence
    attachments = {}
    for name, args in plan:
        if name in ['clean', 'cook']:
            continue
        if name == 'pick':
            r, o, p, g, q, t = args
            trajectories.extend([Trajectory(reversed(t.path), attachments=attachments.values()), close_traj])
            attachments[o] = g
            trajectories.append(Trajectory(t.path, attachments=attachments.values()))
        elif name == 'place':
            r, o, p, g, q, t = args
            trajectories.extend([Trajectory(reversed(t.path), attachments=attachments.values()), open_traj])
            del attachments[o]
            trajectories.append(Trajectory(t.path, attachments=attachments.values()))
        elif name == 'pull':
            t = args[-1]
            trajectories.extend([close_traj, Trajectory(t.path, attachments=attachments.values()), open_traj])
        elif name == 'move':
            t = args[-1]
            trajectories.append(Trajectory(t.path, attachments=attachments.values()))
        else:
            raise NotImplementedError(name)

    return trajectories

def step_trajectories(diagram, diagram_context, context, trajectories, time_step=0.001, teleport=False):
    diagram.Publish(diagram_context)
    user_input('Step?')
    for traj in trajectories:
        if teleport:
            traj.path = traj.path[::len(traj.path)-1]
        for _ in traj.iterate(context):
            diagram.Publish(diagram_context)
            if time_step is None:
                user_input('Continue?')
            else:
                time.sleep(time_step)
    user_input('Finish?')

##################################################


RADIANS_PER_SECOND = np.pi / 4

def get_hold_spline(mbp, context, robot):
    q_knots_kuka = np.zeros((2, 7))
    q_knots_kuka[0] = get_configuration(mbp, context, robot)  # Second is velocity
    return PiecewisePolynomial.ZeroOrderHold([0, 1], q_knots_kuka.T)

def get_path_spline(joints, path):
    distance_fn = get_distance_fn(joints)
    path = [q.positions[:len(joints)] for q in path]
    # Need to be careful that we follow the path well
    # path = waypoints_from_path(joints, path)
    q_knots_kuka = np.vstack(path).T
    distances = [0.] + [distance_fn(q1, q2) for q1, q2 in zip(path, path[1:])]
    # TODO: increase time for pick/place & hold
    t_knots = np.cumsum(distances) / RADIANS_PER_SECOND # TODO: this should be a max
    d, n = q_knots_kuka.shape
    return PiecewisePolynomial.Cubic(
        breaks=t_knots,
        knots=q_knots_kuka,
        knot_dot_start=np.zeros(d),
        knot_dot_end=np.zeros(d))

def convert_splines(mbp, robot, gripper, context, trajectories):
    # TODO: move to trajectory class
    print
    splines, gripper_setpoints = [], []
    for i, traj in enumerate(trajectories):
        traj.path[-1].assign(context)
        joints = traj.path[0].joints
        if len(joints) == 8: # TODO: remove inclusion of door joints
            joints = joints[:7]

        if len(joints) == 2:
            spline = get_hold_spline(mbp, context, robot)
        elif len(joints) == 7:
            spline = get_path_spline(traj.joints, traj.path)
        else:
            raise ValueError(joints)
        d = spline.rows()
        n = spline.get_number_of_segments()
        print('{}) d={}, n={}, duration={:.3f}'.format(i, d, n, spline.duration(n-1)))
        splines.append(spline)
        _, gripper_setpoint = get_configuration(mbp, context, gripper)
        gripper_setpoints.append(gripper_setpoint)
    return splines, gripper_setpoints


def compute_duration(splines, extra_time=5.0):
    from .manipulation_station.robot_plans import PlanBase
    sim_duration = 0.
    for spline in splines:
        if isinstance(spline, PlanBase):
            sim_duration += spline.get_duration() * 1.1
        else:
            sim_duration += spline.end_time() + 0.5
    sim_duration += extra_time
    return sim_duration

##################################################

def simulate_splines(diagram, diagram_context, sim_duration, real_time_rate=1.0):
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(real_time_rate)
    simulator.Initialize()

    diagram.Publish(diagram_context)
    user_input('Simulate?')
    simulator.StepTo(sim_duration)
    user_input('Finish?')