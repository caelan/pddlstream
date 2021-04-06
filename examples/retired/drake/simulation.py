import time

import numpy as np
from pydrake.systems.analysis import Simulator
from pydrake.trajectories import PiecewisePolynomial

from examples.drake.utils import get_configuration, user_input


def step_trajectories(diagram, diagram_context, plant_context, trajectories, time_step=0.001, teleport=False):
    diagram.Publish(diagram_context)
    user_input('Step?')
    for traj in trajectories:
        if teleport:
            traj.path = traj.path[::len(traj.path)-1]
        for _ in traj.iterate(plant_context):
            diagram.Publish(diagram_context)
            if time_step is None:
                user_input('Continue?')
            else:
                time.sleep(time_step)
    user_input('Finish?')

##################################################


def get_hold_spline(mbp, context, robot):
    q_knots_kuka = np.zeros((2, 7))
    q_knots_kuka[0] = get_configuration(mbp, context, robot)  # Second is velocity
    return PiecewisePolynomial.ZeroOrderHold([0, 1], q_knots_kuka.T)


def convert_splines(mbp, robot, gripper, context, trajectories):
    # TODO: move to trajectory class
    print
    splines, gripper_setpoints = [], []
    for i, traj in enumerate(trajectories):
        traj.path[-1].assign(context)
        joints = traj.joints
        if len(joints) == 8: # TODO: remove inclusion of door joints
            joints = joints[:7]

        if len(joints) == 2:
            spline = get_hold_spline(mbp, context, robot)
        elif len(joints) == 7:
            spline = traj.spline()
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