# -*- coding: utf8 -*-

import numpy as np
from pydrake.all import (
    BasicVector,
    LeafSystem,
    PortDataType,
    MathematicalProgram,
    AbstractValue,
)

from pydrake.multibody.multibody_tree import MultibodyForces
from examples.drake.utils import get_model_actuators


plan_types = [
    "JointSpacePlan",
    "TaskSpacePlan",
]

SLEEP = 0.5

class Plan:
    def __init__(self,
                 type = None,
                 trajectory = None,
                 start_time = None,
                 R_WE_ref = None):
        self.type = type
        self.traj = trajectory
        if trajectory is not None:
            self.traj_d = trajectory.derivative(1)
        self.t_start = start_time
        self.R_WE_ref = R_WE_ref

class KukaMultibodyController(LeafSystem):
    def __init__(self, plant, kuka_model_instance,
                 control_period=0.005,
                 print_period=0.5):
        LeafSystem.__init__(self)
        self.set_name("Kuka Controller")

        self.plant = plant
        self.tree = plant.tree()
        self.print_period = print_period
        self.last_print_time = -print_period
        self.control_period = control_period
        self.model_instance = kuka_model_instance
        self.nu = len(get_model_actuators(plant, kuka_model_instance))
        self.nq = plant.num_positions()

        v_dummy = np.arange(plant.num_velocities())
        self.controlled_inds = self.tree.get_velocities_from_array(
            kuka_model_instance, v_dummy).astype(int)


        self.robot_state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   plant.num_positions() +
                                   plant.num_velocities())
        self.plan_input_port = \
            self._DeclareInputPort(PortDataType.kAbstractValued, 0)

        self._DeclareDiscreteState(self.nu)
        self._DeclarePeriodicDiscreteUpdate(period_sec=control_period)
        self._DeclareVectorOutputPort(BasicVector(self.nu),
            self._DoCalcVectorOutput)

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        # (This makes sure relevant event handlers get called.)
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_control_input = discrete_state. \
            get_mutable_vector().get_mutable_value()
        t= context.get_time()
        x = self.EvalVectorInput(
            context, self.robot_state_input_port.get_index()).get_value()
        plan = self.EvalAbstractInput(
            context, self.plan_input_port.get_index()).get_value()
        q = x[:self.nq]
        v = x[self.nq:]

        tree = self.plant.tree()

        q_kuka = tree.get_positions_from_array(self.model_instance, q)
        v_kuka = tree.get_velocities_from_array(self.model_instance, v)

        context = self.plant.CreateDefaultContext()

        x_mutable = tree.get_mutable_multibody_state_vector(context)
        x_mutable[:] = x

        qDDt_desired = np.zeros(self.plant.num_velocities())

        if plan.type == "JointSpacePlan":
            q_kuka_ref = plan.traj.value(t - plan.t_start).flatten()
            v_kuka_ref = plan.traj_d.value(t - plan.t_start).flatten()

            qerr_kuka = (q_kuka_ref - q_kuka)
            verr_kuka = (v_kuka_ref - v_kuka)

            # Get the full LHS of the manipulator equations
            # given the current config and desired accelerations
            qDDt_desired[self.controlled_inds] = 1000.*qerr_kuka + 100*verr_kuka

        lhs = tree.CalcInverseDynamics(context=context,
                                       known_vdot=qDDt_desired,
                                       external_forces=MultibodyForces(tree))
        new_u = lhs[self.controlled_inds]
        new_control_input[:] = new_u

    def _DoCalcVectorOutput(self, context, y_data):
        if (self.print_period and
                context.get_time() - self.last_print_time
                >= self.print_period):
            print "t: ", context.get_time()
            self.last_print_time = context.get_time()
        control_output = context.get_discrete_state_vector().get_value()
        y = y_data.get_mutable_value()
        # Get the ith finger control output
        y[:] = control_output[:]


class HandController(LeafSystem):
    def __init__(self, plant,
                 model_instance,
                 control_period=0.001):
        LeafSystem.__init__(self)
        self.set_name("Hand Controller")

        self.max_force = 100.  # gripper max closing / opening force
        self.plant = plant
        self.model_instance = model_instance

        self.nu = len(get_model_actuators(plant, model_instance))
        self.nq = plant.num_positions()

        self.robot_state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   plant.num_positions() +
                                   plant.num_velocities())

        self.setpoint_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   1)

        self._DeclareDiscreteState(self.nu)
        self._DeclarePeriodicDiscreteUpdate(period_sec=control_period)
        self._DeclareVectorOutputPort(
            BasicVector(self.nu),
            self._DoCalcVectorOutput)

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        # (This makes sure relevant event handlers get called.)
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_control_input = discrete_state. \
            get_mutable_vector().get_mutable_value()
        x = self.EvalVectorInput(
            context, self.robot_state_input_port.get_index()).get_value()

        gripper_width_des = self.EvalVectorInput(
            context, self.setpoint_input_port.get_index()).get_value()

        q_full = x[:self.nq]
        v_full = x[self.nq:]
        tree = self.plant.tree()
        q_hand = tree.get_positions_from_array(self.model_instance, q_full)
        q_hand_des = np.array([-gripper_width_des[0], gripper_width_des[0]])
        v_hand = tree.get_velocities_from_array(self.model_instance, v_full)
        v_hand_des = np.zeros(2)

        qerr_hand = q_hand_des - q_hand
        verr_hand = v_hand_des - v_hand

        Kp = 1000.
        Kv = 100.
        new_control_input[:] = np.clip(
            Kp * qerr_hand + Kv * verr_hand, -self.max_force, self.max_force)

    def _DoCalcVectorOutput(self, context, y_data):
        control_output = context.get_discrete_state_vector().get_value()
        y = y_data.get_mutable_value()
        # Get the ith finger control output
        y[:] = control_output[:]


class ManipStateMachine(LeafSystem):
    '''
    Encodes the high-level logic for the manipulation system.

    This state machine receives a list of trajectories, and sends them to
    the robot in a chronological order. Each trajectory is live through
    the system's kuka_plan_output_port for
    (qtraj_list[i].end_time() + 0.5) seconds, after which qtraj_list[i+1]
    will become live.
    '''
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self.set_name("Manipulation State Machine")
        self.loaded = False

        self.nq = plant.num_positions()
        self.plant = plant

        self.robot_state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   plant.num_positions() +
                                   plant.num_velocities())

        self._DeclareDiscreteState(1)
        self._DeclarePeriodicDiscreteUpdate(period_sec=0.01)
        self.kuka_plan_output_port = \
            self._DeclareAbstractOutputPort(
                lambda: AbstractValue.Make(Plan()), self.CalcPlan)
        self.hand_setpoint_output_port = \
            self._DeclareVectorOutputPort(
                BasicVector(1), self._DoCalcHandSetpointOutput)

    def Load(self, qtraj_list, gripper_setpoint_list):
        assert len(qtraj_list) == len(gripper_setpoint_list)
        self.gripper_setpoint_list = gripper_setpoint_list
        self.qtraj_list = qtraj_list

        self.t_traj = np.zeros(len(qtraj_list) + 1)
        for i in range(len(qtraj_list)):
            self.t_traj[i+1] = self.t_traj[i] + SLEEP + qtraj_list[i].end_time()

        self.current_traj_idx = 0
        self.current_plan = Plan(type=plan_types[0],
                            trajectory=self.qtraj_list[0],
                            start_time=0)
        self.loaded = True

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_state = discrete_state. \
            get_mutable_vector().get_mutable_value()
        # Close gripper after plan has been executed
        new_state[:] = self.gripper_setpoint_list[self.current_traj_idx]

    def CalcPlan(self, context, y_data):
        t = context.get_time()
        x = self.EvalVectorInput(
            context, self.robot_state_input_port.get_index()).get_value()
        q = x[:self.nq]
        v = x[self.nq:]

        new_traj_idx = 0
        for i in range(len(self.qtraj_list)):
            if t >= self.t_traj[i] and t < self.t_traj[i+1]:
                new_traj_idx = i
                break
        if self.current_traj_idx < new_traj_idx:
            self.current_traj_idx = new_traj_idx
            self.current_plan = Plan(type=plan_types[0],
                            trajectory=self.qtraj_list[new_traj_idx],
                            start_time=t)
        y_data.set_value(self.current_plan)


    def _DoCalcHandSetpointOutput(self, context, y_data):
        state = context.get_discrete_state_vector().get_value()
        y = y_data.get_mutable_value()
        # Get the ith finger control output
        y[:] = state[0]