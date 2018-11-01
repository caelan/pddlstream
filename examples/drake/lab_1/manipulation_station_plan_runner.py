import numpy as np
from pydrake.all import (
    BasicVector,
    LeafSystem,
    PortDataType,
    MathematicalProgram,
    AbstractValue,
)
from pydrake.trajectories import (
    PiecewisePolynomial
)

from robot_plans import JointSpacePlan, PlanBase


class KukaPlanRunner(LeafSystem):
    def __init__(self, plant, control_period=0.005, print_period=0.5):
        LeafSystem.__init__(self)
        self.set_name("kuka_plan_runner")

        #TODO: access actuation ports with model_instance

        self.nu = 7
        self.plant = plant
        self.tree = plant.tree()
        self.print_period = print_period
        self.last_print_time = -print_period
        self.control_period = control_period

        self.nq = plant.num_positions()

        self.plan_input_port = \
            self._DeclareInputPort(PortDataType.kAbstractValued, 0)

        self._DeclareDiscreteState(self.nu)
        self._DeclarePeriodicPublish(control_period)
        self.q_robot_reference_output_port = \
            self._DeclareVectorOutputPort(
                BasicVector(self.nu), self._DoCalcVectorOutput)
        
    def _DoCalcVectorOutput(self, context, y_data):
        t= context.get_time()
        plan = self.EvalAbstractInput(
            context, self.plan_input_port.get_index()).get_value()

        t_plan = t - plan.start_time
        new_q = np.zeros(7)

        if plan.type == "JointSpacePlan":
            # The desired joint angle and velocities at this time (t_plan)
            new_q[:] = plan.traj.value(t_plan).flatten()

        if (self.print_period and
                t - self.last_print_time >= self.print_period):
            print "t: ", context.get_time()
            self.last_print_time = context.get_time()

        y = y_data.get_mutable_value()
        y[:] = new_q[:]


class ManipStateMachine(LeafSystem):
    '''
    Encodes the high-level logic for the manipulation system.

    This state machine receives a list of Plans, and sends them to
    the robot in a chronological order. Each trajectory is live through
    the system's kuka_plan_output_port for
    (kuka_plans[i].duration() + 0.5) seconds, after which kuka_plans[i+1]
    will become live.
    '''
    def __init__(self, plant, kuka_plans, gripper_setpoint_list):
        LeafSystem.__init__(self)
        self.set_name("Manipulation State Machine")
        # Append plan_list with a plan that moves the robot from its current position to
        # plan_list[0].traj.value(0)
        kuka_plans.insert(0, JointSpacePlan())
        gripper_setpoint_list.insert(0, 0.05)
        self.move_to_home_duration_sec = 10.0
        kuka_plans[0].duration = self.move_to_home_duration_sec

        # Add a five-second zero order hold to hold the current position of the robot
        kuka_plans.insert(0, JointSpacePlan())
        gripper_setpoint_list.insert(0, 0.05)
        self.zero_order_hold_duration_sec = 5.0
        kuka_plans[0].duration = self.zero_order_hold_duration_sec

        assert len(kuka_plans) == len(gripper_setpoint_list)
        self.gripper_setpoint_list = gripper_setpoint_list
        self.kuka_plans_list = kuka_plans


        self.num_plans = len(kuka_plans)
        self.t_plan = np.zeros(self.num_plans + 1)
        self.t_plan[1] = self.zero_order_hold_duration_sec
        self.t_plan[2] = self.move_to_home_duration_sec + self.t_plan[1]
        for i in range(2, self.num_plans):
            self.t_plan[i + 1] = \
                self.t_plan[i] + kuka_plans[i].get_duration() * 1.1
        print self.t_plan

        self.nq = plant.num_positions()
        self.plant = plant

        self._DeclareDiscreteState(1)
        self._DeclarePeriodicDiscreteUpdate(period_sec=0.01)
        self.kuka_plan_output_port = \
            self._DeclareAbstractOutputPort("iiwa_plan",
                lambda: AbstractValue.Make(PlanBase()), self.GetCurrentPlan)
        self.hand_setpoint_output_port = \
            self._DeclareVectorOutputPort(
                "gripper_setpoint", BasicVector(1), self.CalcHandSetpointOutput)
        self.gripper_force_limit_output_port = \
            self._DeclareVectorOutputPort(
                "force_limit", BasicVector(1), self.CalcForceLimitOutput)
        self.iiwa_position_input_port = \
            self._DeclareInputPort("iiwa_position", PortDataType.kVectorValued, 7)

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_state = discrete_state. \
            get_mutable_vector().get_mutable_value()
        # Close gripper after plan has been executed
        new_state[:] = self.gripper_setpoint_list[self.current_plan_idx]

    def GetCurrentPlan(self, context, y_data):
        t = context.get_time()

        if self.kuka_plans_list[0].traj is None:
            q_current = self.EvalVectorInput(
                context, self.iiwa_position_input_port.get_index()).get_value()
            q0 = self.kuka_plans_list[2].traj.value(0).flatten()

            # zero order hold
            q_knots_kuka = np.zeros((2,7))
            q_knots_kuka[0] = q_current
            qtraj = PiecewisePolynomial.ZeroOrderHold(
                [0, self.zero_order_hold_duration_sec], q_knots_kuka.T)
            self.kuka_plans_list[0] = JointSpacePlan(qtraj)

            # move to the starting position of trajectory plans
            t_knots = np.array(
                [0., self.move_to_home_duration_sec / 2, self.move_to_home_duration_sec])
            q_knots_kuka = np.zeros((3, 7))
            q_knots_kuka[0] = q_current
            q_knots_kuka[2] = q0
            q_knots_kuka[1] = (q_knots_kuka[0] + q_knots_kuka[2]) / 2
            qtraj = PiecewisePolynomial.Cubic(
                t_knots, q_knots_kuka.T, np.zeros(7), np.zeros(7))

            self.kuka_plans_list[1] = JointSpacePlan(qtraj)

            # set current plan
            self.current_plan_idx = 0
            self.current_plan = self.kuka_plans_list[0]
            self.current_plan.set_start_time(0.)

        new_plan_idx = 0
        for i in range(self.num_plans):
            if t >= self.t_plan[i] and t < self.t_plan[i + 1]:
                new_plan_idx = i
                break
        if self.current_plan_idx < new_plan_idx:
            self.current_plan_idx = new_plan_idx
            self.current_plan = self.kuka_plans_list[new_plan_idx]
            self.current_plan.set_start_time(t)
            # print "switching to new plan at %f"%t

        y_data.set_value(self.current_plan)


    def CalcHandSetpointOutput(self, context, y_data):
        state = context.get_discrete_state_vector().get_value()
        y = y_data.get_mutable_value()
        # Get the ith finger control output
        y[:] = state[0]

    def CalcForceLimitOutput(self, context, output):
        output.SetAtIndex(0, 40.0)


