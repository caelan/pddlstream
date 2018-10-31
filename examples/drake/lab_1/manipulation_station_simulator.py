"""
Runs the manipulation_station example with a simple tcl/tk joint slider ui for
directly tele-operating the joints.
"""

import numpy as np
import time

from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import (ManipulationStation,
                                    ManipulationStationHardwareInterface)

from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.util.eigen_geometry import Isometry3

from underactuated.meshcat_visualizer import MeshcatVisualizer
from manipulation_station_plan_runner import KukaPlanRunner, ManipStateMachine
from pydrake.systems.primitives import SignalLogger


def RenderSystemWithGraphviz(system, output_file="system_view.gz"):
    ''' Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. '''
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)

class ManipulationStationSimulator:
    def __init__(self, time_step,
                 object_file_path,
                 object_base_link_name,
                 is_hardware=False):
        self.object_base_link_name = object_base_link_name
        self.time_step = time_step
        self.is_hardware=is_hardware

        # Finalize manipulation station by adding manipuland.
        self.station = ManipulationStation(self.time_step)
        self.station.AddCupboard()
        self.plant = self.station.get_mutable_multibody_plant()
        self.object = AddModelFromSdfFile(
            file_name=object_file_path,
            model_name="object",
            plant=self.station.get_mutable_multibody_plant(),
            scene_graph=self.station.get_mutable_scene_graph() )
        self.station.Finalize()


    def RunSimulation(self, plans_list, gripper_setpoint_list,
                      extra_time=0, real_time_rate=1.0, q0_kuka=np.zeros(7)):
        '''
        Constructs a Diagram that sends commands to ManipulationStation.
        :param plans_list:
        :param gripper_setpoint_list:
        :param extra_time:
        :param real_time_rate:
        :param q0_kuka:
        :return:
        '''
        builder = DiagramBuilder()
        builder.AddSystem(self.station)

        # Add plan runner
        plan_runner = KukaPlanRunner(self.plant)
        builder.AddSystem(plan_runner)
        builder.Connect(plan_runner.get_output_port(0),
                            self.station.GetInputPort("iiwa_position"))

        # Add state machine.
        state_machine = ManipStateMachine(
            plant=self.plant,
            kuka_plans=plans_list,
            gripper_setpoint_list=gripper_setpoint_list)

        builder.AddSystem(state_machine)
        builder.Connect(state_machine.kuka_plan_output_port,
                        plan_runner.plan_input_port)
        builder.Connect(state_machine.hand_setpoint_output_port,
                        self.station.GetInputPort("wsg_position"))
        builder.Connect(state_machine.gripper_force_limit_output_port,
                        self.station.GetInputPort("wsg_force_limit"))
        builder.Connect(self.station.GetOutputPort("iiwa_position_measured"),
                        state_machine.iiwa_position_input_port)

        # Add meshcat visualizer
        scene_graph = self.station.get_mutable_scene_graph()
        viz = MeshcatVisualizer(scene_graph)
        builder.AddSystem(viz)
        builder.Connect(self.station.GetOutputPort("pose_bundle"),
                        viz.get_input_port(0))

        # Add logger
        iiwa_position_command_log = builder.AddSystem(
            SignalLogger(self.station.GetInputPort("iiwa_position").size()))
        iiwa_position_command_log._DeclarePeriodicPublish(0.005)
        builder.Connect(
            plan_runner.get_output_port(0), iiwa_position_command_log.get_input_port(0))

        # build diagram
        diagram = builder.Build()
        viz.load()
        time.sleep(2.0)
        RenderSystemWithGraphviz(diagram)

        # construct simulator
        simulator = Simulator(diagram)

        context = diagram.GetMutableSubsystemContext(
            self.station, simulator.get_mutable_context())

        # set initial state of the robot
        self.station.SetIiwaPosition(q0_kuka, context)
        self.station.SetIiwaVelocity(np.zeros(7), context)
        self.station.SetWsgState(0.05, 0, context)

        # set initial hinge angles of the cupboard.
        left_hinge_joint = self.plant.GetJointByName("left_door_hinge")
        left_hinge_joint.set_angle(
            context=self.station.GetMutableSubsystemContext(self.plant, context), angle=-np.pi/2)

        right_hinge_joint = self.plant.GetJointByName("right_door_hinge")
        right_hinge_joint.set_angle(
            context=self.station.GetMutableSubsystemContext(self.plant, context), angle=np.pi/2)

        # set initial pose of the object
        X_WObject = Isometry3.Identity()
        X_WObject.set_translation([.6, 0, 0])
        self.plant.tree().SetFreeBodyPoseOrThrow(
           self.plant.GetBodyByName(self.object_base_link_name, self.object),
            X_WObject, self.station.GetMutableSubsystemContext(self.plant, context))

        # fix feedforward torque input to 0.
        context.FixInputPort(self.station.GetInputPort(
            "iiwa_feedforward_torque").get_index(), np.zeros(7))

        simulator.set_publish_every_time_step(False)
        simulator.set_target_realtime_rate(real_time_rate)
        simulator.Initialize()
        sim_duration = 0.
        for plan in plans_list:
            sim_duration += plan.get_duration() * 1.1
        sim_duration += extra_time
        print "simulation duration", sim_duration
        simulator.StepTo(sim_duration)

        return iiwa_position_command_log


    def RunRealRobot(self, plans_list, gripper_setpoint_list):
        '''
        Constructs a Diagram that sends commands to ManipulationStationHardwareInterface.
        :param plans_list:
        :param gripper_setpoint_list:
        :param extra_time:
        :param real_time_rate:
        :param q0_kuka:
        :return:
        '''
        builder = DiagramBuilder()
        self.station_hardware = ManipulationStationHardwareInterface()
        builder.AddSystem(self.station_hardware)

        # Add plan runner
        plan_runner = KukaPlanRunner(self.plant)
        builder.AddSystem(plan_runner)
        builder.Connect(plan_runner.get_output_port(0),
                            self.station_hardware.GetInputPort("iiwa_position"))

        # Add state machine.
        state_machine = ManipStateMachine(
            plant=self.plant,
            kuka_plans=plans_list,
            gripper_setpoint_list=gripper_setpoint_list)

        builder.AddSystem(state_machine)
        builder.Connect(state_machine.kuka_plan_output_port,
                        plan_runner.plan_input_port)
        builder.Connect(state_machine.hand_setpoint_output_port,
                        self.station_hardware.GetInputPort("wsg_position"))
        builder.Connect(state_machine.gripper_force_limit_output_port,
                        self.station_hardware.GetInputPort("wsg_force_limit"))
        builder.Connect(self.station_hardware.GetOutputPort("iiwa_position_measured"),
                        state_machine.iiwa_position_input_port)

        # Add logger
        iiwa_position_command_log = builder.AddSystem(
            SignalLogger(self.station_hardware.GetInputPort("iiwa_position").size()))
        builder.Connect(
            plan_runner.get_output_port(0), iiwa_position_command_log.get_input_port(0))

        # build diagram
        diagram = builder.Build()
        RenderSystemWithGraphviz(diagram)

        # construct simulator
        simulator = Simulator(diagram)

        context = diagram.GetMutableSubsystemContext(
            self.station_hardware, simulator.get_mutable_context())

        context.FixInputPort(self.station.GetInputPort(
            "iiwa_feedforward_torque").get_index(), np.zeros(7))

        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        # simulator.set_publish_at_initialization(False)
        sim_duration = 0.
        for plan in plans_list:
            sim_duration += plan.get_duration() * 1.1

        print "sending trajectories in 2 seconds..."
        time.sleep(1.0)
        print "sending trajectories in 1 second..."
        time.sleep(1.0)
        print "sending trajectories now!"
        simulator.StepTo(sim_duration)

        return iiwa_position_command_log
