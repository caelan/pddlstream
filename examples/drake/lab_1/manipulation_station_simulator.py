"""
Runs the manipulation_station example with a simple tcl/tk joint slider ui for
directly tele-operating the joints.
"""

import numpy as np
import time

from pydrake.examples.manipulation_station import (ManipulationStation,
                                    ManipulationStationHardwareInterface)
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.util.eigen_geometry import Isometry3

from underactuated.meshcat_visualizer import MeshcatVisualizer
# from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from manipulation_station_plan_runner import ManipStationPlanRunner
from pydrake.systems.primitives import SignalLogger, Demultiplexer, LogOutput


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

        # Initial pose of the object
        X_WObject = Isometry3.Identity()
        X_WObject.set_translation([.6, 0, 0])
        self.X_WObject = X_WObject

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

        # Add plan runner.
        plan_runner = ManipStationPlanRunner(
            station=self.station,
            kuka_plans=plans_list,
            gripper_setpoint_list=gripper_setpoint_list)

        builder.AddSystem(plan_runner)
        builder.Connect(plan_runner.hand_setpoint_output_port,
                        self.station.GetInputPort("wsg_position"))
        builder.Connect(plan_runner.gripper_force_limit_output_port,
                        self.station.GetInputPort("wsg_force_limit"))


        demux = builder.AddSystem(Demultiplexer(14, 7))
        builder.Connect(
            plan_runner.GetOutputPort("iiwa_position_and_torque_command"),
            demux.get_input_port(0))
        builder.Connect(demux.get_output_port(0),
                        self.station.GetInputPort("iiwa_position"))
        builder.Connect(demux.get_output_port(1),
                        self.station.GetInputPort("iiwa_feedforward_torque"))
        builder.Connect(self.station.GetOutputPort("iiwa_position_measured"),
                        plan_runner.iiwa_position_input_port)
        builder.Connect(self.station.GetOutputPort("iiwa_velocity_estimated"),
                        plan_runner.iiwa_velocity_input_port)

        # Add meshcat visualizer
        scene_graph = self.station.get_mutable_scene_graph()
        viz = MeshcatVisualizer(scene_graph,
                                is_drawing_contact_force = True,
                                plant = self.plant)
        self.viz = viz
        builder.AddSystem(viz)
        builder.Connect(self.station.GetOutputPort("pose_bundle"),
                        viz.GetInputPort("lcm_visualization"))
        builder.Connect(self.station.GetOutputPort("contact_results"),
                        viz.GetInputPort("contact_results"))

        # Add logger
        iiwa_position_command_log = LogOutput(demux.get_output_port(0), builder)
        iiwa_position_command_log._DeclarePeriodicPublish(0.005)

        iiwa_external_torque_log = LogOutput(
            self.station.GetOutputPort("iiwa_torque_external"), builder)
        iiwa_external_torque_log._DeclarePeriodicPublish(0.005)

        iiwa_position_measured_log = LogOutput(
            self.station.GetOutputPort("iiwa_position_measured"), builder)
        iiwa_position_measured_log._DeclarePeriodicPublish(0.005)

        wsg_state_log = LogOutput(
            self.station.GetOutputPort("wsg_state_measured"), builder)
        wsg_state_log._DeclarePeriodicPublish(0.1)

        wsg_command_log = LogOutput(
            plan_runner.hand_setpoint_output_port, builder)
        wsg_command_log._DeclarePeriodicPublish(0.1)


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
        self.station.SetWsgPosition(0.05, context)
        self.station.SetWsgVelocity(0, context)

        # set initial hinge angles of the cupboard.
        # setting hinge angle to exactly 0 or 90 degrees will result in intermittent contact
        # with small contact forces between the door and the cupboard body.
        left_hinge_joint = self.plant.GetJointByName("left_door_hinge")
        left_hinge_joint.set_angle(
            context=self.station.GetMutableSubsystemContext(self.plant, context), angle=-0.001)

        right_hinge_joint = self.plant.GetJointByName("right_door_hinge")
        right_hinge_joint.set_angle(
            context=self.station.GetMutableSubsystemContext(self.plant, context), angle=0.001)

        # set initial pose of the object
        self.plant.tree().SetFreeBodyPoseOrThrow(
           self.plant.GetBodyByName(self.object_base_link_name, self.object),
            self.X_WObject, self.station.GetMutableSubsystemContext(self.plant, context))

        simulator.set_publish_every_time_step(False)
        simulator.set_target_realtime_rate(real_time_rate)
        simulator.Initialize()
        sim_duration = 0.
        for plan in plans_list:
            sim_duration += plan.get_duration() * 1.1
        sim_duration += extra_time
        print "simulation duration", sim_duration
        simulator.StepTo(sim_duration)

        return iiwa_position_command_log, \
               iiwa_position_measured_log, \
               iiwa_external_torque_log, \
               wsg_state_log, \
               wsg_command_log


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
        camera_ids = ["805212060544"]
        station_hardware = ManipulationStationHardwareInterface(camera_ids)
        station_hardware.Connect(wait_for_cameras=False)
        builder.AddSystem(station_hardware)

        # Add plan runner
        # Add plan runner.
        plan_runner = ManipStationPlanRunner(
            station=self.station,
            kuka_plans=plans_list,
            gripper_setpoint_list=gripper_setpoint_list)

        builder.AddSystem(plan_runner)
        builder.Connect(plan_runner.hand_setpoint_output_port,
                        station_hardware.GetInputPort("wsg_position"))
        builder.Connect(plan_runner.gripper_force_limit_output_port,
                        station_hardware.GetInputPort("wsg_force_limit"))


        demux = builder.AddSystem(Demultiplexer(14, 7))
        builder.Connect(
            plan_runner.GetOutputPort("iiwa_position_and_torque_command"),
            demux.get_input_port(0))
        builder.Connect(demux.get_output_port(0),
                        station_hardware.GetInputPort("iiwa_position"))
        builder.Connect(demux.get_output_port(1),
                        station_hardware.GetInputPort("iiwa_feedforward_torque"))
        builder.Connect(station_hardware.GetOutputPort("iiwa_position_measured"),
                        plan_runner.iiwa_position_input_port)
        builder.Connect(station_hardware.GetOutputPort("iiwa_velocity_estimated"),
                        plan_runner.iiwa_velocity_input_port)


        # Add logger
        iiwa_position_measured_log = LogOutput(
            station_hardware.GetOutputPort("iiwa_position_measured"), builder)
        iiwa_position_measured_log._DeclarePeriodicPublish(0.005)

        iiwa_external_torque_log = LogOutput(
            station_hardware.GetOutputPort("iiwa_torque_external"), builder)
        iiwa_external_torque_log._DeclarePeriodicPublish(0.005)

        wsg_state_log = LogOutput(
            station_hardware.GetOutputPort("wsg_state_measured"), builder)
        wsg_state_log._DecalrePeriodicPublish(0.1)

        wsg_command_log = LogOutput(
            plan_runner.hand_setpoint_output_port, builder)
        wsg_command_log._DeclarePeriodicPublish(0.1)


        # build diagram
        diagram = builder.Build()
        RenderSystemWithGraphviz(diagram)

        # construct simulator
        simulator = Simulator(diagram)

        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)

        sim_duration = 0.
        for plan in plans_list:
            sim_duration += plan.get_duration() * 1.1

        print "sending trajectories in 2 seconds..."
        time.sleep(1.0)
        print "sending trajectories in 1 second..."
        time.sleep(1.0)
        print "sending trajectories now!"
        simulator.StepTo(sim_duration)

        return iiwa_position_measured_log, iiwa_external_torque_log, wsg_state_log, wsg_command_log


    def Replay(self):
        self.viz.replay()