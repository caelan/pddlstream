from pydrake.geometry import (ConnectDrakeVisualizer, DispatchLoadMessage)
from pydrake.lcm import DrakeLcm  # Required else "ConnectDrakeVisualizer(): incompatible function arguments."
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import SignalLogger, Demultiplexer, LogOutput


def build_manipulation_station(station):
    from underactuated.meshcat_visualizer import MeshcatVisualizer
    from .manipulation_station.manipulation_station_plan_runner import ManipStationPlanRunner

    builder = DiagramBuilder()
    builder.AddSystem(station)

    # Add plan runner.
    plan_runner = ManipStationPlanRunner(station=station)

    builder.AddSystem(plan_runner)
    builder.Connect(plan_runner.hand_setpoint_output_port,
                    station.GetInputPort("wsg_position"))
    builder.Connect(plan_runner.gripper_force_limit_output_port,
                    station.GetInputPort("wsg_force_limit"))


    demux = builder.AddSystem(Demultiplexer(14, 7))
    builder.Connect(
        plan_runner.GetOutputPort("iiwa_position_and_torque_command"),
        demux.get_input_port(0))
    builder.Connect(demux.get_output_port(0),
                    station.GetInputPort("iiwa_position"))
    builder.Connect(demux.get_output_port(1),
                    station.GetInputPort("iiwa_feedforward_torque"))
    builder.Connect(station.GetOutputPort("iiwa_position_measured"),
                    plan_runner.iiwa_position_input_port)
    builder.Connect(station.GetOutputPort("iiwa_velocity_estimated"),
                    plan_runner.iiwa_velocity_input_port)

    # Add meshcat visualizer
    plant = station.get_mutable_multibody_plant()
    scene_graph = station.get_mutable_scene_graph()
    viz = MeshcatVisualizer(scene_graph,
                            is_drawing_contact_force = True,
                            plant = plant)
    builder.AddSystem(viz)
    builder.Connect(station.GetOutputPort("pose_bundle"),
                    viz.GetInputPort("lcm_visualization"))
    builder.Connect(station.GetOutputPort("contact_results"),
                    viz.GetInputPort("contact_results"))

    # Add logger
    iiwa_position_command_log = LogOutput(demux.get_output_port(0), builder)
    iiwa_position_command_log._DeclarePeriodicPublish(0.005)

    iiwa_external_torque_log = LogOutput(
        station.GetOutputPort("iiwa_torque_external"), builder)
    iiwa_external_torque_log._DeclarePeriodicPublish(0.005)

    iiwa_position_measured_log = LogOutput(
        station.GetOutputPort("iiwa_position_measured"), builder)
    iiwa_position_measured_log._DeclarePeriodicPublish(0.005)

    wsg_state_log = LogOutput(
        station.GetOutputPort("wsg_state_measured"), builder)
    wsg_state_log._DeclarePeriodicPublish(0.1)

    wsg_command_log = LogOutput(
        plan_runner.hand_setpoint_output_port, builder)
    wsg_command_log._DeclarePeriodicPublish(0.1)


    # build diagram
    diagram = builder.Build()
    viz.load()
    #time.sleep(2.0)
    #RenderSystemWithGraphviz(diagram)
    return diagram, plan_runner

##################################################

def add_meshcat_visualizer(scene_graph, builder):
    # https://github.com/rdeits/meshcat-python
    # https://github.com/RussTedrake/underactuated/blob/master/src/underactuated/meshcat_visualizer.py
    from underactuated.meshcat_visualizer import MeshcatVisualizer
    viz = MeshcatVisualizer(scene_graph, draw_timestep=0.033333)
    builder.AddSystem(viz)
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    viz.get_input_port(0))
    viz.load()
    return viz


def add_drake_visualizer(scene_graph, builder):
    lcm = DrakeLcm()
    ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph, lcm=lcm)
    DispatchLoadMessage(scene_graph, lcm) # TODO: only update viewer after a plan is found
    return lcm # Important that variable is saved


def connect_controllers(builder, mbp, robot, gripper, print_period=1.0):
    from examples.drake.manipulation_station.kuka_multibody_controllers import \
        (KukaMultibodyController, HandController, ManipStateMachine)

    iiwa_controller = KukaMultibodyController(plant=mbp,
                                              kuka_model_instance=robot,
                                              print_period=print_period)
    builder.AddSystem(iiwa_controller)

    builder.Connect(iiwa_controller.get_output_port(0),
                    mbp.GetInputPort('iiwa_actuation'))
    builder.Connect(mbp.get_continuous_state_output_port(),
                    iiwa_controller.robot_state_input_port)

    hand_controller = HandController(plant=mbp,
                                     model_instance=gripper)
    builder.AddSystem(hand_controller)
    builder.Connect(hand_controller.get_output_port(0),
                    mbp.GetInputPort('gripper_actuation'))
    builder.Connect(mbp.get_continuous_state_output_port(),
                    hand_controller.robot_state_input_port)

    state_machine = ManipStateMachine(mbp)
    builder.AddSystem(state_machine)
    builder.Connect(mbp.get_continuous_state_output_port(),
                    state_machine.robot_state_input_port)
    builder.Connect(state_machine.kuka_plan_output_port,
                    iiwa_controller.plan_input_port)
    builder.Connect(state_machine.hand_setpoint_output_port,
                    hand_controller.setpoint_input_port)
    return state_machine


def build_diagram(mbp, scene_graph, robot, gripper, meshcat=False, controllers=False):
    builder = DiagramBuilder()
    builder.AddSystem(scene_graph)
    builder.AddSystem(mbp)

    # Connect scene_graph to MBP for collision detection.
    builder.Connect(
        mbp.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(mbp.get_source_id()))
    builder.Connect(
        scene_graph.get_query_output_port(),
        mbp.get_geometry_query_input_port())

    if meshcat:
        vis = add_meshcat_visualizer(scene_graph, builder)
    else:
        vis = add_drake_visualizer(scene_graph, builder)

    state_log = builder.AddSystem(SignalLogger(mbp.get_continuous_state_output_port().size()))
    state_log._DeclarePeriodicPublish(0.02)
    builder.Connect(mbp.get_continuous_state_output_port(), state_log.get_input_port(0))

    state_machine = None
    if controllers:
        state_machine = connect_controllers(builder, mbp, robot, gripper)

    return builder.Build(), state_machine


def RenderSystemWithGraphviz(system, output_file="system_view.gz"):
    ''' Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. '''
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)