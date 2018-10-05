"""
Provides an example translation of `cart_pole_passive_simluation.cc`.
"""

import argparse
import os
import pydrake

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph, DispatchLoadMessage)
from pydrake.lcm import DrakeLcm, DrakeMockLcm # Required else "ConnectDrakeVisualizer(): incompatible function arguments."
from pydrake.multibody.multibody_tree import UniformGravityFieldElement
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

cart_pole_sdf_path = "drake/examples/multibody/cart_pole/cart_pole.sdf"

iiwa_sdf_path = os.path.join(pydrake.getDrakePath(),
    "manipulation", "models", "iiwa_description", "sdf",
    #"iiwa14_no_collision_floating.sdf")
    "iiwa14_no_collision.sdf")

print(iiwa_sdf_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Desired rate relative to real time.  See documentation for "
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "--simulation_time", type=float, default=10.0,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--time_step", type=float, default=0.,
        help="If greater than zero, the plant is modeled as a system with "
             "discrete updates and period equal to this time_step. "
             "If 0, the plant is modeled as a continuous system.")
    args = parser.parse_args()

    #lcm = None
    #lcm = DrakeMockLcm() # Doesn't work
    lcm = DrakeLcm()

    #file_name = FindResourceOrThrow(cart_pole_sdf_path)
    file_name = iiwa_sdf_path
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    cart_pole = builder.AddSystem(MultibodyPlant(time_step=args.time_step))
    AddModelFromSdfFile(
        file_name=file_name, plant=cart_pole, scene_graph=scene_graph)
    cart_pole.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
    cart_pole.Finalize(scene_graph)
    assert cart_pole.geometry_source_is_registered()

    builder.Connect(
        scene_graph.get_query_output_port(),
        cart_pole.get_geometry_query_input_port())
    builder.Connect(
        cart_pole.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(cart_pole.get_source_id()))

    ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph, lcm=lcm)
    diagram = builder.Build()

    diagram_context = diagram.CreateDefaultContext()
    cart_pole_context = diagram.GetMutableSubsystemContext(
        cart_pole, diagram_context)

    #cart_pole_context.FixInputPort(
    #    cart_pole.get_actuation_input_port().get_index(), [0])


    DispatchLoadMessage(scene_graph, lcm)
    diagram.Publish(diagram_context)
    print(dir(lcm))
    #lcm.Publish(diagram_context)

    """
    cart_slider = cart_pole.GetJointByName("CartSlider")
    pole_pin = cart_pole.GetJointByName("PolePin")
    cart_slider.set_translation(context=cart_pole_context, translation=0.)
    pole_pin.set_angle(context=cart_pole_context, angle=2.)

    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(args.target_realtime_rate)
    simulator.Initialize()
    simulator.StepTo(args.simulation_time)
    """

    raw_input("Finish?")


if __name__ == "__main__":
    main()