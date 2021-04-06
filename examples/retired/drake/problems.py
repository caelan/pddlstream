import os

import numpy as np
import pydrake
from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import ManipulationStation, IiwaCollisionModel
from pydrake.geometry import (SceneGraph)
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile

from examples.drake.systems import build_manipulation_station, build_diagram
from examples.drake.iiwa_utils import weld_gripper, DOOR_CLOSED, DOOR_OPEN, open_wsg50_gripper
from examples.drake.utils import get_model_name, weld_to_world, create_transform, get_movable_joints, \
    get_model_bodies, get_bodies, set_joint_position, set_world_pose

IIWA14_SDF_PATH = os.path.join(pydrake.getDrakePath(),
                               "manipulation", "models", "iiwa_description", "sdf",
                               # "iiwa14_no_collision_floating.sdf")
                               # "iiwa14_polytope_collision.sdf")
                               "iiwa14_no_collision.sdf")
# TODO: meshcat fails when using relative import

#IIWA_SDF_PATH = os.path.join(MODELS_DIR, "iiwa_description", "sdf",
#    "iiwa14_no_collision.sdf")

WSG50_SDF_PATH = os.path.join(pydrake.getDrakePath(),
                              "manipulation", "models", "wsg_50_description", "sdf",
                              "schunk_wsg_50.sdf")

TABLE_SDF_PATH = os.path.join(pydrake.getDrakePath(),
                              "examples", "kuka_iiwa_arm", "models", "table",
                              "extra_heavy_duty_table_surface_only_collision.sdf")

MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")

SINK_PATH = os.path.join(MODELS_DIR, "sink.sdf")
STOVE_PATH = os.path.join(MODELS_DIR, "stove.sdf")
BROCCOLI_PATH = os.path.join(MODELS_DIR, "broccoli.sdf")
WALL_PATH = os.path.join(MODELS_DIR, "wall.sdf")

##################################################

class Surface(object):
    def __init__(self, plant, model_index, body_name, visual_index):
        self.plant = plant
        self.model_index = model_index
        self.body_name = body_name
        self.visual_index = visual_index
    def __repr__(self):
        return '{}({},{},{})'.format(self.__class__.__name__,
                                     get_model_name(self.plant, self.model_index),
                                     self.body_name, self.visual_index)

class Task(object):
    def __init__(self, diagram, mbp, scene_graph, robot, gripper,
                 movable=[], surfaces=[], doors=[],
                 initial_positions={}, initial_poses={},
                 goal_poses={}, goal_holding=[], goal_on=[], goal_cooked=[],
                 reset_robot=True, reset_doors=True):
        self.diagram = diagram
        self.mbp = mbp
        self.scene_graph = scene_graph
        self.diagram_context = diagram.CreateDefaultContext()
        self.plant_context = diagram.GetMutableSubsystemContext(mbp, self.diagram_context)
        # context = plant.CreateDefaultContext()

        self.robot = robot
        self.gripper = gripper
        self.movable = movable
        self.surfaces = surfaces
        self.doors = doors

        self.initial_positions = initial_positions
        self.initial_poses = initial_poses
        self.goal_poses = goal_poses
        self.goal_holding = goal_holding
        self.goal_on = goal_on
        self.goal_cooked = goal_cooked
        self.reset_robot = reset_robot
        self.reset_doors = reset_doors
    @property
    def plant(self):
        return self.mbp
    def movable_bodies(self):
        movable = {self.mbp.tree().get_body(index) for index in self.doors}
        for model in [self.robot, self.gripper] + list(self.movable):
            movable.update(get_model_bodies(self.mbp, model))
            #for body in get_model_bodies(self.mbp, model):
            #    #print(self.mbp.tree().get_body(body.index()) in {body}) # True
            #    #print(body.index() in {body.index()}) # False
            #    movable.add(body.index())
        return movable
    def fixed_bodies(self):
        return set(get_bodies(self.mbp)) - self.movable_bodies()
    def set_initial(self):
        for joint, position in self.initial_positions.items():
            set_joint_position(joint, self.plant_context, position)
        for model, pose in self.initial_poses.items():
            set_world_pose(self.plant, self.plant_context, model, pose)
        open_wsg50_gripper(self.plant, self.plant_context, self.gripper)
    def publish(self):
        self.diagram.Publish(self.diagram_context)
    def __repr__(self):
        return '{}(robot={}, gripper={}, movable={}, surfaces={})'.format(
            self.__class__.__name__,
            get_model_name(self.mbp, self.robot),
            get_model_name(self.mbp, self.gripper),
            [get_model_name(self.mbp, model) for model in self.movable],
            self.surfaces)

##################################################

def load_station(time_step=0.0, **kwargs):
    # https://github.com/RobotLocomotion/drake/blob/master/bindings/pydrake/examples/manipulation_station_py.cc
    #object_file_path = FindResourceOrThrow(
    #    "drake/external/models_robotlocomotion/ycb_objects/061_foam_brick.sdf")
    object_file_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf")

    # RuntimeError: Error control wants to select step smaller than minimum allowed (1e-14)
    station = ManipulationStation(time_step, IiwaCollisionModel.kBoxCollision)
    plant = station.get_mutable_multibody_plant()
    scene_graph = station.get_mutable_scene_graph()

    robot = plant.GetModelInstanceByName('iiwa')
    gripper = plant.GetModelInstanceByName('gripper')
    station.AddCupboard()
    brick = AddModelFromSdfFile(file_name=object_file_path, model_name="brick",
                                plant=plant, scene_graph=scene_graph)
    station.Finalize()

    door_names = ['left_door', 'right_door']
    doors = [plant.GetBodyByName(name).index() for name in door_names]

    initial_conf = [0, 0.6 - np.pi / 6, 0, -1.75, 0, 1.0, 0]
    #initial_conf[1] += np.pi / 6
    initial_positions = {
        plant.GetJointByName('left_door_hinge'): -DOOR_CLOSED,
        #plant.GetJointByName('left_door_hinge'): -np.pi / 2,
        plant.GetJointByName('right_door_hinge'): np.pi/2,
    }
    initial_positions.update(zip(get_movable_joints(plant, robot), initial_conf))

    initial_poses = {
        brick: create_transform(translation=[0.3, 0, 0], rotation=[0, 0, np.pi/2]),
    }
    goal_poses = {
        brick: create_transform(translation=[0.8, 0.2, 0.2927], rotation=[0, 0, 5*np.pi/4]),
    }

    diagram, state_machine = build_manipulation_station(station)

    task = Task(diagram, plant, scene_graph, robot, gripper, movable=[brick], doors=doors,
                initial_positions=initial_positions, initial_poses=initial_poses,
                goal_poses=goal_poses, reset_robot=True, reset_doors=False)
    task.station = station

    return task, diagram, state_machine

##################################################

def load_manipulation(time_step=0.0, use_meshcat=True, use_controllers=True, use_external=False):
    if not use_external:
        AMAZON_TABLE_PATH = FindResourceOrThrow(
           "drake/examples/manipulation_station/models/amazon_table_simplified.sdf")
        CUPBOARD_PATH = FindResourceOrThrow(
           "drake/examples/manipulation_station/models/cupboard.sdf")
        IIWA7_PATH = FindResourceOrThrow(
           "drake/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf")
        FOAM_BRICK_PATH = FindResourceOrThrow(
           "drake/examples/manipulation_station/models/061_foam_brick.sdf")
        goal_shelf = 'shelf_lower'
        #goal_shelf = 'shelf_upper'
    else:
        AMAZON_TABLE_PATH = FindResourceOrThrow(
            "drake/external/models_robotlocomotion/manipulation_station/amazon_table_simplified.sdf")
        CUPBOARD_PATH = FindResourceOrThrow(
            "drake/external/models_robotlocomotion/manipulation_station/cupboard.sdf")
        IIWA7_PATH = FindResourceOrThrow(
            "drake/external/models_robotlocomotion/iiwa7/iiwa7_no_collision.sdf")
        FOAM_BRICK_PATH = FindResourceOrThrow(
            "drake/external/models_robotlocomotion/ycb_objects/061_foam_brick.sdf")
        goal_shelf = 'bottom'
    #IIWA7_PATH = os.path.join(MODELS_DIR, "iiwa_description/iiwa7/iiwa7_with_box_collision.sdf")

    plant = MultibodyPlant(time_step=time_step)
    scene_graph = SceneGraph()

    dx_table_center_to_robot_base = 0.3257
    dz_table_top_robot_base = 0.0127
    dx_cupboard_to_table_center = 0.43 + 0.15
    dz_cupboard_to_table_center = 0.02
    cupboard_height = 0.815
    cupboard_x = dx_table_center_to_robot_base + dx_cupboard_to_table_center
    cupboard_z = dz_cupboard_to_table_center + cupboard_height / 2.0 - dz_table_top_robot_base

    robot = AddModelFromSdfFile(file_name=IIWA7_PATH, model_name='iiwa',
                                scene_graph=scene_graph, plant=plant)
    gripper = AddModelFromSdfFile(file_name=WSG50_SDF_PATH, model_name='gripper',
                                  scene_graph=scene_graph, plant=plant)  # TODO: sdf frame/link error
    table = AddModelFromSdfFile(file_name=AMAZON_TABLE_PATH, model_name='table',
                                scene_graph=scene_graph, plant=plant)
    cupboard = AddModelFromSdfFile(file_name=CUPBOARD_PATH, model_name='cupboard',
                                 scene_graph=scene_graph, plant=plant)
    brick = AddModelFromSdfFile(file_name=FOAM_BRICK_PATH, model_name='brick',
                                 scene_graph=scene_graph, plant=plant)

    weld_gripper(plant, robot, gripper)
    weld_to_world(plant, robot, create_transform())
    weld_to_world(plant, table, create_transform(
        translation=[dx_table_center_to_robot_base, 0, -dz_table_top_robot_base]))
    weld_to_world(plant, cupboard, create_transform(
        translation=[cupboard_x, 0, cupboard_z], rotation=[0, 0, np.pi]))
    plant.Finalize(scene_graph)

    shelves = [
        'bottom',
        'shelf_lower',
        'shelf_upper',
        'top',
    ]
    goal_surface = Surface(plant, cupboard, 'top_and_bottom', shelves.index(goal_shelf))
    surfaces = [
        #Surface(plant, table, 'amazon_table', 0),
        #goal_surface,
    ]

    door_names = ['left_door', 'right_door']
    #door_names = []
    doors = [plant.GetBodyByName(name).index() for name in door_names]

    door_position = DOOR_CLOSED  # ~np.pi/2
    #door_position = DOOR_OPEN
    #door_position = np.pi
    #door_position = np.pi/2
    #door_position = np.pi/8
    initial_positions = {
        plant.GetJointByName('left_door_hinge'): -door_position,
        #plant.GetJointByName('right_door_hinge'): door_position,
        plant.GetJointByName('right_door_hinge'): np.pi/2,
    }
    initial_conf = [0, 0.6 - np.pi / 6, 0, -1.75, 0, 1.0, 0]
    #initial_conf[1] += np.pi / 6
    initial_positions.update(zip(get_movable_joints(plant, robot), initial_conf))

    initial_poses = {
        #brick: create_transform(translation=[0.6, 0, 0]),
        brick: create_transform(translation=[0.3, 0, 0], rotation=[0, 0, np.pi/2]),
        #brick: create_transform(translation=[0.2, -0.3, 0], rotation=[0, 0, np.pi/8]),
    }

    goal_poses = {
        brick: create_transform(translation=[0.8, 0.2, 0.2927], rotation=[0, 0, np.pi/8]),
    }
    goal_holding = [
        #brick,
    ]
    goal_on = [
        #(brick, goal_surface),
    ]

    diagram, state_machine = build_diagram(plant, scene_graph, robot, gripper,
                                           meshcat=use_meshcat, controllers=use_controllers)

    task = Task(diagram, plant, scene_graph, robot, gripper, movable=[brick], surfaces=surfaces, doors=doors,
                initial_positions=initial_positions, initial_poses=initial_poses,
                goal_poses=goal_poses, goal_holding=goal_holding, goal_on=goal_on,
                reset_robot=True, reset_doors=False)

    return task, diagram, state_machine

##################################################

def load_tables(time_step=0.0, use_meshcat=True, use_controllers=True):
    plant = MultibodyPlant(time_step=time_step)
    scene_graph = SceneGraph()

    # TODO: meshes aren't supported during collision checking
    robot = AddModelFromSdfFile(file_name=IIWA14_SDF_PATH, model_name='iiwa',
                                scene_graph=scene_graph, plant=plant)
    gripper = AddModelFromSdfFile(file_name=WSG50_SDF_PATH, model_name='gripper',
                                  scene_graph=scene_graph, plant=plant)  # TODO: sdf frame/link error
    table = AddModelFromSdfFile(file_name=TABLE_SDF_PATH, model_name='table',
                                scene_graph=scene_graph, plant=plant)
    table2 = AddModelFromSdfFile(file_name=TABLE_SDF_PATH, model_name='table2',
                                 scene_graph=scene_graph, plant=plant)
    sink = AddModelFromSdfFile(file_name=SINK_PATH, model_name='sink',
                               scene_graph=scene_graph, plant=plant)
    stove = AddModelFromSdfFile(file_name=STOVE_PATH, model_name='stove',
                                scene_graph=scene_graph, plant=plant)
    broccoli = AddModelFromSdfFile(file_name=BROCCOLI_PATH, model_name='broccoli',
                                   scene_graph=scene_graph, plant=plant)
    #wall = AddModelFromSdfFile(file_name=WALL_PATH, model_name='wall',
    #                           scene_graph=scene_graph, plant=mbp)
    wall = None

    table2_x = 0.75
    table_top_z = 0.7655  # TODO: use geometry
    weld_gripper(plant, robot, gripper)
    weld_to_world(plant, robot, create_transform(translation=[0, 0, table_top_z]))
    weld_to_world(plant, table, create_transform())
    weld_to_world(plant, table2, create_transform(translation=[table2_x, 0, 0]))
    weld_to_world(plant, sink, create_transform(translation=[table2_x, 0.25, table_top_z]))
    weld_to_world(plant, stove, create_transform(translation=[table2_x, -0.25, table_top_z]))
    if wall is not None:
        weld_to_world(plant, wall, create_transform(translation=[table2_x / 2, 0, table_top_z]))
    plant.Finalize(scene_graph)

    movable = [broccoli]
    surfaces = [
        Surface(plant, sink, 'base_link', 0), # Could also just pass the link index
        Surface(plant, stove, 'base_link', 0),
    ]

    initial_poses = {
        broccoli: create_transform(translation=[table2_x, 0, table_top_z]),
    }

    diagram, state_machine = build_diagram(plant, scene_graph, robot, gripper,
                                           meshcat=use_meshcat, controllers=use_controllers)

    task = Task(diagram, plant, scene_graph, robot, gripper,
                movable=movable, surfaces=surfaces,
                initial_poses=initial_poses,
                goal_cooked=[broccoli])

    return task, diagram, state_machine

##################################################

PROBLEMS = [
    load_tables,
    load_manipulation,
    load_station,
]