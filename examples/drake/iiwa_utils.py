import random

import numpy as np

from pydrake.multibody.multibody_tree import WeldJoint

from examples.drake.utils import create_transform, get_model_bodies, set_max_joint_positions, set_min_joint_positions


def weld_gripper(mbp, robot_index, gripper_index):
    X_EeGripper = create_transform([0, 0, 0.081], [np.pi / 2, 0, np.pi / 2])
    robot_body = get_model_bodies(mbp, robot_index)[-1]
    gripper_body = get_model_bodies(mbp, gripper_index)[0]
    mbp.AddJoint(WeldJoint(name="weld_gripper_to_robot_ee",
                           parent_frame_P=robot_body.body_frame(),
                           child_frame_C=gripper_body.body_frame(),
                           X_PC=X_EeGripper))

##################################################

WSG50_LEFT_FINGER = 'left_finger_sliding_joint'
WSG50_RIGHT_FINGER = 'right_finger_sliding_joint'

def get_close_wsg50_positions(mbp, model_index):
    left_joint = mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)
    right_joint = mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)
    return [left_joint.upper_limits()[0], right_joint.lower_limits()[0]]


def get_open_wsg50_positions(mbp, model_index):
    left_joint = mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)
    right_joint = mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)
    return [left_joint.lower_limits()[0], right_joint.upper_limits()[0]]


def close_wsg50_gripper(mbp, context, model_index): # 0.05
    set_max_joint_positions(context, [mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)])
    set_min_joint_positions(context, [mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)])


def open_wsg50_gripper(mbp, context, model_index):
    set_min_joint_positions(context, [mbp.GetJointByName(WSG50_LEFT_FINGER, model_index)])
    set_max_joint_positions(context, [mbp.GetJointByName(WSG50_RIGHT_FINGER, model_index)])

##################################################

def get_top_cylinder_grasps(aabb, max_width=np.inf, grasp_length=0): # y is out of gripper
    tool = create_transform(translation=[0, 0, -0.07])
    center, extent = aabb
    w, l, h = 2*extent
    reflect_z = create_transform(rotation=[np.pi / 2, 0, 0])
    translate_z = create_transform(translation=[0, 0, -h / 2 + grasp_length])
    aabb_from_body = create_transform(translation=center).inverse()
    diameter = (w + l) / 2 # TODO: check that these are close
    if max_width < diameter:
        return
    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = create_transform(rotation=[0, 0, theta])
        yield reflect_z.multiply(rotate_z).multiply(translate_z).multiply(tool).multiply(aabb_from_body)

