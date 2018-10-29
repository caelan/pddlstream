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