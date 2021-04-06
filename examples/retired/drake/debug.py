from examples.drake.generators import get_grasp_gen_fn, get_pose_gen
from examples.drake.utils import get_base_body, get_body_pose, user_input, get_model_name


def test_generators(task, diagram, diagram_context):
    mbp = task.mbp
    context = diagram.GetMutableSubsystemContext(mbp, diagram_context)

    # Test grasps
    print(get_base_body(mbp, task.gripper).name())
    print(get_body_pose(context, mbp.GetBodyByName('left_finger', task.gripper)).matrix() -
          get_body_pose(context, get_base_body(mbp, task.gripper)).matrix())
    user_input('Start')
    model = task.movable[0]
    grasp_gen_fn = get_grasp_gen_fn(task)
    for grasp, in grasp_gen_fn(get_model_name(mbp, model)):
        grasp.assign(context)
        diagram.Publish(diagram_context)
        user_input('Continue')

    # Test placements
    user_input('Start')
    pose_gen_fn = get_pose_gen(task, context)
    model = task.movable[0]
    for pose, in pose_gen_fn(get_model_name(mbp, model), task.surfaces[0]):
       pose.assign(context)
       diagram.Publish(diagram_context)
       user_input('Continue')

