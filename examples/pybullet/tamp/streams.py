from examples.pybullet.utils.pybullet_tools.pr2_primitives import iterate_approach_path
from examples.pybullet.utils.pybullet_tools.utils import pairwise_collision, get_distance


#######################################################

def get_cfree_approach_pose_test(problem, collisions=True):
    # TODO: apply this before inverse kinematics as well
    gripper = problem.get_gripper()
    def test(b1, p1, g1, b2, p2):
        if not collisions or (b1 == b2):
            return True
        p2.assign()
        for _ in iterate_approach_path(problem.robot, 'left', gripper, p1, g1, body=b1):
            if pairwise_collision(b1, b2) or pairwise_collision(gripper, b2):
                return False
        return True
    return test


def get_cfree_pose_pose_test(collisions=True):
    def test(b1, p1, b2, p2):
        if not collisions or (b1 == b2):
            return True
        p1.assign()
        p2.assign()
        return not pairwise_collision(b1, b2) #, max_distance=0.001)
    return test


def get_cfree_traj_pose_test(problem, collisions=True):
    def test(c, b2, p2):
        if not collisions:
            return True
        state = c.assign()
        if b2 in state.attachments:
            return True
        p2.assign()
        for _ in c.apply(state):
            state.assign()
            for b1 in state.attachments:
                if pairwise_collision(b1, b2):
                    #wait_for_user()
                    return False
            if pairwise_collision(problem.robot, b2):
                return False
        # TODO: just check collisions with moving links
        return True
    return test


def get_cfree_traj_grasp_pose_test(problem, collisions=True):
    def test(c, a, b1, g, b2, p2):
        return True

        if not collisions or (b1 == b2):
            return True
        state = c.assign()
        if (b1 in state.attachments) or (b2 in state.attachments):
            return True
        p2.assign()
        grasp_attachment = g.get_attachment(problem.robot, a)
        for _ in c.apply(state):
            state.assign()
            grasp_attachment.assign()
            if pairwise_collision(b1, b2):
                return False
            if pairwise_collision(problem.robot, b2):
                return False
        return True
    return test

#######################################################

BASE_CONSTANT = 1
BASE_VELOCITY = 0.25


def distance_fn(q1, q2):
    distance = get_distance(q1.values[:2], q2.values[:2])
    return BASE_CONSTANT + distance / BASE_VELOCITY


def move_cost_fn(t):
    distance = t.distance(distance_fn=lambda q1, q2: get_distance(q1[:2], q2[:2]))
    return BASE_CONSTANT + distance / BASE_VELOCITY