from __future__ import print_function

from examples.discrete_belief.dist import UniformDist
from examples.pybullet.utils.pr2_primitives import State, Pose, Conf
from examples.pybullet.utils.pr2_problems import create_pr2, create_kitchen
from examples.pybullet.utils.pr2_utils import set_arm_conf, get_carry_conf, open_arm, get_other_arm, arm_conf, \
    REST_LEFT_ARM, close_arm
from examples.pybullet.utils.utils import get_name, HideOutput, get_pose, get_bodies


class BeliefTask(object):
    def __init__(self, robot, arms=tuple(), grasp_types=tuple(),
                 class_from_body={}, movable=tuple(), surfaces=tuple(),
                 goal_localized=tuple(), goal_registered=tuple(),
                 goal_holding=tuple(), goal_on=tuple()):
        self.robot = robot
        self.arms = arms
        self.grasp_types = grasp_types
        self.class_from_body = class_from_body
        self.movable = movable
        self.surfaces = surfaces
        self.goal_holding = goal_holding
        self.goal_on = goal_on
        self.goal_localized = goal_localized
        self.goal_registered = goal_registered
    def get_bodies(self):
        return self.movable + self.surfaces


# TODO: operate on histories to do open-world
class BeliefState(State):
    def __init__(self, task, b_on={}, localized=tuple(), registered=tuple(), **kwargs):
        super(BeliefState, self).__init__(**kwargs)
        self.task = task
        self.b_on = b_on
        self.localized = set(localized) # TODO: infer localized from this?
        self.registered = set(registered)
        # TODO: store configurations
        #for body in task.get_bodies():
        #    if body not in localized:
        #        del self.poses[body]
        #    elif body not in registered:
        #        point, quat = self.poses[body].value
        #        self.poses[body] = Pose(body, (point, None))
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__,
                                  list(map(get_name, self.localized)),
                                  list(map(get_name, self.registered)))

#######################################################

def get_localized_rooms(task):
    raise NotImplementedError()


def get_localized_surfaces(task):
    b_on = {}
    for movable in task.movable:
        b_on[movable] =  UniformDist(task.surfaces)
    return BeliefState(task, b_on=b_on, localized=task.surfaces)


def get_localized_movable(task):
    b_on = {} # TODO: delta
    #for movable in problem.movable:
    #    b_on[movable] =  UniformDist(problem.surfaces)
    return BeliefState(task, b_on=b_on, localized=task.surfaces + task.movable)

#######################################################

def get_problem1(arm='left', grasp_type='top'):
    with HideOutput():
        pr2 = create_pr2()
    set_arm_conf(pr2, arm, get_carry_conf(arm, grasp_type))
    open_arm(pr2, arm)
    other_arm = get_other_arm(arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()
    class_from_body = {
        table: 'table',
        cabbage: 'cabbage',
        sink: 'sink',
        stove: 'stove',
    } # TODO: use for debug

    task = BeliefTask(robot=pr2, arms=[arm], grasp_types=[grasp_type],
                      class_from_body=class_from_body, movable=[cabbage], surfaces=[table, sink, stove],
                      #goal_localized=[cabbage],
                      #goal_registered=[cabbage],
                      #goal_holding=[(arm, cabbage)],
                      #goal_on=[(cabbage, table)],
                      goal_on=[(cabbage, sink)],
                      )

    # initial = get_localized_rooms(task)
    initial = get_localized_surfaces(task)
    #initial = get_localized_movable(task)
    # TODO: construct supporting here

    return task, initial