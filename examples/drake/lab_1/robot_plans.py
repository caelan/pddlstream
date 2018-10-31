import numpy as np

PlanTypes = [
    "JointSpacePlan",
    "TaskSpacePlan",
    "PlanarTaskSpacePlan",
    "PlanarHybridPositionForcePlan",
]


class PlanBase:
    def __init__(self,
                 type = None,
                 trajectory = None,):
        self.type = type
        self.traj = trajectory
        self.traj_d = None
        self.duration = None
        if trajectory is not None:
            self.traj_d = trajectory.derivative(1)
            self.duration = trajectory.end_time()

        self.start_time = None

    def get_duration(self):
        return self.duration

    def set_start_time(self, time):
        self.start_time = time


class JointSpacePlan(PlanBase):
    def __init__(self,
                 trajectory=None):
        PlanBase.__init__(self,
                          type=PlanTypes[0],
                          trajectory=trajectory)


'''
trajectory is a 3-dimensional PiecewisePolynomial. It describes the trajectory
    of the origin of the ee frame in world frame.
R_WE_ref is the fixed pose of the end effector while it its origin moves along
    the given trajectory. 
'''
class TaskSpacePlan(PlanBase):
    def __init__(self,
                 trajectory=None,
                 R_WE_ref=None):
        PlanBase.__init__(self,
                          type=PlanTypes[1],
                          trajectory=trajectory)
        self.R_WE_ref = R_WE_ref


'''
trajectory is a 3-dimensional PiecewisePolynomial.
trajectory[0] is the x-position of the gripper in world frame.
trajectory[1] is the y-position of the gripper in world frame.
trajectory[2] is the angle between the ee frame and the world frame. 
'''
class PlanarTaskSpacePlan(PlanBase):
    def __init__(self,
                 trajectory=None):
        PlanBase.__init__(self,
                          type=PlanTypes[2],
                          trajectory=trajectory)


'''
The end effector of a planar robot has three degrees of freedom:
    translation along y, z and rotation about x. 
x_ee_traj is a k-dimensional (k<=3) PiecewisePolynomial that describes the desired trajectory
    of the position-controlled DOFs.
f_ee_traj is a (3-k)-dimensional PiecewisePolynomial that describes the desired trajecotry
    of the force-controlled DOFs (applied by the robot on the world).
The "Task frame" or "constrained frame" (as in Raibert and Craig, 1981) is aligned with the world frame,
    the origin of the task frame is coincident with the body frame of the end effector.
selector is a (3,) numpy array that whether the i-th DOF is force-controlled (selector[i] == 1)
    or position-controlled(selector[i] == 0).
'''
class PlanarHybridPositionForcePlan(PlanBase):
    def __init__(self,
                 x_ee_traj=None,
                 f_ee_traj=None,
                 selector=None):
        PlanBase.__init__(self,
                          type=PlanTypes[3],
                          trajectory=x_ee_traj)
        assert np.isclose(x_ee_traj.end_time(), f_ee_traj.end_time())
        assert x_ee_traj.rows() + f_ee_traj.rows() == 3
        assert selector.shape == (3,)
        self.f_ee_traj = f_ee_traj
        self.selector = selector

        # using notations from De Luca's notes on hybrid force/motion control
        k = x_ee_traj.rows()
        self.T = np.zeros((3, k))
        j = 0
        for i in range(3):
            if selector[i] == 0:
                self.T[i,j] = 1
                j += 1

        j = 0
        self.Y = np.zeros((3, 3-k))
        for i in range(3):
            if selector[i] == 1:
                self.Y[i,j] = 1
                j+= 1



