import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import numpy as np
import scipy.linalg as operations
from matplotlib import pyplot as plt


# Class Inverse Kinematics
# Constructor: Defines values of Start (base_footprint) and End (April_tag base) pose
# Member Functions: i_kine

#           Return: 2-Vector with Left and Right Wheel velocities

class InverseKine:
    def __init__(self, start, end, start_time, deltaT):
        self.R = 0.033
        self.W = 0.16
        self.phi_r = None
        self.phi_l = None
        self.inv_Model = None
        self.start_pose = start  # np.array (3x3)
        self.end_pose = end  # np.array (3x3)
        self.start_time = start_time  # Make Robot start at T = 0 Seconds
        self.end_time = start_time + deltaT  # Make Robot stop at T = 5.0 Seconds

    def i_kine(self, calc_end_pose):
        self.end_pose = calc_end_pose
        # noinspection PyTypeChecker
        inv_Model = ((self.end_time - self.start_time) ** -1) * operations.logm(operations.inv(self.start_pose) @ self.end_pose)

        # omega = [0                  -(r/w)*(rVel-lVel) (r/2)*(rVel+lVel);
        #         (r/w)*(rVel-lVel)    0                  0;
        #         0                    0                  0
        #         ];
        #    Solve for phi_l, phi_r in the linear system:
        #                                                   omega = inv_Model

        # self.phi_l = (self.W * theta_dot + 2 * x_dot) / self.R
        # self.phi_r = (2 * x_dot - self.W * theta_dot) / self.R

        theta_dot = inv_Model[0, 1]
        x_dot = inv_Model[0, 2]
        return x_dot, theta_dot



# Class Forward Kinematics
# Constructor: Defines values of Start (base_footprint) and End (April_tag base) pose
# Member Functions: i_kine
#           Return: 2-Vector with Left and Right Wheel velocities

class ForwardKine:
    def __init__(self, mass, radius, WheelBaseWidth, start_time, end_time, start_pose, rVel, lVel):
        self.Mass = mass  # CHECK IF NEEDED
        self.R = radius  # CHECK IF NEEDED
        self.W = WheelBaseWidth  # CHECK IF NEEDED
        self.phi_r = rVel
        self.phi_l = lVel
        self.start_time = start_time  # Make Robot start at T = 0 Seconds
        self.end_time = end_time  # Make Robot stop at T = 1.0 Seconds
        self.start_pose = start_pose  # np.array (3x3)
        self.end_pose = None  # np.array (3x3)
        self.omega_dot = None
        self.refresh_R = rospy.Rate(10)  # CHECK IF NEEDED

    def f_kine(self):
        # noinspection PyTypeChecker
        # omega = [0                  -(r/w)*(rVel-lVel) (r/2)*(rVel+lVel);
        #         (r/w)*(rVel-lVel)    0                  0;
        #         0                    0                  0
        #         ];
        #    Solve for phi_l, phi_r in the linear system:
        #                                                   omega = inv_Model
        self.omega_dot = np.array([
            [0, (-self.R / self.W) * (self.phi_r - self.phi_l), (self.R / 2) * (self.phi_r + self.phi_l)],
            [(self.R / self.W) * (self.phi_r - self.phi_l), 0, 0],
            [0, 0, 0]
        ])
        self.end_pose = self.start_pose * operations.expm((self.end_time - self.start_time) * self.omega_dot)
        return self.end_pose


def main():
    start = np.identity(3)
    end = np.array([[1, 0, 5], [0, 1, 0], [0, 0, 1]])
    obj = InverseKine(start, end, radius=2, WheelBaseWidth=2)
    vel, twist = obj.i_kine()
    print("Velocity: {} \t Twist:{}".format(vel, twist))


if __name__ == '__main__':
    main()
