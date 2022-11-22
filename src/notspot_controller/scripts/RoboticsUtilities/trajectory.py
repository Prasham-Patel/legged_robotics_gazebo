import numpy as np
from math import atan, sin, cos, pi
from matplotlib import pyplot as plt
import copy

class Trajectory():

    def __init__(self, xi, xf, vi, vf, t0, tf):
        self.xi  = xi
        self.xf = xf
        self.Ti = t0
        self.Tf = tf
        self.vi = vi
        self.vf = vf

        self.x_matrix = np.matrix([[xi], [vi], [xf], [vf]])
        self.cubic_T_Matrix()
        self.cubic_trajectory()


    def cubic_T_Matrix(self):
        self.T_matrix = np.matrix([[1, self.Ti, self.Ti ** 2, self.Ti ** 3],
                                  [0, 1, 2 * self.Ti, 3 * self.Ti ** 2],
                                  [1, self.Tf, self.Tf ** 2, self.Tf ** 3],
                                  [0, 1, 2 * self.Tf, 3 * self.Tf ** 2]])
        self.T_inv = np.linalg.inv(self.T_matrix)

    def cubic_trajectory(self):
        self.cubic_T_Matrix()
        self.x_Co_ef = np.dot(self.T_inv, self.x_matrix)
        

    def at_time(self, T):
        T_d1 = np.matrix([1, T, T**2, T**3])
        T_d2 = np.matrix([0, 1, 2*T, 3*T**2])
        x = np.dot(T_d1, self.x_Co_ef)
        x_dot = np.dot(T_d2, self.x_Co_ef)
        return x, x_dot

class trajectory_controller():

    def __init__(self, kp):
        self.kp = kp # control constant
        self.current_pose = None
        self.previous_pose = None
        self.target_pose = None

    def update_state(self, new_pose):
        self.previous_pose = self.current_pose
        self.current_pose = new_pose

    def update_target(self, new_target):
        self.target_pose = new_target

    def get_velocity_command(self):
        return (self.target_pose - self.current_pose)*self.kp

    def current_error(self):
        return (self.target_pose - self.current_pose)
if __name__ == '__main__':

    primitive = Trajectory(0, 1, 0, 0, 0, 1)
    pos_x = []
    pos_y = []
    time = []
    for i in range(100):
        x, y = primitive.at_time(i/100)
        pos_x.append(x)
        pos_y.append(y)
        time.append(i)
    plt.scatter(time, pos_x)
    plt.show()
