#!/usr/bin/env python3

import numpy as np
from math import sqrt, atan2, sin, cos, pi
from RoboticsUtilities import Transformations

class robot_leg():

    def __init__(self, leg_dimension):
        # Leg dimensions
        self.l1 = leg_dimension[0]
        self.l2 = leg_dimension[1]
        self.l3 = leg_dimension[2]
        self.l4 = leg_dimension[3]

    def inverse_kinematics(self, pose, leg_number):

        i = leg_number
        x = pose[0]

        if i == 1 or i == 2:
            x = x
        else:
            x = x

        y = -pose[1] + self.l2
        z = pose[2]

        u = sqrt(z**2 + y**2)
        h = sqrt(u**2 - self.l2**2)
        theta1 = pi/2 - atan2(h, self.l2) - atan2(y, z)

        H = sqrt((x)**2 + (y - self.l2*cos(theta1))**2 + (z - self.l2*sin(theta1))**2)
        D = (H ** 2 - self.l3 ** 2 - self.l4 ** 2) / (2 * self.l3 * self.l4)
        theta3 = -atan2((sqrt(1 - D ** 2)), D)

        E = sqrt(H**2 - x**2)
        F = (self.l4**2 - H**2 - self.l3**2)/(2*self.l3*H)
        print(atan2((sqrt(1 - F ** 2)), F))
        print(atan2(x, E))
        theta2 = pi - atan2((sqrt(1 - F ** 2)), F) - atan2(x, E)


        # if i%2 == 1:
        #     y = -pose[1]
        #     self.l2 = -self.l2_perm
        # else:
        #     self.l2 = self.l2_perm
        # y = y + self.l2


        # print("xyz", x, y, z)
        # F = sqrt(abs(x ** 2 + y** 2 - self.l2 ** 2))
        #
        # if np.sign(x) < 0:
        #     F = F
        #
        # G = F - self.l1
        # H = sqrt(G ** 2 + z ** 2)
        #
        # print("F, G, H", F, G, H)
        # print("tan(y, x)", atan2(x, y))
        # print("tan2", atan2(F, self.l2))
        #
        # # theta1 = atan2(x, y) + atan2(F, self.l2)
        #
        # D = (H ** 2 - self.l3 ** 2 - self.l4 ** 2) / (2 * self.l3 * self.l4)
        #
        # theta3 = -atan2((sqrt(1 - D ** 2)), D)
        #
        # theta2 = atan2(z, G) - atan2(self.l4 * sin(theta3),
        #                              self.l3 + self.l4 * cos(theta3)) - pi/2

        # if i%2 == 1:
        #     theta1 = -theta1

        return [theta1, theta2, theta3]