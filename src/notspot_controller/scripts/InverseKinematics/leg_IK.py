#!/usr/bin/env python3

import numpy as np
from math import sqrt, atan2, sin, cos, pi

class robot_leg():

    def __init__(self, leg_dimension):
        # Leg dimensions
        self.l1 = leg_dimension[0]
        self.l2 = leg_dimension[1]
        self.l3 = leg_dimension[2]
        self.l4 = leg_dimension[3]

    def inverse_kinematics(self, pose, leg_number):

        x = pose[0]
        y = pose[1] + self.l2
        z = pose[2]
        i = leg_number

        F = sqrt(abs(x ** 2 + y** 2 - self.l2 ** 2))
        if np.sign(x) < 0:
            F = -F

        G = F - self.l1
        H = sqrt(G ** 2 + z ** 2)


        # print("F, G, H", F, G, H)
        # print("tan(y, x)", atan2(y, x))
        # print("tan2", atan2(F, self.l2))

        theta1 = atan2(y, x) + atan2(F, self.l2) - pi/2

        D = (H ** 2 - self.l3 ** 2 - self.l4 ** 2) / (2 * self.l3 * self.l4)

        theta3 = -atan2((sqrt(1 - D ** 2)), D)

        theta2 = atan2(z, G) - atan2(self.l4 * sin(theta3),
                                     self.l3 + self.l4 * cos(theta3)) - pi/2

        return [theta1, theta2, theta3]