#!/usr/bin/env python3

from RoboticsUtilities import trajectory
from RoboticsUtilities import Transformations
from math import floor
import numpy as np

class gait_generator():

    def __init__(self, beta, t_cycle, robot_height, robot_leg):
        # constant
        self.beta = beta
        self.t_cycle = t_cycle
        self.leg_raise_time = 0.05
        self.robot_leg = robot_leg
        self.robot_height = robot_height

        # updates with velocity command
        self.v_body_g = None
        self.yaw_rate = None
        self.l_stride = None



        self.leg_pos = None
        self.robot_body_pos = None
        self.body_pos_init = False
        self.previous_timestamp = 0

    def run(self, sim_time, velocity_command, yaw_rate_command):
        # new v_body_g and l_stride from velocity command
        self.update_gait(velocity_command, yaw_rate_command)

        # get kinematic phase array to time leg movement
        self.get_kinematic_phase()
        # self.kinematic_phase[3] = self.kinematic_phase[0]
        # self.kinematic_phase[2] = self.kinematic_phase[1]

        # settling time stance
        if sim_time < 0:
            stance = [0.0, 0.0, self.robot_height]
            angles = []
            for i in range(1, 5):
                leg_angles = self.robot_leg.inverse_kinematics(stance, i)
                angles.append(leg_angles[0])
                angles.append(leg_angles[1])
                angles.append(leg_angles[2])

            return angles

        else:
            leg_pos = self.get_leg_pos(sim_time, yaw_rate_command)
            angles = []
            for i in range(0, 4):
                # if i == 1 or i == 3:
                #     leg_pos[i][0] = -leg_pos[i][0]
                # leg_pos[i] = np.matmul(Transformations.rotz(yaw_rate_command), leg_pos[i])
                # print("after rotation", leg_pos[i])
                leg_angles = self.robot_leg.inverse_kinematics(leg_pos[i], i+1)
                angles.append(leg_angles[0])
                angles.append(leg_angles[1])
                angles.append(leg_angles[2])

            return angles

    def update_gait(self, velocity_command, yaw_rate):
        self.v_body_g = velocity_command.transpose()[0]
        self.l_stride = self.v_body_g*self.t_cycle
        self.yaw_rate = yaw_rate
        print("v_body", self.v_body_g)
        print("t_cycle", self.t_cycle)
        print("l_stride", self.l_stride)

    def get_kinematic_phase(self):
        "creates kinematic phase array for leg 1 to leg 4"
        leg1_phase = 0
        leg2_phase = 0.5
        leg3_phase = leg1_phase + self.beta
        leg4_phase = leg2_phase + self.beta

        if leg3_phase >= 1:
            leg3_phase = leg3_phase - 1
        if leg4_phase >= 1:
            leg4_phase = leg4_phase - 1

        self.kinematic_phase = [leg1_phase, leg2_phase, leg3_phase, leg4_phase]

    def get_leg_pos(self, sim_time, yaw_rate):
        "gives local position which is added in to default stance"
        leg_raise_height = 0.08
        leg_vz = 0.0

        cycle_number = floor(sim_time/self.t_cycle)
        print("cycle number", cycle_number)

        leg_pos = []
        i = 0

        for phase in self.kinematic_phase:
            x = 0
            y = 0
            z = 0

            rot_velocity = np.asarray([yaw_rate*((-1)**(i+1)), yaw_rate*((-1)**(i)), 0])

            start_phase = phase + self.beta
            end_phase = phase

            if start_phase >= 1:
                start_phase = start_phase - 1
            if end_phase == 0:
                end_phase = 1

            start_phase = start_phase + cycle_number
            end_phase = end_phase + cycle_number

            l_swing = self.l_stride + rot_velocity
            # l_swing[0] = min(0.05, abs(l_swing[0]))*np.sign(l_swing[0])
            # l_swing[1] = min(0.05, abs(l_swing[1]))*np.sign(l_swing[1])

            print("new lswing", l_swing)
            t0 = self.t_cycle*cycle_number
            t1 = self.t_cycle*start_phase
            t2 = self.t_cycle*start_phase + self.t_cycle*self.leg_raise_time
            t3 = self.t_cycle*end_phase - self.t_cycle*self.leg_raise_time
            t4 = self.t_cycle*end_phase

            if sim_time < t1:
                print("phase T1")
                y = 0
                x = 0
                z = 0

            if sim_time >= t1 and sim_time < t2:
                print("phase T2")
                x = 0
                y = 0
                z_traj = trajectory.Trajectory(0, leg_raise_height, 0, leg_vz, t1, t2)
                z = z_traj.at_time(sim_time)[0]

            if sim_time >= t2 and sim_time <= t3:
                print("phase T3")
                x_traj = trajectory.Trajectory(0, l_swing[0], 0, 0, t2, t3)
                y_traj = trajectory.Trajectory(0, l_swing[1], 0, 0, t2, t3)
                z_traj = trajectory.Trajectory(leg_raise_height, leg_raise_height, leg_vz, -leg_vz, t2, t3)

                x = x_traj.at_time(sim_time)[0]
                y = y_traj.at_time(sim_time)[0]
                z = z_traj.at_time(sim_time)[0]

            if sim_time >= t3 and sim_time <= t4:
                print("phase T4")
                x = l_swing[0]
                y = l_swing[1]
                z_traj = trajectory.Trajectory(leg_raise_height, 0, -leg_vz, 0, t3, t4)
                z = z_traj.at_time(sim_time)[0]

            if sim_time > t4:
                print("phase T5")
                x = l_swing[0]
                y = l_swing[1]
                z = 0
            print("answer", x, y, z)
            x_offset = (-l_swing[0]/self.t_cycle)*(sim_time%self.t_cycle)
            y_offset = (-l_swing[1]/self.t_cycle)*(sim_time%self.t_cycle)
            z_offset = self.robot_height

            leg_pos.append([(x + x_offset), (y + y_offset), z_offset - z])
            i=i+1

        print(leg_pos)
        return leg_pos