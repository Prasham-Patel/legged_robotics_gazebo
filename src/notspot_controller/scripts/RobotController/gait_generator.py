#!/usr/bin/env python3

from RoboticsUtilities import trajectory
from RoboticsUtilities import Transformations
from InverseKinematics import leg_IK
from math import floor
import numpy as np
from decimal import Decimal

class gait_generator():

    def __init__(self, beta, t_cycle, robot_height, robot_dimensions, robot_leg, max_l):
        # constant
        self.beta = beta
        self.t_cycle = t_cycle
        self.leg_raise_time = 0.05
        self.robot_leg = robot_leg
        self.robot_height = robot_height
        self.robot_dimensions = robot_dimensions # [length, width]
        self.max_l = max_l # maximum stride length 

        # define leg base frame
        l = self.robot_dimensions[0]/2
        b = self.robot_dimensions[1]/2
        self.l1_base = [l, b]
        self.l2_base = [l, -b]
        self.l3_base = [-l, -b]
        self.l4_base = [-l, b]

        # updates with velocity command
        self.v_body_g = None
        self.yaw_rate = None
        self.l_stride = None

        self.leg_pos = None
        self.robot_body_pos = None
        self.body_pos_init = False
        self.previous_timestamp = 0

        self.phase_times= [[],[],[],[]]

    def run(self, sim_time, velocity_command, yaw_rate_command):
        if np.any(self.v_body_g == None) or np.all(self.v_body_g == 0):
            # new v_body_g and l_stride from velocity command
            self.update_gait(velocity_command, yaw_rate_command)

        # get kinematic phase array to time leg movement
        self.get_kinematic_phase()
        self.kinematic_phase[3] = self.kinematic_phase[0]
        self.kinematic_phase[2] = self.kinematic_phase[1]

        # settling time stance
        if sim_time < 0:
            stance = [0.0, 0.0, self.robot_height]
            angles = []
            for i in range(0, 4):
                leg_angles = self.robot_leg.inverse_kinematics(stance, i)
                angles.append(leg_angles[0])
                angles.append(leg_angles[1])
                angles.append(leg_angles[2])

            return angles

        else:
            leg_pos = self.get_leg_pos(sim_time, velocity_command, yaw_rate_command)
            leg_pos = self.get_leg_rotation_offset(leg_pos, yaw_rate_command)
            #print("rotated", leg_pos)
            angles = []
            for i in range(0, 4):
                leg_angles = self.robot_leg.inverse_kinematics(leg_pos[i], i)
                angles.append(leg_angles[0])
                angles.append(leg_angles[1])
                angles.append(leg_angles[2])

            return angles

    def update_gait(self, velocity_command, yaw_rate):
        self.v_body_g = velocity_command.transpose()[0]
        self.l_stride = self.v_body_g*self.t_cycle if np.any(self.v_body_g * self.t_cycle < self.max_l) else (self.max_l-0.05) * self.t_cycle
        self.yaw_rate = yaw_rate
        #print("v_body", self.v_body_g)
        #print("t_cycle", self.t_cycle)
        #print("l_stride", self.l_stride)

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

    def get_leg_rotation_offset(self, leg_pose, yaw_rate):
        new_leg_pose = []
        for i in range(0, 4):
            l = self.robot_dimensions[0] / 2
            b = self.robot_dimensions[1] / 2
            offset = [l, b, 0]
            #print("yaw", yaw_rate)
            rotated_pose = np.matmul(Transformations.rotz(yaw_rate*(-1**((i+1)%2))), np.add(leg_pose[i], offset))
            new_leg_pose.append(np.subtract(rotated_pose, offset))

        return new_leg_pose

    def get_leg_pos(self, sim_time, velocity_command, yaw_rate):
        "gives local position which is added in to default stance"
        leg_raise_height = 0.03
        leg_vz = 0.0

        cycle_number = floor(sim_time/self.t_cycle)
        #print("cycle number", cycle_number)

        leg_pos = []
        i = 0

        for idx,phase in enumerate(self.kinematic_phase):
            x = 0
            y = 0
            z = 0

            l_swing = self.l_stride*self.beta

            # if we are at the beginning of a new cycle 
            if float((Decimal(sim_time) % Decimal(self.t_cycle))) == 0.0 or len(self.phase_times[idx]) == 0:
                start_phase = phase + self.beta
                end_phase = phase

                if start_phase >= 1:
                    start_phase = start_phase - 1
                if end_phase == 0:
                    end_phase = 1

                start_phase = start_phase + cycle_number
                end_phase = end_phase + cycle_number

                #print("new lswing", l_swing)
                t0 = self.t_cycle*cycle_number
                t1 = self.t_cycle*start_phase
                t2 = self.t_cycle*start_phase + self.t_cycle*self.leg_raise_time
                t3 = self.t_cycle*end_phase - self.t_cycle*self.leg_raise_time
                t4 = self.t_cycle*end_phase

                self.phase_times[idx] = [t0, t1, t2, t3, t4]

            t0 = self.phase_times[idx][0]
            t1 = self.phase_times[idx][1]
            t2 = self.phase_times[idx][2]
            t3 = self.phase_times[idx][3]
            t4 = self.phase_times[idx][4]

            if sim_time <= t0:
                leg_pos.append([0.0, 0.0, self.robot_height])
                self.update_gait(velocity_command, yaw_rate)
                continue

            #print("gait trajectory")
            #print(self.kinematic_phase)
            #print(sim_time)
            #print("l_swing", l_swing, self.t_cycle)
            if sim_time > t0 and sim_time < t1:
                #print("phase T1")
                y = (-l_swing[1] / (self.t_cycle)) * (float((Decimal(sim_time) % Decimal(self.t_cycle))))
                x = (-l_swing[0] / (self.t_cycle)) * (float((Decimal(sim_time) % Decimal(self.t_cycle))))
                z = self.robot_height

            if sim_time >= t1 and sim_time < t2:
                #print("phase T2")
                x = (-l_swing[0] / (self.t_cycle)) * (float((Decimal(sim_time) % Decimal(self.t_cycle))))
                y = (-l_swing[1] / (self.t_cycle)) * (float((Decimal(sim_time) % Decimal(self.t_cycle))))
                z_traj = trajectory.Trajectory(0, leg_raise_height, 0, leg_vz, t1, t2)
                z = self.robot_height - z_traj.at_time(sim_time)[0]

            if sim_time >= t2 and sim_time <= t3:
                #print("phase T3")
                x_traj = trajectory.Trajectory((-l_swing[0] / self.t_cycle) * (t2 % self.t_cycle), ((-l_swing[0] / self.t_cycle) * (t2 % self.t_cycle)) + l_swing[0], 0, 0, t2, t3)
                y_traj = trajectory.Trajectory((-l_swing[1] / self.t_cycle) * (t2 % self.t_cycle), ((-l_swing[1] / self.t_cycle) * (t2 % self.t_cycle) )+ l_swing[1], 0, 0, t2, t3)
                z_traj = trajectory.Trajectory(leg_raise_height, leg_raise_height, leg_vz, -leg_vz, t2, t3)

                x = x_traj.at_time(sim_time)[0]
                y = y_traj.at_time(sim_time)[0]
                z = self.robot_height - z_traj.at_time(sim_time)[0]

            if sim_time >= t3 and sim_time <= t4:
                #print("phase T4")
                x = ((-l_swing[0] / (self.t_cycle)) * (t2 % self.t_cycle)) + l_swing[0] + ((-l_swing[0] / (self.t_cycle)) * ((sim_time - t2) % self.t_cycle))
                y = ((-l_swing[1] / (self.t_cycle)) * (t2 % self.t_cycle)) + l_swing[1] + ((-l_swing[1] / (self.t_cycle)) * ((sim_time - t2) % self.t_cycle))

                z_traj = trajectory.Trajectory(leg_raise_height, 0, -leg_vz, 0, t3, t4)
                z = self.robot_height - z_traj.at_time(sim_time)[0]

            if sim_time > t4:
                #print("phase T5")
                x = ((-l_swing[0] / (self.t_cycle)) * (t2 % self.t_cycle)) + l_swing[0] + (
                            (-l_swing[0] / (self.t_cycle)) * ((sim_time - t2) % self.t_cycle))
                y = ((-l_swing[1] / (self.t_cycle)) * (t2 % self.t_cycle)) + l_swing[1] + (
                            (-l_swing[1] / (self.t_cycle)) * ((sim_time - t2) % self.t_cycle))

                z = self.robot_height
            #print("answer", x, y, z)
            leg_pos.append([x, y, z])
            i=i+1

        #print(leg_pos)
        return leg_pos


if __name__ == '__main__':
    body = [0.1908, 0.080]
    legs = [0.0, 0.04, 0.100, 0.094333]
    # initialize robot gait generator
    beta = 0.75
    t_cycle = 0.5
    robot_height = 0.15
    robot_leg = leg_IK.robot_leg(legs)
    robot_gait = gait_generator.gait_generator(beta, t_cycle, robot_height, body, robot_leg)
