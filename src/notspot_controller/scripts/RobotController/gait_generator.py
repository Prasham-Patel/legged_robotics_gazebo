#!/usr/bin/env python3

from RoboticsUtilities import trajectory
from math import floor
import numpy as np

class gait_generator():

    def __init__(self):
        self.beta = None
        self.t_cycle = None
        self.v_body_g = None
        self.yaw_rate = None
        self.l_stride = None
        self.leg_raise_time = 0.05
        self.leg_pos = None
        self.robot_body_pos = None
        self.previous_timestamp = 0

    def update_gait(self, velocity_command, yaw_rate):
        self.v_body_g = velocity_command
        self.l_stride = self.v_body_g*self.t_cycle
        self.yaw_rate = yaw_rate

    def update_velocity_commands(self, sim_time, velocity_command, yaw_rate_commnad, stance):
        self.get_kinematic_phase()
        self.update_gait(velocity_command, yaw_rate_commnad)
        leg_pos = self.get_leg_pos(sim_time)

        # taking into account ground velocity of robot
        delta_x, delta_y = velocity_command[0]*(sim_time - self.previous_timestamp), velocity_command[1]*(sim_time - self.previous_timestamp)
        if self.robot_body_pos == None:
            self.robot_body_pos = np.reshape(stance, (4, 3))
        leg1 = leg_pos[0] + [delta_x, delta_y, 0] + self.robot_body_pos[0]
        leg2 = leg_pos[1] + [delta_x, delta_y, 0] + self.robot_body_pos[1]
        leg3 = leg_pos[2] + [delta_x, delta_y, 0] + self.robot_body_pos[2]
        leg4 = leg_pos[3] + [delta_x, delta_y, 0] + self.robot_body_pos[3]



        leg_pos = [leg1, leg2, leg3, leg4]
        return leg_pos


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

    def get_leg_pos(self, sim_time):
        "gives local position which is added in to default stance"
        leg_raise_height = 0.01
        leg_vz = 0.2

        cycle_number = floor(sim_time/self.t_cycle)

        leg_pos = []

        for phase in self.kinematic_phase:
            x = 0
            y = 0
            z = 0

            start_phase = phase + self.beta
            end_phase = phase

            if start_phase >= 1:
                start_phase = start_phase - 1
            if end_phase == 0:
                end_phase = 1

            start_phase = start_phase + cycle_number
            end_phase = end_phase + cycle_number

            l_swing = self.l_stride
            t1 = self.t_cycle*start_phase
            t2 = self.t_cycle*start_phase + self.t_cycle*self.leg_raise_time
            t3 = self.t_cycle*end_phase - self.t_cycle*self.leg_raise_time
            t4 = self.t_cycle*end_phase

            if sim_time < t1:
                x = 0
                y = 0
                z = 0

            if sim_time >= t1 and sim_time < t2:
                x = 0
                y = 0
                z_traj = trajectory.Trajectory(0, leg_raise_height, 0, leg_vz, t1, t2)
                z = z_traj.at_time(sim_time)

            if sim_time >= t2 and sim_time <= t3:
                x_traj = trajectory.Trajectory(0, l_swing[0], 0, 0, t2, t3)
                y_traj = trajectory.Trajectory(0, l_swing[1], 0, 0, t2, t3)
                z_traj = trajectory.Trajectory(leg_raise_height, leg_raise_height, leg_vz, -leg_vz, t2, t3)

                x = x_traj.at_time(sim_time)
                y = y_traj.at_time(sim_time)
                z = z_traj.at_time(sim_time)

            if sim_time >= t3 and sim_time <= t4:
                x = l_swing[0]
                y = l_swing[1]
                z_traj = trajectory.Trajectory(leg_raise_height, 0, -leg_vz, 0, t3, t4)
                z = z_traj.at_time(sim_time)
            if sim_time > t4:
                x = l_swing[0]
                y = l_swing[1]
                z = 0

            leg_pos.append(np.asarray([x, y, z]))

        return leg_pos