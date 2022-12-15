#!/usr/bin/env python3
import time

import rospy
from scipy.spatial.transform import Rotation
import numpy as np
from math import pi

from RobotController import gait_generator
from InverseKinematics import leg_IK
from RoboticsUtilities import trajectory, Transformations
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist

def get_robot_states(msg):
    global robot_state
    global robot_velocity_control_x
    global robot_velocity_control_y
    robot_state = msg.pose[msg.name.index("notspot_gazebo")]
    robot_velocity_control_x.update_state(robot_state.position.x)
    robot_velocity_control_y.update_state(robot_state.position.y)

def set_traj(msg):
    global joy_robot_velocity_control_x
    global joy_robot_velocity_control_y 
    global joy_robot_yaw_control
    print("velocity", msg.linear.x, msg.linear.y, msg.angular.z)
    joy_robot_velocity_control_x = msg.linear.x
    joy_robot_velocity_control_y = msg.linear.y
    joy_robot_yaw_control = msg.angular.z


USE_JOY = True
USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.1908, 0.080]
legs = [0.0, 0.04, 0.100, 0.094333]
max_l = np.sum(legs);

# global variable updated by get_robot_states
robot_state = None

# way points, keeping staring and ending velocity 0
waypoints = [[0, 0], [1, -1]]
velocity = [[0, 0], [0, 0]]
yaw = [0, 0]
trajectory_time = 20 # in secs

# initialize trajectory
robot_trajectory_x_axis = trajectory.Trajectory(waypoints[0][0], waypoints[1][0], velocity[0][0], velocity[1][0], 0, trajectory_time)
robot_trajectory_y_axis = trajectory.Trajectory(waypoints[0][1], waypoints[1][1], velocity[0][1], velocity[1][1], 0, trajectory_time)
robot_trajectory_yaw = trajectory.Trajectory(yaw[0], yaw[1], 0, 0, 0, trajectory_time)

#initialize controller
joy_robot_velocity_control_x = 0
joy_robot_velocity_control_y = 0
joy_robot_yaw_control = 0
kp = 0.1
robot_velocity_control_x = trajectory.trajectory_controller(kp*3)
# kp needs to be negative for Y
# cause body frame rotated 180 about x-axis
robot_velocity_control_y = trajectory.trajectory_controller(-kp*3)
robot_yaw_control = trajectory.trajectory_controller(0.00)

rospy.Subscriber("notspot_joy/joy_ramped", Twist, set_traj)

# initialize robot gait generator
beta = 0.5
t_cycle = 0.4
robot_height = 0.15
robot_leg = leg_IK.robot_leg(legs)
robot_gait = gait_generator.gait_generator(beta, t_cycle, robot_height, body, robot_leg, max_l)

command_topics = ["/notspot_controller/FR1_joint/command",
                  "/notspot_controller/FR2_joint/command",
                  "/notspot_controller/FR3_joint/command",
                  "/notspot_controller/FL1_joint/command",
                  "/notspot_controller/FL2_joint/command",
                  "/notspot_controller/FL3_joint/command",
                  "/notspot_controller/RR1_joint/command",
                  "/notspot_controller/RR2_joint/command",
                  "/notspot_controller/RR3_joint/command",
                  "/notspot_controller/RL1_joint/command",
                  "/notspot_controller/RL2_joint/command",
                  "/notspot_controller/RL3_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

rospy.Subscriber("/gazebo/model_states", ModelStates, get_robot_states)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

# current simulation time as trajectory start time
start_time = rospy.get_time()
# time allowed to get gazebo setup
settling_time = 3

while not rospy.is_shutdown():

    # get current_trajectory time
    current_trajectory_time = rospy.get_time() - start_time - settling_time
    #print("current time", current_trajectory_time)
    if USE_JOY:
            if current_trajectory_time > 0 and joy_robot_velocity_control_x != 0:
                robot_orientation = robot_state.orientation
                rot = Rotation.from_quat([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
                robot_yaw_control.update_state(rot.as_euler("xyz", degrees=False)[2])

                velocity_command = np.asarray(np.matmul(Transformations.rotz(rot.as_euler("xyz", degrees=False)[2]), np.asarray([[joy_robot_velocity_control_x], [joy_robot_velocity_control_y], [0]])))
                velocity_command = velocity_command.astype(float)
                #print(rot.as_euler("xyz", degrees=False))
                velocity_command = np.asarray([[joy_robot_velocity_control_x], [joy_robot_velocity_control_y], [0]])
                print(velocity_command)
                joint_angles = robot_gait.run(current_trajectory_time, velocity_command, joy_robot_yaw_control)
            else:
                 joint_angles = robot_gait.run(-1, np.asarray([0, 0]), 0)
    else:
        if (not robot_velocity_control_x.current_pose == None and
                not robot_velocity_control_y.current_pose == None )\
                and current_trajectory_time > 0:

            #print("current_body_pos", robot_state.position.x, robot_state.position.y)
            robot_orientation = robot_state.orientation
            rot = Rotation.from_quat([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
            robot_yaw_control.update_state(rot.as_euler("xyz", degrees=False)[2])

            if current_trajectory_time <= trajectory_time:
                # update target
                robot_velocity_control_x.update_target(robot_trajectory_x_axis.at_time(current_trajectory_time)[0])
                robot_velocity_control_y.update_target(robot_trajectory_y_axis.at_time(current_trajectory_time)[0])
                robot_yaw_control.update_target(robot_trajectory_yaw.at_time(current_trajectory_time)[0])

            # calculate velocity command
            vel_command_x = robot_velocity_control_x.get_velocity_command()
            vel_command_y = robot_velocity_control_y.get_velocity_command()
            yaw_rate_command = robot_yaw_control.get_velocity_command()

            velocity_command = np.asarray(np.matmul(Transformations.rotz(rot.as_euler("xyz", degrees=False)[2]), np.asarray([[vel_command_x], [vel_command_y], [0]])))
            velocity_command = velocity_command.astype(float)
            #print(rot.as_euler("xyz", degrees=False))
            velocity_command = np.asarray([[vel_command_x], [vel_command_y], [0]])

            if current_trajectory_time > trajectory_time and \
                    robot_velocity_control_x.current_error() < 0.05 and \
                    robot_velocity_control_y.current_error() < 0.05 and \
                    robot_yaw_control.current_error() < 0.05:
                velocity_command = [0, 0, 0]
                yaw_rate_command = 0
            #print("velocity command", velocity_command)
            # velocity_command = np.asarray([[0.3], [0], [0]])
            yaw_rate_command = 0.0
            joint_angles = robot_gait.run(current_trajectory_time, velocity_command, yaw_rate_command)
        else:
            #print("stance")
            joint_angles = robot_gait.run(-1, np.asarray([0, 0]), 0)

        #print("angles", joint_angles)

    for i in range(len(joint_angles)):
        publishers[i].publish(joint_angles[i])

    rate.sleep()
