#!/usr/bin/env python3
import time

import rospy
# from rospy import time
from scipy.spatial.transform import Rotation
import numpy as np
from math import pi

from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController, gait_generator
from InverseKinematics import robot_IK
from RoboticsUtilities import trajectory, Transformations
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates

def get_robot_states(msg):
    global robot_state
    global robot_velocity_control_x
    global robot_velocity_control_y
    robot_state = msg.pose[msg.name.index("notspot_gazebo")]
    robot_velocity_control_x.update_state(robot_state.position.x)
    robot_velocity_control_y.update_state(robot_state.position.y)

def get_time(msg):
    print(msg)

USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.1908, 0.080]
legs = [0.0, 0.04, 0.100, 0.094333]

# global variable updated by get_robot_states
robot_state = None

# way points, keeping staring and ending velocity 0
waypoints = [[0, 0], [2, -2]]
direction = [[0, 0], [0, 0]]
yaw = [0, pi/4]
trajectory_time = 20 # in secs

# initialize trajectory
robot_trajectory_x_axis = trajectory.Trajectory(waypoints[0][0], waypoints[1][0], direction[0][0], direction[1][0], 0, trajectory_time)
robot_trajectory_y_axis = trajectory.Trajectory(waypoints[0][1], waypoints[1][1], direction[0][1], direction[1][1], 0, trajectory_time)
robot_trajectory_yaw = trajectory.Trajectory(yaw[0], yaw[1], 0, 0, 0, trajectory_time)

#initialize controller
kp = 0.1
robot_velocity_control_x = trajectory.trajectory_controller(kp)
robot_velocity_control_y = trajectory.trajectory_controller(kp)
robot_yaw_control = trajectory.trajectory_controller(kp)

# initialize robot gait controller
robot_gait = gait_generator.gait_generator()
notspot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

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

if USE_IMU:
    rospy.Subscriber("notspot_imu/base_link_orientation",Imu,notspot_robot.imu_orientation)

rospy.Subscriber("/gazebo/model_states", ModelStates, get_robot_states)
rospy.Subscriber("notspot_joy/joy_ramped",Joy,notspot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

start_time = rospy.get_time()
notspot_robot.use_trot_gait()

while not rospy.is_shutdown():
    leg_positions = notspot_robot.run()
    # notspot_robot.change_controller()

    # get current_trajectory time
    current_trajectory_time = rospy.get_time() - start_time

    if not robot_velocity_control_x.current_pose == None and \
            not robot_velocity_control_y.current_pose == None:
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
        # yaw_rate_command = 0

        velocity_command = np.matmul(Transformations.rotz(rot.as_euler("xyz", degrees=False)[2]), np.asarray([[vel_command_x], [vel_command_y], [0]]))
        print(rot.as_euler("xyz", degrees=False))

        if current_trajectory_time > trajectory_time and \
                robot_velocity_control_x.current_error() < 0.05 and \
                robot_velocity_control_y.current_error() < 0.05 and \
                robot_yaw_control.current_error() < 0.05:
            velocity_command = [0, 0, 0]
            yaw_rate_command = 0

        # leg_positions = robot_gait.update_velocity_commands(current_trajectory_time, velocity_command, yaw_rate_command, notspot_robot.default_stance())

        notspot_robot.trajectory_controller_command(velocity_command[:2], yaw_rate_command)


    dx = notspot_robot.state.body_local_position[0]
    dy = notspot_robot.state.body_local_position[1]
    dz = notspot_robot.state.body_local_position[2]
    
    roll = notspot_robot.state.body_local_orientation[0]
    pitch = notspot_robot.state.body_local_orientation[1]
    yaw = notspot_robot.state.body_local_orientation[2]

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw)

        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
    except:
        pass

    rate.sleep()
