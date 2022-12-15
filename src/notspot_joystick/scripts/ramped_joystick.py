#!/usr/bin/env python3
#Author: lnotspotl

import rospy
from math import fabs
from numpy import array_equal

import os
from tkinter import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from pynput.keyboard import Key, Listener

class PS4_controller(Tk):
    def __init__(self, rate):
        super(PS4_controller, self).__init__()
        rospy.init_node("Joystick_ramped")
        self.publisher = rospy.Publisher("notspot_joy/joy_ramped", Twist, queue_size = 10)
        self.rate = rospy.Rate(rate)

        self.use_button = True

        self.speed_index = 1
        self.available_speeds = [ -0.5, 0.0, 0.5, 1.0]

        self.rot_speed_index = 0
        self.rot_available_speeds = [0.0, 0.5, 1.0, 3.0, 4.0]

        self.changed = False

        self.geometry("100x100")
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)
        self.sensorText = StringVar()
        self.sensorLabel = Label(self, textvariable=self.sensorText)
        self.sensorLabel.pack()
        self.mainloop()


    def keydown(self, event):
        print(event.keysym)
        if event.keysym == "Up":
            self.speed_index += 1
            if self.speed_index >= len(self.available_speeds):
                self.speed_index = 0
            self.changed = True

        if event.keysym == "Down":
            self.speed_index -= 1
            if self.speed_index < 0:
                self.speed_index = len(self.available_speeds) - 1
            rospy.loginfo(f"Joystick speed:{self.available_speeds[self.speed_index]}")
            self.changed = True

        if event.keysym == "Left":
            self.rot_speed_index -= 1
            if self.rot_speed_index < 0:
                self.rot_speed_index = len(self.rot_available_speeds) - 1
            rospy.loginfo(f"Joystick speed:{self.rot_available_speeds[self.rot_speed_index]}")
            self.changed = True
        if event.keysym == "Right":
            self.rot_speed_index += 1
            if self.speed_index >= len(self.rot_available_speeds):
                self.rot_speed_index = len(self.rot_speed_index) - 1
            rospy.loginfo(f"Joystick speed:{self.rot_available_speeds[self.rot_speed_index]}")
            self.changed = True

        self.publish_joy()
    def keyup(self, event):
        pass


    def run(self):
        while not rospy.is_shutdown():
            self.publish_joy()
            self.rate.sleep()

    def publish_joy(self):
        t_now = rospy.Time.now()

        # if the desired value is the same as the last value, there's no
        # need to publish the same message again
        if self.changed:
            # new message
            print("sending")
            twist = Twist()
            twist.linear.x = self.available_speeds[self.speed_index]
            twist.linear.y = 0
            twist.angular.z = 0
            self.publisher.publish(twist)
            self.changed = False

        self.last_send_time = t_now


if __name__ == "__main__":
    joystick = PS4_controller(rate = 30)
    joystick.run()
