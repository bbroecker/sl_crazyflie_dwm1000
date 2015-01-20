#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math

class Teleop:


    def __init__(self):
        self.joy_subscriber_ = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.velocity_publisher_ = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.on_client_ = rospy.ServiceProxy('on', Empty)
        self.off_client_ = rospy.ServiceProxy('off', Empty)

        self.axes = {}
        self.axes["x"] = {"axis": 0, "max": 2}
        self.axes["y"] = {"axis": 0, "max": 2}
        self.axes["z"] = {"axis": 0, "max": 2}
        self.axes["yaw"] = {"axis": 0, "max": 90 * math.pi / 180.0}
        self.button = {"on": 1, "off": 2}

        self.axes["x"]["axis"] = rospy.get_param("~x_axis", 0)
        self.axes["y"]["axis"] = rospy.get_param("~y_axis", 0)
        self.axes["z"]["axis"] = rospy.get_param("~z_axis", 0)
        self.axes["yaw"]["axis"] = rospy.get_param("~yaw_axis", 0)

        self.axes["x"]["max"] = rospy.get_param("~x_velocity_max", 2)
        self.axes["y"]["max"] = rospy.get_param("~y_velocity_max", 2)
        self.axes["z"]["max"] = rospy.get_param("~z_velocity_max", 2)
        self.axes["yaw"]["max"] = rospy.get_param("~yaw_velocity_max", 2)

        self.velocity_ = Twist()


    def joy_callback(self, joy_msg):
        self.velocity_.linear.x = self.get_axis(joy_msg, self.axes['x']['axis']) * self.axes["x"]["max"]
        self.velocity_.linear.y = self.get_axis(joy_msg, self.axes['y']['axis']) * self.axes["y"]["max"]
        self.velocity_.linear.z = self.get_axis(joy_msg, self.axes['z']['axis']) * self.axes["z"]["max"]
        self.velocity_.angular.z = self.get_axis(joy_msg, self.axes['yaw']['axis']) * self.axes["yaw"]["max"]
        self.velocity_publisher_.publish(self.velocity_)

        if self.get_button(joy_msg, self.button['on']):
            self.on_client_()
        if self.get_button(joy_msg, self.button['off']):
            self.off_client_()


    def get_axis(self, joy_msg, axis):
        if axis == 0 or axis > len(joy_msg.axes):
            return 0
        sign = 1
        if axis < 0:
            sign = -1
        return sign * joy_msg.axes[abs(axis) - 1]


    def get_button(self, joy_msgs, button_idx):
        if button_idx <= 0 or button_idx > len(joy_msgs.axes):
            return 0
        return joy_msgs.buttons[button_idx - 1]


if __name__ == '__main__':
    rospy.init_node("sl_crazy_teleop")
    Teleop()
    rospy.spin()