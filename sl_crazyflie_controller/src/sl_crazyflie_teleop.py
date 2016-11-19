#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math
from sl_crazyflie_msgs.msg import Velocity


def convert_value(value, in_min, in_max, out_min, out_max):
    return ((value - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min

class Teleop:


    def __init__(self):
        self.joy_subscriber_ = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.manual_mode_publisher_ = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.velocity_mode_publisher_ = rospy.Publisher("teleop/velocity", Velocity, queue_size=1)
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

        self.axes["x"]["max"] = rospy.get_param("~x_joy_max", 30)
        self.axes["y"]["max"] = rospy.get_param("~y_joy_max", -30)#invert
        self.axes["z"]["max"] = rospy.get_param("~z_joy_max", 60000)
        self.axes["yaw"]["max"] = rospy.get_param("~yaw_joy_max", -200)

        self.x_max_vel = rospy.get_param("~x_vel_max", 0.1)
        self.y_max_vel = rospy.get_param("~x_vel_max", 0.1)
        self.z_max_vel = rospy.get_param("~z_vel_max", 0.1)
        self.yaw_max_vel = rospy.get_param("~yaw_vel_max", 0.5)

        self.joy_value_ = Twist()



    def gen_vel_msg(self, twist):
        assert isinstance(twist, Twist)
        vel = Velocity()
        vel.x = convert_value(twist.linear.x, -self.axes["x"]["max"], self.axes["x"]["max"], -self.x_max_vel,
                              self.x_max_vel)
        vel.y = convert_value(twist.linear.y, -self.axes["y"]["max"], self.axes["y"]["max"], -self.y_max_vel,
                              self.y_max_vel)
        vel.z = convert_value(twist.linear.z, -self.axes["z"]["max"], self.axes["z"]["max"], -self.z_max_vel,
                              self.z_max_vel)
        vel.yaw = convert_value(twist.angular.z, -self.axes["yaw"]["max"], self.axes["yaw"]["max"], -self.yaw_max_vel,
                              self.yaw_max_vel)
        return vel



    def joy_callback(self, joy_msg):
        self.joy_value_.linear.x = self.get_axis(joy_msg, self.axes['x']['axis']) * self.axes["x"]["max"]
        self.joy_value_.linear.y = self.get_axis(joy_msg, self.axes['y']['axis']) * self.axes["y"]["max"]
        self.joy_value_.linear.z = self.get_axis(joy_msg, self.axes['z']['axis']) * self.axes["z"]["max"]
        self.joy_value_.angular.z = self.get_axis(joy_msg, self.axes['yaw']['axis']) * self.axes["yaw"]["max"]

        self.manual_mode_publisher_.publish(self.joy_value_)
        self.velocity_mode_publisher_.publish(self.gen_vel_msg(self.joy_value_))

        # if self.get_button(joy_msg, self.button['on']):
        #     self.on_client_()
        # if self.get_button(joy_msg, self.button['off']):
        #     self.off_client_()


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