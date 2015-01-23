#!/usr/bin/env python
__author__ = 'broecker'
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math

RATE = 50
TIME_OUT = 0.5

class MocapController:
    def __init__(self):

        self.pid_active_button = rospy.get_param("~pid_activate_axis", 11)
        self.pid_active = False
        self.cmd_vel_teleop = Twist()
        self.cmd_vel_pid = Twist()
        self.pid_received = False
        self.teleop_received = False
        self.pid_last_time = None
        self.teleop_last_time = None

        self.hover_stop_srv = rospy.ServiceProxy('hover/stop', Empty)
        self.hold_position_start_srv = rospy.ServiceProxy('hover/start_position_hold', Empty)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.joy_subscriber_ = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.velocity_subscriber_teleop = rospy.Subscriber("teleop/cmd_vel", Twist, self.cmd_vel_callback_teleop)
        self.velocity_subscriber_pid = rospy.Subscriber("hover/cmd_vel", Twist, self.cmd_vel_callback_pid)

    def spin(self):
        r = rospy.Rate(RATE)
        while not self.teleop_received:
            r.sleep()

        while not rospy.is_shutdown():
            cur_time = rospy.Time.now()
            twist = self.cmd_vel_teleop
            if self.pid_active and self.pid_received:
                if (cur_time - self.pid_last_time).to_sec() < TIME_OUT:
                    twist.linear.z = self.cmd_vel_pid.linear.z
                else:
                    twist = Twist()
            else:
                if (cur_time - self.teleop_last_time).to_sec() > TIME_OUT:
                    twist = Twist()
            self.cmd_pub.publish(twist)
            r.sleep()

    def joy_callback(self, joy_msgs):
        if not joy_msgs.buttons[self.pid_active_button] == 0:
            if not self.pid_active:
                self.hold_position_start_srv()
            self.pid_active = True
        else:
            if self.pid_active:
                self.hover_stop_srv()
            self.pid_active = False

    def cmd_vel_callback_teleop(self, twist):
        self.cmd_vel_teleop = twist
        self.teleop_received = True
        self.teleop_last_time = rospy.Time.now()

    def cmd_vel_callback_pid(self, twist):
        self.cmd_vel_pid = twist
        self.pid_received = True
        self.pid_last_time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node("mocap_telecop")
    cont = MocapController()
    cont.spin()













