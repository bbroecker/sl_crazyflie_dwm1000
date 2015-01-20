#!/usr/bin/env python
__author__ = 'broecker'
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math

class mocap_controller:


    def __init__(self):



        self.pid_active_button = rospy.get_param("~pid_activate_axis", 11)
        self.pid_active = False
        self.cmd_vel_teleop = Twist()
        self.cmd_vel_pid = Twist()
        self.pid_received = False
        self.teleop_received = False
        r = rospy.Rate(5)

        self.joy_subscriber_ = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.velocity_subscriber_teleop = rospy.Subscriber("teleop/cmd_vel", Twist, self.cmd_vel_callback_teleop)
        self.velocity_subscriber_pid = rospy.Subscriber("hover/cmd_vel", Twist, self.cmd_vel_callback_pid)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.set_pub = rospy.Publisher('hover/setpoint', Twist, queue_size=10)
        self.pid_toggle_srv = rospy.ServiceProxy('hover/toggle', Empty)
        self.toggle_hover_srv = rospy.ServiceProxy('hover/toggle_hover', Empty)

        while not rospy.is_shutdown():
           #print self.teleop_received, self.pid_active, self.pid_received
            if (self.pid_received or not self.pid_active) and self.teleop_received:
                twist = self.cmd_vel_teleop
                if self.pid_active:
                    #twist.linear.z = 22888.2068396
                    twist.linear.z = self.cmd_vel_pid.linear.z
                self.pid_received = False
                self.teleop_received = False
                self.cmd_pub.publish(twist)
            r.sleep()




    def joy_callback(self, joy_msgs):
        if not joy_msgs.buttons[self.pid_active_button] == 0:
           if self.pid_active == False:
               self.pid_toggle_srv()
               self.toggle_hover_srv()
           self.pid_active = True
        else:
            if self.pid_active == True:
                self.pid_toggle_srv()
            self.pid_active = False


    def cmd_vel_callback_teleop(self, twist):
        self.cmd_vel_teleop = twist
        self.teleop_received = True

    def cmd_vel_callback_pid(self, twist):
        self.cmd_vel_pid = twist
        self.pid_received = True

if __name__ == '__main__':
    rospy.init_node("mocap_telecop")
    mocap_controller()














