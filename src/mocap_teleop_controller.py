__author__ = 'broecker'
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math

class mocap_controller:


    def __init__(self):
        self.joy_subscriber_ = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.velocity_subscriber_ = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        self.pid_active_button = rospy.get_param("~pid_activate_axis", 13)
        self.pid_active = False

    def joy_callback(self, joy_msgs):
        if joy_msgs.axes[self.pid_active_button] is not 0:
           self.pid_active = True
        else:
            self.pid_active = False
    




