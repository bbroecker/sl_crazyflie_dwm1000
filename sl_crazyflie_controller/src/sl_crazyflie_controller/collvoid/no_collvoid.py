#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, Obstacle
from tf import transformations

from collvoid_interface import CollvoidInterface

#placeholder, does nothing
class NoCollvoid(CollvoidInterface):
    def __init__(self):
        CollvoidInterface.__init__(self)


    def is_active(self):
        return False


    def calculate_velocity(self, current_target_velocity):
        assert isinstance(current_target_velocity, Velocity)
        return current_target_velocity



