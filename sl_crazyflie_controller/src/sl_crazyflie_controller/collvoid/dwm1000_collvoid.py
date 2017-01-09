#!/usr/bin/env python
import math
import rospy
from crazyflie_driver.msg import GenericLogData
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, Obstacle
from tf import transformations

from collvoid_interface import CollvoidInterface

ACTIVATION_DISTANCE = 1.5


class DW1000Collvoid(CollvoidInterface):
    def __init__(self, priority):
        CollvoidInterface.__init__(self, priority)
        rospy.Subscriber("log_ranges", GenericLogData, self.range_callback)
        self.dwm_distance = 0
        self.range_active = False



    def range_callback(self, data):
        ranges = data.values[:6]
        min_distance = 2000.0
        self.range_active = False

        state = int(data.values[6])
        valid = [False] * 6
        for i in range(6):
            #is unvalid if distance == 0
            valid[i] = (state & (1 << i)) != 0
            if valid[i] and ranges[i] < min_distance:
                min_distance = ranges[i]






    # calculates the new target velocity based on the collvoid scheme
    # target_Velocity is the current target velocity towards the goal
    def calculate_velocity(self, current_target_velocity):
        return Velocity()

    # tells the main controller if this bahaviour is active
    def is_active(self):
        if not self.range_active and self.dwm_distance:

        return False






