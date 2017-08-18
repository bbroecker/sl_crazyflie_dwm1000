#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from sl_crazyflie_srvs.srv import StartGoalWalk, StartGoalWalkRequest
from std_srvs.srv import Empty
import numpy as np

NN = 13
NEW_GOAL_BUTTON = 11
MIN_X = -1.0
MAX_X = 1.0
MIN_Y = -1.0
MAX_Y = 1.0


def map_value(value, in_min, in_max, out_min, out_max):
    return ((value - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min


class Teleop:
    def __init__(self):
        joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.joy_subscriber_ = rospy.Subscriber(joy_topic, Joy, self.joy_callback)
        self.pid_active_button = rospy.get_param("~pid_activate_axis", 11)

        self.toggle_coordinator = rospy.ServiceProxy("/toggle_random_coordinator", Empty)

        self.prev_pressed = {'nn': False}
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()

    def joy_callback(self, joy_msg):
        if self.is_button_released('nn', joy_msg.buttons[NN]):
            self.toggle_coordinator.call()

    def is_button_released(self, button_name, button_pressed):
        if button_pressed:
            self.prev_pressed[button_name] = True
        elif self.prev_pressed[button_name]:
            self.prev_pressed[button_name] = False
            return True
        return False




if __name__ == '__main__':
    rospy.init_node("sl_random_nn_teleop")
    Teleop()
    rospy.spin()