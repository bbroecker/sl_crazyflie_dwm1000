#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

PATH_FOLLOWING = 13


def map_value(value, in_min, in_max, out_min, out_max):
    return ((value - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min


class Teleop:
    def __init__(self):
        joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.joy_subscriber_ = rospy.Subscriber(joy_topic, Joy, self.joy_callback)
        self.pid_active_button = rospy.get_param("~pid_activate_axis", 11)

        self.start_follow = rospy.ServiceProxy("/start_way_following",Empty)
        self.stop_follow = rospy.ServiceProxy("/cancel_way_following", Empty)

        self.prev_pressed = {'path_following': False}
        self.follow_active = False

    def joy_callback(self, joy_msg):
        if self.is_button_released('path_following', joy_msg.buttons[PATH_FOLLOWING]):
            if self.follow_active:
                print "stop"
                self.follow_active = False
                self.stop_follow.call()
            else:
                self.start_follow.call()
                self.follow_active = True
                print "active"

    def is_button_released(self, button_name, button_pressed):
        if button_pressed:
            self.prev_pressed[button_name] = True
        elif self.prev_pressed[button_name]:
            self.prev_pressed[button_name] = False
            return True
        return False




if __name__ == '__main__':
    rospy.init_node("sl_follow_way_teleop")
    Teleop()
    rospy.spin()