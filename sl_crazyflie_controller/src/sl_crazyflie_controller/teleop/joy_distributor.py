#!/usr/bin/env python

#makes it possible to switch the joystick controll between between drones
import rospy
from sensor_msgs.msg import Joy

ALL = "ALL"
UP = 4
DOWN = 6

class JoyDistributor:
    def __init__(self):

        self.cf_ns = rospy.get_param("~cf_name_spaces")
        self.joy_publisher = []
        self.prev_pressed = {'up': False, 'down': False}
        for ns in self.cf_ns:
            self.joy_publisher.append(rospy.Publisher(ns + "/joy", Joy))
        self.cf_ns.append(ALL)
        self.current_ns_idx = 0
        self.sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

    def joy_callback(self, joy_msgs):
        assert isinstance(joy_msgs, Joy)
        if self.is_button_released('up', joy_msgs.buttons[UP]):
            self.current_ns_idx += 1
            self.current_ns_idx %= len(self.cf_ns)
            rospy.loginfo("Sending joy commands to: %s" % (self.cf_ns[self.current_ns_idx]))
        elif self.is_button_released('down', joy_msgs.buttons[DOWN]):
            self.current_ns_idx -= 1
            if self.current_ns_idx < 0:
                self.current_ns_idx = len(self.cf_ns) - 1
            rospy.loginfo("Sending joy commands to: %s" %(self.cf_ns[self.current_ns_idx]))

        for i in range(len(self.cf_ns)-1):
            if self.cf_ns[self.current_ns_idx] == ALL:
                self.joy_publisher[i].publish(joy_msgs)
            elif self.current_ns_idx == i:
                self.joy_publisher[i].publish(joy_msgs)


    def is_button_released(self, button_name, button_pressed):
        if button_pressed:
            self.prev_pressed[button_name] = True
        elif self.prev_pressed[button_name]:
            self.prev_pressed[button_name] = False
            return True
        return False

if __name__ == '__main__':
    rospy.init_node("JoyDistributor")
    jd = JoyDistributor()
    rospy.spin()
