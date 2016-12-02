#!/usr/bin/env python
import rospy
from sl_crazyflie_msgs.msg import TargetMsg

TIME_OUT = 0.1

class Mixer():
    def __init__(self):
        rospy.init_node("TargetMSGMixer")
        self.teleop_sub = rospy.Subscriber('teleop/external_cmd', TargetMsg, self.teleop_msg_callback)
        self.geofencing_sub = rospy.Subscriber('geofencing/external_cmd', TargetMsg, self.geofencing_msg_callback)
        self.geofencing_sub = rospy.Subscriber('testing/external_cmd', TargetMsg, self.testing_msg_callback)
        self.pub = rospy.Publisher('external_cmd', TargetMsg, queue_size=1)
        self.last_teleop_update = None
        self.last_teleop_msg = None
        self.last_geofencing_update = None
        self.last_geofencing_msg = None
        self.last_testing_update = None
        self.last_testing_msg = None
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_msg()
            rate.sleep()

    def is_valid(self, time):
        return time is not None and (rospy.Time.now() - time).to_sec() < TIME_OUT

    def publish_msg(self):
        if self.is_valid(self.last_geofencing_update):
            # rospy.loginfo("geofencing")
            self.pub.publish(self.last_geofencing_msg)
        elif self.is_valid(self.last_teleop_update):
            self.pub.publish(self.last_teleop_msg)
        elif self.is_valid(self.last_testing_update):
            self.pub.publish(self.last_testing_msg)


    def teleop_msg_callback(self, msg):
        assert isinstance(msg, TargetMsg)
        self.last_teleop_update = rospy.Time.now()
        self.last_teleop_msg = msg

    def geofencing_msg_callback(self, msg):
        assert isinstance(msg, TargetMsg)
        self.last_geofencing_update = rospy.Time.now()
        self.last_geofencing_msg = msg

    def testing_msg_callback(self, msg):
        assert isinstance(msg, TargetMsg)
        self.last_testing_update = rospy.Time.now()
        self.last_testing_msg = msg

if __name__ == '__main__':
    m = Mixer()