#!/usr/bin/env python
__author__ = 'broecker'
# license removed for brevity
import rospy
from std_msgs.msg import String

class sl_crazyflie_controller:

    def __init__(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        #rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber("joy_msgs", String, self.joy_msgs)

        while not rospy.is_shutdown():
         #   hello_str = "hello world %s" % rospy.get_time()
          #  rospy.loginfo(hello_str)
           # pub.publish(hello_str)
            rate.sleep()

    def joy_msgs(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)


if __name__ == '__main__':

    try:
        sl_crazyflie_controller()
    except rospy.ROSInterruptException:
        pass