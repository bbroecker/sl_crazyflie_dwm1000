import rospy
from crazyflie_driver.msg import GenericLogData


class test:
    def __init__(self):
        rospy.Subscriber("/crazyflie1/log_ranges", GenericLogData, self.log_callback)
        self.prev_value = 0.0
        self.last_update = rospy.Time.now()
        rospy.spin()

    def log_callback(self, log_data):
        # print "callback"
        value = log_data.values[1]
        if value != self.prev_value:
            self.prev_value = value
            print "update_rate {}".format(1.0 / (rospy.Time.now() - self.last_update).to_sec())
            self.last_update = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node("test123")
    test()
