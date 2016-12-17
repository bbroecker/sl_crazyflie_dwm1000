#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import Obstacle

from sl_crazyflie_controller.collvoid.simple_collvoid import ObstacleTime

POSE_TOPIC_PARAM = "~OBJECT_ID/pose_topic"
POSE_TOPIC_OUT = "/OBJECT_ID/pose"
DECAY_FACTOR = 3

class PoseManager:
    def __init__(self):
        max_pose_topics = rospy.get_param("~max_pose_topics", 100)
        rate = rospy.get_param("~obstacle_publish_rate", 20)
        self.pose_subs = {}
        self.obstacle_publisher = rospy.Publisher("/obstacle_poses", Obstacle, queue_size=10)
        self.obstacles = {}
        for i in range(max_pose_topics):
            topic_str = POSE_TOPIC_PARAM.replace("ID", str(i))
            if rospy.has_param(topic_str):
                self.pose_subs[i] = rospy.Subscriber(rospy.get_param(topic_str), PoseStamped, self.callback_pose, i)
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            for key, value in self.obstacles.iteritems():
                if (rospy.Time.now() - value.time).to_sec() < (1.0 / rate) * DECAY_FACTOR:
                    self.obstacle_publisher.publish(value.obs)
            r.sleep()

    def callback_pose(self, pose, i):
        assert isinstance(pose, PoseStamped)
        obs = Obstacle()
        obs.x = pose.pose.position.x
        obs.y = pose.pose.position.y
        obs.z = pose.pose.position.z
        obs.id = i

        ot = ObstacleTime(obs, rospy.Time.now())

        self.obstacles[i] = ot


if __name__ == '__main__':
    rospy.init_node("PoseManager")
    pm = PoseManager()





