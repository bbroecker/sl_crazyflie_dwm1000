#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import Obstacle

from sl_crazyflie_controller.collvoid.simple_collvoid import ObstacleTime, get_yaw_from_msg
import numpy as np

POSE_TOPIC_PARAM = "~OBJECT_ID/pose_topic"
POSE_TOPIC_OUT = "/OBJECT_ID/pose"
DECAY_FACTOR = 3


def rotate_vel(x_vel, y_vel, angle):
    x = x_vel * np.cos(angle) - y_vel * np.sin(angle)
    y = x_vel * np.sin(angle) + y_vel * np.cos(angle)
    return x, y

class PoseManager:
    def __init__(self):
        rate = rospy.get_param("~obstacle_publish_rate", 100)
        self.pose_subs = {}
        self.obstacle_publisher = rospy.Publisher("/obstacle_poses", Obstacle, queue_size=10)
        self.obstacles = {}
        topics = rospy.get_param("~pose_topics")
        ids = rospy.get_param("~ids")
        if len(ids) is not len(topics):
            rospy.logwarn("parameter count is not correct")
            return
        self.avg_vel = {}
        for i in ids:
            self.avg_vel[i] = []
        for idx, topic_str in enumerate(topics):
            print idx, topic_str
            self.pose_subs[idx] = rospy.Subscriber(topic_str, PoseStamped, self.callback_pose, ids[idx])
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            for key, value in self.obstacles.iteritems():
                if (rospy.Time.now() - value.time).to_sec() < (1.0 / rate) * DECAY_FACTOR:
                    value.obs.x_vel_local = np.mean([c.obs.x_vel_local for c in self.avg_vel[i]])
                    value.obs.y_vel_local = np.mean([c.obs.y_vel_local for c in self.avg_vel[i]])
                    value.obs.x_vel_global = np.mean([c.obs.x_vel_global for c in self.avg_vel[i]])
                    value.obs.y_vel_global = np.mean([c.obs.y_vel_global for c in self.avg_vel[i]])
                    value.obs.z_vel = np.mean([c.obs.z_vel for c in self.avg_vel[i]])
                    self.obstacle_publisher.publish(value.obs)
                    self.avg_vel[i] = [value]
                    # value.obs.x_vel_local = value.obs.y_vel_local = value.obs.x_vel_global = value.obs.y_vel_global = value.obs.z_vel= 0.0
            # self.obstacles = {}
            r.sleep()

    def callback_pose(self, pose, i):
        assert isinstance(pose, PoseStamped)
        obs = Obstacle()
        obs.x = pose.pose.position.x
        obs.y = pose.pose.position.y
        obs.z = pose.pose.position.z
        obs.x_vel_local = obs.y_vel_local = obs.z_vel = 0.0
        obs.x_vel_global = obs.y_vel_global = 0.0
        obs.yaw = get_yaw_from_msg(pose)
        obs.id = i
        now = rospy.Time.now()
        if i in self.obstacles:
            prev_ob = self.obstacles[i]
            dt = (now - prev_ob.time).to_sec()
            obs.x_vel_global = (obs.x - prev_ob.obs.x) / dt
            obs.y_vel_global = (obs.y - prev_ob.obs.y) / dt
            # if abs(obs.x_vel_global) > 0.01 or abs(obs.y_vel_global) > 0.01:
            #     print "obs manager x: {} y: {}".format(obs.x_vel_global, obs.y_vel_global)
            #     if abs(obs.x_vel_global) > 0.01:
            #         print "x pos  {} x prev_pos {}".format(obs.x, prev_ob.obs.x)
            obs.z_vel = (obs.z - prev_ob.obs.z) / dt
            obs.x_vel_local, obs.y_vel_local = rotate_vel(obs.x_vel_global, obs.y_vel_global, -obs.yaw)

            # obs.x_vel_local = obs.x_vel_global * np.cos(-obs.yaw) - obs.y_vel_local * np.sin(-obs.yaw)
            # obs.y_vel_local = obs.x_vel_global * np.sin(-obs.yaw) + obs.y_vel_local * np.cos(-obs.yaw)

            #     print "obstacle manager x: {} y: {}".format(obs.x_vel_local, obs.y_vel_local)

        ot = ObstacleTime(obs, now)
        self.avg_vel[i].append(ot)
        self.obstacles[i] = ot


if __name__ == '__main__':
    rospy.init_node("PoseManager")
    pm = PoseManager()





