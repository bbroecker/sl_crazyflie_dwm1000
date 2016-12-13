#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity

TIME_OUT = 0.1


class PoseTime:
    def __init__(self, pose, time):
        self.time = time
        self.pose = pose


def xy_distance(pose1, pose2):
    x = pose1.pose.position.x - pose2.pose.position.x
    y = pose1.pose.position.y - pose2.pose.position.y
    return math.sqrt(x ** 2 + y ** 2)


def vel_magnitude(vel):
    assert isinstance(vel, Velocity)
    return math.sqrt(vel.x ** 2 + vel.y ** 2)


class SimpleCollvoid:
    def __init__(self):
        self.obstacle_pose_topics = rospy.get_param("~obstacle_pose_topics")
        self.cf_pose_topic = rospy.get_param("~cf_pose_topic")
        self.min_xy_dist = rospy.get_param("~min_xy_dist")
        self.max_xy_dist = rospy.get_param("~max_xy_dist")
        self.max_xy_avoid_vel = rospy.get_param("~max_xy_avoid_vel")
        self.max_xy_goal_vel = rospy.get_param("~max_xy_goal_vel")
        self.p_factor = 1.5
        self.subs = []

        self.last_pose_dict = {}
        self.last_pose = None
        self.cmd_pub = rospy.Publisher("collvoid_cmd", TargetMsg)
        self.cmd_sub = rospy.Subscriber("external_cmd", TargetMsg, self.external_cmd_callback)
        for topic in self.obstacle_pose_topics:
            self.subs.append(rospy.Subscriber(topic, PoseStamped, self.obs_pose_callback, topic))
        self.subs = rospy.Subscriber(self.cf_pose_topic, PoseStamped, self.pose_callback)

    def obs_pose_callback(self, pose, topic):
        pt = PoseTime(pose, rospy.Time.now())
        self.last_pose_dict[topic] = pt

    def pose_callback(self, pose):
        self.last_pose = pose

    def obstacle_vel(self, cf_pose, obs_pose):
        assert isinstance(cf_pose, PoseStamped)
        assert isinstance(obs_pose, PoseStamped)
        vel = Velocity()
        vel.x = cf_pose.pose.position.x - obs_pose.pose.position.x
        vel.y = cf_pose.pose.position.y - obs_pose.pose.position.y
        return vel

    def invert_vel(self, vel):
        result = Velocity()
        mag = vel_magnitude(vel)
        distance_range = self.max_xy_dist - self.min_xy_dist
        if mag > self.max_xy_dist:
            mag = self.max_xy_dist
        elif mag < self.min_xy_dist:
            mag = self.max_xy_dist
        invert_factor = (distance_range - (mag - self.min_xy_dist)) / distance_range
        result.x = (vel.x / mag) * invert_factor * self.max_xy_avoid_vel * self.p_factor
        result.y = (vel.y / mag) * invert_factor * self.max_xy_avoid_vel * self.p_factor
        return result

    def external_cmd_callback(self, target_msg):
        assert isinstance(target_msg, TargetMsg)
        obstacle_poses = []
        for key, value in self.last_pose_dict.iteritems():
            assert isinstance(value, PoseTime)
            if (rospy.Time.now() - value.time).to_sec() < TIME_OUT and xy_distance(self.last_pose, value.pose) < self.max_xy_dist:
                obstacle_poses.append(value.pose)

        if self.last_pose is None or not self.last_pose_dict or len(obstacle_poses) <= 0:
            self.cmd_pub.publish(target_msg)
        else:

            avoid_vel = Velocity()
            for ob in obstacle_poses:
                vel = self.obstacle_vel(self.last_pose, ob)
                vel = self.invert_vel(vel)
                avoid_vel.x += vel.x
                avoid_vel.y += vel.y
            mag = vel_magnitude(avoid_vel)
            if mag > self.max_xy_avoid_vel:
                avoid_vel.x = avoid_vel.x / mag * self.max_xy_avoid_vel
                avoid_vel.y = avoid_vel.y / mag * self.max_xy_avoid_vel
            current_vel = target_msg.target_velocity
            mag = vel_magnitude(current_vel)
            if mag > self.max_xy_goal_vel:
                current_vel.x = current_vel.x / mag * self.max_xy_goal_vel
                current_vel.y = current_vel.y / mag * self.max_xy_goal_vel

            current_vel.x += avoid_vel.x
            current_vel.y += avoid_vel.y
            target_msg.target_velocity = current_vel
            self.cmd_pub.publish(target_msg)


if __name__ == '__main__':
    rospy.init_node("SimpleCollvoid")
    s = SimpleCollvoid()
    rospy.spin()
