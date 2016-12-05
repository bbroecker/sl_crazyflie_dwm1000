#!/usr/bin/env python
import copy
import math
import rospy
import actionlib
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_pathfollowing.msg import FollowSegmentAction, FollowSegmentActionGoal, LineSegment

UPDATE_RATE = 30
LINE_SEG_BUFFER_TIME = 0.5

def euler_distance_pose(pose1, pose2):
    assert isinstance(pose1, PoseStamped)
    assert isinstance(pose2, PoseStamped)
    return math.sqrt(math.pow(pose1.pose.position.x - pose2.pose.position.x, 2) + math.pow(
        pose1.pose.position.y - pose2.pose.position.y, 2) +
                     math.pow(pose1.pose.position.z - pose2.pose.position.z, 2))
def norm(pose):
    assert isinstance(pose, PoseStamped)
    return math.sqrt(pose.pose.position.x + pose.pose.position.y**2 + pose.pose.position.z**2)

#pose1 - pose2
def pose_diff(pose1, pose2):
    result = copy.deepcopy(pose1)
    result.pose.position.x -= pose2.pose.position.x
    result.pose.position.y -= pose2.pose.position.y
    result.pose.position.z -= pose2.pose.position.z
    return result


class SegmentFollower:
    def __init__(self, name):
        rospy.init_node(name)
        pose_topic = rospy.get_param("pose_topic", "/Robot_1/pose")
        self.last_pose = None
        self.action_srv = actionlib.SimpleActionServer(name, FollowSegmentAction, execute_cb=self.follow_cb, auto_start=False)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback)


    def follow_cb(self, goal):
        assert isinstance(goal, FollowSegmentActionGoal)
        if goal.segment_type is LineSegment.TYPE_ID:
            current_line = goal.line
            assert isinstance(current_line, LineSegment)
            if current_line.start is None:
                current_line.start = self.last_pose
            goal_time = goal.max_time # in sec
            segment_length = euler_distance_pose(current_line.start, current_line.end_point) #in m
            duration = goal_time - LINE_SEG_BUFFER_TIME
            speed = segment_length / duration
            distance_per_update = speed * (1.0 / (UPDATE_RATE * duration))
            direction = pose_diff(current_line.end_point, current_line.start)
            x_step = direction.pose.position.x / self.norm(direction) * distance_per_update
            y_step = direction.pose.position.y / self.norm(direction) * distance_per_update
            z_step = direction.pose.position.z / self.norm(direction) * distance_per_update
            current_x = current_line.start.pose.position.x
            current_y = current_line.start.pose.position.y
            current_z = current_line.start.pose.position.z

            quaternion = [current_line.start.pose.orientation.x, current_line.start.pose.orientation.y, current_line.start.pose.orientation.z, current_line.start.pose.orientation.w]
            current_yaw = tf.transformations.euler_from_quaternion(quaternion)




        pass

    # def loop(self):
    #     r = rospy.Rate(UPDATE_RATE)
    #     while not rospy.is_shutdown():
    #         r.sleep()


    def pose_callback(self, pose):
        self.last_pose = pose


if __name__ == '__main__':
    rospy.init_node("SegmentFollower")
    s = SegmentFollower(rospy.get_name())
    rospy.spin()

