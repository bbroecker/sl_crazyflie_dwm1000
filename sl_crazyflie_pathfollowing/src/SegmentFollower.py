#!/usr/bin/env python
import copy
import math
from threading import Lock

import rospy
import actionlib
import tf
from sl_crazyflie_msgs.msg import FlightMode, TargetMsg, ControlMode
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_pathfollowing.msg import FollowSegmentAction, FollowSegmentActionGoal, LineSegment, \
    FollowSegmentActionFeedback, FollowSegmentActionResult, FollowSegmentGoal, FollowSegmentResult, \
    FollowSegmentFeedback

UPDATE_RATE = 30
LINE_SEG_BUFFER_TIME = 0.5
TARGET_THRESHOLD = 0.05

def euler_distance_pose(pose1, pose2):
    assert isinstance(pose1, PoseStamped)
    assert isinstance(pose2, PoseStamped)
    return math.sqrt(math.pow(pose1.pose.position.x - pose2.pose.position.x, 2) + math.pow(
        pose1.pose.position.y - pose2.pose.position.y, 2) +
                     math.pow(pose1.pose.position.z - pose2.pose.position.z, 2))
def norm(pose):
    assert isinstance(pose, PoseStamped)
    return math.sqrt(pose.pose.position.x**2 + pose.pose.position.y**2 + pose.pose.position.z**2)

#pose1 - pose2
def pose_diff(pose1, pose2):
    result = copy.deepcopy(pose1)
    result.pose.position.x -= pose2.pose.position.x
    result.pose.position.y -= pose2.pose.position.y
    result.pose.position.z -= pose2.pose.position.z
    return result


def limit_angle(angle):
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle

def yaw_of_pose(pose):
    quaternion = [pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w]
    yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw[2]

def quaternon_from_yaw(yaw):
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return quaternion[0], quaternion[1], quaternion[2], quaternion[3]

class SegmentFollower:
    def __init__(self, name):
        rospy.init_node(name)
        pose_topic = rospy.get_param("~pose_topic")
        self.last_pose = None
        self.action_srv = actionlib.SimpleActionServer("follow_segment_action", FollowSegmentAction, execute_cb=self.follow_cb, auto_start=False)
        self.control_command_pub = rospy.Publisher("path_following/external_cmd", TargetMsg, queue_size=1)
        self.action_srv.start()
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback)
        self.change_flight_mode = rospy.ServiceProxy('change_flightmode', ChangeFlightMode)
        self.mode_sub = rospy.Subscriber("flight_mode", FlightMode, self.mode_callback)
        self.current_flight_mode = None
        self.lock = Lock()
        self.is_following = False
        self.action_srv.register_preempt_callback(self.preempted_callback)

    def mode_callback(self, mode):
        assert isinstance(mode, FlightMode)
        self.current_flight_mode = mode

    def is_pos_hold(self):
        return self.current_flight_mode.id is FlightMode.POS_HOLD

    def active_pos_hold(self):
        m = ChangeFlightModeRequest()
        m.mode.id = FlightMode.POS_HOLD
        self.change_flight_mode.call(m)

    def deactivate_pos_hold(self):
        print "deactivate_pos_hold"
        m = ChangeFlightModeRequest()
        m.mode.id = FlightMode.EXTERNAL_CONTROL
        self.change_flight_mode.call(m)

    def preempted_callback(self):
        rospy.loginfo("cancel current goal")
        self.is_following = False

    def follow_line(self, goal):
        print "follow_line"
        self.is_following = True
        assert isinstance(goal, FollowSegmentGoal)
        current_line = goal.line
        assert isinstance(current_line, LineSegment)
        if goal.take_current_pose:
            current_line.start_point = self.last_pose

        goal_time = goal.max_time  # in sec
        segment_length = euler_distance_pose(current_line.start_point, current_line.end_point)  # in m
        # duration = goal_time - LINE_SEG_BUFFER_TIME
        speed = goal.max_travel_vel
        angular_speed = goal.max_angle_vel
        travel_duration = segment_length / speed
        direction = pose_diff(current_line.end_point, current_line.start_point)

        x_speed = direction.pose.position.x / norm(direction) * speed
        y_speed = direction.pose.position.y / norm(direction) * speed
        z_speed = direction.pose.position.z / norm(direction) * speed

        start_yaw = yaw_of_pose(current_line.start_point.pose)
        end_yaw = yaw_of_pose(current_line.end_point.pose)
        rotating = limit_angle(end_yaw - start_yaw)
        current_yaw = start_yaw

        if rotating < 0.0:
            angular_speed *= -1.0

        rot_duration = rotating / angular_speed
        start_time = rospy.Time.now()
        rate = rospy.Rate(30)
        last_update = start_time
        target_pose = copy.deepcopy(current_line.start_point)
        process_percent = 0.0
        while self.is_following and not rospy.is_shutdown() and self.action_srv.is_active() and not self.action_srv.is_new_goal_available():

            #check flight mode
            if self.is_pos_hold():
                self.deactivate_pos_hold()

            #public target msg
            if euler_distance_pose(self.last_pose, current_line.end_point) < TARGET_THRESHOLD:
                result = FollowSegmentResult()
                result.success = True
                #pos_hold
                self.action_srv.set_succeeded(result)
                break

            pasted_time = (rospy.Time.now() - start_time).to_sec()
            #public feedback
            process_percent = pasted_time / travel_duration
            if process_percent > 1.0:
                process_percent = 1.0
            feedback = FollowSegmentFeedback()
            feedback.percent_complete = process_percent
            self.action_srv.publish_feedback(feedback)

            #update target pose
            if pasted_time <= travel_duration:
                target_pose.pose.position.x = current_line.start_point.pose.position.x + x_speed * pasted_time #time * speed = distance
                target_pose.pose.position.y = current_line.start_point.pose.position.y + y_speed * pasted_time
                target_pose.pose.position.z = current_line.start_point.pose.position.z + z_speed * pasted_time
            else:
                target_pose.pose.position = current_line.end_point.pose.position

            if pasted_time <= rot_duration:
                target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w = quaternon_from_yaw(
                    current_yaw + angular_speed * pasted_time)
            else:
                target_pose.pose.orientation = current_line.end_point.pose.orientation

            msg = TargetMsg()
            msg.target_pose = target_pose
            ctrl_mode = ControlMode()
            ctrl_mode.x_mode = ctrl_mode.y_mode = ctrl_mode.z_mode = ctrl_mode.yaw_mode = ControlMode.POSITION
            msg.control_mode = ctrl_mode
            self.control_command_pub.publish(msg)

            rate.sleep()

        if not self.is_following:
            result = FollowSegmentResult()
            result.success = False
            # pos_hold
            self.action_srv.set_aborted(result)

        self.active_pos_hold()

        # self.lock.release()

    def follow_cb(self, goal):
        print "follow_cb"
        assert isinstance(goal, FollowSegmentGoal)
        if goal.segment_type is LineSegment.TYPE_ID:
            if self.is_following:
                self.is_following = False
                # while not self.follow_done:
                #     pass
            # making sure the last follow command finished
            # self.lock.aquire()
            self.follow_line(goal)

    # def loop(self):
    #     r = rospy.Rate(UPDATE_RATE)
    #     while not rospy.is_shutdown():
    #         r.sleep()


    def pose_callback(self, pose):
        self.last_pose = pose


if __name__ == '__main__':
    s = SegmentFollower("SegmentFollower")
    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        r.sleep()

