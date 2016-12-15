#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, ControlMode, FlightMode
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest
from std_srvs.srv import Empty, EmptyResponse

RATE = 20


class PoseFollower:
    def __init__(self):
        self.is_running = False
        rate = rospy.Rate(30)
        self.target_msg = TargetMsg()
        self.target_msg.control_mode = ControlMode()
        self.target_msg.control_mode.x_mode = self.target_msg.control_mode.y_mode = ControlMode.POSITION
        self.target_msg.control_mode.z_mode = self.target_msg.control_mode.yaw_mode = ControlMode.POSITION
        self.tf_listen = tf.TransformListener()
        pose_topic = rospy.get_param("~pose_topic")
        self.pose_frame_id = rospy.get_param("~pose_frame_id")
        self.x_offset = rospy.get_param("~x_offset", 0.0)
        self.y_offset = rospy.get_param("~y_offset", 0.0)
        self.z_offset = rospy.get_param("~z_offset", 0.0)
        self.world_frame = rospy.get_param("~world_frame_id", "/world")
        self.cmd_pub = rospy.Publisher('external_cmd', TargetMsg, queue_size=1)
        self.sub = rospy.Subscriber(pose_topic, PoseStamped, self.callback_follow_pose)
        self.change_mode = rospy.ServiceProxy('change_flightmode', ChangeFlightMode)
        self.start_srv = rospy.Service("external_modes/start_pose_follower", Empty, self.callback_start)
        self.stop_srv = rospy.Service("external_modes/stop_pose_follower", Empty, self.callback_stop)

        while not rospy.is_shutdown():
            if self.is_running:
                self.cmd_pub.publish(self.target_msg)
            rate.sleep()

    def callback_start(self, msg):
        if not self.is_running:
            req = ChangeFlightModeRequest()
            req.mode.id = FlightMode.EXTERNAL_CONTROL
            self.change_mode.call(req)
            self.is_running = True

        return EmptyResponse()

    def callback_stop(self, msg):

        self.is_running = False

        return EmptyResponse()

    def callback_follow_pose(self, pose_msg):
        follow_pose = PoseStamped()
        follow_pose.pose.position.z = self.z_offset
        follow_pose.pose.orientation.w = 1.
        follow_pose.header.stamp = pose_msg.header.stamp
        follow_pose.header.frame_id = self.pose_frame_id
        target = None
        try:
            self.tf_listen.waitForTransform(self.pose_frame_id, pose_msg.header.frame_id, pose_msg.header.stamp,
                                            rospy.Duration(0.2))
            target = self.tf_listen.transformPose(self.world_frame, follow_pose)
        except tf.Exception as e:
            rospy.logwarn("Transform failed: %s", e)
        if target is not None:
            self.target_msg.target_pose = target


if __name__ == '__main__':
    rospy.init_node("SimpleCollvoid")
    s = PoseFollower()

