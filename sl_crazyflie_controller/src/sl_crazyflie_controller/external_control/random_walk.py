#!/usr/bin/env python
import math
import random

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, ControlMode, FlightMode
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest
from std_srvs.srv import Empty, EmptyResponse

def yaw_of_pose(pose):
    if isinstance(pose, PoseStamped):
        quaternion = [pose.pose.orientation.x, pose.pose.orientation.y,
                      pose.pose.orientation.z, pose.pose.orientation.w]
    else:
        quaternion = [pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w]
    yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw[2]

def quaternon_from_yaw(yaw):
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return quaternion[0], quaternion[1], quaternion[2], quaternion[3]

def rotate_vector_by_angle(vector_x, vector_y, angle):
    x = vector_x * math.cos(angle) - vector_y * math.sin(angle)
    y = vector_x * math.sin(angle) + vector_y * math.cos(angle)

    return x, y

class RandomWalker:
    def __init__(self):
        cf_pose_topic = rospy.get_param("~cf_pose_topic")
        self.max_speed = rospy.get_param("~random_walk/max_speed")
        self.border_x_min = rospy.get_param('~random_walk/border/min_x')
        self.border_x_max = rospy.get_param("~random_walk/border/max_x")
        self.border_y_min = rospy.get_param("~random_walk/border/min_y")
        self.border_y_max = rospy.get_param("~random_walk/border/max_y")

        self.center_x_min = rospy.get_param("~random_walk/center/min_x")
        self.center_x_max = rospy.get_param("~random_walk/center/max_x")
        self.center_y_min = rospy.get_param("~random_walk/center/min_y")
        self.center_y_max = rospy.get_param("~random_walk/center/max_y")
        self.buffer = rospy.get_param("~random_walk/buffer_len")
        self.recover_timer = rospy.get_param("~random_walk/recover_timer")




        self.pose_sub = rospy.Subscriber(cf_pose_topic, PoseStamped, self.pose_cb)
        self.last_pose = None
        rate = rospy.Rate(20)
        self.is_outside = False
        self.current_vel = None
        self.is_active = False
        self.current_vel = None
        self.last_time = None
        self.cmd = TargetMsg()
        self.cmd.control_mode.x_mode = self.cmd.control_mode.y_mode = ControlMode.VELOCITY
        self.cmd.control_mode.z_mode = self.cmd.control_mode.yaw_mode = ControlMode.POSITION
        self.start_random_walk = rospy.Service("start_random_walk", Empty, self.start_cb)
        self.stop_random_walk = rospy.Service("stop_random_walk", Empty, self.stop_cb)
        self.change_flight_mode = rospy.ServiceProxy('change_flightmode', ChangeFlightMode)
        self.toggle_dwm_avoid = rospy.ServiceProxy("toggle_dwm_avoid", Empty)
        self.publish_cmd = rospy.Publisher("external_cmd", TargetMsg, queue_size=1)

        while not rospy.is_shutdown():
            if self.is_active and self.last_pose is not None:
                self.cmd.target_velocity = self.update_vel()
                # print self.cmd
                self.cmd.target_pose.header.stamp = rospy.Time.now()
                #print self.cmd
                self.publish_cmd.publish(self.cmd)
            rate.sleep()

    def pose_outside_border(self, buff=0):
        assert isinstance(self.last_pose, PoseStamped)
        x = self.last_pose.pose.position.x
        y = self.last_pose.pose.position.y
        return x < self.border_x_min + buff or x > self.border_x_max - buff or y < self.border_y_min + buff or y > self.border_y_max - buff

    def generate_vel(self):
        print "generate_new_vel"
        current_yaw = yaw_of_pose(self.last_pose)
        x = self.last_pose.pose.position.x
        y = self.last_pose.pose.position.y
        goal_x = random.uniform(self.center_x_min, self.center_x_max)
        goal_y = random.uniform(self.center_y_min, self.center_y_max)
        #rotate goal vector

        goal_vec_x = goal_x - x
        goal_vec_y = goal_y - y
        rotation_angle = -current_yaw
        goal_vec_x, goal_vec_y = rotate_vector_by_angle(goal_vec_x, goal_vec_y, rotation_angle)

        norm = math.sqrt(goal_vec_x**2 + goal_vec_y**2)

        goal_vec_x = goal_vec_x/norm * self.max_speed
        goal_vec_y = goal_vec_y / norm * self.max_speed


        print goal_x, goal_y
        vel = Velocity()
        vel.x = goal_vec_x
        vel.y = goal_vec_y
        vel.z = 0
        vel.yaw = 0
        return vel

    def start_cb(self, req):
        print "start random_walk"
        self.cmd.target_pose = self.last_pose
        self.is_active = True
        cfr = ChangeFlightModeRequest()
        cfr.mode.id = FlightMode.EXTERNAL_CONTROL
        self.change_flight_mode.call(cfr)
        self.current_vel = self.generate_vel()
        cmd = TargetMsg()
        cmd.control_mode = ControlMode()
        cmd.control_mode.x_mode = cmd.control_mode.y_mode = ControlMode.POSITION
        cmd.control_mode.z_mode = cmd.control_mode.yaw_mode = ControlMode.POSITION
        cmd.target_pose = self.last_pose
        self.publish_cmd.publish(cmd)
        self.toggle_dwm_avoid()
        return EmptyResponse()

    def stop_cb(self, req):
        cmd = TargetMsg()
        cmd.control_mode = ControlMode()
        cmd.control_mode.x_mode = cmd.control_mode.y_mode = ControlMode.POSITION
        cmd.control_mode.z_mode = cmd.control_mode.yaw_mode = ControlMode.POSITION
        cmd.target_pose = self.last_pose
        self.publish_cmd.publish(cmd)
        self.is_active = False
        cfr = ChangeFlightModeRequest()
        cfr.mode.id = FlightMode.POS_HOLD
        self.change_flight_mode.call(cfr)
        self.toggle_dwm_avoid()
        return EmptyResponse()

    def update_vel(self):
        if self.last_pose is not None:
            if not self.is_outside and self.pose_outside_border():
                self.is_outside = True
                self.current_vel = self.generate_vel()
                self.last_time = rospy.Time.now()
            elif not self.pose_outside_border(self.buffer) or (self.last_time is not None and (rospy.Time.now() - self.last_time).to_sec() > self.recover_timer):
                self.is_outside = False
        return self.current_vel

    def pose_cb(self, pose):
        assert isinstance(pose, PoseStamped)
        self.last_pose = pose

if __name__ == '__main__':
    rospy.init_node("random_walk")
    rw = RandomWalker()