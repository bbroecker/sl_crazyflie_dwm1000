#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseStamped
from sl_crazyflie_msgs.msg import FlightMode, TargetMsg, ControlMode, Velocity
from tf import transformations
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest

from collvoid_interface import CollvoidInterface
from sl_crazyflie_controller.flightmode_manager import POS_CTRL_MODES


def get_yaw_from_msg(msg):
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = transformations.euler_from_quaternion(q)
    return euler[2]


def rotate_vector_by_angle(vector_x, vector_y, angle):
    x = vector_x * math.cos(angle) - vector_y * math.sin(angle)
    y = vector_x * math.sin(angle) + vector_y * math.cos(angle)
    return x, y

class GeoFenchingNode(CollvoidInterface):
    def __init__(self, priority):
        CollvoidInterface.__init__(self, priority)
        self.min_x = rospy.get_param("~collvoid/geofencing/min_x")
        self.min_y = rospy.get_param("~collvoid/geofencing/min_y")
        self.max_x = rospy.get_param("~collvoid/geofencing/max_x")
        self.max_y = rospy.get_param("~collvoid/geofencing/max_y")
        self.recover_speed = rospy.get_param("~collvoid/geofencing/recover_speed")
        self.buffer_len = rospy.get_param("~collvoid/geofencing/buffer_len")
        self.avoid = False

    def update_cf_pose(self, pose):
        self.current_cf_pose = pose
        if not self.avoid and self.is_outside_fenc(pose):
            self.avoid = True
        elif self.avoid and not self.is_outside_fenc(pose, self.buffer_len):
            self.avoid = False

    def is_active(self):
        if self.avoid:
            print "geofencing!!"
            print self.current_cf_pose
        return self.avoid

    def calculate_velocity(self, current_target_velocity):
        msg = TargetMsg()
        msg.control_mode = ControlMode()
        msg.control_mode.x_mode = msg.control_mode.y_mode = ControlMode.VELOCITY
        msg.control_mode.z_mode = msg.control_mode.yaw_mode = ControlMode.POSITION
        velo = Velocity()
        if self.current_cf_pose.pose.position.x < self.min_x + self.buffer_len:
            velo.x = self.recover_speed
        elif self.current_cf_pose.pose.position.x > self.max_x - self.buffer_len:
            velo.x = -self.recover_speed

        if self.current_cf_pose.pose.position.y < self.min_y + self.buffer_len:
            velo.y = self.recover_speed
        elif self.current_cf_pose.pose.position.y > self.max_y - self.buffer_len:
            velo.y = -self.recover_speed

        norm = math.sqrt(velo.x**2 + velo.y**2)
        velo.x = velo.x / norm * self.recover_speed
        velo.y = velo.y / norm * self.recover_speed

        current_yaw = get_yaw_from_msg(self.current_cf_pose)
        rotation_angle = -current_yaw
        velo.x, velo.y = rotate_vector_by_angle(velo.x, velo.y, rotation_angle)
        return velo

    def is_outside_fenc(self, pose, buffer=0.0):
        assert isinstance(pose, PoseStamped)
        return (pose.pose.position.x < self.min_x + buffer) or (pose.pose.position.y < self.min_y + buffer) or (
        pose.pose.position.x > self.max_x - buffer) or (pose.pose.position.y > self.max_y - buffer)


