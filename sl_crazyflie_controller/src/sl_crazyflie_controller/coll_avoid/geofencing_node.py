#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseStamped
from sl_crazyflie_msgs.msg import FlightMode, TargetMsg, ControlMode, Velocity
from tf import transformations
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest
from sl_crazyflie_controller.flightmode_manager import POS_CTRL_MODES


def get_yaw_from_msg(msg):
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = transformations.euler_from_quaternion(q)
    return euler[2]


def rotate_vector_by_angle(vector_x, vector_y, angle):
    x = vector_x * math.cos(angle) - vector_y * math.sin(angle)
    y = vector_x * math.sin(angle) + vector_y * math.cos(angle)

    return x, y

class GeoFenchingNode:
    def __init__(self):
        pose_topic = rospy.get_param("~pose_topic")
        vel_topic = rospy.get_param("~target_topic", "geofencing/external_cmd")
        self.min_x = rospy.get_param("~min_x", -1.2)
        self.min_y = rospy.get_param("~min_y", -1.2)
        self.max_x = rospy.get_param("~max_x", 0.1)
        self.max_y = rospy.get_param("~max_y", 0.3)
        self.recover_speed = rospy.get_param("recover_speed", 0.3)
        self.buffer_len = rospy.get_param("buffer_len", 0.05)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.callback_robot_pose)
        self.target_pub = rospy.Publisher(vel_topic, TargetMsg)
        self.is_active = False
        self.change_flight_mode = rospy.ServiceProxy('change_flightmode', ChangeFlightMode)
        self.mode_sub = rospy.Subscriber("flight_mode", FlightMode, self.mode_callback)
        self.fight_mode_backup = None
        self.current_flight_mode = FlightMode()
        self.current_flight_mode.id = FlightMode.MANUAL

    def callback_robot_pose(self, pose):
        if not self.is_active and self.is_outside_fenc(pose) and self.current_flight_mode.id in POS_CTRL_MODES:
            self.is_active = True
            self.fight_mode_backup = self.current_flight_mode
            cfm_msg = ChangeFlightModeRequest()
            cfm_msg.mode.id = FlightMode.EXTERNAL_CONTROL
            self.change_flight_mode.call(cfm_msg)
        elif self.is_active and self.is_outside_fenc(pose, self.buffer_len) and self.current_flight_mode.id in POS_CTRL_MODES:
            # calculate velocity
            # publish velocity
            msg = self.calculate_target_msg(pose)
            self.target_pub.publish(msg)
        elif self.is_active:
            cfm_msg = ChangeFlightModeRequest()
            cfm_msg.mode.id = FlightMode.POS_HOLD
            self.is_active = False
            self.change_flight_mode.call(cfm_msg)


    def mode_callback(self, mode):
        self.current_flight_mode = mode

    def calculate_target_msg(self, pose):
        msg = TargetMsg()
        msg.control_mode = ControlMode()
        msg.control_mode.x_mode = msg.control_mode.y_mode = ControlMode.VELOCITY
        msg.control_mode.z_mode = msg.control_mode.yaw_mode = ControlMode.POSITION
        velo = Velocity()
        if pose.pose.position.x < self.min_x + self.buffer_len:
            velo.x = self.recover_speed
        elif pose.pose.position.x > self.max_x - self.buffer_len:
            velo.x = -self.recover_speed

        if pose.pose.position.y < self.min_y + self.buffer_len:
            velo.y = self.recover_speed
        elif pose.pose.position.y > self.max_y - self.buffer_len:
            velo.y = -self.recover_speed

        norm = math.sqrt(velo.x**2 + velo.y**2)
        velo.x = velo.x / norm * self.recover_speed
        velo.y = velo.y / norm * self.recover_speed


        # toDo yaw
        current_yaw = get_yaw_from_msg(pose)
        rotation_angle = -current_yaw

        velo.x, velo.y = rotate_vector_by_angle(velo.x, velo.y, rotation_angle)
        msg.target_velocity = velo
        msg.target_pose = pose
        return msg


    def is_outside_fenc(self, pose, buffer=0.0):
        assert isinstance(pose, PoseStamped)
        return (pose.pose.position.x < self.min_x + buffer) or (pose.pose.position.y < self.min_y + buffer) or (
        pose.pose.position.x > self.max_x - buffer) or (pose.pose.position.y > self.max_y - buffer)


if __name__ == '__main__':
    rospy.init_node('GeoFenchingNode')
    manager = GeoFenchingNode()
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        rate.sleep()
