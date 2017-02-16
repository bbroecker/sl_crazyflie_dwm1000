#!/usr/bin/env python
import math
import random

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, ControlMode, FlightMode
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest, StartGoalWalk, StartGoalWalkRequest, \
    StartGoalWalkResponse
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse

from sl_crazyflie_behavior_trees.BehaviorTree import AIIO, BehaviorTree
from sl_crazyflie_behavior_trees.bt_composites import BTTarget, BTTargetSimulator
from sl_crazyflie_behavior_trees.Geometry.point import Point
from sl_crazyflie_controller.flight_controller import euler_distance_pose

RATE = 100.0
SLOW_RATE = 10.0

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



class RandomGoalWalker:
    def __init__(self):
        cf_pose_topic = rospy.get_param("~cf_pose_topic")
        self.max_velocity = rospy.get_param('~goal_walk/max_velocity')
        self.angle_max_speed = rospy.get_param("~goal_walk/max_angle_vel")
        self.max_angle_increment = rospy.get_param("~goal_walk/max_angle_increment")
        self.goal_distance = rospy.get_param("~goal_walk/goal_distance")
        self.buffer_to_wall = rospy.get_param("~goal_walk/buffer_to_wall")


        self.pose_sub = rospy.Subscriber(cf_pose_topic, PoseStamped, self.pose_cb)
        self.last_pose = None
        rate = rospy.Rate(RATE)
        self.is_outside = False
        self.is_active = False
        self.last_time = None
        self.m_aiIO = AIIO()
        self.m_psi_cmd = 0
        self.m_aiIO.input = [0.0]
        self.state_array = [Point(), 0.0, Point()]
        self.m_ai = BehaviorTree(1, 3 , 1)
        def_dehav_root = BTTargetSimulator(self.state_array, 1.0 / RATE)
        self.m_ai.m_root_node = def_dehav_root
        self.goal_count = 0

        self.cmd = TargetMsg()
        self.x_min = 0
        self.x_max = 0
        self.y_min = 0
        self.y_max = 0
        self.current_vel = Velocity()

        self.cmd.control_mode.x_mode = self.cmd.control_mode.y_mode = ControlMode.VELOCITY
        self.cmd.control_mode.z_mode = self.cmd.control_mode.yaw_mode = ControlMode.POSITION
        self.start_random_walk = rospy.Service("start_goal_walk", StartGoalWalk, self.start_cb)
        self.stop_random_walk = rospy.Service("stop_goal_walk", Empty, self.stop_cb)
        self.change_flight_mode = rospy.ServiceProxy('change_flightmode', ChangeFlightMode)
        self.toggle_dwm_avoid = rospy.ServiceProxy("toggle_dwm_avoid", Empty)
        self.publish_cmd = rospy.Publisher("external_cmd", TargetMsg, queue_size=1)
        self.publish_target = rospy.Publisher("target_pose_walk", PoseStamped, queue_size=1)
        self.publish_goal_count = rospy.Publisher("goal_count", Int32, queue_size=1)
        last_goal_pub = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.is_active and self.last_pose is not None:
                self.update()
                self.cmd.target_velocity = self.current_vel
                # print self.cmd
                self.cmd.target_pose.header.stamp = rospy.Time.now()
                self.publish_target.publish(self.cmd.target_pose)
                #print self.cmd
                self.publish_cmd.publish(self.cmd)
            if (rospy.Time.now() - last_goal_pub).to_sec() > 1.0 / SLOW_RATE:
                self.publish_goal_count.publish(self.goal_count)
                last_goal_pub = rospy.Time.now()
            rate.sleep()


    def update_vel(self, dt):
        # print "generate_new_vel"

        #current point
        self.state_array[0].x = self.last_pose.pose.position.x
        self.state_array[0].y = self.last_pose.pose.position.y
        self.state_array[0].z = self.last_pose.pose.position.z

        rotation_angle = -yaw_of_pose(self.last_pose)
        goal_vec_x, goal_vec_y = rotate_vector_by_angle(self.cmd.target_pose.pose.position.x, self.cmd.target_pose.pose.position.y, rotation_angle)

        #target_point
        self.state_array[2].x = goal_vec_x
        self.state_array[2].y = goal_vec_y
        self.state_array[2].z = self.cmd.target_pose.pose.position.z

        # current yaw
        self.state_array[1] = math.atan2(self.current_vel.y, self.current_vel.x)

        self.m_ai.trigger(self.m_aiIO)
        velocity = self.m_aiIO.output[0] * self.max_velocity
        angle_speed = self.m_aiIO.output[1]
        angle_increment = self.m_aiIO.output[2]

        rotation_angle = ((angle_speed * self.angle_max_speed/180.0 * math.pi * dt) + (angle_increment/180.0 * self.max_angle_increment))

        while rotation_angle < -math.pi:
            rotation_angle += 2 * math.pi

        while rotation_angle > math.pi:
            rotation_angle -= 2 * math.pi

        self.current_vel.x, self.current_vel.y = rotate_vector_by_angle(self.current_vel.x,
                                                                        self.current_vel.y, rotation_angle)
        norm = math.sqrt(self.current_vel.x**2 + self.current_vel.y**2)
        self.current_vel.x /= norm
        self.current_vel.x *= velocity
        self.current_vel.y /= norm
        self.current_vel.y *= velocity



    def update_target(self):
        self.cmd.target_pose.pose.position.x = random.uniform(self.x_min + self.buffer_to_wall, self.x_max - self.buffer_to_wall)
        self.cmd.target_pose.pose.position.y = random.uniform(self.y_min + self.buffer_to_wall, self.y_max - self.buffer_to_wall)
       # self.cmd.target_pose.pose.position.z = self.current_height

    def start_cb(self, req):
        assert isinstance(req, StartGoalWalkRequest)
        self.x_min = req.x_min
        self.x_max = req.x_max
        self.y_min = req.y_min
        self.y_max = req.y_max
        yaw = yaw_of_pose(self.last_pose)
        self.current_vel.x = math.cos(yaw)
        self.current_vel.y = math.sin(yaw)
        print "{0} {1} {2} {3}".format(req.x_min, req.x_max, req.y_min, req.y_max)
        print "start random_walk"
        self.cmd.target_pose = self.last_pose
        self.is_active = True
        cfr = ChangeFlightModeRequest()
        cfr.mode.id = FlightMode.EXTERNAL_CONTROL
        self.change_flight_mode.call(cfr)
        self.update_target()
        cmd = TargetMsg()
        cmd.control_mode = ControlMode()
        cmd.control_mode.x_mode = cmd.control_mode.y_mode = ControlMode.POSITION
        cmd.control_mode.z_mode = cmd.control_mode.yaw_mode = ControlMode.POSITION
        cmd.target_pose = self.last_pose
        self.publish_cmd.publish(cmd)
        self.toggle_dwm_avoid()
        return StartGoalWalkResponse()

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

    def update(self):
        if self.last_pose is not None:
            if euler_distance_pose(self.last_pose, self.cmd.target_pose) < self.goal_distance:
                self.goal_count += 1
                self.update_target()
            self.update_vel(1.0/RATE)


    def pose_cb(self, pose):
        assert isinstance(pose, PoseStamped)
        self.last_pose = pose

if __name__ == '__main__':
    rospy.init_node("RandomGoalWalker")
    rw = RandomGoalWalker()