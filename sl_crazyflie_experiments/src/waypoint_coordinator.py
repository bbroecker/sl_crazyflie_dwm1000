#!/usr/bin/env python
import math
import random
import rospkg
import yaml

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, ControlMode, FlightMode
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
PATH_FOLDER = "path_files"

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

def distance_pose(pose1, pose2):
    x = pose1.pose.position.x - pose2.pose.position.x
    y = pose1.pose.position.y - pose2.pose.position.y
    return np.sqrt(x**2 + y**2)


class Path:
    def __init__(self):
        self.path = []
        self.current_idx = 0
        self.path_len = 0


class DroneInfo:
    def __init__(self, drone_id):
        self.drone_pose = None
        self.goal_pose = None
        self.goal_count = 0
        self.drone_id = drone_id
        self.path = None

class WaypointCoordinator:
    def __init__(self):
        self.waypoint_file = rospy.get_param("~waypoint_coordinator/waypoint_file")

        # self.buffer = rospy.get_param("~random_walk/buffer_len")

        self.drone_pose_topics = rospy.get_param("~waypoint_coordinator/pose_topics")
        self.drone_ns = rospy.get_param("~waypoint_coordinator/namespaces")
        self.drone_info = []
        if len(self.drone_pose_topics) != len(self.drone_ns):
            rospy.logerr("namespace and pose_topics have to have the same length")
            return
        self.goal_publishers = []
        self.start_nn = []
        self.stop_nn = []
        self.goal_counter_publisher = []
        self.is_running = False
        rospack = rospkg.RosPack()
        self.path = rospack.get_path("sl_crazyflie_experiments") + '/' + PATH_FOLDER + '/'
        rospy.Service("/toggle_random_coordinator", Empty, self.toggle_coordinator)
        rospy.Service("/next_goals", Empty, self.next_goal)
        for i in range(len(self.drone_pose_topics)):
            rospy.logwarn(self.drone_pose_topics[i])
            rospy.Subscriber(self.drone_pose_topics[i], PoseStamped, self.pose_cb, callback_args=i)
            self.goal_publishers.append(rospy.Publisher(self.drone_ns[i] + "/goal_pose", PoseStamped, queue_size=1))
            self.start_nn.append(rospy.ServiceProxy(self.drone_ns[i] + "/start_nn_controller", Empty))
            self.stop_nn.append(rospy.ServiceProxy(self.drone_ns[i] + "/stop_nn_controller", Empty))
            self.goal_counter_publisher.append(rospy.Publisher(self.drone_ns[i] + "/goal_counter", Int32, queue_size=1))
            self.drone_info.append(DroneInfo(i))
        self.last_pose = None
        self.read_waypoints()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.is_running:
                self.publish_goal_poses()
                self.publish_goal_counter()
            rate.sleep()

    def read_waypoints(self):
        yaml_f = open(self.path + self.waypoint_file)
        dataMap = yaml.safe_load(yaml_f)
        # self.waypoints[self.name_spaces[idx]] = dataMap
        yaml_f.close()
        dataMap = dataMap['waypoints']
        for i in range(len(dataMap)):
            print dataMap[i]
            self.drone_info[i].path = Path()
            self.drone_info[i].path.path_len = len(dataMap[i]['drone_wp'])
            for wp in dataMap[i]['drone_wp']:
                self.drone_info[i].path.path.append(wp)

    def publish_goal_counter(self):
        for i in range(len(self.drone_info)):
            if self.drone_info[i].goal_pose is not None:
                self.goal_counter_publisher[i].publish(self.drone_info[i].goal_counter)

    def publish_goal_poses(self):
        for i in range(len(self.drone_info)):
            if self.drone_info[i].goal_pose is not None:
                self.drone_info[i].goal_pose.header.stamp = rospy.Time.now()
                idx = self.drone_info[i].path.current_index
                self.drone_info[i].goal_pose.pose.position.x = self.drone_info[i].path.path[idx][0]
                self.drone_info[i].goal_pose.pose.position.y = self.drone_info[i].path.path[idx][1]
                self.goal_publishers[i].publish(self.drone_info[i].goal_pose)

    def pose_outside_border(self, buff=0):
        assert isinstance(self.last_pose, PoseStamped)
        x = self.last_pose.pose.position.x
        y = self.last_pose.pose.position.y
        return x < self.border_x_min + buff or x > self.border_x_max - buff or y < self.border_y_min + buff or y > self.border_y_max - buff

    def generate_new_pose(self, drone_id):
        self.drone_info[drone_id].goal_pose = PoseStamped()
        self.drone_info[drone_id].goal_pose.header.frame_id = "/world"
        self.drone_info[drone_id].goal_pose.header.stamp = rospy.Time.now()
        self.drone_info[drone_id].goal_pose.pose.position.z = self.drone_info[drone_id].drone_pose.pose.position.z


    def pose_cb(self, pose, drone_id):
        self.drone_info[drone_id].drone_pose = pose
        # if self.drone_info[drone_id].goal_pose is not None and self.is_running:
        #     if distance_pose(self.drone_info[drone_id].drone_pose, self.drone_info[drone_id].goal_pose) < self.goal_distance:
        #         self.drone_info[drone_id].goal_counter += 1
        #         self.generate_new_pose(drone_id)

    def toggle_coordinator(self, req):
        self.is_running = not self.is_running
        if self.is_running:
            for i in range(len(self.drone_info)):
                self.generate_new_pose(i)
                self.drone_info[i].goal_counter = 0
                self.drone_info[i].path.current_index = 0
            for i in range(len(self.drone_info)):
                self.start_nn[i].call()
        else:
            for i in range(len(self.drone_info)):
                self.stop_nn[i].call()
        return EmptyResponse()

    def next_goal(self, req):
        if self.is_running:
            for i in range(len(self.drone_info)):
                self.drone_info[i].goal_counter += 1
                self.drone_info[i].path.current_index += 1
                self.drone_info[i].path.current_index %= self.drone_info[i].path.path_len

        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node("WaypointCoordinator")
    rw = WaypointCoordinator()