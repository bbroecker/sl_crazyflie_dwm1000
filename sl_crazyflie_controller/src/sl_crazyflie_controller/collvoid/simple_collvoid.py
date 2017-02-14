#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, Obstacle
from tf import transformations

from collvoid_interface import CollvoidInterface

TIME_OUT = 0.3


class ObstacleTime:
    def __init__(self, obs, time):
        self.time = time
        self.obs = obs


def xy_distance(pose, obs2):
    x = pose.pose.position.x - obs2.x
    y = pose.pose.position.y - obs2.y
    return math.sqrt(x ** 2 + y ** 2)


def vel_magnitude(vel):
    assert isinstance(vel, Velocity)
    return math.sqrt(vel.x ** 2 + vel.y ** 2)


def get_yaw_from_msg(msg):
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = transformations.euler_from_quaternion(q)
    return euler[2]

def rotate_vector_by_angle(vector_x, vector_y, angle):
    x = vector_x * math.cos(angle) - vector_y * math.sin(angle)
    y = vector_x * math.sin(angle) + vector_y * math.cos(angle)

    return x, y


class SimpleCollvoid(CollvoidInterface):
    def __init__(self):
        CollvoidInterface.__init__(self)
        self.my_pose_id = rospy.get_param("~obstacle_manager_id")
        rospy.loginfo("my pose id %d" % self.my_pose_id)
        self.min_xy_dist = rospy.get_param("~collvoid/simple_collvoid/min_xy_dist")
        self.max_xy_dist = rospy.get_param("~collvoid/simple_collvoid/max_xy_dist")
        self.max_xy_avoid_vel = rospy.get_param("~collvoid/simple_collvoid/max_xy_avoid_vel")
        self.max_xy_goal_vel = rospy.get_param("~collvoid/simple_collvoid/max_xy_goal_vel")
        self.p_factor = rospy.get_param("~collvoid/simple_collvoid/error_factor")
        self.sub = rospy.Subscriber("/obstacle_poses", Obstacle, self.obs_pose_callback)
        self.current_obstacles = []
        self.last_pose_dict = {}

    def obs_pose_callback(self, obs):
        if obs.id is not self.my_pose_id:
            assert isinstance(obs, Obstacle)
            ot = ObstacleTime(obs, rospy.Time.now())
            self.last_pose_dict[obs.id] = ot

    def is_active(self):
        return len(self.current_obstacles) > 0

    def update_cf_pose(self, pose):
        self.last_pose = pose
        self.current_obstacles = []
        for key, value in self.last_pose_dict.iteritems():
            assert isinstance(value, ObstacleTime)
            if (rospy.Time.now() - value.time).to_sec() < TIME_OUT and xy_distance(self.last_pose, value.obs) < self.max_xy_dist:
                self.current_obstacles.append(value.obs)

    def calculate_velocity(self, current_target_velocity):
        assert isinstance(current_target_velocity, Velocity)

        avoid_vel = Velocity()
        for ob in self.current_obstacles:
            vel = self.obstacle_vel(self.last_pose, ob)
            vel = self.invert_vel(vel)
            avoid_vel.x += vel.x
            avoid_vel.y += vel.y
        mag = vel_magnitude(avoid_vel)
        if mag > self.max_xy_avoid_vel:
            avoid_vel.x = avoid_vel.x / mag * self.max_xy_avoid_vel
            avoid_vel.y = avoid_vel.y / mag * self.max_xy_avoid_vel
        current_vel = current_target_velocity
        mag = vel_magnitude(current_vel)
        if mag > self.max_xy_goal_vel:
            current_vel.x = current_vel.x / mag * self.max_xy_goal_vel
            current_vel.y = current_vel.y / mag * self.max_xy_goal_vel

        current_vel.x += avoid_vel.x
        current_vel.y += avoid_vel.y
        return current_vel

    def obstacle_vel(self, cf_pose, obs):
        assert isinstance(cf_pose, PoseStamped)
        assert isinstance(obs, Obstacle)
        current_yaw = get_yaw_from_msg(cf_pose)

        rotation_angle = -current_yaw
        x = cf_pose.pose.position.x - obs.x
        y = cf_pose.pose.position.y - obs.y
        rotated_obs_x, rotated_obs_y = rotate_vector_by_angle(x, y, rotation_angle)

        vel = Velocity()
        vel.x = rotated_obs_x
        vel.y = rotated_obs_y
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




