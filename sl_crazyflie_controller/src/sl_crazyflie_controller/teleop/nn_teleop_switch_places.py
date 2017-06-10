#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from sl_crazyflie_srvs.srv import StartGoalWalk, StartGoalWalkRequest
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, Obstacle
from std_srvs.srv import Empty
import copy
import numpy as np

NN = 13
NEW_GOAL_BUTTON = 11
MIN_X = -1.0
MAX_X = 1.0
MIN_Y = -1.0
MAX_Y = 1.0


def map_value(value, in_min, in_max, out_min, out_max):
    return ((value - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min


class Teleop:
    def __init__(self):
        joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.pid_active_button = rospy.get_param("~pid_activate_axis", 11)
        self.switch_with_id = rospy.get_param("~switch_with_id")
        self.start_follow = rospy.ServiceProxy("start_nn_controller", Empty)
        self.stop_follow = rospy.ServiceProxy("stop_nn_controller", Empty)
        goal_publisher = rospy.Publisher("goal_pose", PoseStamped)

        self.prev_pressed = {'nn': False, 'next_goal':False}
        self.nn_active = False
        self.goal_pose = None
        self.last_obstacle_pose = PoseStamped()
        self.last_obstacle_pose.header.frame_id = "world"
        self.sub = rospy.Subscriber("/obstacle_poses", Obstacle, self.obs_pose_callback)
        self.joy_subscriber_ = rospy.Subscriber(joy_topic, Joy, self.joy_callback)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.nn_active:
                goal_publisher.publish(self.goal_pose)
            rate.sleep()

    def obs_pose_callback(self, obs):
        if obs.id is self.switch_with_id:
            assert isinstance(obs, Obstacle)
            self.last_obstacle_pose.pose.position.x = obs.x
            self.last_obstacle_pose.pose.position.y = obs.y
            self.last_obstacle_pose.pose.position.z = obs.z


    def joy_callback(self, joy_msg):
        if self.is_button_released('nn', joy_msg.buttons[NN]):
            if self.nn_active:
                print "stop"
                self.nn_active = False
                self.goal_pose = None
                self.stop_follow.call()
            else:
                self.goal_pose = copy.deepcopy(self.last_obstacle_pose)
                self.goal_pose.header.stamp = rospy.Time.now()
                self.start_follow.call()
                self.nn_active = True
                print "next goal {0}".format(self.goal_pose)
                print "active"
        elif self.nn_active and self.is_button_released('next_goal', joy_msg.buttons[NEW_GOAL_BUTTON]):
            self.goal_pose = copy.deepcopy(self.last_obstacle_pose)
            self.goal_pose.header.stamp = rospy.Time.now()
            print "next goal {0}".format(self.goal_pose)

    def is_button_released(self, button_name, button_pressed):
        if button_pressed:
            self.prev_pressed[button_name] = True
        elif self.prev_pressed[button_name]:
            self.prev_pressed[button_name] = False
            return True
        return False




if __name__ == '__main__':
    rospy.init_node("sl_random_walk_teleop")
    Teleop()
    rospy.spin()