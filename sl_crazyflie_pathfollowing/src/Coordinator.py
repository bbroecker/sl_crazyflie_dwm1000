#!/usr/bin/env python
import copy
import math
import rospkg
import threading
from  threading import Thread

import actionlib
import rospy
import yaml
from std_srvs.srv import Empty, EmptyResponse

import tf
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_pathfollowing.msg import FollowSegmentAction, FollowSegmentActionGoal, LineSegment, \
    FollowSegmentActionFeedback, CircleSegment, FollowSegmentFeedback, FollowSegmentGoal

from SegmentFollower import quaternon_from_yaw

PATH_FOLDER = "path_files"
place_holder = {'pose': None, 'waypoint': 0, 'sync': False}
SWITCH_GOAL_PERCENT = 0.98



class SingleDroneWorker():
    def __init__(self, name_space):
        self.cf_name_space = name_space
        self.way_points_dict = None
        self.is_running = False
        self.thread = None
        self.progress = 0.0
        print name_space + "/follow_segment_action"
        self.client = actionlib.SimpleActionClient(name_space + "/follow_segment_action", FollowSegmentAction)
        self.client.wait_for_server()

    def load_new_waypoints(self, waypoints):
        self.way_points_dict = waypoints

    def join(self):
        if self.thread is not None:
            self.thread.join()

    def start(self):
        self.thread = threading.Thread(target=self.follow_points)
        self.thread.start()

    def stop(self):
        if self.thread is not None:
            self.client.cancel_all_goals()
            self.is_running = False
            self.way_points_dict = None
            self.thread.join()
            self.thread = None

    def array_to_pose(self, a):
        pose = PoseStamped()
        if a is None:
            return pose

        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = a[0]
        pose.pose.position.y = a[1]
        pose.pose.position.z = a[2]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, a[3])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def follow_points(self):
        if self.way_points_dict is None:
            rospy.logwarn("no waypoints are loaded")
            return

        self.is_running = True
        self.way_points_dict.insert(0, place_holder)
        for i in range(len(self.way_points_dict) - 1):
            if not self.is_running:
                break
            self.progress = 0.0
            print "1"
            self.client.cancel_all_goals()
            action = FollowSegmentGoal()
            action.segment_type = LineSegment.TYPE_ID
            line = LineSegment()
            if self.way_points_dict[i]['pose'] is None:
                action.take_current_pose = True
            else:
                action.take_current_pose = False
            line.start_point = self.array_to_pose(self.way_points_dict[i]['pose'])
            line.end_point = self.array_to_pose(self.way_points_dict[i + 1]['pose'])
            action.line = line
            action.max_angle_vel = 0.1
            action.max_travel_vel = 0.3

            self.client.send_goal(action, feedback_cb=self.feedback_cb)
            self.wait_for_action()
            print "2"
        self.way_points_dict = None
        self.is_running = False

    def wait_for_action(self):
        rate = rospy.Rate(5.0)
        while self.progress < SWITCH_GOAL_PERCENT and self.is_running:
            print self.progress
            rate.sleep()

    def feedback_cb(self, feedback):
        assert isinstance(feedback, FollowSegmentFeedback)
        self.progress = feedback.percent_complete


class Coordinator:
    def __init__(self, name):
        self.name_spaces = rospy.get_param("~cf_name_spaces", [""])
        self.repeats = rospy.get_param("~repeat", 2)
        self.waypoint_files = rospy.get_param("~waypoint_files", ["cf1.yaml"])
        if len(self.name_spaces) is not len(self.waypoint_files):
            rospy.loginfo("Each cf namespace, requires a waypoint file")
            return
        self.waypoints = {}
        self.threads = {}
        for name in self.name_spaces:
            self.threads[name] = SingleDroneWorker(name)

        rospack = rospkg.RosPack()
        self.path = rospack.get_path("sl_crazyflie_pathfollowing") + '/' + PATH_FOLDER + '/'
        self.service = rospy.Service('/start_way_following', Empty, self.start_way_following)
        self.service_cancel = rospy.Service('/cancel_way_following', Empty, self.cancel_way_following)
        self.is_running = False
        self.main_thread = None



    def send_waypoints(self):
        i = 0
        while i < self.repeats and self.is_running:
            for idx, f in enumerate(self.waypoint_files):
                yaml_f = open(self.path + f)
                dataMap = yaml.safe_load(yaml_f)
                self.waypoints[self.name_spaces[idx]] = dataMap
                print dataMap
                yaml_f.close()

            while self.has_waypoints() and self.is_running:
                running_tasks = []
                for n in self.name_spaces:
                    wp = []
                    while self.is_running and len(self.waypoints[n]) > 0:
                        w = self.waypoints[n].pop(0)
                        wp.append(w)
                        if w['sync']:
                            break
                    if len(wp) > 0:
                        t = self.threads[n]
                        t.load_new_waypoints(wp)
                        print "start!!"
                        t.start()
                        running_tasks.append(t)
                for t in running_tasks:
                    print "join!!"
                    t.join()
            i += 1

    def cancel_way_following(self, cb):
        self.stop_threads()
        return EmptyResponse()


    def stop_threads(self):
        if self.is_running:
            self.is_running = False
            for key in self.threads.keys():
                self.threads[key].stop()
            self.main_thread.join()
            self.main_thread = None


    def start_way_following(self, msg):
        self.stop_threads()
        self.is_running = True
        self.main_thread = threading.Thread(target=self.send_waypoints)
        self.main_thread.start()
        return EmptyResponse()

    def has_waypoints(self):
        result = False
        for n in self.name_spaces:
            if n in self.waypoints and len(self.waypoints[n]) > 0:
                result = True
                break
        return result

        # for f in waypoint_files:
        #     with open(path + f, 'r') as stream:
        #         try:
        #             print(yaml.load(stream))
        #         except yaml.YAMLError as exc:
        #             print(exc)





if __name__ == '__main__':
    rospy.init_node("Coordinator")
    s = Coordinator(rospy.get_name())
    rospy.spin()