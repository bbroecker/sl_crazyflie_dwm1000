#!/usr/bin/env python
import copy
import math
import numpy as np
from threading import Lock

import rospy
import actionlib
import tf
from dynamic_reconfigure.server import Server
from sl_crazyflie_msgs.msg import FlightMode, TargetMsg, ControlMode, Velocity
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_pathfollowing.msg import FollowSegmentAction, FollowSegmentActionGoal, LineSegment, \
    FollowSegmentActionFeedback, FollowSegmentActionResult, FollowSegmentGoal, FollowSegmentResult, \
    FollowSegmentFeedback

from sl_crazyflie_controller.cfg import pid_cfgConfig

from pid import PidController


UPDATE_RATE = 30
LINE_SEG_BUFFER_TIME = 0.5
TARGET_THRESHOLD = 0.02
MAX_YAW_DIFF = math.pi / 8.0

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

def rotate_vector_by_angle(vector_x, vector_y, angle):
    x = vector_x * math.cos(angle) - vector_y * math.sin(angle)
    y = vector_x * math.sin(angle) + vector_y * math.cos(angle)

    return x, y


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


        kp_x_speed = 3.3
        ki_x_speed = 0.0
        kd_x_speed = 0.15

        kp_y_speed = 3.3
        ki_y_speed = 0.0
        kd_y_speed = 0.15

        # pid yaw
        p_yaw_pos = 1.0
        i_yaw_pos = 0.0
        d_yaw_pos = 0.0

        # climb speed
        kp_climb_rate = 3.0
        ki_climb_rate = 0
        kd_climb_rate = 0.6

        self.max_altitude_error = 2.0


        self.target_2d_pose = None
        self.max_xy_error = 1.5
        self.pitch_roll_cap = 20.0

        self.pid_xy_x_speed = PidController(kp_x_speed, ki_x_speed, kd_x_speed, -0.5, 0.5, 6)
        self.pid_xy_y_speed = PidController(kp_y_speed, ki_y_speed, kd_y_speed, -0.5, 0.5, 6)
        self.pid_climb_rate = PidController(kp_climb_rate, ki_climb_rate, kd_climb_rate, -0.4, 0.4, 6)

        self.max_yaw_angle_error = 1.7
        self.max_yaw_cmd = 200
        self.pid_yaw_angle = PidController(p_yaw_pos, i_yaw_pos, d_yaw_pos, -1, 1)
        self.dyn_server = Server(pid_cfgConfig, self.callback_dynreconf)


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
        m = ChangeFlightModeRequest()
        m.mode.id = FlightMode.EXTERNAL_CONTROL
        self.change_flight_mode.call(m)

    def preempted_callback(self):
        self.is_following = False

    def follow_line(self, goal):
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
        rate = rospy.Rate(200)
        last_update = rospy.Time.now()
        target_pose = copy.deepcopy(current_line.start_point)
        process_percent = 0.0

        # check flight mode
        if self.is_pos_hold():
            self.deactivate_pos_hold()

        while self.is_following and not rospy.is_shutdown() and self.action_srv.is_active() and not self.action_srv.is_new_goal_available():

            #public target msg
            if euler_distance_pose(self.last_pose, current_line.end_point) < TARGET_THRESHOLD:
                result = FollowSegmentResult()
                result.success = True
                #pos_hold
                self.action_srv.set_succeeded(result)
                break

            # check flight mode
            if self.is_pos_hold():
                self.deactivate_pos_hold()
            current_time = rospy.Time.now()
            pasted_time = (current_time - start_time).to_sec()
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
            ctrl_mode.x_mode = ctrl_mode.y_mode = ctrl_mode.z_mode = ctrl_mode.yaw_mode = ControlMode.VELOCITY
            msg.control_mode = ctrl_mode
            msg.target_velocity = self.calculate_velocity(self.last_pose, target_pose, (current_time - last_update).to_sec())
            last_update = current_time
            self.control_command_pub.publish(msg)

            rate.sleep()

        if not self.is_following:
            result = FollowSegmentResult()
            result.success = False
            # pos_hold
            self.action_srv.set_aborted(result)
        rospy.loginfo("finished line!")
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

    def calc_xy_velocity(self, pid_xy_speed, error, dt):
        current_error = error
        if current_error > self.max_xy_error:
            current_error = self.max_xy_error
        elif current_error < - self.max_xy_error:
            current_error = -self.max_xy_error
        target_speed = pid_xy_speed.update(current_error, dt)
        return target_speed

    def calc_yaw_vel(self, current_yaw, target_yaw, dt):

        # ALTITUDE TO CLIMB_RATE PID
        yaw_angle_error = limit_angle(target_yaw - current_yaw)

        # rospy.loginfo("target altitude error: %s", str(current_target_altitude_error))

        if yaw_angle_error > self.max_yaw_angle_error:
            yaw_angle_error = self.max_yaw_angle_error

        if yaw_angle_error < -self.max_yaw_angle_error:
            yaw_angle_error = -self.max_yaw_angle_error
        target_angle_speed = self.pid_yaw_angle.update(yaw_angle_error, dt)
        return target_angle_speed


    def calc_z_vel(self, target_altitude, current_altitude, dt):

        # ALTITUDE TO CLIMB_RATE PID
        current_target_altitude_error = target_altitude - current_altitude

        # rospy.loginfo("target altitude error: %s", str(current_target_altitude_error))

        if current_target_altitude_error > self.max_altitude_error:
            current_target_altitude_error = self.max_altitude_error

        if current_target_altitude_error < -self.max_altitude_error:
            current_target_altitude_error = -self.max_altitude_error

        target_climb_rate = self.pid_climb_rate.update(current_target_altitude_error, dt)
        return target_climb_rate

    def calculate_velocity(self, current_pose_msg, target_pose, dt):
        vel = Velocity()

        target_2d_pose = np.array(
            [target_pose.pose.position.x, target_pose.pose.position.y])

        current_altitude = current_pose_msg.pose.position.z
        target_altitude = target_pose.pose.position.z
        vel.z = self.calc_z_vel(target_altitude, current_altitude, dt)
        current_position = np.array([current_pose_msg.pose.position.x, current_pose_msg.pose.position.y])

        current_yaw = yaw_of_pose(current_pose_msg.pose)
        current_target_yaw = yaw_of_pose(target_pose.pose)
        vel.yaw = self.calc_yaw_vel(current_yaw, current_target_yaw, dt)



        # try to avoid gitter
        target_vector = target_2d_pose - current_position
        # target_vector_yaw = math.atan2(target_vector[1], target_vector[0])
        rotation_angle = -current_yaw
        rotated_target_x, rotated_target_y = rotate_vector_by_angle(target_vector[0], target_vector[1],
                                                                    rotation_angle)

        # print "target x {0} target y {1}".format(target_speed_x,target_speed_y)
        vel.x = self.calc_xy_velocity(self.pid_xy_x_speed, rotated_target_x, dt)
        vel.y = self.calc_xy_velocity(self.pid_xy_y_speed, rotated_target_y, dt)

        # rospy.logdebug("pitch %f, roll %f, current yaw %f", x, -y, current_yaw)
        # rospy.logdebug("local_speed_y %f", local_speed_y)
        # rospy.loginfo("target vector %s \n roated target= %f, %f", str(target_vector), rotated_target_x, rotated_target_y)
        return vel






    def callback_dynreconf(self, config, level):

        # rospy.loginfo("kp_thrust callback %s", kp_thrust)
        # rospy.loginfo("kd_thrust callback %s", kd_thrust)
        kp_climb_rate = config['KP_climb_rate']
        ki_climb_rate = config['KI_climb_rate']
        kd_climb_rate = config['KD_climb_rate']

        kp_x_speed = config['KP_x_speed']
        ki_x_speed = config['KI_x_speed']
        kd_x_speed = config['KD_x_speed']

        kp_y_speed = config['KP_y_speed']
        ki_y_speed = config['KI_y_speed']
        kd_y_speed = config['KD_y_speed']

        #pid yaw
        p_yaw_pos = config['P_yaw_pos']
        i_yaw_pos = config['I_yaw_pos']
        d_yaw_pos = config['D_yaw_pos']

        self.max_xy_error = config['max_xy_error']
        self.pitch_roll_cap = config['pitch_roll_cap']

        self.max_altitude_error = config['max_altitude_error']

        self.pid_climb_rate.set_pid_parameters(kp_climb_rate, ki_climb_rate, kd_climb_rate)

        self.pid_xy_x_speed.set_pid_parameters(kp_x_speed, ki_x_speed, kd_x_speed)
        self.pid_xy_y_speed.set_pid_parameters(kp_y_speed, ki_y_speed, kd_y_speed)

        self.pid_yaw_angle.set_pid_parameters(p_yaw_pos, i_yaw_pos, d_yaw_pos)

        rospy.loginfo("Dynreconf callback %s", str(config))
        return config


if __name__ == '__main__':
    s = SegmentFollower("SegmentFollower")
    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        r.sleep()

