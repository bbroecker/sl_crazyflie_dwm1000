#!/usr/bin/env python
import copy
import math

import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sl_crazyflie_controller.cfg import pid_cfgConfig
from sl_crazyflie_msgs.msg import TargetMsg, ControlMode, Velocity
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from tf import transformations

from sl_crazyflie_controller.pid_controller.pid import PidController, yaw_of_pose, rotate_vector_by_angle, limit_angle

# constants
# update 30Hz
RATE = 200
MAX_THRUST = 65000.0
MAX_YAW_DIFF = math.pi / 8
TIME_THRESHOLD = 0.1
WAND_DISTANCE = 1.0
# TARGET_FRAME_ID = 'Robot_2/base_link'
# CRAZY_FLIE_FRAME_ID = 'Robot_1/base_link'
WORLD_FRAME_ID = '/world'
AVG_SPEED_SAMPLE_SIZE = RATE/50
TARGET_SPEED_SAMPLE_SIZE = 1


def thrust_to_percent(thrust):
    return thrust / MAX_THRUST


def percent_to_thrust(percent):
    return percent * MAX_THRUST



class VelocityController(object):
    def __init__(self):


        cf_pose_topic = rospy.get_param("~cf_pose_topic")

        self.pub_cmd_vel = rospy.Publisher("hover/cmd_vel", Twist, queue_size=10)
        self.pub_target_height = rospy.Publisher("hover/target_height", Float32, queue_size=10)
        self.pub_target_climb_rate = rospy.Publisher("hover/target_climb_rate", Float32, queue_size=10)
        self.pub_current_climb_rate = rospy.Publisher("hover/current_climb_rate", Float32, queue_size=10)
        self.pub_current_x_speed = rospy.Publisher("hover/current_x_speed", Float32, queue_size=10)
        self.pub_target_x_speed = rospy.Publisher("hover/target_x_speed", Float32, queue_size=10)
        self.pub_current_y_speed = rospy.Publisher("hover/current_y_speed", Float32, queue_size=10)
        self.pub_target_y_speed = rospy.Publisher("hover/target_y_speed", Float32, queue_size=10)
        self.pub_thrust_percentage = rospy.Publisher("hover/thrust_percentage", Float32, queue_size=1)

        self.pub_wand_target = rospy.Publisher("hover/wand_target", PoseStamped, queue_size=10)
        self.prev_x_vel = 0.0
        self.max_vel = 0.0
        self.max_accel = 0.0
        # std parameters
        self.running = False
        self.last_update_time = rospy.get_time()
        self.last_pose_msg = None

        self.min_thrust = 0.25
        self.target_altitude = 0

        self.prev_altitude = None
        self.prev_position = None

        self.avg_global_x_speed = []
        self.avg_local_x_speed = []
        self.avg_global_y_speed = []
        self.avg_local_y_speed = []

        self.avg_climb_rate = []

        self.target_speed_x_avg_list = []
        self.target_speed_y_avg_list = []
        self.target_climb_rate_list = []

        self.take_off_height = 1



        self.is_running = False


        # pid xy

        self.kp_x_vel = rospy.get_param("~pid_velocity/x/kp")
        self.ki_x_vel = rospy.get_param("~pid_velocity/x/ki")
        self.kd_x_vel = rospy.get_param("~pid_velocity/x/kd")
        self.max_x_vel_d_filter = rospy.get_param("~pid_velocity/x/d_filter_count")

        self.kp_y_vel = rospy.get_param("~pid_velocity/y/kp")
        self.ki_y_vel = rospy.get_param("~pid_velocity/y/ki")
        self.kd_y_vel = rospy.get_param("~pid_velocity/y/kd")
        self.max_y_vel_d_filter = rospy.get_param("~pid_velocity/y/d_filter_count")

        self.kp_z_vel = rospy.get_param("~pid_velocity/z/kp")
        self.ki_z_vel = rospy.get_param("~pid_velocity/z/ki")
        self.kd_z_vel = rospy.get_param("~pid_velocity/z/kd")
        self.max_z_vel_d_filter = rospy.get_param("~pid_velocity/z/d_filter_count")
        self.max_thrust = rospy.get_param("~pid_velocity/z/max_output")

        self.kp_yaw_vel = rospy.get_param("~pid_velocity/yaw/kp")
        self.ki_yaw_vel = rospy.get_param("~pid_velocity/yaw/ki")
        self.kd_yaw_vel = rospy.get_param("~pid_velocity/yaw/kd")
        self.max_yaw_vel_d_filter = rospy.get_param("~pid_velocity/yaw/d_filter_count")
        self.max_yaw_cmd = rospy.get_param("~pid_velocity/yaw/max_output")

        self.nominal_thrust = rospy.get_param("~pid_velocity/Nom_thrust")

        self.pitch_roll_cap = max(rospy.get_param("~pid_velocity/x/max_output"), rospy.get_param("~pid_velocity/y/max_output"))

        self.load_param = True

        self.pid_xy_pitch = PidController(self.kp_x_vel, self.ki_x_vel, self.kd_x_vel, -7, 7, self.max_x_vel_d_filter)
        self.pid_xy_roll = PidController(self.kp_y_vel, self.ki_y_vel, self.kd_y_vel, -7, 7, self.max_y_vel_d_filter)
        self.pid_thrust = PidController(self.kp_z_vel, self.ki_z_vel, self.kd_z_vel, 0, 0.75, self.max_z_vel_d_filter)

        self.pid_yaw_angle_speed = PidController(self.kp_yaw_vel, self.ki_yaw_vel, self.kd_yaw_vel, -1, 1, self.max_yaw_vel_d_filter)

        self.last_yaw = None

        # self.pid_yaw = PidController(KP_yaw, KI_yaw, KD_yaw, Ilimit_yaw)
        # self.target_pose_yaw = 0

        # self.dyn_server = Server(pid_cfgConfig, self.callback_dynreconf)

        # x, y or z are increase about the step size, the String has to contain x,y, or z

        rospy.loginfo("Started hover pid controller")




    def calc_update_avg_speeds(self, local_x, local_y, global_x, global_y):
        if len(self.avg_global_x_speed) >= AVG_SPEED_SAMPLE_SIZE:
            self.avg_global_x_speed.pop(0)
            self.avg_global_y_speed.pop(0)
            self.avg_local_x_speed.pop(0)
            self.avg_local_y_speed.pop(0)
        self.avg_global_x_speed.append(global_x)
        self.avg_global_y_speed.append(global_y)
        self.avg_local_x_speed.append(local_x)
        self.avg_local_y_speed.append(local_y)
        return float(sum(self.avg_local_x_speed)) / len(self.avg_local_x_speed), \
               float(sum(self.avg_local_y_speed)) / len(self.avg_local_y_speed), \
               float(sum(self.avg_global_x_speed)) / len(self.avg_global_x_speed), \
               float(sum(self.avg_global_y_speed)) / len(self.avg_global_y_speed)

    def update_cmd_twist(self, current_pose_msg, target_vel, dt):
        assert isinstance(target_vel, Velocity)

        current_time = rospy.get_time()

        self.last_update_time = current_time

        current_altitude = current_pose_msg.pose.position.z
        current_thrust_cmd = self.update_thrust(current_altitude, target_vel, dt)
        current_position = np.array([current_pose_msg.pose.position.x, current_pose_msg.pose.position.y])

        current_yaw = yaw_of_pose(current_pose_msg)
        yaw_cmd = self.update_yaw(current_yaw, target_vel, dt)
        x = 0
        y = 0

        # try to avoid gitter
        diff_angle = limit_angle(current_yaw - self.last_yaw)
        # if diff_angle > math.pi / 2:
        #    diff_angle -= math.pi
        #    rospy.logwarn("diff yaw properly flipped")
        # elif diff_angle < math.pi / 2:
        #    diff_angle += math.pi
        #    rospy.logwarn("diff yaw properly flipped")

        if abs(diff_angle) < MAX_YAW_DIFF:
            # target_vector_yaw = math.atan2(target_vector[1], target_vector[0])
            rotation_angle = -current_yaw

            global_speed_x = (current_position[0] - self.prev_position[0]) / dt
            global_speed_y = (current_position[1] - self.prev_position[1]) / dt
            local_speed_x, local_speed_y = rotate_vector_by_angle(global_speed_x, global_speed_y, rotation_angle)

            local_speed_x, local_speed_y, global_speed_x, global_speed_y = self.calc_update_avg_speeds(
                local_speed_x, local_speed_y, global_speed_x, global_speed_y)
            #vel and access check
            # u = False
            # if abs(local_speed_x) > self.max_vel:
            #     self.max_vel = abs(local_speed_x)
            #     u = True
            # if abs((local_speed_x - self.prev_x_vel)/dt) > self.max_accel:
            #     self.max_accel = abs((local_speed_x - self.prev_x_vel)/dt)
            #     u = True
            # print "speed: {0} accel {1}".format(abs(local_speed_x),abs((local_speed_x - self.prev_x_vel)/dt))
            # self.prev_x_vel = local_speed_x
                # print "target x {0} target y {1}".format(target_speed_x,target_speed_y)
            x = self.calc_xy_vel(target_vel.x, local_speed_x, dt, self.pid_xy_pitch,
                               self.target_speed_x_avg_list, [self.pub_target_x_speed, self.pub_current_x_speed])
            y = self.calc_xy_vel(target_vel.y, local_speed_y, dt, self.pid_xy_roll,
                               self.target_speed_y_avg_list, [self.pub_target_y_speed, self.pub_current_y_speed])

            # rospy.logdebug("pitch %f, roll %f, current yaw %f", x, -y, current_yaw)
            # rospy.logdebug("local_speed_y %f", local_speed_y)
            # rospy.loginfo("target vector %s \n roated target= %f, %f", str(target_vector), rotated_target_x, rotated_target_y)
        else:
            rospy.logwarn('yaw flipped %f', abs(diff_angle))

        #normalize and only allow combined max angle
        norm = math.sqrt(x**2 + y**2)
        if norm > self.pitch_roll_cap:
            x = x/norm * self.pitch_roll_cap
            y = y/norm * self.pitch_roll_cap

        self.last_yaw = current_yaw
        self.prev_altitude = current_altitude
        self.prev_position = current_position

        cmd_twist = Twist()
        cmd_twist.linear.z = percent_to_thrust(current_thrust_cmd)
        # TODO: check if x and y are correct!!
        cmd_twist.linear.x = x
        cmd_twist.linear.y = -y
        cmd_twist.angular.z = -yaw_cmd
        # sd_twist.angular.z = yaw
        # self.nominal_thrust = cmd_twist.linear.z

        # publish velocity
        return cmd_twist



    def calc_xy_vel(self, target_speed, current_speed, dt, pid_pitch_roll, target_speed_list, publisher=None):

        target_speed_list.append(target_speed)
        # avg target speed for ramping
        if len(target_speed_list) > TARGET_SPEED_SAMPLE_SIZE:
            target_speed_list.pop(0)
        target_speed = float(sum(target_speed_list)) / len(target_speed_list)

        speed_error = target_speed - current_speed

        if (publisher is not None):
            # targetspeed
            publisher[0].publish(target_speed)
            # current speed
            publisher[1].publish(current_speed)

        roll_pitch_cmd = pid_pitch_roll.update(speed_error, dt)
        rospy.logdebug('current I %f ki %f speed error %f', pid_pitch_roll.i_term, pid_pitch_roll.ki, speed_error)

        return roll_pitch_cmd

    def update_yaw(self, current_yaw, target_speed, dt):
        assert isinstance(target_speed, Velocity)
        target_angle_speed = target_speed.yaw

        current_angle_speed = limit_angle(current_yaw - self.last_yaw) / dt

        angle_speed_error = target_angle_speed - current_angle_speed

        # climb_rate to thrust pid

        yaw_cmd = self.pid_yaw_angle_speed.update(angle_speed_error, dt)

        # rospy.loginfo("target climb rate : %s thrust %s", str(target_climb_rate), str(thrust))

        if yaw_cmd < -self.max_yaw_cmd:
            yaw_cmd = -self.max_yaw_cmd
        elif yaw_cmd > self.max_yaw_cmd:
            yaw_cmd = self.max_yaw_cmd

        # publish debug msgs

        return yaw_cmd



    def update_thrust(self, current_altitude, target_speed, dt):
        assert isinstance(target_speed, Velocity)

        target_climb_rate = target_speed.z

        self.target_climb_rate_list.append(target_climb_rate)
        if len(self.target_climb_rate_list) > TARGET_SPEED_SAMPLE_SIZE:
            self.target_climb_rate_list.pop(0)
        target_climb_rate = float(sum(self.target_climb_rate_list)) / len(self.target_climb_rate_list)

        current_climb_rate = (current_altitude - self.prev_altitude) / dt


        self.avg_climb_rate.append(current_climb_rate)
        if len(self.avg_climb_rate) > AVG_SPEED_SAMPLE_SIZE:
            self.avg_climb_rate.pop(0)
        current_climb_rate = float(sum(self.avg_climb_rate)) / len(self.avg_climb_rate)

        climb_rate_error = target_climb_rate - current_climb_rate

        # climb_rate to thrust pid
        thrust = self.pid_thrust.update(climb_rate_error, dt)

        # rospy.loginfo("target climb rate : %s thrust %s", str(target_climb_rate), str(thrust))
        current_thrust_cmd = self.nominal_thrust + thrust

        if current_thrust_cmd < self.min_thrust:
            current_thrust_cmd = self.min_thrust
        elif current_thrust_cmd > self.max_thrust:
            current_thrust_cmd = self.max_thrust

            # publish debug msgs
        self.pub_target_height.publish(Float32(self.target_altitude))
        self.pub_current_climb_rate.publish(Float32(current_climb_rate))
        self.pub_target_climb_rate.publish(Float32(target_climb_rate))
        self.pub_thrust_percentage.publish(Float32(current_thrust_cmd))

        return current_thrust_cmd



    def reset_averager(self):
        self.avg_global_x_speed = []
        self.avg_local_x_speed = []
        self.avg_global_x_speed = []
        self.avg_local_y_speed = []
        self.avg_climb_rate = []
        self.target_speed_x_avg_list = []
        self.target_speed_y_avg_list = []
        self.target_climb_rate_list = []

    def reset(self, current_pose):
        # self.paused = not self.paused
        self.last_update_time = rospy.get_time()
        self.prev_altitude = current_pose.pose.position.z
        self.prev_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y])
        self.target_altitude = self.prev_altitude
        self.target_2d_pose = np.array([current_pose.pose.position.x, current_pose.pose.position.y])
        self.last_yaw = yaw_of_pose(current_pose)
        self.pid_xy_pitch.reset_pid()
        self.pid_xy_roll.reset_pid()
        self.pid_thrust.reset_pid()
        self.reset_averager()





    def callback_dynreconf(self, config, level):

        # rospy.loginfo("kp_thrust callback %s", kp_thrust)
        # rospy.loginfo("kd_thrust callback %s", kd_thrust)
        if self.load_param:
            self.load_param = False
            self.update_dyn_conf(config)
        else:
            self.update_pid(config)
        return config


    def update_dyn_conf(self, config):
        config['KP_x_vel'] = self.kp_x_vel
        config['KI_x_vel'] = self.ki_x_vel
        config['KD_x_vel'] = self.kd_x_vel
        config['max_x_vel_d_filter'] = self.max_x_vel_d_filter

        config['KP_y_vel'] = self.kp_y_vel
        config['KI_y_vel'] = self.ki_y_vel
        config['KD_y_vel'] = self.kd_y_vel
        config['max_y_vel_d_filter'] = self.max_y_vel_d_filter

        config["KP_z_vel"] = self.kp_z_vel
        config["KI_z_vel"] = self.ki_z_vel
        config["KD_z_vel"] = self.kd_z_vel
        config['max_z_vel_d_filter'] = self.max_z_vel_d_filter
        config['max_thrust'] = self.max_thrust

        config['KP_yaw_vel'] = self.kp_yaw_vel
        config['KI_yaw_vel'] = self.ki_yaw_vel
        config['KD_yaw_vel'] = self.kd_yaw_vel
        config['max_yaw_vel_d_filter'] = self.max_yaw_vel_d_filter
        config['max_yaw_vel_output'] = self.max_yaw_cmd

        config["Nom_thrust"] = self.nominal_thrust
        config['max_xy_vel_output'] = self.pitch_roll_cap



    def update_pid(self, config):
        self.kp_x_vel = config['KP_x_vel']
        self.ki_x_vel = config['KI_x_vel']
        self.kd_x_vel = config['KD_x_vel']
        self.max_x_vel_d_filter = config['max_x_vel_d_filter']

        self.kp_y_vel = config['KP_y_vel']
        self.ki_y_vel = config['KI_y_vel']
        self.kd_y_vel = config['KD_y_vel']
        self.max_y_vel_d_filter = config['max_y_vel_d_filter']

        self.kp_z_vel = config["KP_z_vel"]
        self.ki_z_vel = config["KI_z_vel"]
        self.kd_z_vel = config["KD_z_vel"]
        self.max_z_vel_d_filter = config['max_z_vel_d_filter']
        self.max_thrust = config['max_thrust']

        self.kp_yaw_vel = config['KP_yaw_vel']
        self.ki_yaw_vel = config['KI_yaw_vel']
        self.kd_yaw_vel = config['KD_yaw_vel']
        self.max_yaw_vel_d_filter = config['max_yaw_vel_d_filter']
        self.max_yaw_cmd = config['max_yaw_vel_output']


        self.nominal_thrust = config["Nom_thrust"]

        self.pitch_roll_cap = config['max_xy_vel_output']

        self.pid_thrust.set_pid_parameters(self.kp_z_vel, self.ki_z_vel, self.kd_z_vel, self.max_z_vel_d_filter)

        self.pid_xy_pitch.set_pid_parameters(self.kp_x_vel, self.ki_x_vel, self.kd_x_vel, self.max_x_vel_d_filter)
        self.pid_xy_roll.set_pid_parameters(self.kp_y_vel, self.ki_y_vel, self.kd_y_vel, self.max_y_vel_d_filter)

        self.pid_yaw_angle_speed.set_pid_parameters(self.kp_yaw_vel, self.ki_yaw_vel, self.kd_yaw_vel, self.max_yaw_vel_d_filter)


