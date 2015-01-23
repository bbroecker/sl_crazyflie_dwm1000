#!/usr/bin/env python
import math
import rospy
from tf import transformations
from pid import PidController
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from sl_crazyflie.cfg import pid_cfgConfig
import numpy as np
import copy

# constants
# update 30Hz
RATE = 50
MAX_THRUST = 65000.0
MAX_YAW_DIFF = math.pi / 8


def thrust_to_percent(thrust):
    return thrust / MAX_THRUST


def percent_to_thrust(percent):
    return percent * MAX_THRUST


def get_yaw_from_msg(msg):
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = transformations.euler_from_quaternion(q)
    return euler[2]


class HoverController(object):
    def __init__(self):
        rospy.init_node('pid_hover')

        self.pub_cmd_vel = rospy.Publisher("hover/cmd_vel", Twist, queue_size=10)
        self.pub_target_height = rospy.Publisher("hover/target_height", Float32, queue_size=10)
        self.pub_target_climb_rate = rospy.Publisher("hover/target_climb_rate", Float32, queue_size=10)
        self.pub_current_climb_rate = rospy.Publisher("hover/current_climb_rate", Float32, queue_size=10)
        self.pub_thrust_percentage = rospy.Publisher("hover/thrust_percentage", Float32, queue_size=1)

        #std parameters
        self.paused = True
        self.last_update_time = rospy.get_time()
        self.last_pose_msg = None

        #pid altitude
        kp_thrust = 0.6
        ki_thrust = 1.2
        kd_thrust = 0

        kp_climb_rate = 0.5
        ki_climb_rate = 0
        kd_climb_rate = 0

        self.nominal_thrust = 0.7
        self.max_thrust = 0.9
        self.min_thrust = 0.25
        self.target_altitude = 0
        self.max_altitude_error = 0.5
        self.prev_altitude = None

        self.pid_thrust = PidController(kp_thrust, ki_thrust, kd_thrust)
        self.pid_climb_rate = PidController(kp_climb_rate, ki_climb_rate, kd_climb_rate)

        #pid xy
        kp_xy = 2.
        ki_xy = 4.
        kd_xy = 0

        self.target_2d_pose = None
        self.max_xy_error = 0.5

        self.pitch_roll_cap = 15.0
        self.pid_xy_pitch = PidController(kp_xy, ki_xy, kd_xy)
        self.pid_xy_roll = PidController(kp_xy, ki_xy, kd_xy)

        self.last_yaw = None

        # self.pid_yaw = PidController(KP_yaw, KI_yaw, KD_yaw, Ilimit_yaw)
        #self.target_pose_yaw = 0

        self.dyn_server = Server(pid_cfgConfig, self.callback_dynreconf)

        rospy.Service('hover/stop', Empty, self.callback_stop)
        rospy.Service('hover/start_hold_position', Empty, self.callback_hold_position)
        # toDo make service call for target pose

        rospy.Subscriber('/Robot_1/pose', PoseStamped, self.callback_optitrack_pose)

        rospy.loginfo("Started hover pid controller")

    def callback_optitrack_pose(self, data):
        self.last_pose_msg = data

    def spin(self):
        rospy.loginfo("Hover controller spinning")
        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.update(copy.copy(self.last_pose_msg))
            r.sleep()

    def update(self, current_pose_msg):
        if not self.paused:
            current_time = rospy.get_time()
            dt = float(current_time - self.last_update_time)
            self.last_update_time = current_time

            current_altitude = current_pose_msg.pose.position.z
            current_thrust_cmd = self.update_thrust(current_altitude, dt)

            #toDo yaw
            current_yaw = get_yaw_from_msg(current_pose_msg)
            x = 0
            y = 0

            if abs(current_yaw - self.last_yaw) < MAX_YAW_DIFF:
                current_position = np.array([current_pose_msg.pose.position.x, current_pose_msg.pose.position.y])
                target_vector = self.target_2d_pose - current_position
                #target_vector_yaw = math.atan2(target_vector[1], target_vector[0])
                diff_yaw = -current_yaw
                rotated_target_x = target_vector[0] * math.cos(diff_yaw) - target_vector[1] * math.sin(diff_yaw)
                rotated_target_y = target_vector[0] * math.sin(diff_yaw) + target_vector[1] * math.cos(diff_yaw)
                x = self.update_xy(rotated_target_x, dt, self.pid_xy_pitch)
                y = self.update_xy(rotated_target_y, dt, self.pid_xy_roll)

                rospy.loginfo("pitch %f, yaw %f, current yaw %f", x, y, current_yaw)
                rospy.loginfo("target vector %s roated target= %f, %f", str(target_vector), rotated_target_x, rotated_target_y)
            else:
                rospy.logwarn('yaw flipped %f', abs(current_yaw - self.last_yaw))

            cmd_twist = Twist()
            cmd_twist.linear.z = percent_to_thrust(current_thrust_cmd)
            #TODO: check if x and y are correct!!
            cmd_twist.linear.x = x
            cmd_twist.linear.y = y
            #sd_twist.angular.z = yaw
            #self.nominal_thrust = cmd_twist.linear.z
            self.last_yaw = current_yaw

            self.prev_altitude = current_altitude
            # publish velocity
            self.pub_cmd_vel.publish(cmd_twist)

    def update_xy(self, error, dt, pid):
        current_error = error
        if current_error > self.max_xy_error:
            current_error = self.max_xy_error
        elif current_error < - self.max_xy_error:
            current_error = -self.max_xy_error

        roll_pitch_cmd = pid.update(current_error, dt)

        if roll_pitch_cmd > self.pitch_roll_cap:
            roll_pitch_cmd = self.pitch_roll_cap
        elif roll_pitch_cmd < - self.pitch_roll_cap:
            roll_pitch_cmd = -self.pitch_roll_cap

        return roll_pitch_cmd

    def update_thrust(self, current_altitude, dt):
        #ALTITUDE TO CLIMB_RATE PID
        current_target_altitude_error = self.target_altitude - current_altitude

        if current_target_altitude_error > self.max_altitude_error:
            current_target_altitude_error = self.max_altitude_error

        if current_target_altitude_error < -self.max_altitude_error:
            current_target_altitude_error = -self.max_altitude_error

        target_climb_rate = self.pid_climb_rate.update(current_target_altitude_error, dt)
        current_climb_rate = (current_altitude - self.prev_altitude) / dt
        climb_rate_error = target_climb_rate - current_climb_rate

        #climb_rate to thrust pid
        thrust = self.pid_thrust.update(climb_rate_error, dt)
        current_thrust_cmd = self.nominal_thrust + thrust

        if current_thrust_cmd < self.min_thrust:
            current_thrust_cmd = self.min_thrust
        elif current_thrust_cmd > self.max_thrust:
            current_thrust_cmd = self.max_thrust

            #publish debug msgs
        self.pub_target_height.publish(Float32(self.target_altitude))
        self.pub_current_climb_rate.publish(Float32(current_climb_rate))
        self.pub_target_climb_rate.publish(Float32(target_climb_rate))
        self.pub_thrust_percentage.publish(Float32(current_thrust_cmd))

        return current_thrust_cmd

    def callback_stop(self, req):
        self.paused = True
        return EmptyResponse()

    def callback_hold_position(self, req):
        # self.paused = not self.paused
        self.last_update_time = rospy.get_time()
        self.prev_altitude = self.last_pose_msg.pose.position.z
        self.target_altitude = self.prev_altitude
        self.target_2d_pose = np.array([self.last_pose_msg.pose.position.x, self.last_pose_msg.pose.position.y])
        self.last_yaw = get_yaw_from_msg(self.last_pose_msg)
        self.pid_thrust.reset_pid()
        self.paused = False

        rospy.loginfo("Holding position")

        return EmptyResponse()

    def callback_dynreconf(self, config, level):
        kp_thrust = config["KP_thrust"]
        ki_thrust = config["KI_thrust"]
        kd_thrust = config["KD_thrust"]
        kp_climb_rate = config['KP_climb_rate']
        ki_climb_rate = config['KI_climb_rate']
        kd_climb_rate = config['KD_climb_rate']

        kp_xy = config['KP_xy']
        ki_xy = config['KI_xy']
        kd_xy = config['KD_xy']

        self.max_altitude_error = config['max_altitude_error']
        self.nominal_thrust = config["Nom_thrust"]

        self.pid_thrust.set_pid_parameters(kp_thrust, ki_thrust, kd_thrust)
        self.pid_climb_rate.set_pid_parameters(kp_climb_rate, ki_climb_rate, kd_climb_rate)

        self.pid_xy_pitch.set_pid_parameters(kp_xy, ki_xy, kd_xy)
        self.pid_xy_roll.set_pid_parameters(kp_xy, ki_xy, kd_xy)

        rospy.loginfo("kp %f ki %f kd %f", kp_xy, ki_xy, kd_xy)
        return config


if __name__ == '__main__':
    try:
        cont = HoverController()
        cont.spin()
    except rospy.ROSInterruptException:
        pass
