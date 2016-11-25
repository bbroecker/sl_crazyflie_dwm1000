#!/usr/bin/env python
import math
import rospy
from sl_crazyflie_msgs.msg import FlightMode, TargetMsg, ControlMode
from sl_crazyflie_srvs.srv import ChangeFlightModeRequest
from tf import transformations
from pid import PidController
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeTargetPose, ChangeTargetPoseResponse, StartWanding, \
    StartWandingResponse
from dynamic_reconfigure.server import Server
from sl_crazyflie_controller.cfg import pid_cfgConfig
import numpy as np
import copy
import tf

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
AVG_SPEED_SAMPLE_SIZE = 4
TARGET_SPEED_SAMPLE_SIZE = RATE/3


def thrust_to_percent(thrust):
    return thrust / MAX_THRUST


def percent_to_thrust(percent):
    return percent * MAX_THRUST


def get_yaw_from_msg(msg):
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = transformations.euler_from_quaternion(q)
    return euler[2]


def rotate_vector_by_angle(vector_x, vector_y, angle):
    x = vector_x * math.cos(angle) - vector_y * math.sin(angle)
    y = vector_x * math.sin(angle) + vector_y * math.cos(angle)

    return x, y


class HoverController(object):
    def __init__(self):
        rospy.init_node('pid_hover')

        cf_pose_topic = rospy.get_param("cf_pose_topic", "/Robot_1/pose")

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
        self.pub_target_pose = rospy.Publisher("hover/target_pose", PoseStamped, queue_size=10)
        self.pub_current_pose = rospy.Publisher("hover/current_pose", PoseStamped, queue_size=10)
        # std parameters
        self.running = False
        self.last_update_time = rospy.get_time()
        self.last_pose_msg = None

        # pid altitude
        kp_thrust = 1.4
        ki_thrust = 0
        kd_thrust = 0.01

        kp_climb_rate = 1.0
        ki_climb_rate = 0
        kd_climb_rate = 0

        self.nominal_thrust = 0.7
        self.max_thrust = 0.9
        self.min_thrust = 0.25
        self.target_altitude = 0
        self.max_altitude_error = 2.0
        self.prev_altitude = None
        self.prev_position = None

        self.avg_global_x_speed = []
        self.avg_local_x_speed = []
        self.avg_global_y_speed = []
        self.avg_local_y_speed = []

        self.target_speed_x_avg_list = []
        self.target_speed_y_avg_list = []
        self.target_climb_rate_list = []

        self.take_off_height = 1
        self.pos_3d_control_active = False

        self.pid_thrust = PidController(kp_thrust, ki_thrust, kd_thrust, 0, 0.75, True)
        self.pid_climb_rate = PidController(kp_climb_rate, ki_climb_rate, kd_climb_rate, -0.4, 0.4)

        # pid xy
        kp_x = 11.7
        ki_x = 0
        kd_x = 0.

        kp_x_speed = 2.0
        ki_x_speed = 0.4
        kd_x_speed = 0.2

        kp_y = 11.7
        ki_y = 0.
        kd_y = 0.

        kp_y_speed = 2.0
        ki_y_speed = 0.4
        kd_y_speed = 0.2

        #pid yaw
        p_yaw_pos = 0.5
        i_yaw_pos = 0.0
        d_yaw_pos = 0.0

        p_yaw_angle_speed = 50.0
        i_yaw_angle_speed = 0.0
        d_yaw_angle_speed = 0.0

        self.target_2d_pose = None
        self.max_xy_error = 1.5
        self.pitch_roll_cap = 20.0

        self.pid_xy_pitch = PidController(kp_x, ki_x, kd_x, -10,10)
        self.pid_xy_roll = PidController(kp_y, ki_y, kd_y, -10, 10)
        self.pid_xy_x_speed = PidController(kp_x_speed, ki_x_speed, kd_x_speed, -0.5, 0.5)
        self.pid_xy_y_speed = PidController(kp_y_speed, ki_y_speed, kd_y_speed, -0.5, 0.5)

        self.max_yaw_angle_error = 1.7
        self.max_yaw_cmd = 200
        self.pid_yaw_angle = PidController(p_yaw_pos, i_yaw_pos, d_yaw_pos, -1, 1)
        self.pid_yaw_angle_speed = PidController(p_yaw_angle_speed, i_yaw_angle_speed, d_yaw_angle_speed, -1, 1)

        self.last_yaw = None
        self.last_target_pose_msg = None
        self.last_target_msg = None  # TargetMsg()
        self.last_target_msg_time = None
        # self.pid_yaw = PidController(KP_yaw, KI_yaw, KD_yaw, Ilimit_yaw)
        # self.target_pose_yaw = 0

        self.dyn_server = Server(pid_cfgConfig, self.callback_dynreconf)
        rospy.Service('hover/stop_pid', Empty, self.callback_stop_pos_control)
        rospy.Service('hover/start_pid', Empty, self.callback_start_pos_control)
        # x, y or z are increase about the step size, the String has to contain x,y, or z
        rospy.Subscriber(cf_pose_topic, PoseStamped, self.callback_optitrack_pose)
        rospy.Subscriber('main_crtl/target_msg', TargetMsg, self.callback_target_msg)

        rospy.loginfo("Started hover pid controller")

    def callback_optitrack_pose(self, pose):
        self.last_pose_msg = pose

    def callback_target_msg(self, msg):
        assert isinstance(msg, TargetMsg)
        self.last_target_msg = msg
        self.last_target_msg_time = rospy.Time.now()

    def spin(self):
        rospy.loginfo("Hover controller spinning")
        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.update(copy.deepcopy(self.last_pose_msg))
            r.sleep()

    def limit_angle(self, angle):
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

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

    def update(self, current_pose_msg):
        if self.running and self.last_target_msg is not None:
            current_time = rospy.get_time()
            self.last_target_pose_msg = self.last_target_msg.target_pose
            self.target_altitude = self.last_target_pose_msg.pose.position.z
            self.target_2d_pose = np.array(
                [self.last_target_pose_msg.pose.position.x, self.last_target_pose_msg.pose.position.y])
            dt = float(current_time - self.last_update_time)
            self.last_update_time = current_time

            current_altitude = current_pose_msg.pose.position.z
            current_thrust_cmd = self.update_thrust(current_altitude, dt)
            current_position = np.array([current_pose_msg.pose.position.x, current_pose_msg.pose.position.y])

            # toDo yaw
            current_yaw = get_yaw_from_msg(current_pose_msg)
            current_target_yaw = get_yaw_from_msg(self.last_target_pose_msg)
            yaw_cmd = self.update_yaw(current_yaw, current_target_yaw, dt)
            x = 0
            y = 0

            # try to avoid gitter
            diff_angle = self.limit_angle(current_yaw - self.last_yaw)
            # if diff_angle > math.pi / 2:
            #    diff_angle -= math.pi
            #    rospy.logwarn("diff yaw properly flipped")
            # elif diff_angle < math.pi / 2:
            #    diff_angle += math.pi
            #    rospy.logwarn("diff yaw properly flipped")

            if abs(diff_angle) < MAX_YAW_DIFF:
                target_vector = self.target_2d_pose - current_position
                # target_vector_yaw = math.atan2(target_vector[1], target_vector[0])
                rotation_angle = -current_yaw
                rotated_target_x, rotated_target_y = rotate_vector_by_angle(target_vector[0], target_vector[1],
                                                                            rotation_angle)
                global_speed_x = (current_position[0] - self.prev_position[0]) / dt
                global_speed_y = (current_position[1] - self.prev_position[1]) / dt
                local_speed_x, local_speed_y = rotate_vector_by_angle(global_speed_x, global_speed_y, rotation_angle)

                target_speed_x = None
                target_speed_y = None
                local_speed_x, local_speed_y, global_speed_x, global_speed_y = self.calc_update_avg_speeds(
                    local_speed_x, local_speed_y, global_speed_x, global_speed_y)

                if self.last_target_msg.control_mode.x_mode is ControlMode.VELOCITY:
                    target_speed_x = self.last_target_msg.target_velocity.x
                if self.last_target_msg.control_mode.y_mode is ControlMode.VELOCITY:
                    target_speed_y = self.last_target_msg.target_velocity.y
                    # print "target x {0} target y {1}".format(target_speed_x,target_speed_y)
                x = self.update_xy(rotated_target_x, local_speed_x, dt, self.pid_xy_pitch, self.pid_xy_x_speed,
                                   self.target_speed_x_avg_list, [self.pub_target_x_speed, self.pub_current_x_speed],
                                   target_speed_x)
                y = self.update_xy(rotated_target_y, local_speed_y, dt, self.pid_xy_roll, self.pid_xy_y_speed,
                                   self.target_speed_y_avg_list, [self.pub_target_y_speed, self.pub_current_y_speed],
                                   target_speed_y)

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

            cmd_twist = Twist()
            cmd_twist.linear.z = percent_to_thrust(current_thrust_cmd)
            # TODO: check if x and y are correct!!
            cmd_twist.linear.x = x
            cmd_twist.linear.y = -y
            cmd_twist.angular.z = -yaw_cmd
            # sd_twist.angular.z = yaw
            # self.nominal_thrust = cmd_twist.linear.z
            self.last_yaw = current_yaw

            self.prev_altitude = current_altitude
            self.prev_position = current_position
            # publish velocity
            self.pub_cmd_vel.publish(cmd_twist)

            if self.target_2d_pose is not None:
                target_pose = PoseStamped()
                target_pose.pose.position.x = self.target_2d_pose[0]
                target_pose.pose.position.y = self.target_2d_pose[1]
                target_pose.pose.position.z = self.target_altitude
                target_pose.header.stamp = rospy.Time.now()

                current_pose = PoseStamped()
                current_pose.pose = current_pose_msg.pose
                current_pose.header.stamp = rospy.Time.now()

                self.pub_current_pose.publish(current_pose)
                self.pub_target_pose.publish(target_pose)

    def update_xy(self, error, current_speed, dt, pid_pitch_roll, pid_xy_speed, target_speed_list, publisher=None,
                  target_speed=None):
        if target_speed is None:
            current_error = error
            if current_error > self.max_xy_error:
                current_error = self.max_xy_error
            elif current_error < - self.max_xy_error:
                current_error = -self.max_xy_error
            target_speed = pid_xy_speed.update(current_error, dt)


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

    def update_yaw(self, current_yaw, target_yaw, dt):
        assert isinstance(self.last_target_msg, TargetMsg)
        if self.last_target_msg.control_mode.yaw_mode is ControlMode.POSITION:
            # ALTITUDE TO CLIMB_RATE PID
            yaw_angle_error = self.limit_angle(target_yaw - current_yaw)

            # rospy.loginfo("target altitude error: %s", str(current_target_altitude_error))

            if yaw_angle_error > self.max_yaw_angle_error:
                yaw_angle_error = self.max_yaw_angle_error

            if yaw_angle_error < -self.max_yaw_angle_error:
                yaw_angle_error = -self.max_yaw_angle_error
            target_angle_speed = self.pid_yaw_angle.update(yaw_angle_error, dt)
        else:
            target_angle_speed = self.last_target_msg.target_velocity.yaw

        current_angle_speed = self.limit_angle(current_yaw - self.last_yaw) / dt

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



    def update_thrust(self, current_altitude, dt):
        assert isinstance(self.last_target_msg, TargetMsg)
        if self.last_target_msg.control_mode.z_mode is ControlMode.POSITION:
            # ALTITUDE TO CLIMB_RATE PID
            current_target_altitude_error = self.target_altitude - current_altitude

            # rospy.loginfo("target altitude error: %s", str(current_target_altitude_error))

            if current_target_altitude_error > self.max_altitude_error:
                current_target_altitude_error = self.max_altitude_error

            if current_target_altitude_error < -self.max_altitude_error:
                current_target_altitude_error = -self.max_altitude_error

            target_climb_rate = self.pid_climb_rate.update(current_target_altitude_error, dt)
        else:
            target_climb_rate = self.last_target_msg.target_velocity.z

        self.target_climb_rate_list.append(target_climb_rate)
        if len(self.target_climb_rate_list) > TARGET_SPEED_SAMPLE_SIZE:
            self.target_climb_rate_list.pop(0)
        target_climb_rate = float(sum(self.target_climb_rate_list)) / len(self.target_climb_rate_list)

        current_climb_rate = (current_altitude - self.prev_altitude) / dt
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

    def callback_stop_pos_control(self, req):
        self.running = False
        self.pid_climb_rate.reset_pid()
        self.pid_xy_pitch.reset_pid()
        self.pid_xy_roll.reset_pid()
        self.pid_xy_x_speed.reset_pid()
        self.pid_xy_y_speed.reset_pid()
        self.pid_thrust.reset_pid()
        self.running = False
        self.pos_3d_control_active = False
        return EmptyResponse()

    def reset_averager(self):
        self.avg_global_x_speed = []
        self.avg_local_x_speed = []
        self.avg_global_x_speed = []
        self.avg_local_y_speed = []
        self.target_speed_x_avg_list = []
        self.target_speed_y_avg_list = []
        self.target_climb_rate_list = []

    def callback_start_pos_control(self, req):
        # self.paused = not self.paused
        self.last_update_time = rospy.get_time()
        self.prev_altitude = self.last_pose_msg.pose.position.z
        self.prev_position = np.array([self.last_pose_msg.pose.position.x, self.last_pose_msg.pose.position.y])
        self.target_altitude = self.prev_altitude
        self.target_2d_pose = np.array([self.last_pose_msg.pose.position.x, self.last_pose_msg.pose.position.y])
        self.last_yaw = get_yaw_from_msg(self.last_pose_msg)
        self.pid_climb_rate.reset_pid()
        self.pid_xy_pitch.reset_pid()
        self.pid_xy_roll.reset_pid()
        self.pid_xy_x_speed.reset_pid()
        self.pid_xy_y_speed.reset_pid()
        self.pid_thrust.reset_pid()
        self.reset_averager()
        self.running = True
        self.pos_3d_control_active = True
        rospy.loginfo("Takeoff")
        rospy.loginfo("3D position mode on")
        return EmptyResponse()

    def callback_dynreconf(self, config, level):
        kp_thrust = config["KP_thrust"]
        ki_thrust = config["KI_thrust"]
        kd_thrust = config["KD_thrust"]
        # rospy.loginfo("kp_thrust callback %s", kp_thrust)
        # rospy.loginfo("kd_thrust callback %s", kd_thrust)
        kp_climb_rate = config['KP_climb_rate']
        ki_climb_rate = config['KI_climb_rate']
        kd_climb_rate = config['KD_climb_rate']

        kp_x = config['KP_x']
        ki_x = config['KI_x']
        kd_x = config['KD_x']
        kp_x_speed = config['KP_x_speed']
        ki_x_speed = config['KI_x_speed']
        kd_x_speed = config['KD_x_speed']

        kp_y = config['KP_y']
        ki_y = config['KI_y']
        kd_y = config['KD_y']
        kp_y_speed = config['KP_y_speed']
        ki_y_speed = config['KI_y_speed']
        kd_y_speed = config['KD_y_speed']

        #pid yaw
        p_yaw_pos = config['P_yaw_pos']
        i_yaw_pos = config['I_yaw_pos']
        d_yaw_pos = config['D_yaw_pos']

        p_yaw_angle_speed = config['P_yaw_speed']
        i_yaw_angle_speed = config['I_yaw_speed']
        d_yaw_angle_speed = config['D_yaw_speed']

        self.max_altitude_error = config['max_altitude_error']
        self.nominal_thrust = config["Nom_thrust"]

        self.max_xy_error = config['max_xy_error']
        self.pitch_roll_cap = config['pitch_roll_cap']

        self.pid_thrust.set_pid_parameters(kp_thrust, ki_thrust, kd_thrust)
        self.pid_climb_rate.set_pid_parameters(kp_climb_rate, ki_climb_rate, kd_climb_rate)

        self.pid_xy_pitch.set_pid_parameters(kp_x, ki_x, kd_x)
        self.pid_xy_roll.set_pid_parameters(kp_y, ki_y, kd_y)
        self.pid_xy_x_speed.set_pid_parameters(kp_x_speed, ki_x_speed, kd_x_speed)
        self.pid_xy_y_speed.set_pid_parameters(kp_y_speed, ki_y_speed, kd_y_speed)

        self.pid_yaw_angle.set_pid_parameters(p_yaw_pos, i_yaw_pos, d_yaw_pos)
        self.pid_yaw_angle_speed.set_pid_parameters(p_yaw_angle_speed, i_yaw_angle_speed, d_yaw_angle_speed)

        rospy.loginfo("Dynreconf callback %s", str(config))
        return config


if __name__ == '__main__':
    try:
        cont = HoverController()
        cont.spin()
    except rospy.ROSInterruptException:
        pass
