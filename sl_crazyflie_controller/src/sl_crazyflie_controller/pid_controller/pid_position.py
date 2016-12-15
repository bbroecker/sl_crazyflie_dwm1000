import numpy as np

import rospy
from dynamic_reconfigure.server import Server
from sl_crazyflie_controller.cfg import pid_cfgConfig
from sl_crazyflie_msgs.msg import Velocity

from pid import PidController, yaw_of_pose, rotate_vector_by_angle, limit_angle


class PositonController:

    def __init__(self):
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



    def reset(self):
        # self.paused = not self.paused
        self.pid_climb_rate.reset_pid()

        self.pid_xy_x_speed.reset_pid()
        self.pid_xy_y_speed.reset_pid()




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

    def pose_to_yaw_vel(self, current_pose_msg, target_pose, dt):
        current_yaw = yaw_of_pose(current_pose_msg.pose)
        current_target_yaw = yaw_of_pose(target_pose.pose)
        yaw_vel = self.calc_yaw_vel(current_yaw, current_target_yaw, dt)
        return yaw_vel

    def pose_to_xy_vel(self, current_pose_msg, target_pose, dt):
        target_2d_pose = np.array(
            [target_pose.pose.position.x, target_pose.pose.position.y])
        current_position = np.array([current_pose_msg.pose.position.x, current_pose_msg.pose.position.y])
        current_yaw = yaw_of_pose(current_pose_msg.pose)

        # try to avoid gitter
        target_vector = target_2d_pose - current_position
        # target_vector_yaw = math.atan2(target_vector[1], target_vector[0])
        rotation_angle = -current_yaw
        rotated_target_x, rotated_target_y = rotate_vector_by_angle(target_vector[0], target_vector[1],
                                                                    rotation_angle)

        # print "target x {0} target y {1}".format(target_speed_x,target_speed_y)
        x = self.calc_xy_velocity(self.pid_xy_x_speed, rotated_target_x, dt)
        y = self.calc_xy_velocity(self.pid_xy_y_speed, rotated_target_y, dt)

        return x, y



    # def calculate_velocity(self, current_pose_msg, target_pose, dt):
    #     vel = Velocity()
    #
    #     target_2d_pose = np.array(
    #         [target_pose.pose.position.x, target_pose.pose.position.y])
    #
    #     current_altitude = current_pose_msg.pose.position.z
    #     target_altitude = target_pose.pose.position.z
    #     vel.z = self.calc_z_vel(target_altitude, current_altitude, dt)
    #     current_position = np.array([current_pose_msg.pose.position.x, current_pose_msg.pose.position.y])
    #
    #     current_yaw = yaw_of_pose(current_pose_msg.pose)
    #     current_target_yaw = yaw_of_pose(target_pose.pose)
    #     vel.yaw = self.calc_yaw_vel(current_yaw, current_target_yaw, dt)
    #
    #     # try to avoid gitter
    #     target_vector = target_2d_pose - current_position
    #     # target_vector_yaw = math.atan2(target_vector[1], target_vector[0])
    #     rotation_angle = -current_yaw
    #     rotated_target_x, rotated_target_y = rotate_vector_by_angle(target_vector[0], target_vector[1],
    #                                                                 rotation_angle)
    #
    #     # print "target x {0} target y {1}".format(target_speed_x,target_speed_y)
    #     vel.x = self.calc_xy_velocity(self.pid_xy_x_speed, rotated_target_x, dt)
    #     vel.y = self.calc_xy_velocity(self.pid_xy_y_speed, rotated_target_y, dt)
    #
    #     # rospy.logdebug("pitch %f, roll %f, current yaw %f", x, -y, current_yaw)
    #     # rospy.logdebug("local_speed_y %f", local_speed_y)
    #     # rospy.loginfo("target vector %s \n roated target= %f, %f", str(target_vector), rotated_target_x, rotated_target_y)
    #     return vel


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

        # pid yaw
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