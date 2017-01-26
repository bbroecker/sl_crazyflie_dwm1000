import numpy as np

import rospy
from dynamic_reconfigure.server import Server
from sl_crazyflie_controller.cfg import pid_cfgConfig
#from sl_crazyflie_controller.cfg import pid_positionConfig
from sl_crazyflie_msgs.msg import Velocity

from pid import PidController, yaw_of_pose, rotate_vector_by_angle, limit_angle


class PositonController:

    def __init__(self):

        self.kp_x = rospy.get_param("~pid_position/x/kp")
        self.ki_x = rospy.get_param("~pid_position/x/ki")
        self.kd_x = rospy.get_param("~pid_position/x/kd")
        self.max_x_error = rospy.get_param("~pid_position/x/max_error")
        self.max_x_d_filter = rospy.get_param("~pid_position/x/d_filter_count")

        self.kp_y = rospy.get_param("~pid_position/y/kp")
        self.ki_y = rospy.get_param("~pid_position/y/ki")
        self.kd_y = rospy.get_param("~pid_position/y/kd")
        self.max_y_error = rospy.get_param("~pid_position/y/max_error")
        self.max_y_d_filter = rospy.get_param("~pid_position/y/d_filter_count")

        self.kp_z = rospy.get_param("~pid_position/z/kp")
        self.ki_z = rospy.get_param("~pid_position/z/ki")
        self.kd_z = rospy.get_param("~pid_position/z/kd")
        self.max_altitude_error = rospy.get_param("~pid_position/z/max_error")
        self.max_z_d_filter = rospy.get_param("~pid_position/z/d_filter_count")

        self.kp_yaw = rospy.get_param("~pid_position/yaw/kp")
        self.ki_yaw = rospy.get_param("~pid_position/yaw/ki")
        self.kd_yaw = rospy.get_param("~pid_position/yaw/kd")
        self.max_yaw_angle_error = rospy.get_param("~pid_position/yaw/max_error")
        self.max_yaw_d_filter = rospy.get_param("~pid_position/yaw/d_filter_count")
        self.load_cfg_params = True
        self.target_2d_pose = None

        self.pid_xy_x_speed = PidController(self.kp_x, self.ki_x, self.kd_x, -0.5, 0.5, self.max_x_d_filter)
        self.pid_xy_y_speed = PidController(self.kp_y, self.ki_y, self.kd_y, -0.5, 0.5, self.max_y_d_filter)
        self.pid_climb_rate = PidController(self.kp_z, self.ki_z, self.kd_z, -0.4, 0.4, self.max_z_d_filter)

        self.pid_yaw_angle = PidController(self.kp_yaw, self.ki_yaw, self.kd_yaw, -1, 1, self.max_yaw_d_filter)




    def reset(self):
        # self.paused = not self.paused
        self.pid_climb_rate.reset_pid()

        self.pid_xy_x_speed.reset_pid()
        self.pid_xy_y_speed.reset_pid()




    def calc_xy_velocity(self, pid_xy_speed, error, max_error, dt):
        current_error = error
        if current_error > max_error:
            current_error = max_error
        elif current_error < - max_error:
            current_error = -max_error
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
        x = self.calc_xy_velocity(self.pid_xy_x_speed, rotated_target_x, self.max_x_error, dt)
        y = self.calc_xy_velocity(self.pid_xy_y_speed, rotated_target_y, self.max_y_error, dt)

        return x, y




    def callback_dynreconf(self, config, level):
        # rospy.loginfo("kp_thrust callback %s", kp_thrust)
        # rospy.loginfo("kd_thrust callback %s", kd_thrust)
        if self.load_cfg_params:
            self.load_cfg_params = False
            self.update_dyn_conf(config)
        else:
            self.update_pid(config)

        rospy.loginfo("Dynreconf callback %s", str(config))
        return config

    def update_dyn_conf(self, config):
        config['KP_z'] = self.kp_z
        config['KI_z'] = self.ki_z
        config['KD_z'] = self.kd_z
        config['max_z_error'] = self.max_altitude_error
        config['max_z_d_filter'] = self.max_z_d_filter

        config['KP_x'] = self.kp_x
        config['KI_x'] = self.ki_x
        config['KD_x'] = self.kd_x
        config['max_x_error'] = self.max_x_error
        config['max_x_d_filter'] = self.max_x_d_filter

        config['KP_y'] = self.kp_y
        config['KI_y'] = self.ki_y
        config['KD_y'] = self.kd_y
        config['max_y_error'] = self.max_y_error
        config['max_y_d_filter'] = self.max_y_d_filter
        # pid yaw
        config['KP_yaw'] = self.kp_yaw
        config['KI_yaw'] = self.ki_yaw
        config['KD_yaw'] = self.kd_yaw
        config['max_yaw_error'] = self.max_yaw_angle_error
        config['max_yaw_d_filter'] = self.max_yaw_d_filter


    def update_pid(self, config):
        self.kp_z = config['KP_z']
        self.ki_z = config['KI_z']
        self.kd_z = config['KD_z']
        self.max_altitude_error = config['max_z_error']
        self.max_z_d_filter = config['max_z_d_filter']

        self.kp_x = config['KP_x']
        self.ki_x = config['KI_x']
        self.kd_x = config['KD_x']
        self.max_x_error = config['max_x_error']
        self.max_x_d_filter = config['max_x_d_filter']

        self.kp_y = config['KP_y']
        self.ki_y = config['KI_y']
        self.kd_y = config['KD_y']
        self.max_y_error = config['max_y_error']
        self.max_y_d_filter = config['max_y_d_filter']
        # pid yaw
        self.kp_yaw = config['KP_yaw']
        self.ki_yaw = config['KI_yaw']
        self.kd_yaw = config['KD_yaw']
        self.max_yaw_angle_error = config['max_yaw_error']
        self.max_yaw_d_filter = config['max_yaw_d_filter']

        self.pid_climb_rate.set_pid_parameters(self.kp_z, self.ki_z, self.kd_z, self.max_z_d_filter)
        self.pid_xy_x_speed.set_pid_parameters(self.kp_x, self.ki_x, self.kd_x, self.max_x_d_filter)
        self.pid_xy_y_speed.set_pid_parameters(self.kp_y, self.ki_y, self.kd_y, self.max_y_d_filter)

        self.pid_yaw_angle.set_pid_parameters(self.kp_yaw, self.ki_yaw, self.kd_yaw, self.max_yaw_d_filter)
