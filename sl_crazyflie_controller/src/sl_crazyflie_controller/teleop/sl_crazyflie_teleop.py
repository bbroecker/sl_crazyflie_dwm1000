#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest
from std_srvs.srv import Empty
import math
from sl_crazyflie_msgs.msg import Velocity, FlightMode, TargetMsg, ControlMode
from sl_crazyflie_controller.flightmode_manager import POS_CTRL_MODES, INTERNAL_TARGET_MODES

TAKEOFF = 3
STEP_SIZE = 0.15
WANDING = 0
DEADMAN_SWITCH = 10
DISARM = 14
THROW = 12
MANUAL = 15


def map_value(value, in_min, in_max, out_min, out_max):
    return ((value - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min


class Teleop:
    def __init__(self):
        target_topic = rospy.get_param("~target_topic", "external_cmd")
        joy_topic = rospy.get_param("~joy_topic", "joy")
        self.pid_tuning_active = rospy.get_param("~pid_tuning_active", False)

        self.manual_mode_publisher_ = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.target_msg_publisher_ = rospy.Publisher(target_topic, TargetMsg, queue_size=1)
        self.change_flight_mode = rospy.ServiceProxy('change_flightmode', ChangeFlightMode)
        self.start_wanding = rospy.ServiceProxy('external_modes/start_pose_follower', Empty)
        self.stop_wanding = rospy.ServiceProxy('external_modes/stop_pose_follower', Empty)

        self.on_client_ = rospy.ServiceProxy('on', Empty)
        self.off_client_ = rospy.ServiceProxy('off', Empty)
        self.is_wanding = False

        self.pid_active_button = rospy.get_param("~pid_activate_axis", 11)

        self.prev_pressed = {'takeoff': False, 'up': False, 'down': False, 'left': False, 'right': False,
                             'forward': False, 'backward': False, 'wanding': False, 'disarm': False, 'throw': False, 'manual' : False}

        self.current_flight_mode_id = None
        self.controller_active = False

        self.axes = {}
        self.axes["x"] = {"axis": 0, "max": 2}
        self.axes["y"] = {"axis": 0, "max": 2}
        self.axes["z"] = {"axis": 0, "max": 2}
        self.axes["yaw"] = {"axis": 0, "max": 90 * math.pi / 180.0}
        self.button = {"on": 1, "off": 2}

        self.axes["x"]["axis"] = rospy.get_param("~x_axis", 0)
        self.axes["y"]["axis"] = rospy.get_param("~y_axis", 0)
        self.axes["z"]["axis"] = rospy.get_param("~z_axis", 0)
        self.axes["yaw"]["axis"] = rospy.get_param("~yaw_axis", 0)

        self.axes["x"]["max"] = rospy.get_param("~x_joy_max", 30)
        self.axes["y"]["max"] = rospy.get_param("~y_joy_max", -30)  # invert
        self.axes["z"]["max"] = rospy.get_param("~z_joy_max", 60000)
        self.axes["yaw"]["max"] = rospy.get_param("~yaw_joy_max", -200)

        self.x_max_vel = rospy.get_param("~x_vel_max", 0.1)
        self.y_max_vel = rospy.get_param("~x_vel_max", 0.1)
        self.z_max_vel = rospy.get_param("~z_vel_max", 0.1)
        self.yaw_max_vel = rospy.get_param("~yaw_vel_max", 0.5)
        self.joy_value_ = Twist()
        self.joy_subscriber_ = rospy.Subscriber("flight_mode", FlightMode, self.flight_mode_callback)
        self.joy_subscriber_ = rospy.Subscriber(joy_topic, Joy, self.joy_callback)

    def gen_target_msg(self, twist):
        assert isinstance(twist, Twist)
        vel = Velocity()
        vel.x = map_value(twist.linear.x, -self.axes["x"]["max"], self.axes["x"]["max"], -self.x_max_vel,
                          self.x_max_vel)
        vel.y = map_value(twist.linear.y, -self.axes["y"]["max"], self.axes["y"]["max"], -self.y_max_vel,
                          self.y_max_vel)
        vel.z = map_value(twist.linear.z, -self.axes["z"]["max"], self.axes["z"]["max"], -self.z_max_vel,
                          self.z_max_vel)
        # print "x: {0} y:{1}".format(vel.z, vel.y)
        vel.yaw = map_value(twist.angular.z, -self.axes["yaw"]["max"], self.axes["yaw"]["max"], -self.yaw_max_vel,
                            self.yaw_max_vel)

        msg = TargetMsg()
        msg.target_velocity = vel
        msg.control_mode.x_mode = msg.control_mode.y_mode = msg.control_mode.z_mode = msg.control_mode.yaw_mode = ControlMode.VELOCITY
        return msg

    def joy_callback(self, joy_msg):
        self.check_button_events(joy_msg)
        self.joy_value_.linear.x = self.get_axis(joy_msg, self.axes['x']['axis']) * self.axes["x"]["max"]
        self.joy_value_.linear.y = self.get_axis(joy_msg, self.axes['y']['axis']) * self.axes["y"]["max"]
        self.joy_value_.linear.z = self.get_axis(joy_msg, self.axes['z']['axis']) * self.axes["z"]["max"]
        self.joy_value_.angular.z = self.get_axis(joy_msg, self.axes['yaw']['axis']) * self.axes["yaw"]["max"]

        self.manual_mode_publisher_.publish(self.joy_value_)
        # if deadman switch....
        if self.controller_active:
            self.target_msg_publisher_.publish(self.gen_target_msg(self.joy_value_))

            # if self.get_button(joy_msg, self.button['on']):
            #     self.on_client_()
            # if self.get_button(joy_msg, self.button['off']):
            #     self.off_client_()

    def flight_mode_callback(self, mode):
        assert isinstance(mode, FlightMode)
        self.current_flight_mode_id = mode.id

    def check_button_events(self, joy_msgs):
        ch_flm = ChangeFlightModeRequest()
        ch_flm.mode.id = -1
        if self.is_button_released('takeoff', joy_msgs.buttons[TAKEOFF]):
            if self.current_flight_mode_id not in POS_CTRL_MODES or self.current_flight_mode_id == FlightMode.LAND:
                ch_flm.mode.id = FlightMode.TAKEOFF
            else:
                ch_flm.mode.id = FlightMode.LAND


        if self.is_button_released('wanding', joy_msgs.buttons[WANDING]):
            if self.is_wanding:
                ch_flm.mode.id = FlightMode.POS_HOLD
                self.stop_wanding.call()
                self.is_wanding = False
            else:
                # ch_flm.mode.id = FlightMode.WANDING
                self.start_wanding.call()
                self.is_wanding = True

        if joy_msgs.buttons[DEADMAN_SWITCH]:
            if not self.controller_active:
                self.controller_active = True
                if not self.pid_tuning_active:
                    self.mode_id_backup = self.current_flight_mode_id
                    ch_flm.mode.id = FlightMode.EXTERNAL_CONTROL

        elif self.controller_active:
            self.controller_active = False
            if not self.pid_tuning_active:
                ch_flm.mode.id = self.mode_id_backup

        if self.is_button_released('throw', joy_msgs.buttons[THROW]):
            if self.current_flight_mode_id is not FlightMode.THROW_LAUNCH:
                ch_flm.mode.id = FlightMode.THROW_LAUNCH

        if self.is_button_released('manual', joy_msgs.buttons[MANUAL]):
            ch_flm.mode.id = FlightMode.MANUAL

        if self.is_button_released('disarm', joy_msgs.buttons[DISARM]):
            if self.current_flight_mode_id is FlightMode.DISARM:
                ch_flm.mode.id = FlightMode.MANUAL
            else:
                # ch_flm.mode.id = FlightMode.WANDING
                ch_flm.mode.id = FlightMode.DISARM

        if ch_flm.mode.id != -1:
            if ch_flm.mode.id in INTERNAL_TARGET_MODES and self.is_wanding:
                self.is_wanding = False
                self.stop_wanding.call()
            self.change_flight_mode(ch_flm)
            self.current_flight_mode_id = ch_flm.mode.id


    def get_axis(self, joy_msg, axis):
        if axis == 0 or axis > len(joy_msg.axes):
            return 0
        sign = 1
        if axis < 0:
            sign = -1
        return sign * joy_msg.axes[abs(axis) - 1]

    def is_button_released(self, button_name, button_pressed):
        if button_pressed:
            self.prev_pressed[button_name] = True
        elif self.prev_pressed[button_name]:
            self.prev_pressed[button_name] = False
            return True
        return False

    def get_button(self, joy_msgs, button_idx):
        if button_idx <= 0 or button_idx > len(joy_msgs.axes):
            return 0
        return joy_msgs.buttons[button_idx - 1]


if __name__ == '__main__':
    rospy.init_node("sl_crazy_teleop")
    Teleop()
    rospy.spin()
