#!/usr/bin/env python
from sl_crazyflie_msgs.msg import FlightMode

__author__ = 'broecker'
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sl_crazyflie_srvs.srv import ChangeTargetPose, ChangeTargetPoseRequest, StartWanding, ChangeFlightMode, \
    ChangeFlightModeRequest
import math

RATE = 50
TIME_OUT = 0.5
CONTROLLER_RP_THRESH = 0.2
UP = 12
DOWN = 14
FORWARD = 4
BACKWARD = 6
LEFT = 7
RIGHT = 5
TAKEOFF = 3
STEP_SIZE = 0.15
WANDING = 0

class MocapController:
    def __init__(self):

        self.pid_active_button = rospy.get_param("~pid_activate_axis", 11)
        self.pid_active = False
        self.position_control_active = False
        self.pid_last_time = None
        self.teleop_last_time = None
        self.wanding = False
        self.prev_pressed = {'takeoff': False, 'up': False, 'down': False, 'left': False, 'right': False, 'forward': False, 'backward': False, 'wanding': False}

        self.change_flight_mode = rospy.ServiceProxy('change_flightmode', ChangeFlightMode)

        self.joy_subscriber_ = rospy.Subscriber("joy", Joy, self.joy_callback)

    def spin(self):
        r = rospy.Rate(RATE)

        while not rospy.is_shutdown():
            r.sleep()

    def joy_callback(self, joy_msgs):
        # if not joy_msgs.buttons[self.pid_active_button] == 0:
        #     if not self.pid_active:
        #         self.hold_position_start_srv()
        #     self.pid_active = True
        # else:
        #     if self.pid_active:
        #         self.hover_stop_srv()
        #     self.pid_active = False
        ch_flm = ChangeFlightModeRequest()
        if self.is_button_released('takeoff', joy_msgs.buttons[TAKEOFF]):
            if not self.position_control_active:
                self.position_control_active = True
                ch_flm.mode.id = FlightMode.TAKEOFF
            else:
                ch_flm.mode.id = FlightMode.LAND
                self.position_control_active = False
            self.change_flight_mode(ch_flm)

        if self.is_button_released('wanding', joy_msgs.buttons[WANDING]):
            if self.wanding:
                ch_flm.mode.id = FlightMode.POS_HOLD
                self.change_flight_mode(ch_flm)
                self.wanding = False
            else:
                ch_flm.mode.id = FlightMode.WANDING
                #ch_flm.mode.id = FlightMode.TEST_VEL_JOY
                res = self.change_flight_mode(ch_flm)
                self.wanding = res.success


        req = ChangeTargetPoseRequest()
        if self.is_button_released('up', joy_msgs.buttons[UP]):
            req.pose.position.z = STEP_SIZE
        if self.is_button_released('down', joy_msgs.buttons[DOWN]):
            req.pose.position.z = -STEP_SIZE
        if self.is_button_released('left', joy_msgs.buttons[LEFT]):
            req.pose.position.y = STEP_SIZE
        if self.is_button_released('right', joy_msgs.buttons[RIGHT]):
            req.pose.position.y = -STEP_SIZE
        if self.is_button_released('forward', joy_msgs.buttons[FORWARD]):
            req.pose.position.x = STEP_SIZE
        if self.is_button_released('backward', joy_msgs.buttons[BACKWARD]):
            req.pose.position.x = -STEP_SIZE
        pose = req.pose.position
        # if not pose.x == 0 or not pose.y == 0 or not pose.z == 0:
        #     self.change_target_pose_srv(req)
        #toDo add target service




    def is_button_released(self, button_name, button_pressed):
        if button_pressed:
            self.prev_pressed[button_name] = True
        elif self.prev_pressed[button_name]:
            self.prev_pressed[button_name] = False
            return True
        return False




if __name__ == '__main__':
    rospy.init_node("mocap_telecop")
    cont = MocapController()
    cont.spin()













