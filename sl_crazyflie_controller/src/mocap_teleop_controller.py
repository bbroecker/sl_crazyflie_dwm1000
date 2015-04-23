#!/usr/bin/env python
__author__ = 'broecker'
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sl_crazyflie_srvs.srv import ChangeTargetPose, ChangeTargetPoseRequest, StartWanding
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
        self.cmd_vel_teleop = Twist()
        self.cmd_vel_pid = Twist()
        self.pid_received = False
        self.teleop_received = False
        self.pid_last_time = None
        self.teleop_last_time = None
        self.wanding = False
        self.prev_pressed = {'takeoff': False, 'up': False, 'down': False, 'left': False, 'right': False, 'forward': False, 'backward': False, 'wanding': False}

        self.hover_stop_srv = rospy.ServiceProxy('hover/stop', Empty)
        self.hold_position_start_srv = rospy.ServiceProxy('hover/start_hold_position', Empty)
        self.change_target_pose_srv = rospy.ServiceProxy('hover/change_target_pose', ChangeTargetPose)
        self.toggle_position_control_srv = rospy.ServiceProxy('hover/toggle_position_control', Empty)

        self.start_wanding_srv = rospy.ServiceProxy('hover/start_wanding', StartWanding)
        self.stop_wanding_srv = rospy.ServiceProxy('hover/stop_wanding', Empty)

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.joy_subscriber_ = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.velocity_subscriber_teleop = rospy.Subscriber("teleop/cmd_vel", Twist, self.cmd_vel_callback_teleop)
        self.velocity_subscriber_pid = rospy.Subscriber("hover/cmd_vel", Twist, self.cmd_vel_callback_pid)

    def spin(self):
        r = rospy.Rate(RATE)
        while not self.teleop_received:
            r.sleep()

        while not rospy.is_shutdown():
            cur_time = rospy.Time.now()
            twist = self.cmd_vel_teleop
            if (self.pid_active or self.position_control_active) and self.pid_received:
                if (cur_time - self.pid_last_time).to_sec() < TIME_OUT:
                    #pass
                    twist.linear.z = self.cmd_vel_pid.linear.z
                    if abs(twist.linear.y) < CONTROLLER_RP_THRESH:
                        twist.linear.y = self.cmd_vel_pid.linear.y
                    if abs(twist.linear.x) < CONTROLLER_RP_THRESH:
                        twist.linear.x = self.cmd_vel_pid.linear.x
                else:
                    twist = Twist()
            else:
                if (cur_time - self.teleop_last_time).to_sec() > TIME_OUT:
                    twist = Twist()
            self.cmd_pub.publish(twist)
            r.sleep()

    def joy_callback(self, joy_msgs):
        if not joy_msgs.buttons[self.pid_active_button] == 0:
            if not self.pid_active:
                self.hold_position_start_srv()
            self.pid_active = True
        else:
            if self.pid_active:
                self.hover_stop_srv()
            self.pid_active = False

        if self.is_button_released('takeoff', joy_msgs.buttons[TAKEOFF]):
            self.toggle_position_control_srv()
            if not self.position_control_active:
                self.position_control_active = True
            else:
                self.position_control_active = False

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
        if not pose.x == 0 or not pose.y == 0 or not pose.z == 0:
            self.change_target_pose_srv(req)

        if self.is_button_released('wanding', joy_msgs.buttons[WANDING]):
            if self.wanding:
                self.stop_wanding_srv()
                self.wanding = False
            else:
                res = self.start_wanding_srv()
                self.wanding = res.success


    def is_button_released(self, button_name, button_pressed):
        if button_pressed:
            self.prev_pressed[button_name] = True
        elif self.prev_pressed[button_name]:
            self.prev_pressed[button_name] = False
            return True
        return False


    def cmd_vel_callback_teleop(self, twist):
        self.cmd_vel_teleop = twist
        self.teleop_received = True
        self.teleop_last_time = rospy.Time.now()

    def cmd_vel_callback_pid(self, twist):
        self.cmd_vel_pid = twist
        self.pid_received = True
        self.pid_last_time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node("mocap_telecop")
    cont = MocapController()
    cont.spin()













