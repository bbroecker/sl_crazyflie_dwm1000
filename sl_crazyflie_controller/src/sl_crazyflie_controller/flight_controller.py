#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseStamped
from sl_crazyflie_msgs.msg import FlightMode, TargetMsg, ControlMode, Velocity
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest, ChangeFlightModeResponse
from std_srvs.srv import Empty
import copy

from flightmode_manager import FlightModeManager, POS_CTRL_MODES
from sl_crazyflie_controller.pid_controller.pid_position import PositonController

RATE = 200.0
POSE_TIME_OUT = 0.5
CONTROLLER_RP_THRESH = 0.05
TAKEOFF_HEIGHT = 1.3
POS_THRESHOLD = 0.04
LAND_HEIGHT = 0.14
LAND_VEL = -0.25
TAKEOFF_VEL = 0.25
Z_VEL_SAMPLE_SIZE = 10
MAX_THROW_HEIGHT = 2.5
THROW_MOTOR_SPEED = 30000 #45500
MIN_THROW_SPEED = 0.6 #m/s
FLIGHT_MODE_UPDATERATE = 20  # Publish current flight mode in 2 Hz
# TARGET_FRAME_ID = 'Robot_2/base_link'
# CRAZY_FLIE_FRAME_ID = 'Robot_1/base_link'
WORLD_FRAME_ID = '/world'




def euler_distance_pose(pose1, pose2):
    assert isinstance(pose1, PoseStamped)
    assert isinstance(pose2, PoseStamped)
    return math.sqrt(math.pow(pose1.pose.position.x - pose2.pose.position.x, 2) + math.pow(
        pose1.pose.position.y - pose2.pose.position.y, 2) +
                     math.pow(pose1.pose.position.z - pose2.pose.position.z, 2))



class FlightController:
    def __init__(self):

        self.pid_received = False
        self.teleop_received = False
        self.cmd_vel_teleop = None
        self.velocity_teleop = None
        ##Timestamp
        ##Target pose for specific modes (takeoff, pos_hold, land)
        self.last_pose_msg = None

        self.tf_listen = tf.TransformListener()
        self.tune_pid = rospy.get_param("~pid_tuning_active", False)
        self.external_cmd_topic = rospy.get_param("~external_cmd_topic")
        self.target_pose = None
        self.target_velocity = None

        self.target_msg = TargetMsg()
        cf_pose_topic = rospy.get_param("~cf_pose_topic")

        self.pub_target_msg = rospy.Publisher("main_crtl/target_msg", TargetMsg, queue_size=1)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_flight_mode = rospy.Publisher("flight_mode", FlightMode, queue_size=1)


        self.last_fightmode_update = rospy.Time.now()
        self.last_external_cmd = None
        self.last_external_cmd_update = None
        self.mode_manager = FlightModeManager()

        self.cf_pose_sub = rospy.Subscriber(cf_pose_topic, PoseStamped, self.callback_cf_pose)
        self.manual_mode_subscriber_teleop = rospy.Subscriber("teleop/cmd_vel", Twist, self.cmd_vel_callback_teleop)
        # self.velocity_subscriber_teleop = rospy.Subscriber("teleop/velocity", Velocity, self.velocity_callback_teleop)
        # self.velocity_subscriber_geofencing = rospy.Subscriber("/geofencing/velocity", Velocity,
        #                                                        self.velocity_callback_geofencing)
        self.velocity_subscriber_pid = rospy.Subscriber("hover/cmd_vel", Twist, self.cmd_vel_callback_pid)
        self.external_ctrl_sub = rospy.Subscriber(self.external_cmd_topic, TargetMsg, self.external_ctrl_callback)
        service = rospy.Service('change_flightmode', ChangeFlightMode, self.callback_change_flightmode)
        self.pid_position = PositonController()


    def main_loop(self):
        r = rospy.Rate(RATE)
        while not self.teleop_received:
            r.sleep()

        while not rospy.is_shutdown():
            self.send_vel_cmd()
            ##check for fail safe options
            mode_changed = self.mode_manager.update_mode(self.last_pose_msg, self.target_pose)
            if mode_changed:
                self.update_target_pose()
            self.update_target_msg()
            self.target_msg.target_velocity = self.convert_target_msg_to_vel(self.target_msg)
            self.publish_target_msg()
            if (rospy.Time.now() - self.last_fightmode_update).to_sec() > 1.0 / FLIGHT_MODE_UPDATERATE:
                self.pub_flight_mode.publish(self.mode_manager.current_flightmode)
                self.last_fightmode_update = rospy.Time.now()
            r.sleep()

    def callback_change_flightmode(self, mode):
        print "callback_change_flightmode %s" % mode.mode.id
        assert isinstance(mode, ChangeFlightModeRequest)
        mode_changed = self.mode_manager.change_flightmode(mode.mode.id)
        if mode_changed:
            self.update_target_pose()

        return ChangeFlightModeResponse()

    def convert_target_msg_to_vel(self, target_msg):
        assert isinstance(target_msg, TargetMsg)
        vel = target_msg.target_velocity
        if target_msg.control_mode.x_mode == ControlMode.POSITION or target_msg.control_mode.y_mode == ControlMode.POSITION:
            vel.x, vel.y = self.pid_position.pose_to_xy_vel(self.last_pose_msg, target_msg.target_pose)
        if target_msg.control_mode.z_mode == ControlMode.POSITION:
            vel.z = self.pid_position.calc_z_vel(self.target_msg.target_pose.pose.position.z, self.last_pose_msg.pose.position.z)
        if target_msg.control_mode.yaw_mode == ControlMode.POSITION:
            vel.yaw = self.pid_position.pose_to_yaw_vel(self.last_pose_msg, target_msg.target_pose)
        return vel


    def update_target_pose(self):
        if self.mode_manager.current_flightmode.id is FlightMode.TAKEOFF:
            rospy.logwarn("TAKEOFF")
            self.target_pose = copy.deepcopy(self.last_pose_msg)
            self.target_pose.pose.position.z = TAKEOFF_HEIGHT
        elif self.mode_manager.current_flightmode.id is FlightMode.POS_HOLD:
            self.target_pose = copy.deepcopy(self.last_pose_msg)
        elif self.mode_manager.current_flightmode.id is FlightMode.LAND:
            rospy.logwarn("LAND")
            self.target_pose = copy.deepcopy(self.last_pose_msg)
            self.target_pose.pose.position.z = 0

        if self.mode_manager.current_flightmode.id in POS_CTRL_MODES and self.mode_manager.prev_flightmode.id not in POS_CTRL_MODES:
            rospy.logwarn("START_AGAIN")
            self.start_mocap_control()
        elif self.mode_manager.current_flightmode.id not in POS_CTRL_MODES:
            rospy.logwarn("STOPPP!!!!")
            self.stop_mocap_control()




    def external_ctrl_callback(self, target_msg):
        assert isinstance(target_msg, TargetMsg)
        self.last_external_cmd = target_msg
        self.last_external_cmd_update = rospy.Time.now()

    def start_mocap_control(self):
        self.pid_position.reset()

    def stop_mocap_control(self):
        self.pid_position.reset()


    def callback_cf_pose(self, pose_msg):
        self.last_pose_update = rospy.Time.now()
        self.last_pose_msg = pose_msg

    def publish_target_msg(self):
        # update time stamp for modes fixed target positions
        if self.target_pose is not None:
            self.pub_target_msg.publish(self.target_msg)

    def update_target_msg(self):
        self.target_msg.control_mode = self.gen_control_mode()
        self.target_msg.target_pose = self.gen_target_pose()
        self.target_msg.target_velocity = self.gen_target_vel()

    def gen_target_vel(self):
        vel = Velocity()
        if self.mode_manager.current_flightmode.id is FlightMode.LAND:
            vel.z = LAND_VEL
        if self.mode_manager.current_flightmode.id is FlightMode.TAKEOFF:
            vel.z = TAKEOFF_VEL

        if (self.mode_manager.current_flightmode.id is FlightMode.EXTERNAL_CONTROL and self.last_external_cmd is not None) or (self.tune_pid and self.external_control_active()):
            vel = self.last_external_cmd.target_velocity
        # if self.last_geo_fencing_update is not None and (
        #     rospy.Time.now() - self.last_geo_fencing_update).to_sec() <= POSE_TIME_OUT:
        #     vel.x = self.last_geo_fencing_vel.x
        #     vel.y = self.last_geo_fencing_vel.y
        return vel

    def gen_target_pose(self):
        if self.mode_manager.current_flightmode.id in [FlightMode.TAKEOFF, FlightMode.POS_HOLD, FlightMode.LAND]:
            self.target_pose.header.stamp = rospy.Time.now()
        elif (self.mode_manager.current_flightmode.id is FlightMode.EXTERNAL_CONTROL and self.last_external_cmd is not None) or (self.tune_pid and self.external_control_active()):
            self.target_pose = self.last_external_cmd.target_pose
        return self.target_pose

    def joy_active(self):
        return abs(self.velocity_teleop.x) > CONTROLLER_RP_THRESH or abs(
            self.velocity_teleop.y) > CONTROLLER_RP_THRESH or abs(self.velocity_teleop.yaw) > CONTROLLER_RP_THRESH

    def external_control_active(self):
        if self.last_external_cmd is None:
            return False
        velo = self.last_external_cmd.target_velocity
        max_value = max(abs(velo.yaw), max(abs(velo.z), max(abs(velo.x), abs(velo.y))))
        return max_value > CONTROLLER_RP_THRESH

    def gen_control_mode(self):
        mode = ControlMode()
        mode.x_mode = mode.y_mode = mode.z_mode = mode.yaw_mode = ControlMode.POSITION
        if self.mode_manager.current_flightmode.id in [FlightMode.TAKEOFF, FlightMode.LAND]:
            mode.z_mode = ControlMode.VELOCITY
        elif (self.mode_manager.current_flightmode.id is FlightMode.EXTERNAL_CONTROL and self.last_external_cmd is not None) or (self.tune_pid and self.external_control_active()):
            if self.last_external_cmd.control_mode is not None:
                mode = self.last_external_cmd.control_mode

        # if self.last_geo_fencing_update is not None and (
        #     rospy.Time.now() - self.last_geo_fencing_update).to_sec() <= POSE_TIME_OUT:
        #     mode.x_mode = mode.y_mode = ControlMode.VELOCITY
        return mode


    def send_vel_cmd(self):
        cur_time = rospy.Time.now()
        current_mode = self.mode_manager.current_flightmode.id
        twist = self.cmd_vel_teleop
        if current_mode is FlightMode.DISARM:
            twist = Twist()
        elif current_mode is FlightMode.THROW_LAUNCH_STABILIZE:
                twist.linear.z = THROW_MOTOR_SPEED
        elif current_mode is FlightMode.MANUAL:
            twist = self.cmd_vel_teleop
        elif current_mode in POS_CTRL_MODES and self.pid_received:
            if (cur_time - self.pid_last_time).to_sec() < POSE_TIME_OUT:
                # pass
                twist.linear.z = self.cmd_vel_pid.linear.z
                if self.mode_manager.current_flightmode.id in POS_CTRL_MODES:
                    twist.linear.y = self.cmd_vel_pid.linear.y
                    twist.linear.x = self.cmd_vel_pid.linear.x
                    twist.angular.z = self.cmd_vel_pid.angular.z
            else:
                if (cur_time - self.teleop_last_time).to_sec() > POSE_TIME_OUT:
                    twist = Twist()
        self.cmd_pub.publish(twist)

    def cmd_vel_callback_teleop(self, twist):
        self.cmd_vel_teleop = twist
        self.teleop_received = True
        self.teleop_last_time = rospy.Time.now()

    def velocity_callback_teleop(self, vel):
        self.velocity_teleop = vel

    def cmd_vel_callback_pid(self, twist):
        self.cmd_vel_pid = twist
        self.pid_received = True
        self.pid_last_time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('FlightModeManager')
    manager = FlightController()
    manager.main_loop()
