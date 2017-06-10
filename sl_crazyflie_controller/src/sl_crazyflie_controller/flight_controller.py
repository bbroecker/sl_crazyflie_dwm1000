#!/usr/bin/env python
import rospy
import tf
import math

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, PoseStamped
from sl_crazyflie_controller.cfg import pid_cfgConfig
from sl_crazyflie_msgs.msg import FlightMode, TargetMsg, ControlMode, Velocity
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest, ChangeFlightModeResponse
from std_srvs.srv import Empty
import copy

from flightmode_manager import FlightModeManager, POS_CTRL_MODES, INTERNAL_TARGET_MODES
from sl_crazyflie_controller.collvoid.collvoid_controller import CollvoidController
from sl_crazyflie_controller.pid_controller.pid_position import PositonController
from sl_crazyflie_controller.pid_controller.pid_velocity import VelocityController

RATE = 200.0
POSE_TIME_OUT = 0.5
CONTROLLER_RP_THRESH = 0.05
TAKEOFF_HEIGHT = 1.6
POS_THRESHOLD = 0.04
LAND_HEIGHT = 0.14
LAND_VEL = -0.25
TAKEOFF_VEL = 0.25
Z_VEL_SAMPLE_SIZE = 10
MAX_THROW_HEIGHT = 2.5
THROW_MOTOR_SPEED = 30000  # 45500
MIN_THROW_SPEED = 0.6  # m/s
FLIGHT_MODE_UPDATERATE = 2  # Publish current flight mode in 2 Hz
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
        self.target_velocity = None

        self.target_msg = TargetMsg()
        self.target_msg.target_pose = PoseStamped()
        cf_pose_topic = rospy.get_param("~cf_pose_topic")

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_flight_mode = rospy.Publisher("flight_mode", FlightMode, queue_size=1)

        self.last_fightmode_update = rospy.Time.now()
        self.last_external_cmd = None
        self.last_external_cmd_update = None
        self.is_collvoid_active = False
        self.mode_manager = FlightModeManager()
        self.pid_position = PositonController()
        self.pid_velocity = VelocityController()
        self.collvoid_controller = CollvoidController()

        self.pub_target_pose = rospy.Publisher("hover/target_pose", PoseStamped, queue_size=10)
        self.pub_current_pose = rospy.Publisher("hover/current_pose", PoseStamped, queue_size=10)
        self.cf_pose_sub = rospy.Subscriber(cf_pose_topic, PoseStamped, self.callback_cf_pose)
        self.manual_mode_subscriber_teleop = rospy.Subscriber("teleop/cmd_vel", Twist, self.cmd_vel_callback_teleop)
        # self.velocity_subscriber_teleop = rospy.Subscriber("teleop/velocity", Velocity, self.velocity_callback_teleop)
        # self.velocity_subscriber_geofencing = rospy.Subscriber("/geofencing/velocity", Velocity,
        #                                                        self.velocity_callback_geofencing)
        self.external_ctrl_sub = rospy.Subscriber(self.external_cmd_topic, TargetMsg, self.external_ctrl_callback)
        service = rospy.Service('change_flightmode', ChangeFlightMode, self.callback_change_flightmode)
        self.dyn_server = Server(pid_cfgConfig, self.callback_dynreconf)


    def main_loop(self):
        r = rospy.Rate(RATE)
        while not self.teleop_received:
            r.sleep()

        last_update = rospy.Time.now()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_update).to_sec()
            ##check for fail safe options
            mode_changed = self.mode_manager.update_mode(self.last_pose_msg, self.target_msg.target_pose)
            if mode_changed:
                self.update_target_msg()


            cmd_vel = Twist()
            if self.mode_manager.current_flightmode.id in POS_CTRL_MODES:
                if self.pid_tune_movement_active():
                    # still keep the current target position but allow external vel cmd
                    # is helpful for tuning position control
                    target_velocity = self.calc_target_velocities(self.last_external_cmd, dt)

                else:
                    target_velocity = self.calc_target_velocities(self.target_msg, dt)
                coll_active, target_velocity = self.collvoid_controller.calc_collvoid_velocity(self.last_pose_msg, target_velocity)
                # if coll_active:
                #     print "before {0} after {1}".format(z, target_velocity.z)
                if self.is_collvoid_active and not coll_active:
                    self.target_msg.target_pose = copy.copy(self.last_pose_msg)
                self.is_collvoid_active = coll_active
                cmd_vel = self.pid_velocity.update_cmd_twist(self.last_pose_msg, target_velocity, dt)
                self.pub_current_pose.publish(self.last_pose_msg)
                self.pub_target_pose.publish(self.target_msg.target_pose)

            self.send_vel_cmd(cmd_vel)
            if (rospy.Time.now() - self.last_fightmode_update).to_sec() > 1.0 / FLIGHT_MODE_UPDATERATE:
                self.pub_flight_mode.publish(self.mode_manager.current_flightmode)
                self.last_fightmode_update = rospy.Time.now()
            last_update = current_time
            r.sleep()

    def pid_tune_movement_active(self):
        return self.tune_pid and self.external_control_active()

    def callback_change_flightmode(self, mode):
        assert isinstance(mode, ChangeFlightModeRequest)
        mode_changed = self.mode_manager.change_flightmode(mode.mode.id)
        if mode_changed:
            self.update_target_msg()

        return ChangeFlightModeResponse()

    def calc_target_velocities(self, target_msg, dt):
        assert isinstance(target_msg, TargetMsg)
        vel = target_msg.target_velocity
        if target_msg.control_mode.x_mode == ControlMode.POSITION or target_msg.control_mode.y_mode == ControlMode.POSITION:
            vel.x, vel.y = self.pid_position.pose_to_xy_vel(self.last_pose_msg, target_msg.target_pose, dt)
        if target_msg.control_mode.z_mode == ControlMode.POSITION:
            vel.z = self.pid_position.calc_z_vel(self.target_msg.target_pose.pose.position.z,
                                                 self.last_pose_msg.pose.position.z, dt)
        if target_msg.control_mode.yaw_mode == ControlMode.POSITION:
            vel.yaw = self.pid_position.pose_to_yaw_vel(self.last_pose_msg, target_msg.target_pose, dt)
        return vel



    def update_target_msg(self):

        if self.mode_manager.current_flightmode.id in POS_CTRL_MODES and self.mode_manager.prev_flightmode.id not in POS_CTRL_MODES:
            rospy.logwarn("START_AGAIN")
            self.start_mocap_control()
        elif self.mode_manager.current_flightmode.id not in POS_CTRL_MODES:
            rospy.logwarn("STOPPP!!!!")
            self.stop_mocap_control()

        if self.mode_manager.current_flightmode.id in INTERNAL_TARGET_MODES:
            self.target_msg.control_mode.x_mode = self.target_msg.control_mode.y_mode = ControlMode.POSITION
            self.target_msg.control_mode.z_mode = self.target_msg.control_mode.yaw_mode = ControlMode.POSITION
            self.target_msg.target_velocity = Velocity()

            if self.mode_manager.current_flightmode.id is FlightMode.TAKEOFF:
                rospy.logwarn("TAKEOFF")
                self.target_msg.control_mode.z_mode = ControlMode.VELOCITY
                self.target_msg.target_pose = copy.deepcopy(self.last_pose_msg)
                self.target_msg.target_pose.pose.position.z = TAKEOFF_HEIGHT
                self.target_msg.target_velocity.z = TAKEOFF_VEL
            elif self.mode_manager.current_flightmode.id is FlightMode.POS_HOLD:
                self.target_msg.target_pose = copy.deepcopy(self.last_pose_msg)
            elif self.mode_manager.current_flightmode.id is FlightMode.LAND:
                rospy.logwarn("LAND")
                self.target_msg.control_mode.z_mode = ControlMode.VELOCITY
                self.target_msg.target_pose = copy.deepcopy(self.last_pose_msg)
                self.target_msg.target_pose.pose.position.z = 0
                self.target_msg.target_velocity.z = LAND_VEL




    def external_ctrl_callback(self, target_msg):
        assert isinstance(target_msg, TargetMsg)
        self.last_external_cmd = target_msg
        if self.mode_manager.current_flightmode.id is FlightMode.EXTERNAL_CONTROL:
            self.target_msg = self.last_external_cmd

    def start_mocap_control(self):
        self.pid_position.reset()
        self.pid_velocity.reset(self.last_pose_msg)

    def stop_mocap_control(self):
        self.pid_position.reset()
        self.pid_velocity.reset(self.last_pose_msg)

    def callback_cf_pose(self, pose_msg):
        self.last_pose_update = rospy.Time.now()
        self.last_pose_msg = pose_msg


    def joy_active(self):
        return abs(self.velocity_teleop.x) > CONTROLLER_RP_THRESH or abs(
            self.velocity_teleop.y) > CONTROLLER_RP_THRESH or abs(self.velocity_teleop.yaw) > CONTROLLER_RP_THRESH

    def external_control_active(self):
        if self.last_external_cmd is None:
            return False
        velo = self.last_external_cmd.target_velocity
        max_value = max(abs(velo.yaw), max(abs(velo.z), max(abs(velo.x), abs(velo.y))))
        return max_value > CONTROLLER_RP_THRESH


    def send_vel_cmd(self, cmd_vel_pid):
        cur_time = rospy.Time.now()
        current_mode = self.mode_manager.current_flightmode.id
        twist = self.cmd_vel_teleop
        if current_mode is FlightMode.DISARM:
            twist = Twist()
        elif current_mode is FlightMode.THROW_LAUNCH_STABILIZE:
            twist.linear.z = THROW_MOTOR_SPEED
        elif current_mode is FlightMode.MANUAL:
            twist = self.cmd_vel_teleop
        elif current_mode in POS_CTRL_MODES:
            twist.linear.z = cmd_vel_pid.linear.z
            if self.mode_manager.current_flightmode.id in POS_CTRL_MODES:
                twist.linear.y = cmd_vel_pid.linear.y
                twist.linear.x = cmd_vel_pid.linear.x
                twist.angular.z = cmd_vel_pid.angular.z

        self.cmd_pub.publish(twist)

    def cmd_vel_callback_teleop(self, twist):
        self.cmd_vel_teleop = twist
        self.teleop_received = True
        self.teleop_last_time = rospy.Time.now()

    def velocity_callback_teleop(self, vel):
        self.velocity_teleop = vel

    def callback_dynreconf(self, config, level):
        self.pid_velocity.callback_dynreconf(config, level)
        self.pid_position.callback_dynreconf(config, level)
        rospy.loginfo("Dynreconf callback %s", str(config))
        return config




if __name__ == '__main__':
    rospy.init_node('FlightModeManager')
    manager = FlightController()
    manager.main_loop()
