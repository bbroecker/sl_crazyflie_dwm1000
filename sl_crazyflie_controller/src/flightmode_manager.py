#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseStamped
from sl_crazyflie_msgs.msg import FlightMode, TargetMsg, ControlMode, Velocity
from sl_crazyflie_srvs.srv import ChangeFlightMode, ChangeFlightModeRequest, ChangeFlightModeResponse
from std_srvs.srv import Empty
import copy


RATE = 200.0
POSE_TIME_OUT = 0.5
CONTROLLER_RP_THRESH = 0.05
TAKEOFF_HEIGHT = 1.3
WAND_DISTANCE = 0.5
POS_THRESHOLD = 0.04
LAND_HEIGHT = 0.10
LAND_VEL = -0.25
TAKEOFF_VEL = 0.25
Z_VEL_SAMPLE_SIZE = 10
MAX_THROW_HEIGHT = 2.5
THROW_MOTOR_SPEED = 30000 #45500
MIN_THROW_SPEED = 0.6 #m/s
FLIGHT_MODE_UPDATERATE = 2  # Publish current flight mode in 2 Hz
# TARGET_FRAME_ID = 'Robot_2/base_link'
# CRAZY_FLIE_FRAME_ID = 'Robot_1/base_link'
WORLD_FRAME_ID = '/world'
POS_CTRL_MODES = [FlightMode.LAND, FlightMode.TAKEOFF, FlightMode.POS_HOLD, FlightMode.WANDING, FlightMode.POS_FOLLOW,
                  FlightMode.TEST_VEL_JOY,
                  FlightMode.EXTERNAL_CONTROL]
VEL_CTRL_MODES = [FlightMode.TEST_VEL_JOY]


def euler_distance_pose(pose1, pose2):
    assert isinstance(pose1, PoseStamped)
    assert isinstance(pose2, PoseStamped)
    return math.sqrt(math.pow(pose1.pose.position.x - pose2.pose.position.x, 2) + math.pow(
        pose1.pose.position.y - pose2.pose.position.y, 2) +
                     math.pow(pose1.pose.position.z - pose2.pose.position.z, 2))

class ThrowLaunchState:
    DETECT_THROW, STABILIZE = range(2)

class FlightModeManager:
    def __init__(self):
        self.current_flightmode = FlightMode()
        self.current_flightmode.id = FlightMode.MANUAL
        self.pid_received = False
        self.teleop_received = False
        self.cmd_vel_teleop = None
        self.velocity_teleop = None
        self.last_wand_pose = None
        ##Timestamp
        self.last_wand_update = None
        ##Target pose for specific modes (takeoff, pos_hold, land)
        self.last_pose_update = None
        self.last_pose_msg = None
        self.prev_pose_msg = None
        self.current_thow_launch_state = None

        self.last_geo_fencing_update = None
        self.last_geo_fencing_vel = None

        self.tf_listen = tf.TransformListener()
        self.tune_pid = rospy.get_param("~pid_tuning_active", False)
        self.target_pose = None
        self.target_velocity = None
        self.avg_z_vel_list = [0.0 for i in range(Z_VEL_SAMPLE_SIZE)]
        self.avg_z_vel = None



        self.target_msg = TargetMsg()
        self.wand_frame_id = rospy.get_param("wand_frame_id", "/Robot_2/base_link")
        cf_pose_topic = rospy.get_param("cf_pose_topic", "/Robot_1/pose")
        wand_pose_topic = rospy.get_param("wand_pose_topic", "/Robot_2/pose")
        self.wand_frame_id = rospy.get_param("wand_frame_id", "/Robot_2/base_link")

        self.pub_target_msg = rospy.Publisher("main_crtl/target_msg", TargetMsg, queue_size=1)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_flight_mode = rospy.Publisher("flight_mode", FlightMode, queue_size=1)

        self.wand_pose_sub = rospy.Subscriber(wand_pose_topic, PoseStamped, self.callback_wand_pose)
        self.cf_pose_sub = rospy.Subscriber(cf_pose_topic, PoseStamped, self.callback_cf_pose)
        self.manual_mode_subscriber_teleop = rospy.Subscriber("teleop/cmd_vel", Twist, self.cmd_vel_callback_teleop)
        # self.velocity_subscriber_teleop = rospy.Subscriber("teleop/velocity", Velocity, self.velocity_callback_teleop)
        # self.velocity_subscriber_geofencing = rospy.Subscriber("/geofencing/velocity", Velocity,
        #                                                        self.velocity_callback_geofencing)
        self.velocity_subscriber_pid = rospy.Subscriber("hover/cmd_vel", Twist, self.cmd_vel_callback_pid)
        self.external_ctrl_sub = rospy.Subscriber("external_cmd", TargetMsg, self.external_ctrl_callback)

        rospy.wait_for_service('hover/stop_pid')
        rospy.wait_for_service('hover/start_pid')
        self.stop_mocap_ctrl = rospy.ServiceProxy('hover/stop_pid', Empty)
        self.start_mocap_ctrl = rospy.ServiceProxy('hover/start_pid', Empty)
        rospy.Service('change_flightmode', ChangeFlightMode, self.callback_change_flightmode)
        self.last_fightmode_update = rospy.Time.now()
        self.last_external_cmd = None
        self.last_external_cmd_update = None

    def main_loop(self):
        r = rospy.Rate(RATE)
        while not self.teleop_received:
            r.sleep()

        while not rospy.is_shutdown():
            self.send_vel_cmd()
            ##check for fail safe options
            self.check_for_fallback_mode()
            self.update_target_msg()
            self.publish_target_msg()
            if (rospy.Time.now() - self.last_fightmode_update).to_sec() > 1.0 / FLIGHT_MODE_UPDATERATE:
                self.pub_flight_mode.publish(self.current_flightmode)
                self.last_fightmode_update = rospy.Time.now()
            r.sleep()

    def external_ctrl_callback(self, target_msg):
        assert isinstance(target_msg, TargetMsg)
        self.last_external_cmd = target_msg
        self.last_external_cmd_update = rospy.Time.now()


    def check_for_fallback_mode(self):
        if self.current_flightmode.id is FlightMode.DISARM:
            return
        new_mode = ChangeFlightModeRequest()
        new_mode.mode.id = self.current_flightmode.id
        change_mode = True
        if self.current_flightmode.id is FlightMode.THROW_LAUNCH:
            change_mode = False
            if self.current_thow_launch_state is ThrowLaunchState.DETECT_THROW and self.avg_z_vel > MIN_THROW_SPEED:
                self.current_thow_launch_state = ThrowLaunchState.STABILIZE
                print "STABILIZE"
            elif self.current_thow_launch_state is ThrowLaunchState.STABILIZE and (self.avg_z_vel < MIN_THROW_SPEED or self.last_pose_msg.pose.position.z >= MAX_THROW_HEIGHT):
                change_mode = True
                self.current_thow_launch_state = ThrowLaunchState.DETECT_THROW
                print "POS_HOLD"
                new_mode.mode.id = FlightMode.POS_HOLD
        elif self.current_flightmode.id in POS_CTRL_MODES and (
            rospy.Time.now() - self.last_pose_update).to_sec() > POSE_TIME_OUT:
            new_mode.mode.id = FlightMode.MANUAL
        elif self.current_flightmode.id is FlightMode.WANDING and (
            rospy.Time.now() - self.last_wand_update).to_sec() > POSE_TIME_OUT:
            new_mode.mode.id = FlightMode.POS_HOLD
        elif self.current_flightmode.id is FlightMode.LAND:
            if self.last_pose_msg.pose.position.z <= LAND_HEIGHT:
                new_mode.mode.id = FlightMode.MANUAL
                print "<= LAND_HEIGHT"
            else:
                change_mode = False
        elif self.current_flightmode.id is FlightMode.TAKEOFF:
            if self.last_pose_msg.pose.position.z >= self.target_pose.pose.position.z:
                new_mode.mode.id = FlightMode.POS_HOLD
            else:
                change_mode = False
        elif self.current_flightmode.id is FlightMode.EXTERNAL_CONTROL and self.last_external_cmd_update is not None and (
            rospy.Time.now() - self.last_external_cmd_update).to_sec() > POSE_TIME_OUT:
            new_mode.mode.id = FlightMode.POS_HOLD
        else:
            change_mode = False
        if change_mode:
            print "change mode!! arg"
            self.callback_change_flightmode(new_mode)

    def velocity_callback_geofencing(self, vel):
        self.last_geo_fencing_update = rospy.Time.now()
        self.last_geo_fencing_vel = vel

    def callback_cf_pose(self, pose_msg):
        prev_update_time = self.last_pose_update
        self.prev_pose_msg = copy.deepcopy(self.last_pose_msg)
        self.last_pose_update = rospy.Time.now()
        self.last_pose_msg = pose_msg
        if self.prev_pose_msg is not None:
            #update z velocity
            cf_z_vel = (pose_msg.pose.position.z - self.prev_pose_msg.pose.position.z) / (self.last_pose_update - prev_update_time).to_sec()
            self.avg_z_vel_list.append(cf_z_vel)
            if len(self.avg_z_vel_list) > Z_VEL_SAMPLE_SIZE:
                self.avg_z_vel_list.pop(0)
            self.avg_z_vel = sum(self.avg_z_vel_list) / len(self.avg_z_vel_list)
            #check for mocap jumps
            if euler_distance_pose(self.prev_pose_msg, self.last_pose_msg) > 0.1:
                print "arg!!!! mocap {0}".format(euler_distance_pose(self.prev_pose_msg, self.last_pose_msg))


    def callback_wand_pose(self, pose_msg):
        self.last_wand_update = rospy.Time.now()
        if self.current_flightmode.id is FlightMode.WANDING:
            wand_target = PoseStamped()
            wand_target.pose.position.z = WAND_DISTANCE
            wand_target.pose.orientation.w = 1.
            wand_target.header.stamp = pose_msg.header.stamp
            wand_target.header.frame_id = self.wand_frame_id
            target = None
            try:
                self.tf_listen.waitForTransform(self.wand_frame_id, pose_msg.header.frame_id, pose_msg.header.stamp,
                                                rospy.Duration(0.2))
                target = self.tf_listen.transformPose(WORLD_FRAME_ID, wand_target)
            except tf.Exception as e:
                rospy.logwarn("Transform failed: %s", e)
            if target is not None:
                self.target_pose = target

    def callback_change_flightmode(self, msg):
        assert isinstance(msg, ChangeFlightModeRequest)
        response = ChangeFlightModeResponse()
        response.success = True
        if msg.mode.id is FlightMode.WANDING and (rospy.Time.now() - self.last_wand_update).to_sec() > POSE_TIME_OUT:
            rospy.logwarn("Can't do wand-mode, didn't see the wand in a while :)")
            response.success = False
        elif msg.mode.id is FlightMode.TAKEOFF:
            rospy.logwarn("TAKEOFF")
            # set takeoff target height
            self.target_pose = copy.deepcopy(self.last_pose_msg)
            self.target_pose.pose.position.z = TAKEOFF_HEIGHT
        elif msg.mode.id is FlightMode.POS_HOLD:
            self.target_pose = copy.deepcopy(self.last_pose_msg)
        elif msg.mode.id is FlightMode.LAND:
            rospy.logwarn("LAND")
            self.target_pose = copy.deepcopy(self.last_pose_msg)
            self.target_pose.pose.position.z = 0
        elif msg.mode.id is FlightMode.THROW_LAUNCH:
            self.current_thow_launch_state = ThrowLaunchState.DETECT_THROW
        if response.success:
            prev_flightmode = self.current_flightmode.id
            self.current_flightmode.id = msg.mode.id
            rospy.logwarn("current_mode == %d" % (msg.mode.id))
            if self.current_flightmode.id in POS_CTRL_MODES and prev_flightmode not in POS_CTRL_MODES:
                rospy.logwarn("START_AGAIN")
                self.start_mocap_ctrl()
            elif self.current_flightmode.id not in POS_CTRL_MODES:
                rospy.logwarn("STOPPP!!!!")
                self.stop_mocap_ctrl()
        return response

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
        if self.current_flightmode.id is FlightMode.LAND:
            vel.z = LAND_VEL
        if self.current_flightmode.id is FlightMode.TAKEOFF:
            vel.z = TAKEOFF_VEL

        if (self.current_flightmode.id is FlightMode.EXTERNAL_CONTROL and self.last_external_cmd is not None) or (self.tune_pid and self.external_control_active()):
            vel = self.last_external_cmd.target_velocity
        # if self.last_geo_fencing_update is not None and (
        #     rospy.Time.now() - self.last_geo_fencing_update).to_sec() <= POSE_TIME_OUT:
        #     vel.x = self.last_geo_fencing_vel.x
        #     vel.y = self.last_geo_fencing_vel.y
        return vel

    def gen_target_pose(self):
        if self.current_flightmode.id in [FlightMode.TAKEOFF, FlightMode.POS_HOLD, FlightMode.LAND]:
            self.target_pose.header.stamp = rospy.Time.now()
        elif (self.current_flightmode.id is FlightMode.EXTERNAL_CONTROL and self.last_external_cmd is not None) or (self.tune_pid and self.external_control_active()):
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
        if self.current_flightmode.id in [FlightMode.TAKEOFF, FlightMode.LAND]:
            mode.z_mode = ControlMode.VELOCITY
        elif (self.current_flightmode.id is FlightMode.EXTERNAL_CONTROL and self.last_external_cmd is not None) or (self.tune_pid and self.external_control_active()):
            if self.last_external_cmd.control_mode is not None:
                mode = self.last_external_cmd.control_mode

        # if self.last_geo_fencing_update is not None and (
        #     rospy.Time.now() - self.last_geo_fencing_update).to_sec() <= POSE_TIME_OUT:
        #     mode.x_mode = mode.y_mode = ControlMode.VELOCITY
        return mode




    def send_vel_cmd(self):
        cur_time = rospy.Time.now()
        current_mode = self.current_flightmode.id
        twist = self.cmd_vel_teleop
        if current_mode is FlightMode.DISARM:
            twist = Twist()
        elif current_mode is FlightMode.THROW_LAUNCH:
            if self.current_thow_launch_state is ThrowLaunchState.STABILIZE:
                twist.linear.z = THROW_MOTOR_SPEED
        elif current_mode is FlightMode.MANUAL:
            twist = self.cmd_vel_teleop
        elif current_mode in POS_CTRL_MODES and self.pid_received:
            if (cur_time - self.pid_last_time).to_sec() < POSE_TIME_OUT:
                # pass
                twist.linear.z = self.cmd_vel_pid.linear.z
                if self.current_flightmode.id in POS_CTRL_MODES:
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
    manager = FlightModeManager()
    manager.main_loop()
