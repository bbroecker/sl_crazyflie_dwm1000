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
WAND_DISTANCE = 0.5
POS_THRESHOLD = 0.04
LAND_HEIGHT = 0.14
LAND_VEL = -0.25
TAKEOFF_HEIGHT = 1.6
TAKEOFF_VEL = 0.25
Z_VEL_SAMPLE_SIZE = 10
MAX_THROW_HEIGHT = 2.5
THROW_MOTOR_SPEED = 30000 #45500
MIN_THROW_SPEED = 0.6 #m/s
FLIGHT_MODE_UPDATERATE = 20  # Publish current flight mode in 2 Hz
# TARGET_FRAME_ID = 'Robot_2/base_link'
# CRAZY_FLIE_FRAME_ID = 'Robot_1/base_link'
WORLD_FRAME_ID = '/world'
POS_CTRL_MODES = [FlightMode.LAND, FlightMode.TAKEOFF, FlightMode.POS_HOLD, FlightMode.EXTERNAL_CONTROL]
INTERNAL_TARGET_MODES = [FlightMode.LAND, FlightMode.TAKEOFF, FlightMode.POS_HOLD, FlightMode.THROW_LAUNCH, FlightMode.THROW_LAUNCH_STABILIZE]


def euler_distance_pose(pose1, pose2):
    assert isinstance(pose1, PoseStamped)
    assert isinstance(pose2, PoseStamped)
    return math.sqrt(math.pow(pose1.pose.position.x - pose2.pose.position.x, 2) + math.pow(
        pose1.pose.position.y - pose2.pose.position.y, 2) +
                     math.pow(pose1.pose.position.z - pose2.pose.position.z, 2))


class FlightModeManager:
    def __init__(self):
        self.current_flightmode = FlightMode()

        #Starts in MANUAL mode
        self.current_flightmode.id = FlightMode.MANUAL

        ##Target pose for specific modes (takeoff, pos_hold, land)
        self.last_pose_update = None
        self.last_pose_msg = None
        self.prev_pose_msg = None
        self.prev_flightmode = FlightMode()
        self.prev_flightmode.id = self.current_flightmode.id


        self.tf_listen = tf.TransformListener()
        self.avg_z_vel_list = [0.0 for i in range(Z_VEL_SAMPLE_SIZE)]
        self.avg_z_vel = None

    #I think this checks for bad modes to go into and stops it from happening
    def check_for_fallback_mode(self, target_pose):

        if self.current_flightmode.id is FlightMode.DISARM:
            return

        change_mode = True

        if self.current_flightmode.id is FlightMode.THROW_LAUNCH and self.avg_z_vel > MIN_THROW_SPEED:
            new_mode_id = FlightMode.THROW_LAUNCH_STABILIZE
        elif self.current_flightmode.id is FlightMode.THROW_LAUNCH_STABILIZE and (self.avg_z_vel < MIN_THROW_SPEED or self.last_pose_msg.pose.position.z >= MAX_THROW_HEIGHT):
            new_mode_id = FlightMode.POS_HOLD
        elif self.current_flightmode.id in POS_CTRL_MODES and (
            rospy.Time.now() - self.last_pose_update).to_sec() > POSE_TIME_OUT:
            new_mode_id = FlightMode.MANUAL
        elif self.current_flightmode.id is FlightMode.LAND:
            if self.last_pose_msg.pose.position.z <= LAND_HEIGHT:
                new_mode_id = FlightMode.MANUAL
            else:
                change_mode = False
        elif self.current_flightmode.id is FlightMode.TAKEOFF:
            if self.last_pose_msg.pose.position.z >= TAKEOFF_HEIGHT:
                new_mode_id = FlightMode.POS_HOLD
            else:
                change_mode = False
        else:
            change_mode = False

        #Actually changes flight mode if this function has allowed it
        if change_mode:
            change_mode = self.change_flightmode(new_mode_id)

        return change_mode

    def update_mode(self, cf_pose, target_pose):
        self.update_cf_pose(cf_pose)
        return self.check_for_fallback_mode(target_pose)

    def update_cf_pose(self, pose_msg):
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
            # if euler_distance_pose(self.prev_pose_msg, self.last_pose_msg) > 0.1:
            #     print "arg!!!! mocap {0}".format(euler_distance_pose(self.prev_pose_msg, self.last_pose_msg))



    def change_flightmode(self, mode_id):

        print "Flight mode: ", mode_id

        #hack deactive DISARM FIRST
        changed = True
        if self.current_flightmode.id == FlightMode.DISARM and mode_id is not FlightMode.MANUAL or self.current_flightmode.id == mode_id:
            changed = False
        else:
            self.prev_flightmode.id = self.current_flightmode.id
            self.current_flightmode.id = mode_id

        return changed
