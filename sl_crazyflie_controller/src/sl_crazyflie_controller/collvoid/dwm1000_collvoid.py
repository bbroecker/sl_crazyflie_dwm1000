#!/usr/bin/env python
import copy
import math
import rospy
from crazyflie_driver.msg import GenericLogData
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, Obstacle
from std_srvs.srv import Empty, EmptyResponse
from tf import transformations

from collvoid_interface import CollvoidInterface

ACTIVATION_DISTANCE = 0.9


class DWM1000Collvoid(CollvoidInterface):
    def __init__(self):
        print "HELLO!!!!!!!! DWM1000Collvoid"

        CollvoidInterface.__init__(self)
        rospy.Subscriber("log_ranges", GenericLogData, self.range_callback)
        start_srvs = rospy.Service("toggle_dwm_avoid", Empty, self.toggle_enable_callback)
        self.dwm_distance = 0
        self.range_active = False
        self.enable = False
        self.is_running = False
        self.reference_vel = Velocity()
        self.ae = 1.0
        self.last_update = rospy.Time.now()



    def range_callback(self, data):
        ranges = data.values[:6]
        min_distance = 2000.0

        state = int(data.values[6])
        valid = [False] * 6
        for i in range(6):
            #is unvalid if distance == 0
            valid[i] = (state & (1 << i)) != 0
            if ranges[i] < min_distance and ranges[i] != 0:
                min_distance = ranges[i]


        if min_distance < ACTIVATION_DISTANCE:
            self.last_update = rospy.Time.now()
            self.dwm_distance = min_distance
            self.range_active = True
        else:
            self.range_active = False
            self.is_running = False


    def toggle_enable_callback(self, req):
        self.enable = not self.enable
        return EmptyResponse()

    def rotate_vel_by_speed(self, vel, rotation_speed, dt):
        angle = rotation_speed * dt
        return self.rotate_vel_by_angle(vel, angle)


    def rotate_vel_by_angle(self, vel, angle):
        assert isinstance(vel, Velocity)
        result_vel = copy.deepcopy(vel)
        angle = angle / 180 * math.pi
        vector_x = vel.x
        vector_y = vel.y
        result_vel.x = vector_x * math.cos(angle) - vector_y * math.sin(angle)
        result_vel.y = vector_x * math.sin(angle) + vector_y * math.cos(angle)
        return result_vel

    def cut_max_speed(self, vel, max_vel):
        result_vel = copy.deepcopy(vel)
        norm = math.sqrt(vel.x**2 + vel.y**2)
        if norm > max_vel:
            vel.x /= norm
            vel.x *= max_vel
            vel.y /= norm
            vel.y *= max_vel
        return result_vel



    # calculates the new target velocity based on the collvoid scheme
    # target_Velocity is the current target velocity towards the goal
    def calculate_velocity(self, current_target_velocity):
        rotation_speed = 0.0
        dt = (rospy.Time.now() - self.last_update).to_sec()
        if not self.is_running:
            self.is_running = True
            self.reference_vel = current_target_velocity
            self.ae = 1.0
        if self.dwm_distance < ACTIVATION_DISTANCE:
            self.reference_vel = self.cut_max_speed(self.reference_vel, 0.15)
            rotation_speed = 30.0
            if self.dwm_distance < 0.5 and self.ae > 0.6:
                self.reference_vel = self.rotate_vel_by_angle(self.reference_vel, 120.0)
                self.ae = 0.0
            else:
                if self.dwm_distance > 0.5:
                    self.reference_vel = self.rotate_vel_by_speed(self.reference_vel, rotation_speed, dt)
        print self.reference_vel
        self.last_update = rospy.Time.now()
        return self.reference_vel


    # tells the main controller if this bahaviour is active
    def is_active(self):
        active = (self.enable and self.range_active)
        if active:
            print "DW1000Collvoid active"
        return active






