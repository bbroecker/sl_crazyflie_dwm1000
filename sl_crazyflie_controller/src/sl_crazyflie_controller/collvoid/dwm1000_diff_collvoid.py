#!/usr/bin/env python
import copy
import math
import rospy
from crazyflie_driver.msg import GenericLogData
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import TargetMsg, Velocity, Obstacle
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse
from tf import transformations

from collvoid_interface import CollvoidInterface
from sl_crazyflie_behavior_trees.AI.BehaviourTree import BehaviorTree, AIIO
from sl_crazyflie_behavior_trees.bt_node import BTNodeState
from sl_crazyflie_controller.sensor.dwm1000_distance_sensor import DWM10000DistanceSensor, INFINITY


class ControlCmd:
    def __init__(self):
        self.vel = 0.0
        self.psi_vel = 0.0
        self.abs_psi = 0.0


class DWM1000DiffCollvoid(CollvoidInterface):
    def __init__(self):
        CollvoidInterface.__init__(self)
        genome_file = rospy.get_param("~collvoid/Genome_file")
        self.max_angle_vel = rospy.get_param("~collvoid/dwm1000_diff_collvoid/max_angle_vel")
        self.max_angle_increment = rospy.get_param("~collvoid/dwm1000_diff_collvoid/max_angle_increment")
        self.max_vel = rospy.get_param("~collvoid/dwm1000_diff_collvoid/max_vel")
        self.timeout = rospy.get_param("~collvoid/dwm1000_diff_collvoid/timeout")
        pub_rate = rospy.get_param("~collvoid/dwm1000_diff_collvoid/publish_rate")
        self.start_srvs = rospy.Service("toggle_dwm_avoid", Empty, self.toggle_enable_callback)
        is_active_publisher = rospy.Publisher('dwm_coll_void_active', Bool, queue_size=1)
        self.dwm_sensor = DWM10000DistanceSensor()
        self.bt = BehaviorTree.load_from_file(genome_file)
        self.m_aiIO = AIIO()
        self.m_aiIO.input = [0] * self.bt.m_nIn
        self.m_aiIO.output = [0] * self.bt.m_nOut
        self.cmd = ControlCmd()
        self.enabled = False
        self.last_update = rospy.Time.now()
        self.active = False
        self.reference_vector = None

        rate = rospy.Rate(pub_rate)

        while not rospy.is_shutdown():
            is_active_publisher.publish(self.active)
            rate.sleep()

    def toggle_enable_callback(self, req):
        self.enabled = not self.enabled
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

    def set_speed(self, vel, speed):
        result_vel = copy.deepcopy(vel)
        norm = math.sqrt(vel.x**2 + vel.y**2)
        vel.x /= norm
        vel.x *= speed
        vel.y /= norm
        vel.y *= speed
        return result_vel



    # calculates the new target velocity based on the collvoid scheme
    # target_Velocity is the current target velocity towards the goal
    def calculate_velocity(self, current_target_velocity):
        if self.reference_vector is None:
            self.reference_vector = copy.deepcopy(current_target_velocity)
        dt = (rospy.Time.now() - self.last_update).to_sec()
        speed = self.cmd.vel * self.max_vel
        rotation_speed = self.cmd.psi_vel * self.max_angle_vel
        angle_increment = self.cmd.abs_psi * self.max_angle_increment
        self.reference_vector = self.set_speed(self.reference_vector, speed)
        self.reference_vector = self.rotate_vel_by_speed(self.reference_vector, rotation_speed, dt)
        self.reference_vector = self.rotate_vel_by_angle(self.reference_vector, angle_increment)

        self.last_update = rospy.Time.now()

        return self.reference_vector

    def get_sensor_data(self):
        current_time = rospy.Time.now()
        distances = self.dwm_sensor.sensor_data.distances
        last_updates = self.dwm_sensor.sensor_data.last_update
        closure_rates = self.dwm_sensor.sensor_data.closure_rate
        result_dis = None
        result_rate = None
        for idx, distance in distances:
            if (current_time - last_updates[idx]).to_sec() < self.timeout:
                if result_dis is None or distance < result_dis:
                    result_dis = distance
                    result_rate = closure_rates[idx]

        return result_dis, result_rate


    # tells the main controller if this bahaviour is active
    def is_active(self):
        distance, closure_rate = self.get_sensor_data()
        if not self.enabled or distance is None or closure_rate is None:
            self.reference_vector = None
            return False
        else:
            self.m_aiIO.input[0] = distance
            if self.bt.m_nIn > 1:
                self.m_aiIO.input[1] = closure_rate
            state = self.bt.trigger(self.m_aiIO)
            self.active = (state == BTNodeState.Success)
            if self.active:
                self.cmd.vel = self.m_aiIO.output[0]
                self.cmd.psi_vel = self.m_aiIO.output[1]
                self.cmd.abs_psi = self.m_aiIO.output[2]
            return self.active






