#!/usr/bin/env python
import rospy

from sl_crazyflie_controller.collvoid.collvoid_interface import CollvoidInterface
from sl_crazyflie_controller.collvoid.geofencing_node import GeoFenchingNode
from sl_crazyflie_controller.collvoid.simple_collvoid import SimpleCollvoid

class CollvoidController:
    def __init__(self):
        simple_collvoid_active = rospy.get_param("~collvoid/simple_collvoid_active", False)
        simple_collvoid_priority = rospy.get_param("~collvoid/simple_collvoid_priority", 2)
        geofencing_active = rospy.get_param("~collvoid/geofencing_active", False)
        geofencing_priority = rospy.get_param("~collvoid/geofencing_priority", 1)

        self.avoid_behaviours = []
        if simple_collvoid_active:
            self.avoid_behaviours.append(SimpleCollvoid(simple_collvoid_priority))
        if geofencing_active:
            self.avoid_behaviours.append(GeoFenchingNode(geofencing_priority))
        self.avoid_behaviours.sort(key=lambda x: x.priority, reverse=False)

    def calc_collvoid_velocity(self, current_pose, current_velocity):
        new_vel = current_velocity
        for behavoiour in self.avoid_behaviours:
            assert isinstance(behavoiour, CollvoidInterface)
            behavoiour.update_cf_pose(current_pose)
            if behavoiour.is_active():
                new_vel = behavoiour.calculate_velocity(current_velocity)
                break
        return new_vel