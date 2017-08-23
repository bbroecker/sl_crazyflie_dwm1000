#!/usr/bin/env python
import importlib
import os
import pkgutil

import rospy
import sys

from sl_crazyflie_controller.collvoid.collvoid_interface import CollvoidInterface
# from sl_crazyflie_controller.collvoid.geofencing_node import GeoFenchingNode
# from sl_crazyflie_controller.collvoid.simple_collvoid import SimpleCollvoid
#
# from sl_crazyflie_controller.collvoid.dwm1000_collvoid import DWM1000Collvoid


class CollvoidController:
    def __init__(self):
        controllers_names = rospy.get_param("~collvoid/controllers")
        self.controllers = []

        for c in controllers_names:
            self.controllers.append(get_child_class(c, CollvoidInterface)())

        self.avoid_behaviours = []


    def calc_collvoid_velocity(self, current_pose, current_velocity):
        active = False
        new_vel = current_velocity
        for behaviour in self.controllers:
            assert isinstance(behaviour, CollvoidInterface)
            #Updates current pose regardless of whether behaviour is active
            behaviour.update_cf_pose(current_pose)
            if behaviour.is_active():
                active = True
                #Only calculates velocity if behaviour is active
                new_vel = behaviour.calculate_velocity(current_velocity)
                break
        return active, new_vel

def find_possible_child_classes(dirname):
    names = []
    pkg_dir = os.path.dirname(dirname)
    for (module_loader, name, ispkg) in pkgutil.iter_modules([pkg_dir]):
        names.append(name)
    return names


def all_subclasses(cls):
    """
    Recursively search for subclasses
    """
    return cls.__subclasses__() + [g for s in cls.__subclasses__()
                                   for g in all_subclasses(s)]


def get_child_class(module_name, class_type):
    module_path = sys.modules[class_type.__module__].__file__
    possible_child_classes = find_possible_child_classes(module_path)
    if module_name not in possible_child_classes:
        raise ImportError("Cannot import %s\n available:\n%s" % (module_name, str(possible_child_classes)))
    parent_package = '.'.join(class_type.__module__.split('.')[:-1])
    importlib.import_module('.' + module_name, parent_package)
    for cls in all_subclasses(class_type):
        if module_name.lower().replace('_', '') == cls.__name__.lower():
            return cls
    raise ImportError("Cannot import %s\n available:\n%s, did you name your class correctly?"
                      % (module_name, str(possible_child_classes)))
