#!/usr/bin/env python
import collections
import copy
import xml.etree.ElementTree as ET

import math

from bt_node import BTNode, BTNodeState
import bt_factory

from sl_crazyflie_behavior_trees.Geometry.g_vector import GVector
from sl_crazyflie_behavior_trees.bt_workspace import BTWorkspace


class BTSelect(BTNode):
    def __init__(self, other=None):
        if other is None:
            BTNode.__init__(self, "BTSelect", True)
        else:
            BTNode.__init__(self, other)
        self.m_running_index = 0


    def tick_fcn(self, workspace):
        for i in range(self.m_running_index, len(self.m_children)):
            node_state = self.m_children[i].tick(workspace)
            if node_state is BTNodeState.Success:
                self.m_running_index = 0
                return BTNodeState.Success
            elif node_state is BTNodeState.Running:
                self.m_running_index = i
                return BTNodeState.Running
        self.m_running_index = 0
        return BTNodeState.Failure

    def clone(self):
        return copy.deepcopy(self)

    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        for i in range(len(self.m_children)):
            self.m_children[i].save(node)

    def load_attributes(self, node):
        assert isinstance(node, ET.Element)

        for child_node in node:
            self.m_children.append(bt_factory.BTFactory.get_child_from_node(child_node))
            if self.m_children[-1] is None:
                self.m_children.pop()
                return False
        return True

class BTSequence(BTNode):
    def __init__(self, other=None):
        if other is None:
            BTNode.__init__(self, "BTSequence", True)
            self.m_running_index = 0
        else:
            BTNode.__init__(self, other)
            self.m_running_index = 0

    def tick_fcn(self, workspace):
        for i in range(self.m_running_index, len(self.m_children)):
            node_state = self.m_children[i].tick(workspace)
            if node_state is BTNodeState.Failure:
                self.m_running_index = 0
                return BTNodeState.Failure
            elif node_state is BTNodeState.Running:
                self.m_running_index = i
                return BTNodeState.Running
        self.m_running_index = 0
        return BTNodeState.Success

    def clone(self):
        return copy.deepcopy(self)

    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        for i in range(len(self.m_children)):
            self.m_children[i].save(node)

    def load_attributes(self, node):
        assert isinstance(node, ET.Element)

        for child_node in node:
            self.m_children.append(bt_factory.BTFactory.get_child_from_node(child_node))
            if self.m_children[-1] is None:
                self.m_children.pop()
                return False
        return True

class BTTarget(BTNode):
    def __init__(self, state_array, dt):
        if isinstance(state_array, BTTarget):
            other = state_array
            BTNode.__init__(self, other)
            self.state_array = other.state_array
        else:
            BTNode.__init__(self, "BTTarget", False)
            self.state_array = state_array
        self.dt = dt
        self.prev_heading = self.state_array[1]
        self.prev_error = None
        self.d_buffer = collections.deque(maxlen=3)

    def clone(self):
        return copy.deepcopy(self)

    def save_attributes(self, node):
        pass

    def load_attributes(self, node):
        return False

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        current_pos = self.state_array[0]
        current_heading = self.state_array[1]
        target = self.state_array[2]

        angle_speed = ((current_heading - self.prev_heading)/math.pi)*180 / self.dt


        # print "angle speed: {0}/s".format(angle_speed)


        k1 = 5
        k2 = 10
        # print "pos x {0} y {1}".format(current_pos.x, current_pos.y)
        # print "target x {0} y {1}".format(target.x, target.y)
        pos_to_target = GVector(current_pos, target)

        error_angle = current_heading - pos_to_target.angle();
        if self.prev_error is None:
            self.prev_error = error_angle
        # print "current_heading {0} targetAngle {1}".format(current_heading, error_angle)

        while error_angle > math.pi:
            error_angle -= 2 * math.pi

        while error_angle < -math.pi:
            error_angle += 2 * math.pi


        Vcmd = (1 - (abs(error_angle) / math.pi)) * 1.1;
        if Vcmd > 1:
            Vcmd = 1
        error_angle *= -1
        e = (error_angle) * 0.8
        d = (error_angle - self.prev_error)/ self.dt * 0.00
        self.d_buffer.append(d)
        #omega = -(Vcmd * 0.5 / pos_to_target.norm()) * (k2 * error_angle + 1 + k1 * math.sin(error_angle))
        omega = e + sum(self.d_buffer) / len(self.d_buffer)

        # print "e {0} d {1}".format(e, d)

        omega = (omega / math.pi)
        # omega = error_angle/math.pi * 0.07
        omega = 1 if omega > 1.0 else omega
        omega = -1 if omega < -1.0 else omega
        # print "omega {0}".format(omega)

        workspace.set_par(0, Vcmd)
        workspace.set_par(1, omega)
        workspace.set_par(2, 0)
        self.prev_heading = current_heading
        self.prev_error = error_angle

        return BTNodeState.Success


class BTWhile(BTNode):
    def __init__(self, other=None):
        if other is None:
            BTNode.__init__(self, "BTWhile", True)
            self.m_running_index = 0
        else:
            BTNode.__init__(self, other)
            self.m_running_index = 0

    def tick_fcn(self, workspace):
        for i in range(self.m_running_index, len(self.m_children)):
            node_state = self.m_children[i].tick(workspace)
            if node_state is BTNodeState.Failure:
                self.m_running_index = 0
                return BTNodeState.Running
            elif node_state is BTNodeState.Running:
                self.m_running_index = i
                return BTNodeState.Running
        self.m_running_index = 0
        return BTNodeState.Success

    def clone(self):
        return copy.deepcopy(self)

    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        for i in range(len(self.m_children)):
            self.m_children[i].save(node)

    def load_attributes(self, node):
        assert isinstance(node, ET.Element)

        for child_node in node:
            self.m_children.append(bt_factory.BTFactory.get_child_from_node(child_node))
            if self.m_children[-1] is None:
                self.m_children.pop()
                return False
        return True
