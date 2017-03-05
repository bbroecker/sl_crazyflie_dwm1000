#!/usr/bin/env python
import collections
import copy
import xml.etree.ElementTree as ET

import math

import rospy
from std_msgs.msg import Float32

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
    def __init__(self, state_array, dt, id = 1):
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

        angle_speed = (current_heading - self.prev_heading) / self.dt
        if angle_speed > math.pi:
            angle_speed = math.pi
        if angle_speed < -math.pi:
            angle_speed = -math.pi



        k1 = 5
        k2 = 10
        # print "pos x {0} y {1}".format(current_pos.x, current_pos.y)
        # print "target x {0} y {1}".format(target.x, target.y)
        pos_to_target = GVector(current_pos, target)

        error_angle = current_heading - pos_to_target.angle()

        # print "current_heading {0} targetAngle {1}".format(current_heading, error_angle)

        while error_angle > math.pi:
            error_angle -= 2 * math.pi

        while error_angle < -math.pi:
            error_angle += 2 * math.pi

        # Vcmd = (posToTarget.norm()) * (1 - (fabs(targetAngle) / M_PI));
        Vcmd = (pos_to_target.norm() * 1.1) * (1 - (abs(error_angle) / math.pi)) * 1.0;
        if Vcmd > 1:
            Vcmd = 1
        error_angle *= -1
        target_angle_speed = error_angle * 0.8

        omega = (target_angle_speed / math.pi)
        # omega = error_angle/math.pi * 0.07
        omega = 1 if omega > 1.0 else omega
        omega = -1 if omega < -1.0 else omega
        # print "omega {0}".format(omega)

        workspace.set_par(0, Vcmd)
        workspace.set_par(1, omega)
        workspace.set_par(2, 0)
        self.prev_heading = current_heading
        # self.prev_error = speed_error

        return BTNodeState.Success


class BTTargetDiffdrive(BTNode):
    def __init__(self, state_array, dt, max_turn_vel = 180):
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
        self.d_buffer = collections.deque(maxlen=4)
        self.i_sum = collections.deque(maxlen=20)

        self.max_turn_vel = max_turn_vel

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


        k1 = 5
        k2 = 10
        # print "pos x {0} y {1}".format(current_pos.x, current_pos.y)
        # print "target x {0} y {1}".format(target.x, target.y)
        pos_to_target = GVector(current_pos, target)

        error_angle = pos_to_target.angle() - current_heading

        # print "current_heading {0} targetAngle {1}".format(current_heading, error_angle)

        while error_angle > math.pi:
            error_angle -= 2 * math.pi

        while error_angle < -math.pi:
            error_angle += 2 * math.pi

        v = 1 if (pos_to_target.norm()*1.2 > 1) else pos_to_target.norm() * 1.2
        # Vcmd = (posToTarget.norm()) * (1 - (fabs(targetAngle) / M_PI));
        # Vcmd = (1 - (abs(error_angle) / math.pi))
        Vcmd = (v) - (abs(error_angle) / math.pi)
        if Vcmd > 1:
            Vcmd = 1
        if Vcmd <= 0.25:
            Vcmd = 0.25

        # if self.prev_error is None:
        #     self.prev_error = error_angle
        #
        # self.i_sum.append(error_angle)
        # d = (error_angle - self.prev_error)/self.dt
        # # print "d1 {0} d2 {1}".format(d, d2)
        #
        # self.d_buffer.append(d)
        #
        # target_angle_speed = error_angle * 0.45 + sum(self.i_sum) * 0.0025 + sum(self.d_buffer) / len(self.d_buffer) * 0.1
        target_angle_speed = ((error_angle * 3.0) / math.pi)/180 * self.max_turn_vel

        omega = target_angle_speed
        # omega = error_angle / math.pi
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


class BTGreaterID(BTNode):
    def __init__(self, state_array):
        if isinstance(state_array, BTGreaterID):
            other = state_array
            BTNode.__init__(self, other)
            self.state_array = other.state_array
        else:
            BTNode.__init__(self, "BTGreaterID", False)
            self.state_array = state_array


    def clone(self):
        return copy.deepcopy(self)

    def save_attributes(self, node):
        pass

    def load_attributes(self, node):
        return False

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        my_id = self.state_array[0]
        other_id = self.state_array[1]

        if my_id > other_id:
            return BTNodeState.Success
        else:
            return BTNodeState.Failure


class BTTargetHolonomic(BTNode):
    def __init__(self, state_array, dt, id = 1):
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
        self.d_buffer = collections.deque(maxlen=4)
        self.i_sum = collections.deque(maxlen=20)

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



        k1 = 5
        k2 = 10
        # print "pos x {0} y {1}".format(current_pos.x, current_pos.y)
        # print "target x {0} y {1}".format(target.x, target.y)
        pos_to_target = GVector(current_pos, target)
        # print "current heading {0}".format(current_heading)
        error_angle = pos_to_target.angle() - current_heading

        # print "current_heading {0} targetAngle {1}".format(current_heading, error_angle)

        while error_angle > math.pi:
            error_angle -= 2 * math.pi

        while error_angle < -math.pi:
            error_angle += 2 * math.pi

        Vcmd = pos_to_target.norm() * 3.0
        # Vcmd = (posToTarget.norm()) * (1 - (fabs(targetAngle) / M_PI));
        # Vcmd = (1 - (abs(error_angle) / math.pi))
        # Vcmd = (v) - (abs(error_angle) / math.pi)
        if Vcmd > 1:
            Vcmd = 1
        if Vcmd < -1:
            Vcmd = 1




        workspace.set_par(0, Vcmd)
        workspace.set_par(1, 0.00)
        workspace.set_par(2, error_angle/math.pi)

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
