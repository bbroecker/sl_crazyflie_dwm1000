#!/usr/bin/env python
import copy
import xml.etree.ElementTree as ET

from bt_node import BTNode, BTNodeState
import bt_factory


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
