#!/usr/bin/env python
import copy

import xml.etree.ElementTree as ET

import rospy


class BTNodeState:
    Failure = 0
    Success = 1
    Running = 2


class BTNode:
    def __init__(self, node_type_name=None, is_composite=None):
        if is_composite is not None:
            self.m_node_type_name = node_type_name
            self.m_children = []
            self.m_is_composite = is_composite
            self.m_tick_counter = 0
            self.m_was_ticked = False
        else:
            other = node_type_name
            assert isinstance(other, BTNode)
            self.m_node_type_name = other.m_node_type_name
            self.m_children = copy.deepcopy(other.m_children)
            self.m_is_composite = other.m_is_composite
            self.m_tick_counter = 0
            self.m_was_ticked = False

    def get_type_name(self):
        return self.m_node_type_name

    def is_child(self, child):
        assert isinstance(child, BTNode)
        for c in self.m_children:
            if c == child:
                rospy.loginfo("is child works")
                return True
        return False

    def add_child(self, child):
        self.m_children.append(child)

    def pop_child(self, child):
        ret = None
        if child is None:
            ret = self.m_children.pop()
            #pop last child
            return ret;

        found_idx = None
        for idx, c in enumerate(self.m_children):
            if c is child:
                found_idx = idx
                break

        if found_idx is not None:
            ret = self.m_children.pop(found_idx)

        return ret

    def get_num_children(self):
        return len(self.m_children)

    def clone(self):
        raise NotImplementedError("Should have implemented this clone()")

    def tick_fcn(self, workspace):
        raise NotImplementedError("Should have implemented this tick_fcn()")

    def load_attributes(self, node):
        raise NotImplementedError("Should have implemented this load_attributes()")

    def save_attributes(self, node):
        raise NotImplementedError("Should have implemented this save_attributes()")

    def tick(self, workspace):
        self.m_was_ticked = True
        self.m_tick_counter += 1
        return self.tick_fcn(workspace)

    def get_tick_count(self):
        return self.m_tick_counter

    def reset_tick_count(self):
        self.m_tick_counter = 0

    def reset_tick_marker(self):
        self.m_was_ticked = False
        for child in self.m_children:
            child.reset_tick_marker()

    def save(self, node):
        this_node = ET.SubElement(node,self.m_nodeTypeName)
        self.saveAttributes(this_node)


    def load(self, node, node_idx):
        rospy.loginfo("weird logic!")
        node_index_counter = 0
        this_node = node.find(self.m_node_type_name)
        for n in this_node:

            if node_index_counter < node_idx:
                continue
            return self.loadAttributes(n);

        return False



    def get_child(self, index):
        return self.m_children[index]

    def getChildCount(self):
        return len(self.m_children)


