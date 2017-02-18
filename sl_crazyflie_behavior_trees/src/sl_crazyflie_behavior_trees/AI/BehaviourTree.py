#!/usr/bin/env python
import copy

import rospy
import xml.etree.ElementTree as ET

from sl_crazyflie_behavior_trees.bt_node import BTNode
from sl_crazyflie_behavior_trees.bt_workspace import BTWorkspace
import sl_crazyflie_behavior_trees.bt_factory


class AIIO:

    def __init__(self):
        self.input = []
        self.output = []

class BehaviorTree:

    def __init__(self, a1, a2=None, a3=None):
        other = None
        if a2 is not None and a3 is not None:
            self.m_root_node = None
            self.m_nIn = a1
            self.m_nOut = a2
            self.m_nPars = a3
            self.m_workspace = BTWorkspace(a1, a2, a3)
        elif a2 is not None and a3 is None:
            other = a2
            assert isinstance(other, BehaviorTree)
            self.m_root_node = a3
            self.m_nIn = other.m_nIn
            self.m_nOut = other.m_nOut
            self.m_workspace = copy.deepcopy(other.m_workspace)
        else:
            other = a2
            assert isinstance(other, BehaviorTree)
            self.m_root_node = copy.deepcopy(other.m_rootNode)
            self.m_nIn = other.m_nIn
            self.m_nOut = other.m_nOut
            self.m_workspace = copy.deepcopy(other.m_workspace)



    # def on_start():
    #     // Clean the BTWorkspace by constructing a new one - there may be better options!
    #     delete m_workspace;
    #     BTWorkspace *tmp = new BTWorkspace(*m_workspace);
    #     m_workspace = tmp;

    def save_data(self, node):
        assert isinstance(node, ET.Element)
        bt_node = ET.SubElement(node, "BehaviorTree")

        workspace_node = ET.SubElement(bt_node, "BTWorkspace")
        self.m_workspace.save(workspace_node);

        tree_node = ET.SubElement(bt_node, "Tree")
        self.m_root_node.save(tree_node)

    def save(self, file_name):

        doc = ET.ElementTree()
        # Save the genome-specific data
        self.save_data(doc)
        doc.write(file_name)

    def trigger(self, aiio_data):
        result = False
        assert isinstance(aiio_data, AIIO)
        for i in range(self.m_nIn):
            self.m_workspace.set_in(i, aiio_data.input[i])

        assert isinstance(self.m_workspace, BTWorkspace)
        # print "1. workspace {0}".format(self.m_workspace.m_workspaceData)
        if self.m_root_node is not None:
            self.m_root_node.reset_tick_marker()
            result = self.m_root_node.tick(self.m_workspace)
        # print "2. workspace {0}".format(self.m_workspace.m_workspaceData)

        aiio_data.output = []
        for i in range(self.m_nOut):
            aiio_data.output.append(self.m_workspace.get_out(i))

        return result

    def get_root(self):
        return self.m_root_node

    def set_workspace_par(self, index, value):
        self.m_workspace.set_par(index, value)

    def load_data(self, node):
        assert isinstance(node, ET.Element)

        tree_node = node.find("Tree")
        bt_node = tree_node[0]
        print "tree{0}  bt{1}".format(tree_node, bt_node)
        self.m_root_node = sl_crazyflie_behavior_trees.bt_factory.BTFactory.get_child_from_node(bt_node)
        return self.m_root_node is not None


    def get_num_in(self):
        return self.m_nIn

    def get_num_out(self):
        return self.m_nOut

    @staticmethod
    def load_from_node(node):
        assert isinstance(node, ET.Element)
        workspace_node = node.find("BTWorkspace")
        assert isinstance(workspace_node, ET.Element)
        numIn  = int(workspace_node.attrib["nIn"])
        numOut = int(workspace_node.attrib["nOut"])
        numPar = int(workspace_node.attrib["nPar"])
        print numIn, numOut, numPar

        bt = BehaviorTree(numIn, numOut, numPar)
        bt.load_data(node)

        if bt.m_root_node is None:
            return None
        else:
            return bt

    @staticmethod
    def load_from_file(file_name):

        tree = ET.parse(file_name)
        doc = tree.getroot()

        # bt_node = doc.find("BehaviorTree")
        # print bt_node
        return BehaviorTree.load_from_node(doc)