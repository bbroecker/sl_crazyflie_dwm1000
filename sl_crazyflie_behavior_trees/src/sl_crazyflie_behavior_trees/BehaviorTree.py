#!/usr/bin/env python
import copy

import rospy
import xml.etree.ElementTree as ET

from sl_crazyflie_behavior_trees.src.sl_crazyflie_behavior_trees.bt_workspace import BTWorkspace

class AIIO:

    def __init__(self):
        self.input = []
        self.output = []

class BehaviorTree:

    def __init__(self, numIn, numOut, numPars):
        self.m_root_node = None
        self.m_nIn = numIn
        self.m_nOut = numOut
        self.m_workspace = BTWorkspace(numIn,numOut,numPars)

    def __init__(self, other):
        assert isinstance(other, BehaviorTree)
        self.m_root_node = copy.deepcopy(other.m_rootNode)
        self.m_nIn = other.m_nIn
        self.m_nOut = other.m_nOut
        self.m_workspace = copy.deepcopy(other.m_workspace)

    def __init__(self, other, root_node):
        self.m_root_node = root_node
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
        assert isinstance(aiio_data, AIIO)
        for i in range(self.m_nIn):
            self.m_workspace.setIn(i, aiio_data.input[i])

        self.m_root_node.resetTickMarker()
        self.m_root_node.tick(self.m_workspace)

        aiio_data.output = []
        for i in range(self.m_nOut):
            aiio_data.output.append(self.m_workspace.get_out(i))

    def get_root(self):
        return self.m_root_node

    def set_workspace_par(self, index, value):
        self.m_workspace.set_par(index, value)

    def load_data(self, node):
        assert isinstance(node, ET.Element)

        tree_node = node.find("Tree");
        bt_node = tree_node[0]

        m_rootNode = BTNode.get_child_from_node(bt_node);

        return m_rootNode is not None


    def load_from_node(self, node):
        assert isinstance(node, ET.Element)
        workspace_node = node.find("BTWorkspace");
        assert isinstance(workspace_node, ET.Element)
        numIn  = workspace_node.attrib["nIn"]
        numOut = workspace_node.attrib["nOut"]
        numPar = workspace_node.attrib["nPar"]

        bt = BehaviorTree(numIn, numOut, numPar);
        bt.load_data(node)

        if bt.m_rootNode is None:
            return None
        else:
            return bt


    def load_from_file(self, file_name):

        tree = ET.parse(file_name)
        doc = tree.getroot()

        bt_node = doc.find("BehaviorTree")
        return self.load_from_node(bt_node)

