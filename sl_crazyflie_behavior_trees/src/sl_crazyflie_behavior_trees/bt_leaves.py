#!/usr/bin/env python
import copy
import xml.etree.ElementTree as ET
from bt_node import BTNode, BTNodeState
from bt_workspace import BTWorkspace


class BTConditionC(BTNode):

    def __init__(self, other=None):
        if other is None:
            BTNode.__init__(self, "BTConditionC", False)
            self.m_compare_to = 0
            self.m_condition_type_more_than = False
            self.m_data_index = 0
        else:
            assert isinstance(other, BTConditionC)
            BTNode.__init__(self, other)
            self.m_compare_to = other.m_compare_to
            self.m_condition_type_more_than = other.m_condition_type_more_than
            self.m_data_index = other.m_data_index


    def clone(self):
        return copy.deepcopy(self)

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        if self.m_condition_type_more_than:
            if workspace.get_par(self.m_data_index) > self.m_compare_to:
                return BTNodeState.Success
        elif workspace.get_par(self.m_data_index) < self.m_compare_to:
            return BTNodeState.Success

        return BTNodeState.Failure


    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        if self.m_condition_type_more_than:
            node.attrib["compareType"] = "moreThan"
        else:
            node.attrib["compareType"] = "lessThan"

        node.attrib["dataIndex"] = self.m_data_index
        node.attrib["compareTo"] = self.m_compare_to

    def load_attributes(self, node):
        cmp_type = node.attrib["compareType"]
        print cmp_type
        if cmp_type == "moreThan":
            self.m_condition_type_more_than = True
        elif cmp_type == "lessThan":
            self.m_condition_type_more_than = False
        else:
            return False

        self.m_data_index = int(node.attrib["dataIndex"])
        self.m_compare_to = float(node.attrib["compareTo"])

        return True



class BTConditionP(BTNode):

    def __init__(self, other=None):
        if other is None:
            BTNode.__init__(self, "BTConditionP", False)
            self.m_condition_type_more_than = False
            self.m_data_index_first = 0
            self.m_data_index_second = 0

        else:
            assert isinstance(other, BTConditionP)
            BTNode.__init__(self, other)
            self.m_condition_type_more_than = other.m_condition_type_more_than
            self.m_data_index_first = other.m_data_index_first
            self.m_data_index_second = other.m_data_index_second

    def clone(self):
        return copy.deepcopy(self)

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        if self.m_condition_type_more_than:
            if workspace.get_par(self.m_data_index_first) > workspace.get_par(self.m_data_index_second):
                return BTNodeState.Success
        elif workspace.get_par(self.m_data_index_first) < workspace.get_par(self.m_data_index_second):
            return BTNodeState.Success

        return BTNodeState.Failure

    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        if self.m_condition_type_more_than:
            node.attrib["compareType"] = "moreThan"
        else:
            node.attrib["compareType"] = "lessThan"

        node.attrib["dataIndexFirst"] = self.m_data_index_first
        node.attrib["dataIndexSecond"] = self.m_data_index_second

    def load_attributes(self, node):
        cmp_type = node.attrib["compareType"]
        if cmp_type == "moreThan":
            self.m_condition_type_more_than = True
        elif cmp_type == "lessThan":
            self.m_condition_type_more_than = False
        else:
            return False

        self.m_data_index_first = int(node.attrib["dataIndexFirst"])
        self.m_data_index_second = int(node.attrib["dataIndexSecond"])

        return True



class BTWhileConditionC(BTNode):

    def __init__(self, a1=None):
        if a1 is None:
            BTNode.__init__(self, "BTWhileConditionC", False)
            self.m_compare_to = 0
            self.m_condition_type_more_than = False
            self.m_data_index = 0

        else:
            other = a1
            assert isinstance(other, BTWhileConditionC)
            BTNode.__init__(self, other)
            self.m_compare_to = other.m_compare_to
            self.m_condition_type_more_than = other.m_condition_type_more_than
            self.m_data_index = other.m_data_index

    def clone(self):
        return copy.deepcopy(self)

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        if self.m_condition_type_more_than:
            if workspace.get_par(self.m_data_index) > self.m_compare_to:
                return BTNodeState.Running
        elif workspace.get_par(self.m_data_index) < self.m_compare_to:
            return BTNodeState.Running

        return BTNodeState.Success

    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        if self.m_condition_type_more_than:
            node.attrib["compareType"] = "moreThan"
        else:
            node.attrib["compareType"] = "lessThan"

        node.attrib["dataIndex"] = self.m_data_index
        node.attrib["compareTo"] = self.m_compare_to

    def load_attributes(self, node):
        cmp_type = node.attrib["compareType"]
        if cmp_type == "moreThan":
            self.m_condition_type_more_than = True
        elif cmp_type == "lessThan":
            self.m_condition_type_more_than = False
        else:
            return False

        self.m_data_index = int(node.attrib["dataIndex"])
        self.m_compare_to = int(node.attrib["compareTo"])

        return True


class BTWhileConditionP(BTNode):

    def __init__(self, a1=None):
        if a1 is None:
            BTNode.__init__(self, "BTWhileConditionP", False)
            self.m_condition_type_more_than = False
            self.m_data_index_first = 0
            self.m_data_index_second = 0
        else:
            other = a1
            assert isinstance(other, BTWhileConditionP)
            BTNode.__init__(self, other)
            self.m_condition_type_more_than = other.m_condition_type_more_than
            self.m_data_index_first = other.m_data_index_first
            self.m_data_index_second = other.m_data_index_second

    def clone(self):
        return copy.deepcopy(self)

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        if self.m_condition_type_more_than:
            if workspace.get_par(self.m_data_index_first) > workspace.get_par(self.m_data_index_second):
                return BTNodeState.Running
        elif workspace.get_par(self.m_data_index_first) < workspace.get_par(self.m_data_index_second):
            return BTNodeState.Running

        return BTNodeState.Success

    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        if self.m_condition_type_more_than:
            node.attrib["compareType"] = "moreThan"
        else:
            node.attrib["compareType"] = "lessThan"

        node.attrib["dataIndexFirst"] = self.m_data_index_first
        node.attrib["dataIndexSecond"] = self.m_data_index_second

    def load_attributes(self, node):
        cmp_type = node.attrib["compareType"]
        if cmp_type == "moreThan":
            self.m_condition_type_more_than = True
        elif cmp_type == "lessThan":
            self.m_condition_type_more_than = False
        else:
            return False

        self.m_data_index_first = int(node.attrib["dataIndexFirst"])
        self.m_data_index_second = int(node.attrib["dataIndexSecond"])

        return True


class BTSetC(BTNode):
    def __init__(self, a1=None, a2 = None):
        if a1 is None and a2 is None:
            BTNode.__init__(self, "BTSetC", False)
            self.m_data_index = 0
            self.m_value = 0
            self.m_set_type_absolute = True
        elif a2 is None:
            other = a1
            assert isinstance(other, BTSetC)
            BTNode.__init__(self, other)
            self.m_data_index = other.m_data_index
            self.m_value = other.m_value
            self.m_set_type_absolute = other.m_set_type_absolute
        else:
            data_index = a1
            value = a2
            BTNode.__init__(self, "BTSetC", False)
            self.m_data_index = data_index
            self.m_value = value
            self.m_set_type_absolute = True

    def clone(self):
        return copy.deepcopy(self)

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        if self.m_set_type_absolute:
            workspace.set_par(self.m_data_index, self.m_value)
        else:
            workspace.inc_par(self.m_data_index, self.m_value)

        return BTNodeState.Success

    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        if not self.m_set_type_absolute:
            node.attrib["setType"] = "incremental"

        node.attrib["dataIndex"] = self.m_data_index
        node.attrib["value"] = self.m_value

    def load_attributes(self, node):
        print "load_attributes"
        assert isinstance(node, ET.Element)
        cmp_type = "absolute"
        if "setType" in node.attrib:
            cmp_type = node.attrib["setType"]

        if cmp_type is "absolute":
            self.m_set_type_absolute = True
        elif cmp_type is "incremental":
            self.m_set_type_absolute = False
        else:
            return False

        self.m_data_index = int(node.attrib["dataIndex"])
        self.m_value = float(node.attrib["value"])

        return True


class BTSetP(BTNode):

    def __init__(self, other = None):
        if other is None:
            BTNode.__init__(self, "BTSetP", False)
            self.m_data_index = 0
            self.m_value = 0
            self.m_set_type_absolute = True
        else:
            assert isinstance(other, BTSetP)
            BTNode.__init__(self, other)
            self.m_data_index = other.m_data_index
            self.m_value = other.m_value
            self.m_set_type_absolute = other.m_set_type_absolute

    def clone(self):
        return copy.deepcopy(self)

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        if self.m_set_type_absolute:
            workspace.set_par(self.m_data_index, workspace.get_par(self.m_value))
        else:
            workspace.inc_par(self.m_data_index, workspace.get_par(self.m_value))

        return BTNodeState.Success


    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        if not self.m_set_type_absolute:
            node.attrib["setType"] = "incremental"

        node.attrib["dataIndex"] = self.m_data_index
        node.attrib["value"] = self.m_value


    def load_attributes(self, node):
        assert isinstance(node, ET.Element)
        cmp_type = "absolute"
        if "setType" in node.attrib:
            cmp_type = node.attrib["setType"]
        #cmp_type = node.attrib["setType"].as_string("absolute");

        if cmp_type == "absolute":
            self.m_set_type_absolute = True
        elif cmp_type == "incremental":
            self.m_set_type_absolute = False
        else:
            return False

        self.m_data_index = int(node.attrib["dataIndex"])
        self.m_value = float(node.attrib["value"])

        return True


class BTProdP(BTNode):
    def __init__(self, other=None):
        if other is None:
            BTNode.__init__("BTProdP", False)
            self.m_data_index1 = 0
            self.m_data_index2 = 0
            self.m_data_to = 0

        else:
            assert isinstance(other, BTProdP)
            BTNode.__init__(self, other)
            self.m_data_index1 = other.m_data_index1
            self.m_data_index2 = other.m_data_index2
            self.m_data_to = other.m_data_to

    def clone(self):
        return copy.deepcopy(self)

    def tick_fcn(self, workspace):
        assert isinstance(workspace, BTWorkspace)
        if self.m_dataTo == -1:
            workspace.set_par(self.m_data_index1 - workspace.get_num_in(), workspace.get_par(self.m_data_index1) * workspace.get_par(self.m_data_index1))
        else:
            workspace.set_par(self.m_data_to, workspace.get_par(self.m_data_index1) * workspace.get_par(self.m_data_index2));

        return BTNodeState.Success

    def save_attributes(self, node):
        assert isinstance(node, ET.Element)
        node.attrib["dataIndex1"] = self.m_data_index1
        node.attrib["dataIndex2"] = self.m_data_index2

        if self.m_data_index1 is not self.m_data_to:
            node.attrib["dataTo"] = self.m_data_to

    def load_attributes(self, node):
        assert isinstance(node, ET.Element)

        self.m_data_index1 = node.attrib["dataIndex1"]
        self.m_data_index2 = node.attrib["dataIndex2"]
        self.m_data_to = -1
        if "dataTo" in node.attrib:
            self.m_data_to = int(node.attrib["dataTo"])
        # self.m_data_to = node.attrib["dataTo").as_int(-1);

        return True







