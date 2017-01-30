#!/usr/bin/env python
import rospy
import xml.etree.ElementTree as ET


class BTWorkspace:

    def __init__(self, a1, a2 = None, a3 = None):
        self.m_numIn = None
        self.m_numOut = None
        self.m_numPar = None
        self.m_workspaceData = None
        if isinstance(a1, BTWorkspace):
            other = a1
            self.init_workspace(other.m_numIn, other.m_numOut, other.m_numPar)
        else:
            self.init_workspace(a1, a2, a3)
            # def init_workspace(self, numIn, numOut, numPars):

    def init_workspace(self,  numIn, numOut, numPars):
        self.m_numIn = numIn
        self.m_numOut = numOut
        self.m_numPar = numPars
        #init array with size (numIn + numPars + numOut)
        self.m_workspaceData = [0] * (numIn + numPars + numOut)

    def set_in(self, pos, val):
        if pos > self.m_numIn:
            rospy.logerr("BTWorkspace::setInput: index out of range")
        self.m_workspaceData[pos] = val

    def get_out(self, pos):
        if pos > self.m_numOut:
            rospy.logerr("BTWorkspace::getOutput: index out of range")
        return self.m_workspaceData[self.m_numIn+pos]

    def set_par(self, pos, val):
        if pos > (self.m_numOut + self.m_numPar):
            rospy.logerr("BTWorkspace::setPar: index out of range")
        self.m_workspaceData[self.m_numIn + pos] = val;

    def inc_par(self, pos, val):
        if pos > self.m_numOut + self.m_numPar:
            rospy.logerr("BTWorkspace::incPar: index out of range")
        self.m_workspaceData[self.m_numIn + pos] += val

    def get_par(self, pos):
        if pos >= len(self.m_workspaceData):
            rospy.logerr("BTWorkspace::getPar: index out of range")
        return self.m_workspaceData[pos]

    def get_workspace_size(self):
        return len(self.m_workspaceData)

    def get_num_in(self):
        return self.m_numIn

    def get_num_read(self):
        return self.m_numIn + self.m_numOut + self.m_numPar

    def get_num_write(self):
        return self.m_numOut + self.m_numPar

    def getNumPar(self):
        return self.m_numPar

    def save(self, node):
        assert isinstance(node, ET.Element)
        node.attrib["nIn"] = self.m_numIn
        node.attrib["nOut"] = self.m_numOut
        node.attrib["nPar"] = self.m_numPar

