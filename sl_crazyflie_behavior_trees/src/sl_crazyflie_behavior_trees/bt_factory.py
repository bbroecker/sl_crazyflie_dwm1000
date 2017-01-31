#!/usr/bin/env python
import xml.etree.ElementTree as ET

import rospy

import bt_composites
import bt_leaves


class BTFactory:
    @staticmethod
    def get_child_from_node(node):
        assert isinstance(node, ET.Element)
        type_name = node.tag


        return_node = None
        if type_name == "BTSelect":
            return_node = bt_composites.BTSelect()

        elif type_name == "BTSequence":
            return_node = bt_composites.BTSequence()

        elif type_name == "BTWhile":
            return_node = bt_composites.BTWhile()

        elif type_name == "BTConditionC":
            return_node = bt_leaves.BTConditionC()

        elif type_name == "BTConditionP":
            return_node = bt_leaves.BTConditionP()

        elif type_name == "BTWhileConditionP":
            return_node = bt_leaves.BTWhileConditionP()

        elif type_name == "BTWhileConditionC":
            return_node = bt_leaves.BTWhileConditionC()

        elif type_name == "BTSetC":
            return_node = bt_leaves.BTSetC()

        elif type_name == "BTSetP":
            return_node = bt_leaves.BTSetP()

            # //    else if (strcmp(typeName, "BTDelay") == 0)
            # //        returnNode = new BTDelay();
            #
            # //    else if (strcmp(typeName, "BTTurnAround") == 0)
            # //        returnNode = new BTTurnAround();
            #
            # //    else if (strcmp(typeName, "BTStop") == 0)
            # //        returnNode = new BTStop();
            #
            # //    else if (strcmp(typeName, "BTSetProportional") == 0)
            # //        returnNode = new BTSetProportional();
            #
            # //    else if (strcmp(typeName, "BTPeakDetector") == 0)
            # //        returnNode = new BTPeakDetector();

        elif type_name == "BTProdP":
            return_node = bt_leaves.BTProdP()

        if return_node == None:
            return return_node

        if not return_node.load_attributes(node):
            return_node = None

        return return_node

