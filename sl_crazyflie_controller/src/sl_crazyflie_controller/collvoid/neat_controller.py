#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from collvoid_interface import CollvoidInterface
import copy

class NEATController(CollvoidInterface):

    def __init__(self):

        print("NEATController!")

        #Implements interface
        CollvoidInterface.__init__(self)

        #self.update_rate
        self.last_velocity = None
        self.last_update = None
        self.goal_pose = None

        #Subscribing to a topic called "goal_pose"
        rospy.Subscriber("goal_pose", PoseStamped, self.goal_pose_callback)


    def calculate_velocity(self, current_target_velocity):

        print("Calculating velocity")

        #Update dt
        # if self.last_update is None:        #Just for first time step I think
        #     dt = 1.0 / self.update_rate
        # else:
        #     dt = (rospy.Time.now() - self.last_update).to_sec()

        #Check last velocity is not none
        #if self.last_velocity is None:
        #    self.last_velocity = copy.deepcopy(current_target_velocity)

        # if (dt >= 1.0 / self.update_rate) and self.goal_pose is not None:
        #     self.last_velocity.x = 0.0
        #     self.last_velocity.y = 0.0
        #     self.last_velocity.z = 0.0

        #if self.goal_pose is not None:
        #    self.last_velocity.x = 0.0
        #    self.last_velocity.y = 0.0
        #    self.last_velocity.z = 0.0

        #print(self.last_velocity)

        # self.last_velocity.x = 0.0
        # self.last_velocity.y = 0.0
        # self.last_velocity.z = 0.0

        print(current_target_velocity)
        #return self.last_velocity
        return current_target_velocity

        #TESTING

        # vel = copy.deepcopy(current_target_velocity)
        # print(vel)
        #
        # vel.x = 0.0
        # vel.y = 0.0
        # vel.z = 0.0

        #return vel

    def is_active(self):

        print("Is active")

        return True

    #Called when topic 'goal_pose' receives something
    #Just sets the goal pose in this class
    def goal_pose_callback(self, pose):
        self.goal_pose = pose
