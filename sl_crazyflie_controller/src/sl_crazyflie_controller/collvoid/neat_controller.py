#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from collvoid_interface import CollvoidInterface
import copy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64MultiArray

class NEATController(CollvoidInterface):

    def __init__(self):

        print("NEATController!")

        #Implements interface
        CollvoidInterface.__init__(self)

        #self.update_rate
        self.last_velocity = None
        #self.last_update = None
        self.current_pose = None
        self.goal_pose = None
        self.enabled = False

        #rospy.Subscriber("/NEAT_outputs", Float64MultiArray, self.network_outputs_received)
        #rospy.Subscriber("/crazyflie1/NEAT_outputs", Float64MultiArray, self.network_outputs_received)
        rospy.Subscriber("NEAT_outputs", Float64MultiArray, self.network_outputs_received)

        #Services to start and stop the network control
        self.start_nn_controller_srvs = rospy.Service("start_nn_controller", Empty, self.start_callback)
        self.stop_nn_controller_srvs = rospy.Service("stop_nn_controller", Empty, self.stop_callback)

        rospy.Subscriber("goal_pose", PoseStamped, self.goal_pose_callback)


    def calculate_velocity(self, current_target_velocity):

        #print("Calculating velocity")

        #Update dt
        # if self.last_update is None:        #Just for first time step I think
        #     dt = 1.0 / self.update_rate
        # else:
        #     dt = (rospy.Time.now() - self.last_update).to_sec()

        #Check last velocity is not none
        #if self.last_velocity is None:
        self.last_velocity = copy.deepcopy(current_target_velocity)

        #Set network output as speeds
        self.last_velocity.x = self.x_speed
        self.last_velocity.y = self.y_speed
        #self.last_velocity.z = 0.0

        #print(self.last_velocity)

        return self.last_velocity


    def is_active(self):

        #print("Am I active?")

        return self.enabled

    #Called when 'start_nn_controller' srv receives something
    #Starts the controller
    def start_callback(self, req):
        print "Start Network"
        self.enabled = True
        return EmptyResponse()

    #Called when 'top_nn_controller' srv receives something
    #Stops the controller
    def stop_callback(self, req):
        print "Stop Network"
        self.enabled = False
        #self.goal_pose = None
        #self.last_update = None
        #self.avg_buffer = collections.deque(maxlen=self.avg_buffer_size)
        #self.nn.reset()
        return EmptyResponse()

    def network_outputs_received(self, input):
        #print("Network output received")
        #print(input.data)
        self.x_speed = input.data[0]
        self.y_speed = input.data[1]

    #Called when topic 'goal_pose' receives something
    #Just sets the goal pose in this class
    def goal_pose_callback(self, pose):
        #print(pose)
        self.goal_pose = pose

    def update_cf_pose(self, pose):
        #print("Update current pose")
        self.current_pose = pose
