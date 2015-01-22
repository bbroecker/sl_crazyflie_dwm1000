#!/usr/bin/env python
import rospy

from pid import PidController
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from sl_crazyflie.cfg import pid_cfgConfig
import tf
import copy


class hoverController(object):
 
    def __init__(self):
        rospy.init_node('pid_hover')
                
        self.pub_cmd_vel = rospy.Publisher("hover/cmd_vel", Twist, queue_size=10)
        self.pub_setpoint = rospy.Publisher("hover/setpoint_z", Float32, queue_size=10)
        self.pub_setpointdz = rospy.Publisher("hover/setpoint_dz", Float32, queue_size=10)
        self.pub_currentdz = rospy.Publisher("hover/current_dz", Float32, queue_size=10)


        AFFE = 100 # 150
        KP_thrust = 20 * AFFE #30
        KD_thrust = 500 * AFFE #500
        KI_thrust = 30 * AFFE #40

        self.Ilimit_thrust = 10000
    
        KP_xy = 1
        KI_xy = 10
        KD_xy = 1
        Ilimit_xy = 40

        KP_yaw = 1
        KI_yaw = 0
        KD_yaw = 0
        Ilimit_yaw = 40

        self.paused = True

        self.max_percentage = 1.0

        self.nominal_thrust = 45000
        self.max_thrust = 60000 * self.max_percentage
        self.min_thrust = 15000

        self.setpoint_pose_z = 0
        self.setpoint_pose_x = 0 
        self.setpoint_pose_y = 0
        self.setpoint_pose_yaw = 0

        self.lastCall_pose = rospy.get_time()

        self.lastZ = None
        self.cont = None
        self.last_msg = None
        self.prev_height = None


        self.pid_thrust = PidController(KP_thrust, KI_thrust, KD_thrust, self.Ilimit_thrust)
        self.pid_xy_x = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        self.pid_xy_y = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        self.pid_yaw = PidController(KP_yaw, KI_yaw, KD_yaw, Ilimit_yaw)
        self.error_thrust = 0

        print "started hover pid controller"
    
    def callback_pose(self, data):
        self.lastZ = data.pose.position.z
        self.last_msg = data

    def update(self, data):
        if not self.paused:
            dt = float(rospy.get_time() - self.lastCall_pose)
            self.lastCall_pose = rospy.get_time()
            #better?
            #dt = float(data.header.stamp - self.lastCall_pose)
            #self.lastcall_pose = data.header.stamp

            error_x = data.pose.position.x - self.setpoint_pose_x
            error_y = data.pose.position.y - self.setpoint_pose_y
            quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(quat)
            error_yaw = euler[2] - self.setpoint_pose_yaw
            #PID CHANGE CRAZYFLIE
           # error_thrust = self.setpoint_pose_z - data.pose.position.z
            climbrate = (self.prev_height - data.pose.position.z)*0.1
            #error_thrust = self.prev_height - data.pose.position.z

            error_thrust = (self.setpoint_pose_z - data.pose.position.z)*0.05
            thrust_cap = 0.05
            if error_thrust > thrust_cap:
                error_thrust = thrust_cap
            elif error_thrust < -thrust_cap:
                error_thrust = -thrust_cap

            error_thrust = climbrate - error_thrust

            self.pub_currentdz.publish(Float32(data.pose.position.z - self.prev_height))
            self.pub_setpointdz.publish(Float32(error_thrust))


            self.prev_height = data.pose.position.z
            #-----
            #old
            #error_thrust = data.pose.position.z - self.setpoint_pose_z
            #-----

            thrust = self.nominal_thrust + self.pid_thrust.update(error_thrust, dt)
            if thrust < self.min_thrust:
                thrust = self.min_thrust
            elif thrust > self.max_thrust:
                thrust = self.max_thrust
                    
            #x = self.pid_xy_x.update(error_x, dt)
            #y = self.pid_xy_y.update(error_y, dt)
            #yaw = self.pid_yaw.update(error_yaw, dt)


            print 'thrust error', error_thrust, 'thrust', thrust, 'nominal thrust', self.nominal_thrust

            cmd_twist = Twist()
            cmd_twist.linear.z = thrust
            #cmd_twist.linear.x = x
            #cmd_twist.linear.y = y
            #sd_twist.angular.z = yaw
            #self.nominal_thrust = cmd_twist.linear.z

            self.pub_cmd_vel.publish(cmd_twist)
            msg = Float32()
            msg.data = self.setpoint_pose_z
            self.pub_setpoint.publish(msg)


    def callback_toggle(self, data):
        self.paused = not self.paused
        self.lastCall_pose = rospy.get_time() #slightly fishy ;)
        return EmptyResponse()

    def callback_toggle_hover(self, data):
        #self.paused = not self.paused
        self.lastCall_pose = rospy.get_time() #slightly fishy ;)
        self.prev_height = self.lastZ
        self.setpoint_pose_z = self.lastZ
        self.setpoint_pose_x = 0
        self.setpoint_pose_y = 0
        self.setpoin_pose_yaw = 0
        self.pid_thrust.zero()
        print '-'*10
        print self.paused, self.lastZ
        return EmptyResponse()
        
    def callback_setpoint(self, data):
        self.setpoint_pose_z = data.linear.z
        self.setpoint_pose_x = data.linear.x 
        self.setpoint_pose_y = data.linear.y
        self.setpoin_pose_yaw = data.angular.z        

    
    def callback_dynreconf(self, config, level):
        #rospy.loginfo("""Reconfiugre Request: {int_param}, {double_param},\ 
        #{str_param}, {bool_param}, {size}""".format(**config))
        KP_thrust = config["KP_thrust"]
        KI_thrust = config["KI_thrust"]
        KD_thrust = config["KD_thrust"]
        self.nominal_thrust = config["Nom_thrust"]
        self.pid_thrust = PidController(KP_thrust, KI_thrust, KD_thrust, self.Ilimit_thrust)



        #self.pid_xy_x = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        #self.pid_xy_y = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        #self.pid_yaw = PidController(KP_yaw, KI_yaw, KD_yaw, Ilimit_yaw)

        #print config
        #print level
        return config

    def listener(self):
        self.dyn_server = Server(pid_cfgConfig, self.callback_dynreconf)
        rospy.Subscriber('/Robot_1/pose', PoseStamped, self.callback_pose)
        rospy.Service('hover/toggle', Empty, self.callback_toggle)
        rospy.Service('hover/toggle_hover', Empty, self.callback_toggle_hover)
        rospy.Subscriber('hover/setpoint', Twist, self.callback_setpoint)
        print "Hover controller spinning"

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update(copy.copy(self.last_msg))
            r.sleep()
        

if __name__ == '__main__':
    try:
        conti = hoverController()
        conti.listener()
    except rospy.ROSInterruptException: pass
