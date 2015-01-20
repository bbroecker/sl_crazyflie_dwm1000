#!/usr/bin/env python
import rospy

from pid import PidController
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import tf


class hoverController(object):
 
    def __init__(self):
        rospy.init_node('pid_hover')
                
        self.pub_cmd_vel = rospy.Publisher("hover/cmd_vel", Twist, queue_size=10)

        KP_thrust = 15000
        KI_thrust = 0
        KD_thrust = 0
        Ilimit_thrust = 0 
    
        KP_xy = 1
        KI_xy = 10
        KD_xy = 1
        Ilimit_xy = 40

        KP_yaw = 1
        KI_yaw = 0
        KD_yaw = 0
        Ilimit_yaw = 40

        self.paused = True

        self.nominal_thrust = 41120.2847958
        self.max_thrust = 60000
        self.min_thrust = 15000

        self.setpoint_pose_z = 0
        self.setpoint_pose_x = 0 
        self.setpoint_pose_y = 0
        self.setpoint_pose_yaw = 0

        self.lastCall_pose = rospy.get_time()

        self.lastZ = None

        self.pid_thrust = PidController(KP_thrust, KI_thrust, KD_thrust, Ilimit_thrust)
        self.pid_xy_x = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        self.pid_xy_y = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        self.pid_yaw = PidController(KP_yaw, KI_yaw, KD_yaw, Ilimit_yaw)

        print "started hover pid controller"
    
    def callback_pose(self, data):
        self.lastZ = data.pose.position.z
        if not self.paused:
            dt = float(rospy.get_time() - self.lastCall_pose)
            self.lastcall_pose = rospy.get_time()
            #better?
            #dt = float(data.header.stamp - self.lastCall_pose)
            #self.lastcall_pose = data.header.stamp

            error_x = data.pose.position.x - self.setpoint_pose_x
            error_y = data.pose.position.y - self.setpoint_pose_y
            quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,data.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(quat)
            error_yaw = euler[2] - self.setpoint_pose_yaw
            error_thrust = data.pose.position.z - self.setpoint_pose_z

            thrust = self.nominal_thrust + self.pid_thrust.update(error_thrust, dt)
            x = self.pid_xy_x.update(error_x, dt)
            y = self.pid_xy_y.update(error_y, dt)
            yaw = self.pid_yaw.update(error_yaw, dt)


            #print 'thrust error', error_thrust, 'thrust', thrust

            cmd_twist = Twist()
            cmd_twist.linear.z = thrust
            cmd_twist.linear.x = x
            cmd_twist.linear.y = y
            cmd_twist.angular.z = yaw
        
            self.pub_cmd_vel.publish(cmd_twist)

    def callback_toggle(self, data):
        self.paused = not self.paused
        self.lastCall_pose = rospy.get_time() #slightly fishy ;)
        return EmptyResponse()

    def callback_toggle_hover(self, data):
        #self.paused = not self.paused
        self.lastCall_pose = rospy.get_time() #slightly fishy ;)
        self.setpoint_pose_z = self.lastZ
        self.setpoint_pose_x = 0
        self.setpoint_pose_y = 0
        self.setpoin_pose_yaw = 0
        print '-'*10
        print self.paused, self.lastZ
        return EmptyResponse()
        
    def callback_setpoint(self, data):
        self.setpoint_pose_z = data.linear.z
        self.setpoint_pose_x = data.linear.x 
        self.setpoint_pose_y = data.linear.y
        self.setpoin_pose_yaw = data.angular.z        

    def listener(self):
        rospy.Subscriber('/Robot_1/pose', PoseStamped, self.callback_pose)
        rospy.Service('hover/toggle', Empty, self.callback_toggle)
        rospy.Service('hover/toggle_hover', Empty, self.callback_toggle_hover)
        rospy.Subscriber('hover/setpoint', Twist, self.callback_setpoint)
        print "Hover controller spinning"
        rospy.spin()    
        

if __name__ == '__main__':
    try:
        conti = hoverController()
        conti.listener()
    except rospy.ROSInterruptException: pass
