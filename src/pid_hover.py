#!/usr/bin/env python
import rospy

from pid import PidController
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from sl_crazyflie.cfg import pid_cfgConfig
import copy

#constants
# update 30Hz
RATE = 50
MAX_THRUST = 65000.0

def thrust_to_percent(thrust):
    return thrust/MAX_THRUST

def percent_to_thrust(percent):
    return percent * MAX_THRUST

class HoverController(object):
    def __init__(self):
        rospy.init_node('pid_hover')

        self.pub_cmd_vel = rospy.Publisher("hover/cmd_vel", Twist, queue_size=10)
        self.pub_target_height = rospy.Publisher("hover/target_height", Float32, queue_size=10)
        self.pub_target_climb_rate = rospy.Publisher("hover/target_climb_rate", Float32, queue_size=10)
        self.pub_current_climb_rate = rospy.Publisher("hover/current_climb_rate", Float32, queue_size=10)
        self.pub_thrust_percentage = rospy.Publisher("hover/thrust_percentage", Float32, queue_size=1)

        kp_thrust = 2
        kd_thrust = 0
        ki_thrust = 0

        kp_climb_rate = 10
        ki_climb_rate = 0
        kd_climb_rate = 0

        self.i_limit_thrust_min = -10000
        self.i_limit_thrust_max = 10000

        self.i_climb_rate_min = -10000
        self.i_climb_rate_max = 10000

        self.paused = True

        self.max_percentage = 0.9

        self.nominal_thrust = 0.7
        self.max_thrust = self.max_percentage
        self.min_thrust = 0.25

        self.target_altitude = 0
        self.max_altitude_error = 0

        self.target_pose_x = 0
        self.target_pose_y = 0
        self.target_pose_yaw = 0

        self.last_update_time = rospy.get_time()

        self.last_pose_msg = None
        self.prev_altitude = None

        self.pid_thrust = PidController(kp_thrust, ki_thrust, kd_thrust, i_min=self.i_limit_thrust_min,
                                        i_max=self.i_limit_thrust_max)

        self.pid_climb_rate = PidController(kp_climb_rate, ki_climb_rate, kd_climb_rate,
                                            i_min=self.i_climb_rate_min, i_max=self.i_climb_rate_max)

        self.error_thrust = 0

        # self.pid_xy_x = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        # self.pid_xy_y = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        # self.pid_yaw = PidController(KP_yaw, KI_yaw, KD_yaw, Ilimit_yaw)

        self.dyn_server = Server(pid_cfgConfig, self.callback_dynreconf)
        rospy.Subscriber('/Robot_1/pose', PoseStamped, self.callback_optitrack_pose)

        rospy.Service('hover/stop', Empty, self.callback_stop)
        rospy.Service('hover/start_hold_position', Empty, self.callback_hold_position)
        # toDo make service call for target pose
        # rospy.Subscriber('hover/setpoint', Twist, self.callback_set_target_pose)

        rospy.loginfo("Started hover pid controller")

    def callback_optitrack_pose(self, data):
        self.last_pose_msg = data

    def spin(self):
        rospy.loginfo("Hover controller spinning")
        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.update(copy.copy(self.last_pose_msg))
            r.sleep()

    def update(self, current_pose_msg):
        if not self.paused:
            current_time = rospy.get_time()
            dt = float(current_time - self.last_update_time)
            self.last_update_time = current_time

            #toDo x,y and yaw
            #error_x = current_pose_msg.pose.position.x - self.target_pose_x
            #error_y = current_pose_msg.pose.position.y - self.target_pose_y
            #quat = [current_pose_msg.pose.orientation.x, current_pose_msg.pose.orientation.y,
            #current_pose_msg.pose.orientation.z, current_pose_msg.pose.orientation.w]
            #euler = tf.transformations.euler_from_quaternion(quat)
            #error_yaw = euler[2] - self.target_pose_yaw

            # PID CHANGE CRAZYFLIE
            # error_thrust = self.setpoint_pose_z - data.pose.position.z

            #ALTITUDE TO CLIMB_RATE PID
            current_altitude = current_pose_msg.pose.position.z
            current_target_altitude_error = self.target_altitude - current_altitude

            if current_target_altitude_error > self.max_altitude_error:
                current_target_altitude_error = self.max_altitude_error

            if current_target_altitude_error < -self.max_altitude_error:
                current_target_altitude_error = -self.max_altitude_error

            target_climb_rate = self.pid_climb_rate.update(current_target_altitude_error, dt)

            current_climb_rate = (current_altitude - self.prev_altitude) / dt
            #current_climb_rate_error = target_climb_rate - current_climb_rate

            climb_rate_error = target_climb_rate - current_climb_rate

            #climb_rate to thrust pid
            thrust = self.pid_thrust.update(climb_rate_error, dt)
            current_thrust_cmd = self.nominal_thrust + thrust

            if current_thrust_cmd < self.min_thrust:
                current_thrust_cmd = self.min_thrust
            elif current_thrust_cmd > self.max_thrust:
                current_thrust_cmd = self.max_thrust

            #x = self.pid_xy_x.update(error_x, dt)
            #y = self.pid_xy_y.update(error_y, dt)
            #yaw = self.pid_yaw.update(error_yaw, dt)

            cmd_twist = Twist()
            cmd_twist.linear.z = percent_to_thrust(current_thrust_cmd)
            #cmd_twist.linear.x = x
            #cmd_twist.linear.y = y
            #sd_twist.angular.z = yaw
            #self.nominal_thrust = cmd_twist.linear.z

            self.prev_altitude = current_altitude
            # publish velocity
            self.pub_cmd_vel.publish(cmd_twist)

            #publish debug msgs
            self.pub_target_height.publish(Float32(self.target_altitude))
            self.pub_current_climb_rate.publish(Float32(current_climb_rate))
            self.pub_target_climb_rate.publish(Float32(target_climb_rate))
            self.pub_thrust_percentage.publish(Float32(current_thrust_cmd))

    def callback_stop(self, req):
        self.paused = True
        return EmptyResponse()

    def callback_hold_position(self, req):
        # self.paused = not self.paused
        self.last_update_time = rospy.get_time()
        self.prev_altitude = self.last_pose_msg.pose.position.z
        self.target_altitude = self.prev_altitude
        self.target_pose_x = 0
        self.target_pose_y = 0
        self.target_pose_yaw = 0
        self.pid_thrust.reset_pid()
        self.paused = False

        rospy.loginfo("Holding position")

        return EmptyResponse()

    def callback_set_target_pose(self, data):
        self.target_altitude = data.linear.z
        self.target_pose_x = data.linear.x
        self.target_pose_y = data.linear.y
        self.target_pose_yaw = data.angular.z

    def callback_dynreconf(self, config, level):

        kp_thrust = config["KP_thrust"]
        ki_thrust = config["KI_thrust"]
        kd_thrust = config["KD_thrust"]

        kp_climb_rate = config['KP_climb_rate']
        ki_climb_rate = config['KI_climb_rate']
        kd_climb_rate = config['KD_climb_rate']
        self.max_altitude_error = config['max_altitude_error']

        self.nominal_thrust = config["Nom_thrust"]
        self.pid_thrust.set_pid_parameters(kp_thrust, ki_thrust, kd_thrust)
        self.pid_climb_rate.set_pid_parameters(kp_climb_rate, ki_climb_rate, kd_climb_rate)

        rospy.loginfo("kp %f ki %f kd %f", kp_thrust, ki_thrust, kd_thrust)
        # self.pid_xy_x = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        # self.pid_xy_y = PidController(KP_xy, KI_xy, KD_xy, Ilimit_xy)
        #self.pid_yaw = PidController(KP_yaw, KI_yaw, KD_yaw, Ilimit_yaw)
        #print config
        #print level
        return config


if __name__ == '__main__':
    try:
        cont = HoverController()
        cont.spin()
    except rospy.ROSInterruptException:
        pass
