#!/usr/bin/env python
import copy
import math

import rospy
from geometry_msgs.msg import PoseStamped
from sl_crazyflie_msgs.msg import Velocity, Obstacle
from std_srvs.srv import Empty, EmptyResponse
from tf import transformations

from collvoid_interface import CollvoidInterface
from sl_crazyflie_controller.collvoid.simple_collvoid import ObstacleTime
from sl_crazyflie_controller.pid_controller.pid import quaternon_from_yaw
from sl_crazyflie_controller.sensor.dwm1000_distance_sensor import DWM10000DistanceSensor
from tf_nn_sim.Config.NNConfigAngle import NNConfigAngle
from tf_nn_sim.Config.NNConfigMovement import NNConfigMovement
from tf_nn_sim.networks.dqn import DQN
import tensorflow as tf
import numpy as np

from tf_nn_sim.networks.rnn_dynamic import RRNDynamicModel
from tf_nn_sim.networks.rrn_cnn import GeneralRRNDiscreteModel
from tf_nn_sim.networks.rrn_dqn import DQN_RNN

NUM_ACTIONS = 10
import collections
NN_GOAL_DISTANCE_OFFSET = 0.0
# NN_GOAL_DISTANCE_OFFSET = 0.0

#Could be a range mapping
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def wrap_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

#Mini class, probably to just store data
class ControlCmd:
    def __init__(self):
        self.vel = 0.0
        self.psi_vel = 0.0
        self.abs_psi = 0.0


def get_yaw_from_msg(msg):
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = transformations.euler_from_quaternion(q)
    return euler[2]


class NNMovementWrapper:
    def __init__(self, movement_cfg, angle_cfg):

        #Come from Basti's simulator
        movement_cfg.epsilon = 0.0
        self.movement_cfg = movement_cfg
        self.angle_cfg = angle_cfg
        self.network_movement = None
        movement_cfg.batch_size = 1
        movement_cfg.keep_prob = 1.0

        self.target_goal_vel = Velocity()

        #tf is tensorflow
        self.graph = tf.Graph().as_default()

        #I think these are the 2 networks
        #Value network
        initializer_movement = tf.random_normal_initializer(stddev=movement_cfg.init_scale)
        with tf.variable_scope("Movement_Model", reuse=None, initializer=initializer_movement) as scope:
            self.network_movement = DQN_RNN(num_states=7, num_actions=NUM_ACTIONS,
                                               config=movement_cfg, namespace="DQN",
                                               is_training=False,
                                               log_path=movement_cfg.log_folder,
                                               weight_path=movement_cfg.weight_folder, variable_scope=scope,
                                               num_drones=1)

        #Angle network
        initializer_angle = tf.random_normal_initializer(stddev=angle_cfg.init_scale)
        with tf.variable_scope("Angle_Model_Discrete", reuse=None, initializer=initializer_angle) as angle_scope:
            self.angle_predict_network = GeneralRRNDiscreteModel(num_drones=1,
                                                            config=angle_cfg,
                                                            namespace="angle",
                                                            is_training=False,
                                                            log_path=angle_cfg.log_folder,
                                                            weight_path=angle_cfg.weight_folder,
                                                            variable_scope=angle_scope)

        #All initialising tensorflow stuff
        init = tf.global_variables_initializer()
        config = tf.ConfigProto(
            device_count={'GPU': 0}
        )
        self.sess = tf.Session(config=config)
        self.sess.run(init)
        print('Loading Movement Model...')
        self.network_movement.load_weight(self.sess)
        self.angle_predict_network.load_weight(self.sess)
        self.rrn_states_obstacle = self.angle_predict_network.gen_init_state(self.sess, 1)
        self.rrn_states_goal = self.angle_predict_network.gen_init_state(self.sess, 1)
        self.rnn_movement_state = self.network_movement.gen_init_state(self.sess, 1)


    def get_new_goal_state(self, my_vx, my_vy, distance, dt):

        #Velocity range translations
        my_vx = translate(my_vx, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x,
                                     self.angle_cfg.max_x)
        my_vy = translate(my_vy, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x, self.angle_cfg.max_x)

        #Distance translations
        dist_norm = translate(distance, 0, self.angle_cfg.max_sensor_distance, self.angle_cfg.min_x,
                              self.angle_cfg.max_x)

        s = [my_vx, my_vy, 0.0, 0.0, dist_norm, dt]
        # print "my_vel: {} my_angle: {} distance: {} dt: {}".format(velocity, velocity_angle, distance, dt)

        #Angle prediction network
        predict_angle, self.rrn_states_goal = self.angle_predict_network.predict_angle(self.sess, s,
                                                                                       self.rrn_states_goal)

        # predict_angle = translate(predict_angle, self.angle_cfg.min_y, self.angle_cfg.max_y, -np.pi, np.pi)

        predict_angle = self.discrete_to_angle(predict_angle)
        dx = distance * np.cos(predict_angle)
        dy = distance * np.sin(predict_angle)

        # return [velocity, velocity_angle, 0.0, 0.0, goal_distance, 1.0 / self.update_rate]

        return dx, dy, predict_angle

    #Used only in this class
    def discrete_to_angle(self, discrete_angle):
        range_size = 2 * np.pi
        step_size = range_size / float(self.angle_cfg.output_size - 1.0)
        return step_size * discrete_angle - np.pi


    def get_new_obstacle_state(self, my_vx, my_vy, o_vx, o_vy, distance, dt):

        #Translations again
        my_vx = translate(my_vx, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x,
                                     self.angle_cfg.max_x)
        my_vy = translate(my_vy, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x, self.angle_cfg.max_x)

        o_vx = translate(o_vx, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x,
                                   self.angle_cfg.max_x)
        o_vy = translate(o_vy, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x, self.angle_cfg.max_x)

        dist = translate(distance, 0, self.angle_cfg.max_sensor_distance, self.angle_cfg.min_x, self.angle_cfg.max_x)
        s = [my_vx, my_vy, o_vx, o_vy, dist, dt]

        # print "my_vel: {} my_angle: {} distance: {} dt: {} | o_vel {} o_angel".format(my_velocity, my_velocity_angle,
        #                                                                               distance, dt, other_vel,
        #                                                                               other_vel_angle)

        #Uses angle predict network
        predict_angle, self.rrn_states_obstacle = self.angle_predict_network.predict_angle(self.sess, s,
                                                                                           self.rrn_states_obstacle)
        predict_angle = self.discrete_to_angle(predict_angle)

        # predict_angle = translate(predict_angle, self.angle_cfg.min_y, self.angle_cfg.max_y, -np.pi, np.pi)
        # predict_angle = wrap_angle(predict_angle)

        dx = distance * np.cos(predict_angle)
        dy = distance * np.sin(predict_angle)

        # return [velocity, velocity_angle, 0.0, 0.0, goal_distance, 1.0 / self.update_rate]

        return dx, dy, predict_angle

    #Resets network
    def reset(self):
        self.target_goal_vel = Velocity()
        self.rrn_states_obstacle = self.angle_predict_network.gen_init_state(self.sess, 1)
        self.rrn_states_goal = self.angle_predict_network.gen_init_state(self.sess, 1)
        self.rnn_movement_state = self.network_movement.gen_init_state(self.sess, 1)

    #goal offset = hack for nn, think it far away but it's not
    def best_velocity(self, v_x, v_y, goal_dx, goal_dy, obstacle_dx, obstacle_dy, delta_time, goal_distance_offset = 0):

        #Translations
        g_dx = translate(goal_dx, -self.movement_cfg.max_sensor_distance,
                                               self.movement_cfg.max_sensor_distance,
                                               self.movement_cfg.min_state_out, self.movement_cfg.max_state_out)
        g_dy = translate(goal_dy, -self.movement_cfg.max_sensor_distance,
                                               self.movement_cfg.max_sensor_distance,
                                               self.movement_cfg.min_state_out, self.movement_cfg.max_state_out)
        o_dx = translate(obstacle_dx, -self.movement_cfg.max_sensor_distance,
                                               self.movement_cfg.max_sensor_distance,
                                               self.movement_cfg.min_state_out, self.movement_cfg.max_state_out)
        o_dy = translate(obstacle_dy, -self.movement_cfg.max_sensor_distance,
                                               self.movement_cfg.max_sensor_distance,
                                               self.movement_cfg.min_state_out, self.movement_cfg.max_state_out)
        v_x = translate(v_x, -self.movement_cfg.max_velocity, self.movement_cfg.max_velocity,
                                              self.movement_cfg.min_state_out, self.movement_cfg.max_state_out)
        v_y = translate(v_y, -self.movement_cfg.max_velocity, self.movement_cfg.max_velocity,
                                              self.movement_cfg.min_state_out, self.movement_cfg.max_state_out)
        update_tick = translate(delta_time, 0.0, 1.0, self.movement_cfg.min_state_out,
                                     self.movement_cfg.max_state_out)

        s = [v_x, v_y, g_dx, g_dy, o_dx, o_dy, update_tick]

        action = None

        #Epsilon greedy strategy
        #Why are you taking non-greedy actions online?
        if np.random.rand(1) < self.movement_cfg.epsilon:
            action = np.random.randint(0, NUM_ACTIONS)
            # actions[d_id] = 2
            # a = 5
        else:
            #Get action from network
            action, self.rnn_movement_state = self.network_movement.predict_action(self.sess, self.rnn_movement_state, s)

        action = action[0]

        #Damping? Braking?
        damping_per_tick = 1.0 - self.movement_cfg.damping_per_sec * delta_time
        damping_per_tick = 0.0 if damping_per_tick < 0.0 else damping_per_tick
        brake_per_tick = 1.0 - self.movement_cfg.brake_per_sec * delta_time
        brake_per_tick = 0.0 if brake_per_tick < 0.0 else brake_per_tick


        accel = self.movement_cfg.acceleration * delta_time
        diagonal = np.sqrt((accel ** 2) / 2.0)

        if np.sqrt(goal_dx ** 2 + goal_dy ** 2) - goal_distance_offset < 0.15 and np.sqrt(obstacle_dx ** 2 + obstacle_dy ** 2) > 0.4:
            self.target_goal_vel.x = 0.0
            self.target_goal_vel.y = 0.0
        else:

            #I think these are the 10 discrete acceleration actions
            if action == 0:
                self.target_goal_vel.y -= accel
            elif action == 1:
                self.target_goal_vel.y += accel
            elif action == 2:
                self.target_goal_vel.x += accel
            elif action == 3:
                self.target_goal_vel.x -= accel
            elif action == 4:
                self.target_goal_vel.x += diagonal
                self.target_goal_vel.y += diagonal
            elif action == 5:
                self.target_goal_vel.x += diagonal
                self.target_goal_vel.y -= diagonal
            elif action == 6:
                self.target_goal_vel.x -= diagonal
                self.target_goal_vel.y += diagonal
            elif action == 7:
                self.target_goal_vel.x -= diagonal
                self.target_goal_vel.y -= diagonal
            elif action == 8:
                self.target_goal_vel.x *= brake_per_tick
                self.target_goal_vel.y *= brake_per_tick
            elif action == 9:
                self.target_goal_vel.x *= damping_per_tick
                self.target_goal_vel.y *= damping_per_tick

        #Returns target velocity as a result of it's chosen acceleration
        self.target_goal_vel = self.cut_velocity(self.target_goal_vel, self.movement_cfg.speed_limit)

        return self.target_goal_vel

    #Used in this class
    def cut_velocity(self, vel, max_velocity):
        result_vel = copy.deepcopy(vel)
        norm = math.sqrt(vel.x ** 2 + vel.y ** 2)
        if norm > max_velocity:
            print "is faster ", norm, max_velocity
            result_vel.x /= norm
            result_vel.x *= max_velocity
            result_vel.y /= norm
            result_vel.y *= max_velocity
        return result_vel

#Main class
class NNControllerDWM1000Discrete(CollvoidInterface):
    def __init__(self):
        #Implements interface
        CollvoidInterface.__init__(self)

        #Receiving parameters from launch files
        self.my_pose_id = rospy.get_param("~obstacle_manager_id")
        self.dwm1000_active = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_active")
        self.timeout = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm_timeout")
        self.avg_buffer_size = rospy.get_param("~collvoid/nn_controller_dwm1000/avg_buffer_size")
        movement_config_file = rospy.get_param("~collvoid/nn_controller_dwm1000/movement_config_file")
        angle_config_file = rospy.get_param("~collvoid/nn_controller_dwm1000/angle_config_file")
        self.dwm_goal_active = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_goal_active")
        self.dwm_obstacle_active = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_obstacle_active")
        self.deactivate_angle_network = rospy.get_param("~collvoid/nn_controller_dwm1000/deactivate_angle_network", False)
        if self.dwm_goal_active:
            self.dwm_goal_id = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_goal_id")
        if self.dwm_obstacle_active:
            self.dwm_obstacle_ids = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_obstacle_ids")
            # rospy.logerr("obstacle ids {}".format(self.dwm_obstacle_ids[0]))
        if self.dwm_goal_active or self.dwm1000_active or self.dwm_obstacle_active:
            self.dwm_distance_offset = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm_distance_offset")

        #From Basti's simulator
        self.movement_cfg_predict = NNConfigMovement(movement_config_file)
        self.movement_cfg_predict.speed_limit = 0.4
        self.movement_cfg_predict.keep_prob = 1.0
        self.angle_cfg_predict = NNConfigAngle(angle_config_file)
        self.angle_cfg_predict.keep_prob = 1.0

        self.accuracy_list = []

        #The above class
        self.nn = NNMovementWrapper(self.movement_cfg_predict, self.angle_cfg_predict)

        #These are got from the paramters above which are in the simulator
        self.update_rate = self.movement_cfg_predict.update_rate
        self.max_velocity = self.movement_cfg_predict.max_velocity
        self.max_sensor_distance = self.movement_cfg_predict.max_sensor_distance

        #Start some services
        self.start_nn_controller_srvs = rospy.Service("start_nn_controller", Empty, self.start_callback)
        self.stop_nn_controller_srvs = rospy.Service("stop_nn_controller", Empty, self.stop_callback)

        self.goal_pose = None

        #Subscribing to a topic called "goal_pose"
        rospy.Subscriber("goal_pose", PoseStamped, self.goal_pose_callback)

        #Start publishers to some topics
        self.pre_goal_angle_pub = rospy.Publisher("pre_goal_angle", PoseStamped)
        self.pre_obs_angle_pub = rospy.Publisher("pre_obstacle_angle", PoseStamped)

        #DWM1000 sensor
        self.dwm_sensor = None
        if self.dwm1000_active or self.dwm_goal_active or self.dwm_obstacle_active:
            self.dwm_sensor = DWM10000DistanceSensor(self.my_pose_id, self.timeout, filter_size=1,
                                                     distance_offset=self.dwm_distance_offset)
        #The data class above
        self.cmd = ControlCmd()

        #Initialising some variables
        self.enabled = False
        self.last_update = None
        self.active = False
        self.last_velocity = None
        self.current_pose = None
        self.prev_pose = None

        #Another subscription to a topic
        self.sub = rospy.Subscriber("/obstacle_poses", Obstacle, self.obs_pose_callback)

        self.current_obstacles = []
        self.last_pose_dict = {}
        self.avg_buffer = collections.deque(maxlen=self.avg_buffer_size)

    #Called when topic 'goal_pose' receives something
    #Just sets the goal pose in this class
    def goal_pose_callback(self, pose):
        self.goal_pose = pose

    #Called when topic 'obstacle_poses' receives something
    def obs_pose_callback(self, obs):
        if obs.id is not self.my_pose_id:
            assert isinstance(obs, Obstacle)
            ot = ObstacleTime(obs, rospy.Time.now())
            self.last_pose_dict[obs.id] = ot

    #Called from collvoid_controller.py
    def update_cf_pose(self, pose):
        self.current_pose = pose
        self.current_obstacles = []
        for key, value in self.last_pose_dict.iteritems():
            assert isinstance(value, ObstacleTime)
            if (rospy.Time.now() - value.time).to_sec() < self.timeout:
                self.current_obstacles.append(value.obs)

    #Called when 'start_nn_controller' srv receives something
    #Starts the controller
    def start_callback(self, req):
        print "start_callback"
        self.enabled = True
        # if self.goal_pose is None:
        #     self.goal_pose = copy.deepcopy(self.current_pose)
        #     print "goal_pose", self.goal_pose
        #     print "current_pose", self.goal_pose
        return EmptyResponse()

    #Called when 'top_nn_controller' srv receives something
    #Stops the controller
    def stop_callback(self, req):
        print "stop_callback"
        self.enabled = False
        self.goal_pose = None
        self.last_update = None
        self.avg_buffer = collections.deque(maxlen=self.avg_buffer_size)
        self.nn.reset()
        return EmptyResponse()


    def rotate_vel_by_speed(self, vel, rotation_speed, dt):
        angle = rotation_speed * dt
        return self.rotate_vel_by_angle(vel, angle)

    #Used in the function above
    def rotate_vel_by_angle(self, vel, angle):
        assert isinstance(vel, Velocity)
        result_vel = copy.deepcopy(vel)
        angle = angle / 180.0 * math.pi
        vector_x = vel.x
        vector_y = vel.y
        result_vel.x = vector_x * math.cos(angle) - vector_y * math.sin(angle)
        result_vel.y = vector_x * math.sin(angle) + vector_y * math.cos(angle)
        return result_vel

    def rotate_vel(self, x_vel, y_vel, angle):
        x = x_vel * np.cos(angle) - y_vel * np.sin(angle)
        y = x_vel * np.sin(angle) + y_vel * np.cos(angle)
        return x, y

    def set_speed(self, vel, speed):
        result_vel = copy.deepcopy(vel)
        norm = math.sqrt(vel.x ** 2 + vel.y ** 2)
        vel.x /= norm
        vel.x *= speed
        vel.y /= norm
        vel.y *= speed
        return result_vel

    # calculates the new target velocity based on the collvoid scheme
    # target_Velocity is the current target velocity towards the goal
    def calculate_velocity(self, current_target_velocity):

        #Update dt
        if self.last_update is None:        #Just for first time step I think
            dt = 1.0 / self.update_rate
        else:
            dt = (rospy.Time.now() - self.last_update).to_sec()

        #First time step
        if self.last_velocity is None:
            self.last_velocity = copy.deepcopy(current_target_velocity)
        # print "nn incoming {0}".format(current_target_velocity.z)

        g_distance = 3.0                #DWM sensor distance
        if self.dwm_goal_active:        #This was set to false
            #Get DWM sensor data
            g_distance = self.dwm_sensor.get_distance(self.dwm_goal_id)

            #Could not get distance from DWM
            if g_distance is None:
                rospy.logwarn("DWM has no distance!!!")

        #Perform update if there is a distance and a goal pose and enough time has passed since last update
        if (dt >= 1.0 / self.update_rate) and self.goal_pose is not None and g_distance is not None:

            if self.prev_pose is None:                              #Just for first time step I think
                self.prev_pose = copy.deepcopy(self.current_pose)

            #Updating old and new velocities
            vel_x = self.last_velocity.x
            vel_y = self.last_velocity.y
            self.last_velocity = copy.deepcopy(current_target_velocity)

            #Seems to be finding difference between goal pose and current pose
            if not self.dwm_goal_active:
                gx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
                gy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
                gx, gy = self.rotate_vel(gx, gy, -get_yaw_from_msg(self.current_pose))
                g_distance = np.sqrt(gx ** 2 + gy ** 2)
                g_distance += NN_GOAL_DISTANCE_OFFSET

            # vel_x = (self.current_pose.pose.position.x - self.prev_pose.pose.position.x) / dt
            # vel_y = (self.current_pose.pose.position.y - self.prev_pose.pose.position.y) / dt
            # vel_x, vel_y = self.rotate_vel(vel_x, vel_y, -get_yaw_from_msg(self.current_pose))

            #Ignoring obstacles
            if (not self.dwm_obstacle_active and len(self.current_obstacles) <= 0) or (
                self.dwm_obstacle_active and self.dwm_sensor.get_closest(self.dwm_obstacle_ids) is None):
                ox = self.max_sensor_distance * np.cos(0.0)
                oy = self.max_sensor_distance * np.sin(0.0)
                print "IGNORE OBSTACLES"


            else:
                #Gets closest object from list of obstacles
                obstacle = self.get_closes_obs()

                #Distance to obstacle
                ox = obstacle.x - self.current_pose.pose.position.x
                oy = obstacle.y - self.current_pose.pose.position.y
                ox, oy = self.rotate_vel(ox, oy, -get_yaw_from_msg(self.current_pose))
                o_distance = np.sqrt(ox ** 2 + oy ** 2)


                if self.dwm_obstacle_active:
                    #Get closest obstacle from DWM
                    tmp = self.dwm_sensor.get_closest(self.dwm_obstacle_ids)
                    #Overide the o_distance above with distance from DWM
                    if tmp is not None:
                        o_distance = tmp
                    else:
                        rospy.logwarn("OBSTACLE DISTANCE IS NOT AVAILABLE!")

                else:
                    print "dwm obstacle not active x {} y {} distance {}".format(ox, oy, o_distance)

                #Angle network not activated
                #I think this returns distance and angle to obstacle from the angle network
                if self.deactivate_angle_network:
                    print "Angle network NOT ACTIVE!!!"
                    ox_, oy_, angle = self.nn.get_new_obstacle_state(vel_x, vel_y, obstacle.x_vel_local, obstacle.y_vel_local,
                                                               o_distance, dt)
                else:
                    ox, oy, angle = self.nn.get_new_obstacle_state(vel_x, vel_y, obstacle.x_vel_local, obstacle.y_vel_local,
                                                               o_distance, dt)
                #Publishes predicted angle
                self.publish_predict_angle(self.pre_obs_angle_pub, angle)

            # real_angle = math.atan2(gy, gx)
            # real_angle_translate = translate(real_angle, -np.pi, np.pi, -1.0, 1.0)

            #Get new goal state
            #Both of these use the angle prediction network
            if self.deactivate_angle_network:
                gx_, gy_, angle = self.nn.get_new_goal_state(vel_x, vel_y, g_distance, dt)
            else:
                gx, gy, angle = self.nn.get_new_goal_state(vel_x, vel_y, g_distance, dt)

            # gx, gy, angle = self.nn.get_new_goal_state(vel_x, vel_y, g_distance, 1.0 / self.update_rate)
            # gx, gy, angle = self.nn.get_new_goal_state(vel_x, vel_y, g_distance, 1.0 / self.update_rate)

            #Translating and publishing
            angle_trans = translate(angle, -np.pi, np.pi, -1.0, 1.0)
            self.publish_predict_angle(self.pre_goal_angle_pub, angle)

            # self.publish_predict_angle(self.pre_goal_angle_pub, real_angle)
            # print "real_angle: {} pre angle: {}".format(real_angle, angle_trans)
            # er = (((real_angle_translate - angle_trans) + 1.0) % 2.0) - 1.0
            # accuracy_list.append((1.0 - abs(er)) / 1.0)
            # self.accuracy_list.append((1.0 - abs(er)) / 1.0)
            # print dt
            # if len(self.accuracy_list) > int(1.0 / dt) * 5.0:
            #     a = sum(self.accuracy_list) / len(self.accuracy_list)
            #     error = (((a * 1.0) - 1.0) * -1.0) * 180.0
            #     print "accuracy: {} avg_error: {}".format(a, error)
            #     self.accuracy_list = []

            #Get best velocity, this is from the DQN network
            tmp = self.nn.best_velocity(vel_x, vel_y, gx, gy, ox, oy, dt, goal_distance_offset=NN_GOAL_DISTANCE_OFFSET)

            # print "prev x: {0} y: {1} after {2} {3} {4}".format(self.last_velocity.x, self.last_velocity.y, tmp.x, tmp.y, self.max_velocity)

            self.last_velocity.x = 0
            self.last_velocity.y = 0
            self.avg_buffer.append(tmp)
            b_size = len(self.avg_buffer)

            #Is this taking an average?
            for v in self.avg_buffer:
                self.last_velocity.x += v.x / b_size
                self.last_velocity.y += v.y / b_size

            #Update previous pose
            self.prev_pose = copy.deepcopy(self.current_pose)

            #Update time
            self.last_update = rospy.Time.now()

        return self.last_velocity

    #Publishes predict angle from network
    def publish_predict_angle(self, publisher, angle):
        tmp = copy.deepcopy(self.current_pose)
        # assert isinstance(tmp, PoseStamped)
        # tmp.header.
        yaw = get_yaw_from_msg(tmp)
        global_angle = wrap_angle(yaw + angle)
        # global_angle = wrap_angle(angle)
        # print "global angle: ", 180.0 * (global_angle / np.pi)
        tmp.header.stamp = rospy.Time.now()
        tmp.pose.orientation.x, tmp.pose.orientation.y, tmp.pose.orientation.z, tmp.pose.orientation.w = quaternon_from_yaw(
            global_angle)
        publisher.publish(tmp)

    #Gets closest object from list of objects
    def get_closes_obs(self):
        min_distance = 100000.0
        result = Obstacle()
        my_x = self.current_pose.pose.position.x
        my_y = self.current_pose.pose.position.y
        for o in self.current_obstacles:
            assert isinstance(o, Obstacle)
            dist = np.sqrt((my_x - o.x) ** 2 + (my_y - o.y) ** 2)
            if dist < min_distance:
                result = o
                min_distance = dist
        return result

    #Seems to not be used in this file
    def get_sensor_data(self):
        current_time = rospy.Time.now()
        distances = self.dwm_sensor.sensor_data.distances
        last_updates = self.dwm_sensor.sensor_data.last_update
        closure_rates = self.dwm_sensor.sensor_data.closure_rate
        result_dis = None
        result_rate = None
        for idx, distance in distances:
            if (current_time - last_updates[idx]).to_sec() < self.timeout:
                if result_dis is None or distance < result_dis:
                    result_dis = distance
                    result_rate = closure_rates[idx]

        return result_dis, result_rate

    # tells the main controller if this bahaviour is active
    def is_active(self):
        return self.enabled
