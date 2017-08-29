#!/usr/bin/env python
import copy
import math
from time import sleep

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sl_crazyflie_msgs.msg import Velocity, Obstacle
from std_srvs.srv import Empty, EmptyResponse
from tf import transformations

from collvoid_interface import CollvoidInterface
from sl_crazyflie_controller.collvoid.simple_collvoid import ObstacleTime
from sl_crazyflie_controller.pid_controller.pid import quaternon_from_yaw
from sl_crazyflie_controller.sensor.dwm1000_distance_sensor import DWM10000DistanceSensor
from tf_nn_sim.Config.NNConfigAngle import NNConfigAngle
from tf_nn_sim.Config.NNConfigMovement import NNConfigMovement
import tensorflow as tf
import numpy as np

from tf_nn_sim.networks.rrn_cnn import GeneralRRNDiscreteModel
from tf_nn_sim.networks.rrn_cnn_multitask_join import GeneralRRNDiscreteModelMultitaskJointLoss
from tf_nn_sim.networks.rrn_dqn import DQN_RNN
from tf_nn_sim.v2.network_wrapper import AngleNetworkWrapper
from tf_nn_sim.v2.particle_filter.particle_filter_nn import ParticleFilterNN, ParticleFilter2_5D_Cfg
import threading

NUM_ACTIONS = 10
import collections

NN_GOAL_DISTANCE_OFFSET = 0.0
GOAL_OFFSET_PARTICLE = 0.0

# NN_GOAL_DISTANCE_OFFSET = 0.0

class ObstacleCfg(ParticleFilter2_5D_Cfg):
    def __init__(self):
        super(ObstacleCfg, self).__init__()
        self.sample_size = 70
        self.x_limit = [-2.0, 2.0]
        self.resample_x_limit = [-0.2, 0.2]
        self.y_limit = [-2.0, 2.0]
        self.resample_y_limit = [-0.2, 0.2]
        self.yaw_limit = [-np.pi, np.pi]
        self.resample_yaw_limit = [-np.pi * 0.1, np.pi * 0.1]
        self.respawn_yaw_limit = [-np.pi * 0.1, np.pi * 0.1]
        self.respawn_y_limit = [-0.01, 0.01]
        self.respawn_x_limit = [-0.01, 0.01]
        self.new_spawn_samples = 0.1
        self.resample_prob = 0.95


class GoalCfg(ParticleFilter2_5D_Cfg):
    def __init__(self):
        super(GoalCfg, self).__init__()
        self.sample_size = 30
        self.x_limit = [-2.0, 2.0]
        self.resample_x_limit = [-0.05, 0.05]
        self.y_limit = [-2.0, 2.0]
        self.resample_y_limit = [-0.05, 0.05]
        self.yaw_limit = [-np.pi, np.pi]
        self.resample_yaw_limit = [-np.pi * 0.1, np.pi * 0.1]
        self.respawn_yaw_limit = [-np.pi * 0.1, np.pi * 0.1]
        self.respawn_y_limit = [-0.01, 0.01]
        self.respawn_x_limit = [-0.01, 0.01]
        self.new_spawn_samples = 0.10
        self.resample_prob = 0.95
        self.variance = 0.0312979988458
        # self.variance = 0.3


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    output = rightMin + (valueScaled * rightSpan)
    output = max(output, rightMin)
    output = min(output, rightMax)

    return output

def wrap_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


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
    def __init__(self, movement_cfg, angle_cfg, frame_id):
        movement_cfg.epsilon = 0.0
        self.movement_cfg = movement_cfg
        self.angle_cfg = angle_cfg
        self.network_movement = None
        movement_cfg.batch_size = 1
        movement_cfg.keep_prob = 1.0
        self.target_goal_vel = Velocity()
        self.first_call = False

        self.graph = tf.Graph().as_default()
        initializer_movement = tf.random_normal_initializer(stddev=movement_cfg.init_scale)
        with tf.variable_scope("Movement_Model", reuse=None, initializer=initializer_movement) as scope:
            self.network_movement = DQN_RNN(num_states=7, num_actions=NUM_ACTIONS,
                                            config=movement_cfg, namespace="DQN",
                                            is_training=False,
                                            log_path=movement_cfg.log_folder,
                                            weight_path=movement_cfg.weight_folder, variable_scope=scope,
                                            num_drones=1)
        initializer_angle = tf.random_normal_initializer(stddev=angle_cfg.init_scale)
        with tf.variable_scope("Angle_Model_Discrete", reuse=None, initializer=initializer_angle) as angle_scope:
            self.angle_predict_network = GeneralRRNDiscreteModelMultitaskJointLoss(num_drones=1,
                                                                                   config=angle_cfg,
                                                                                   namespace="angle",
                                                                                   is_training=False,
                                                                                   log_path=angle_cfg.log_folder,
                                                                                   weight_path=angle_cfg.weight_folder,
                                                                                   variable_scope=angle_scope)

        init = tf.global_variables_initializer()
        config = tf.ConfigProto(
            device_count={'GPU': 0}
        )
        self.sess = tf.Session(config=config)
        self.sess.run(init)
        print('Loading Movement Model...')
        self.network_movement.load_weight(self.sess)
        self.angle_predict_network.load_weight(self.sess)
        self.angle_networks_wrapper = AngleNetworkWrapper(self.sess, self.angle_predict_network, 3, ObstacleCfg(),
                                                          GoalCfg(), frame_id)
        self.rnn_movement_state = self.network_movement.gen_init_state(self.sess, 1)

    def get_new_goal_state(self, my_vx, my_vy, distance, dt, goal_offset = None):
        self.angle_networks_wrapper.update_goal_particle_filter(my_vx, my_vy, 0.0, 0.0, distance, dt)
        goal_x, goal_y, goal_yaw = self.angle_networks_wrapper.get_goal_estimate_pose()
        predict_angle = np.arctan2(goal_y, goal_x)

        if goal_offset is not None:
            length = np.sqrt(goal_x**2 + goal_y**2)
            length += goal_offset
            goal_x = np.cos(predict_angle) * length
            goal_y = np.sin(predict_angle) * length


        # return [velocity, velocity_angle, 0.0, 0.0, goal_distance, 1.0 / self.update_rate]
        # print "nn: x: {} y {} , other x: {} y: {}".format(dx, dy, x, y)
        # return dx, dy, predict_angle
        return goal_x, goal_y, predict_angle

    def discrete_to_angle(self, discrete_angle):
        range_size = 2 * np.pi
        step_size = range_size / float(self.angle_cfg.output_size - 1.0)
        return step_size * discrete_angle - np.pi

    def get_new_obstacle_state(self, my_vx, my_vy, o_vx, o_vy, distance, dt):
        my_vx_t = translate(my_vx, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x,
                            self.angle_cfg.max_x)
        my_vy_t = translate(my_vy, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x,
                            self.angle_cfg.max_x)

        o_vx_t = translate(o_vx, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x,
                           self.angle_cfg.max_x)
        o_vy_t = translate(o_vy, -self.angle_cfg.max_velocity, self.angle_cfg.max_velocity, self.angle_cfg.min_x,
                           self.angle_cfg.max_x)

        dist = translate(distance, 0, self.angle_cfg.max_sensor_distance, self.angle_cfg.min_x, self.angle_cfg.max_x)
        s = [my_vx_t, my_vy_t, o_vx_t, o_vy_t, dist, dt]
        # print "my_vel: {} my_angle: {} distance: {} dt: {} | o_vel {} o_angel".format(my_velocity, my_velocity_angle,
        #                                                                               distance, dt, other_vel,
        #                                                                               other_vel_angle)

        predict_angle, predict_orientation, self.rrn_states_obstacle = self.angle_predict_network.predict_angle_orientation(
            self.sess, s,
            self.rrn_states_obstacle)
        predict_angle = self.discrete_to_angle(predict_angle[0])
        predict_orientation = self.discrete_to_angle(predict_orientation[0])
        # predict_angle = translate(predict_angle, self.angle_cfg.min_y, self.angle_cfg.max_y, -np.pi, np.pi)
        # predict_angle = wrap_angle(predict_angle)
        dx = distance * np.cos(predict_angle)
        dy = distance * np.sin(predict_angle)
        if self.first_call:
            print "RESET?"
            self.particles_obs.reset(dx, dy, predict_orientation)

        # print o_vx, o_vy
        self.particles_obs.update_samples(my_vx, my_vy, o_vx, o_vy, dt, distance, dx, dy, predict_orientation)

        x, y, yaw = self.particles_obs.local_pose()
        # predict_angle = np.arctan2(y, x)

        # return [velocity, velocity_angle, 0.0, 0.0, goal_distance, 1.0 / self.update_rate]
        return x, y, predict_angle

    def reset(self):
        self.target_goal_vel = Velocity()
        self.rrn_states_obstacle = self.angle_predict_network.gen_init_state(self.sess, 1)
        self.rrn_states_goal = self.angle_predict_network.gen_init_state(self.sess, 1)
        self.rnn_movement_state = self.network_movement.gen_init_state(self.sess, 1)

    # goal offset = hack for nn, think it far away but it's not
    def best_velocity(self, v_x, v_y, goal_dx, goal_dy, obstacle_dx, obstacle_dy, delta_time, goal_distance_offset=0):
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
        # print s
        action = None
        if np.random.rand(1) < self.movement_cfg.epsilon:
            action = np.random.randint(0, NUM_ACTIONS)
            # actions[d_id] = 2
            # a = 5
        else:
            action, self.rnn_movement_state = self.network_movement.predict_action(self.sess, self.rnn_movement_state,
                                                                                   s)

        action = action[0]
        damping_per_tick = 1.0 - self.movement_cfg.damping_per_sec * delta_time
        damping_per_tick = 0.0 if damping_per_tick < 0.0 else damping_per_tick
        brake_per_tick = 1.0 - self.movement_cfg.brake_per_sec * delta_time
        brake_per_tick = 0.0 if brake_per_tick < 0.0 else brake_per_tick

        accel = self.movement_cfg.acceleration * delta_time
        diagonal = np.sqrt((accel ** 2) / 2.0)
        if np.sqrt(goal_dx ** 2 + goal_dy ** 2) - goal_distance_offset < 0.15 and np.sqrt(
                                obstacle_dx ** 2 + obstacle_dy ** 2) > 0.4:
            self.target_goal_vel.x = 0.0
            self.target_goal_vel.y = 0.0
        else:
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

        self.target_goal_vel = self.cut_velocity(self.target_goal_vel, self.movement_cfg.speed_limit)

        return self.target_goal_vel

    def cut_velocity(self, vel, max_velocity):
        result_vel = copy.deepcopy(vel)
        norm = math.sqrt(vel.x ** 2 + vel.y ** 2)
        if norm > max_velocity:
            # print "is faster ", norm, max_velocity
            result_vel.x /= norm
            result_vel.x *= max_velocity
            result_vel.y /= norm
            result_vel.y *= max_velocity
        return result_vel


class NNControllerDWM1000DiscreteParticleMulti(CollvoidInterface):
    def __init__(self):
        CollvoidInterface.__init__(self)
        self.my_pose_id = rospy.get_param("~obstacle_manager_id")
        self.dwm1000_active = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_active")
        self.timeout = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm_timeout")
        self.avg_buffer_size = rospy.get_param("~collvoid/nn_controller_dwm1000/avg_buffer_size")
        movement_config_file = rospy.get_param("~collvoid/nn_controller_dwm1000/movement_config_file")
        angle_config_file = rospy.get_param("~collvoid/nn_controller_dwm1000/angle_config_file")
        self.dwm_goal_active = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_goal_active")
        self.dwm_obstacle_active = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_obstacle_active")
        self.cf_frame_id = rospy.get_param("~collvoid/nn_controller_dwm1000/cf_frame_id")
        self.deactivate_angle_network = rospy.get_param("~collvoid/nn_controller_dwm1000/deactivate_angle_network",
                                                        False)
        if self.dwm_goal_active:
            self.dwm_goal_id = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_goal_id")
        if self.dwm_obstacle_active:
            self.dwm_obstacle_ids = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm1000_obstacle_ids")
            # rospy.logerr("obstacle ids {}".format(self.dwm_obstacle_ids[0]))
        if self.dwm_goal_active or self.dwm1000_active or self.dwm_obstacle_active:
            self.dwm_distance_offset = rospy.get_param("~collvoid/nn_controller_dwm1000/dwm_distance_offset")
        self.movement_cfg_predict = NNConfigMovement(movement_config_file)
        self.movement_cfg_predict.speed_limit = 0.5
        self.movement_cfg_predict.keep_prob = 1.0
        self.angle_cfg_predict = NNConfigAngle(angle_config_file)
        self.angle_cfg_predict.keep_prob = 1.0
        self.accuracy_list = []
        self.nn = NNMovementWrapper(self.movement_cfg_predict, self.angle_cfg_predict, self.cf_frame_id)
        self.update_rate = self.movement_cfg_predict.update_rate
        self.max_velocity = self.movement_cfg_predict.max_velocity
        self.max_sensor_distance = self.movement_cfg_predict.max_sensor_distance
        self.start_nn_controller_srvs = rospy.Service("start_nn_controller", Empty, self.start_callback)
        self.stop_nn_controller_srvs = rospy.Service("stop_nn_controller", Empty, self.stop_callback)
        self.goal_pose = None
        rospy.Subscriber("goal_pose", PoseStamped, self.goal_pose_callback)
        self.pre_goal_angle_pub = rospy.Publisher("pre_goal_angle", PoseStamped, queue_size=1)
        self.pre_obs_angle_pub = rospy.Publisher("pre_obstacle_angle", PoseStamped, queue_size=1)
        self.pre_goal_particle_pub = rospy.Publisher("goal_particle", PoseArray, queue_size=1)
        self.pre_obs_particle_pub = rospy.Publisher("obs_particle", PoseArray, queue_size=1)
        self.dwm_sensor = None
        if self.dwm1000_active or self.dwm_goal_active or self.dwm_obstacle_active:
            self.dwm_sensor = DWM10000DistanceSensor(self.my_pose_id, self.timeout, filter_size=1,
                                                     distance_offset=self.dwm_distance_offset)

        self.cmd = ControlCmd()
        self.vel_x = self.vel_y = self.gx = self.gy = self.ox = self.oy = None
        self.enabled = False
        self.last_update_thread = None
        self.last_update = None
        self.active = False
        self.last_velocity = None
        self.current_pose = None
        self.prev_pose = None
        self.prev_obstacle = {}
        self.sub = rospy.Subscriber("/obstacle_poses", Obstacle, self.obs_pose_callback)
        self.current_obstacles = []
        self.last_pose_dict = {}
        self.avg_buffer = collections.deque(maxlen=self.avg_buffer_size)
        self.thread = threading.Thread(target=self.update_thread)
        self.thread.start()

    def update_thread(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue
            self.update_predictions()
            r.sleep()

    def goal_pose_callback(self, pose):
        self.goal_pose = pose

    def obs_pose_callback(self, obs):
        if obs.id is not self.my_pose_id:
            assert isinstance(obs, Obstacle)
            ot = ObstacleTime(obs, rospy.Time.now())
            # if obs.id == 2:
            #     print "controller x: {} y: {}".format(obs.x_vel_local, obs.y_vel_local)
            self.last_pose_dict[obs.id] = ot

    def update_cf_pose(self, pose):
        self.current_pose = pose

    def update_obstacles(self):
        self.current_obstacles = []
        for key, value in self.last_pose_dict.iteritems():
            assert isinstance(value, ObstacleTime)
            if (rospy.Time.now() - value.time).to_sec() < self.timeout:
                self.current_obstacles.append(value.obs)

    def start_callback(self, req):
        print "start_callback"
        self.enabled = True
        self.nn.first_call = True
        # if self.goal_pose is None:
        #     self.goal_pose = copy.deepcopy(self.current_pose)
        #     print "goal_pose", self.goal_pose
        #     print "current_pose", self.goal_pose
        return EmptyResponse()

    def stop_callback(self, req):
        print "stop_callback"
        self.enabled = False
        # self.goal_pose = None
        self.last_update_thread = None
        self.nn.first_call = False
        self.avg_buffer = collections.deque(maxlen=self.avg_buffer_size)
        self.nn.reset()
        return EmptyResponse()

    def rotate_vel_by_speed(self, vel, rotation_speed, dt):
        angle = rotation_speed * dt
        return self.rotate_vel_by_angle(vel, angle)

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

    def update_predictions(self):
        start = rospy.Time.now()
        if self.last_update_thread is None:
            dt = 1.0 / self.update_rate
        else:
            dt = (rospy.Time.now() - self.last_update_thread).to_sec()

        # print "nn incoming {0}".format(current_target_velocity.z)
        self.update_obstacles()
        prev_obstacle = None
        tmp_id = None
        g_distance = 3.0
        if self.dwm_goal_active:
            g_distance = self.dwm_sensor.get_distance(self.dwm_goal_id)
            # print "dwm active"
            if g_distance is None:
                rospy.logwarn("DWM has no distance!!!")
        if self.goal_pose is None:
            self.goal_pose = PoseStamped()
            self.goal_pose.header.stamp = rospy.Time.now()
        if (dt >= 1.0 / self.update_rate) and self.goal_pose is not None and g_distance is not None:
            if self.prev_pose is None:
                self.prev_pose = copy.deepcopy(self.current_pose)
            # vel_x = self.last_velocity.x
            # vel_y = self.last_velocity.y
            if not self.dwm_goal_active:
                gx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
                gy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
                self.gx, self.gy = self.rotate_vel(gx, gy, -get_yaw_from_msg(self.current_pose))
                g_distance = np.sqrt(gx ** 2 + gy ** 2)
                g_distance += NN_GOAL_DISTANCE_OFFSET
                print "g_distance ", g_distance

            self.vel_x = (self.current_pose.pose.position.x - self.prev_pose.pose.position.x) / dt
            self.vel_y = (self.current_pose.pose.position.y - self.prev_pose.pose.position.y) / dt
            # d = np.sqrt((self.vel_x * dt) ** 2 + (self.vel_y * dt) ** 2)
            # print "main distance: {}".format(d)
            self.vel_x, self.vel_y = self.rotate_vel(self.vel_x, self.vel_y, -get_yaw_from_msg(self.current_pose))
            if (not self.dwm_obstacle_active and len(self.current_obstacles) <= 0) or (
                        self.dwm_obstacle_active and self.dwm_sensor.get_closest(self.dwm_obstacle_ids) is None):
                self.ox = self.max_sensor_distance * np.cos(0.0)
                self.oy = self.max_sensor_distance * np.sin(0.0)
                # print "IGNORE OBSTACLES"

            else:
                for obstacle in self.current_obstacles:
                    ox = obstacle.x - self.current_pose.pose.position.x
                    oy = obstacle.y - self.current_pose.pose.position.y
                    if obstacle.id not in self.prev_obstacle:
                        self.prev_obstacle[obstacle.id] = obstacle
                    prev_obstacle = copy.deepcopy(self.prev_obstacle[obstacle.id])
                    tmp_id = obstacle.id
                    o_vx = (obstacle.x - prev_obstacle.x) / dt
                    o_vy = (obstacle.y - prev_obstacle.y) / dt
                    o_vx, o_vy = self.rotate_vel(o_vx, o_vy, -obstacle.yaw)
                    self.prev_obstacle[obstacle.id] = obstacle
                    # print o_vy, o_vx
                    # o_vx = obstacle.x_vel_local
                    # o_vy = obstacle.y_vel_local
                    self.ox, self.oy = self.rotate_vel(ox, oy, -get_yaw_from_msg(self.current_pose))
                    o_distance = np.random.np.sqrt(ox ** 2 + oy ** 2) + np.random.normal(
                        self.nn.angle_networks_wrapper.particle_obs_cfg.mean,
                        self.nn.angle_networks_wrapper.particle_obs_cfg.variance)
                    if self.dwm_obstacle_active:
                        tmp = self.dwm_sensor.get_distance(obstacle.id)
                        if tmp is not None:
                            o_distance = tmp
                        else:
                            rospy.logwarn("OBSTACLE DISTANCE IS NOT AVAILABLE!")
                    self.nn.angle_networks_wrapper.append_obs_state(obstacle.id, self.vel_x, self.vel_y, o_vx, o_vy,
                                                                    o_distance, dt)
                self.nn.angle_networks_wrapper.update_obs_particle_filter()
                # else:
                #     print "dwm obstacle not active x {} y {} distance {}".format(ox, oy, o_distance)
                if self.deactivate_angle_network:
                    print "Angle network NOT ACTIVE!!!"
                    ox_, oy_, angle = self.nn.angle_networks_wrapper.get_obs_estimate_pose()
                else:
                    self.ox, self.oy, angle = self.nn.angle_networks_wrapper.get_obs_estimate_pose()

                self.publish_predict_angle(self.pre_obs_angle_pub, angle)

            # real_angle = math.atan2(gy, gx)
            # real_angle_translate = translate(real_angle, -np.pi, np.pi, -1.0, 1.0)
            if self.deactivate_angle_network:
                gx_, gy_, angle = self.nn.get_new_goal_state(self.vel_x, self.vel_y, g_distance, dt, GOAL_OFFSET_PARTICLE)
            else:
                self.gx, self.gy, angle = self.nn.get_new_goal_state(self.vel_x, self.vel_y, g_distance, dt, GOAL_OFFSET_PARTICLE)

            # if self.my_pose_id == 1:
            #     if abs(round(obstacle.x_vel_local, 2)) > abs(round(obstacle.y_vel_local,2)):
            #         print "x axis"
            #     else:
            #         print "y axis"
            self.publish_goal_particles()
            self.publish_obs_particles()
            # gx, gy, angle = self.nn.get_new_goal_state(vel_x, vel_y, g_distance, 1.0 / self.update_rate)
            # gx, gy, angle = self.nn.get_new_goal_state(vel_x, vel_y, g_distance, 1.0 / self.update_rate)
            angle_trans = translate(angle, -np.pi, np.pi, -1.0, 1.0)
            self.publish_predict_angle(self.pre_goal_angle_pub, angle)
            self.last_update_thread = rospy.Time.now()
            self.prev_pose = copy.deepcopy(self.current_pose)
            # self.update_prev_obstacles(tmp_id)
            if prev_obstacle is not None:
                distance = np.sqrt((prev_obstacle.x - self.prev_obstacle[tmp_id].x) ** 2 + (
                prev_obstacle.y - self.prev_obstacle[tmp_id].y) ** 2)
                # print "distance : {}".format(distance)

                # print (rospy.Time.now() - start).to_sec()

    def update_prev_obstacles(self, skip_id=None):
        for o in self.current_obstacles:
            if skip_id is not None and skip_id == o.id:
                continue
            self.prev_obstacle[o.id] = o

    def publish_goal_particles(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.cf_frame_id
        for p in self.nn.angle_networks_wrapper.get_goal_particles():
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternon_from_yaw(p.yaw)
            pose_array.poses.append(pose)
        self.pre_goal_particle_pub.publish(pose_array)

    def publish_obs_particles(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.cf_frame_id
        l = self.nn.angle_networks_wrapper.get_obs_particles()
        if l is not None:
            for p in self.nn.angle_networks_wrapper.get_obs_particles():
                pose = Pose()
                pose.position.x = p.x
                pose.position.y = p.y
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternon_from_yaw(p.yaw)
                pose_array.poses.append(pose)
            self.pre_obs_particle_pub.publish(pose_array)

    # calculates the new target velocity based on the collvoid scheme
    # target_Velocity is the current target velocity towards the goal
    def calculate_velocity(self, current_target_velocity):
        if self.dwm_goal_active:
            g_distance = self.dwm_sensor.get_distance(self.dwm_goal_id)
            if g_distance is None:
                rospy.logwarn("DWM has no distance!!!")
        if self.last_velocity is None:
            self.last_velocity = copy.deepcopy(current_target_velocity)
        if self.last_update is None:
            dt = 1.0 / self.update_rate
        else:
            dt = (rospy.Time.now() - self.last_update).to_sec()

        if dt >= 1.0 / self.update_rate:
            vel_x = self.last_velocity.x
            vel_y = self.last_velocity.y
            self.last_velocity = copy.deepcopy(current_target_velocity)
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
            if self.ox is None:
                print "OX CRAP", self.vel_x, self.vel_y, self.gx, self.gy, self.ox, self.oy, dt
                self.ox = self.max_sensor_distance * np.cos(0.0)
                self.oy = self.max_sensor_distance * np.sin(0.0)
            # print vel_y, vel_y
            # tmp = self.nn.best_velocity(self.vel_x, self.vel_y, self.gx, self.gy, self.ox, self.oy, dt,
            #                             goal_distance_offset=NN_GOAL_DISTANCE_OFFSET)
            tmp = self.nn.best_velocity(self.nn.target_goal_vel.x, self.nn.target_goal_vel.y, self.gx, self.gy, self.ox, self.oy, dt,
                                        goal_distance_offset=NN_GOAL_DISTANCE_OFFSET)
            # print "prev x: {0} y: {1} after {2} {3} {4}".format(self.last_velocity.x, self.last_velocity.y, tmp.x, tmp.y, self.max_velocity)

            self.last_velocity.x = 0
            self.last_velocity.y = 0
            self.avg_buffer.append(tmp)
            b_size = len(self.avg_buffer)
            for v in self.avg_buffer:
                self.last_velocity.x += v.x / b_size
                self.last_velocity.y += v.y / b_size
            self.nn.first_call = False
            self.last_update = rospy.Time.now()

        # print self.last_velocity
        return self.last_velocity

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
        # self.update_predictions()
        return self.enabled
