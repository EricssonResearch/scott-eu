from __future__ import print_function
from __future__ import division

import threading
import cv2
import numpy as np
import rospy
import message_filters
from gym import Env
from gym.spaces import Discrete, Dict, Box
from gym.utils.seeding import np_random
from turtlebot2i_edge import PickAndPlaceNavigator, VrepSceneController
from cv_bridge import CvBridge
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import Image
from kobuki_msgs.msg import BumperEvent
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_edge.srv import GenerateSceneGraph, GenerateSceneGraphRequest, GenerateSceneGraphResponse
from turtlebot2i_edge.srv import MeasureNetwork


class TaskOffloadingEnv(Env):
    metadata = {'render.modes': ['human', 'ansi']}

    action_space = Discrete(3)
    observation_space = Dict(
        rtt=Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
        throughput=Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
        risk_value=Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
        temporal_coherence=Box(low=0, high=1, shape=(1,), dtype=np.float32)
    )

    def __init__(self, max_rtt, bandwidth, max_duration_throughput, max_risk_value,
                 network_image_size, depth_image_size,
                 robot_compute_power, robot_transmit_power, w_latency, w_energy,
                 pick_goals, place_goals, pick_and_place_per_episode,
                 vrep_simulation=False, vrep_host='localhost', vrep_port=19997, vrep_scene_graph_extraction=False):
        super(TaskOffloadingEnv, self).__init__()

        if not vrep_simulation and vrep_scene_graph_extraction:
            raise ValueError('Cannot extract scene graph from V-REP if it is not a simulation')

        self.max_rtt = max_rtt / 1e3
        self.bandwidth = bandwidth
        self.max_duration_throughput = max_duration_throughput
        self.max_risk_value = max_risk_value
        self.network_image_size = network_image_size
        self.depth_image_size = depth_image_size
        self.robot_compute_power = robot_compute_power
        self.robot_transmit_power = robot_transmit_power
        self.w_latency = w_latency
        self.w_energy = w_energy
        self.pick_and_place_per_episode = pick_and_place_per_episode
        self.vrep_simulation = vrep_simulation
        self.vrep_scene_graph_extraction = vrep_scene_graph_extraction

        self._pick_and_place_navigator = PickAndPlaceNavigator(pick_goals, place_goals)
        self._cv_bridge = CvBridge()                        # for conversions ROS message <-> image
        self._vrep_scene_controller = VrepSceneController()
        if vrep_simulation:
            self._vrep_scene_controller.open_connection(vrep_host, vrep_port)

        rospy.loginfo('Waiting for ROS service to generate scene graph on robot...')
        self._generate_scene_graph_on_robot = rospy.ServiceProxy('generate_scene_graph_proxy', GenerateSceneGraph)
        self._generate_scene_graph_on_robot.wait_for_service()

        rospy.loginfo('Waiting for ROS service to generate scene graph on edge...')
        self._generate_scene_graph_on_edge = rospy.ServiceProxy('edge/generate_scene_graph_proxy', GenerateSceneGraph)
        self._generate_scene_graph_on_edge.wait_for_service()

        rospy.loginfo('Waiting for ROS service to measure the network...')
        self._measure_network = rospy.ServiceProxy('measure_network', MeasureNetwork)
        self._measure_network.wait_for_service()

        self._collision = False
        self._collision_lock = threading.Lock()
        rospy.Subscriber('events/bumper', BumperEvent, self._save_collision)

        self._scene_graph_last = None
        self.scene_graph_from_edge = False
        self._scene_graph_pub = rospy.Publisher('scene_graph', SceneGraph, queue_size=1)

        self._risk_value = None
        self._risk_lock = threading.Lock()
        rospy.Subscriber('safety/risk_val', Float64, self._save_risk_value, queue_size=1)

        self.image_rgb_observation = None       # to use in next action (ROS message)
        self.image_rgb_last = None              # last image used to compute scene graph effectively, action!=2 (image)
        self.image_depth_observation = None
        self.image_depth_last = None

        self._image_rgb_current = None          # most recent from topic (ROS message)
        self._image_depth_current = None
        self._image_current_lock = threading.Lock()

        # with real scene graph generation the depth image is required
        # with scene graph extraction from V-REP, the depth camera could be disabled for a faster simulation
        if vrep_scene_graph_extraction:
            rospy.Subscriber('camera/rgb/raw_image', Image, self._save_camera_image_rgb, queue_size=1)
        else:
            camera_rgb_sub = message_filters.Subscriber('camera/rgb/raw_image', Image)
            camera_depth_sub = message_filters.Subscriber('camera/depth/raw_image', Image)
            camera_sub = message_filters.TimeSynchronizer([camera_rgb_sub, camera_depth_sub], queue_size=1)
            camera_sub.registerCallback(self._save_camera_image)

        self.observation = None
        self._observation_last = None
        self._action = None
        self._reward = None
        self._step = None

        self._rng = None
        self.seed()
        self.reset()

        # only for TaskOffloadingLogger
        self.latency = None
        self.energy = None
        self.published = False

    def step(self, action):
        assert self.action_space.contains(action)

        if action == 0:
            response = self._compute_on_robot()
        elif action == 1:
            response = self._compute_on_edge()
        elif action == 2:
            response = self._use_last_output()
        else:
            raise ValueError

        # there is no scene graph if action=1 and the network is not available
        if response.scene_graph.sg_data != '':
            self._scene_graph_pub.publish(response.scene_graph)
            self.published = True
        else:
            rospy.logwarn('Bad decision, no scene graph available')
            self.published = False

        self._observation_last = self.observation
        self._action = action
        self._step += 1
        self.observation = self._observe()
        self._reward = self._get_reward(response)
        done = self._done()

        return self.observation, self._reward, done, {}

    def reset(self):
        self._pick_and_place_navigator.stop()
        if self.vrep_simulation:
            self._vrep_scene_controller.reset()
        self._pick_and_place_navigator.start()

        self.image_rgb_observation = None
        self.image_rgb_last = None
        self.image_depth_observation = None
        self.image_depth_last = None
        with self._image_current_lock:
            self._image_rgb_current = None
            self._image_depth_current = None
        with self._risk_lock:
            self._risk_value = None
        with self._collision_lock:
            self._collision = False

        # compute once on robot so that action=2 is valid
        self.observation = self._observe()
        response = self._compute_on_robot()
        self._scene_graph_last = response.scene_graph
        self.scene_graph_from_edge = False

        self.observation = self._observe()
        self._step = 1

        rospy.loginfo('Environment reset')
        return self.observation

    def render(self, mode='human'):
        # TODO: render graphically (e.g. video with camera image + data overlapped)
        if mode == 'human':
            rospy.loginfo('Step = %d' % self._step)
            rospy.loginfo('Last observation = %s' % str(self._observation_last))
            rospy.loginfo('New observation = %s' % str(self.observation))
            rospy.loginfo('Action = %s' % str(self._action))
            rospy.loginfo('Reward = %f' % self._reward)

        elif mode == 'ansi':
            output = ('Step = %d\n' % self._step) + \
                     ('Last observation = %s\n' % self._observation_last) + \
                     ('New observation = %s\n' % self.observation) + \
                     ('Action = %d\n' % self._action) + \
                     ('Reward = %f' % self._reward)
            return output

        else:
            super(TaskOffloadingEnv, self).render(mode=mode)

    def close(self):
        self._pick_and_place_navigator.stop()
        if self.vrep_simulation:
            self._vrep_scene_controller.close_connection()

    def seed(self, seed=None):
        self._rng, seed_ = np_random(seed)
        self._pick_and_place_navigator.seed(seed)
        if seed is not None:
            rospy.loginfo('Seed set to %d' % seed)
        return [seed_]

    def _observe(self):
        response = self._measure_network(
            max_rtt=rospy.Duration.from_sec(self.max_rtt),
            max_duration_throughput=rospy.Duration.from_sec(self.max_duration_throughput)
        )
        rtt = response.rtt.to_sec() * 1e3
        throughput = response.throughput

        # most recent images (while loop because the camera image messages are filtered by the time synchronizer)
        self.image_rgb_observation = None
        while self.image_rgb_observation is None:
            with self._image_current_lock:
                self.image_rgb_observation = self._image_rgb_current
                self.image_depth_observation = self._image_depth_current
            if self.image_rgb_observation is None:
                rospy.wait_for_message('camera/rgb/raw_image', Image)

        # temporal coherence is in [0,1] (pixels are scaled to [0,1])
        image_rgb = self._cv_bridge.imgmsg_to_cv2(self.image_rgb_observation, 'rgb8')
        if self.image_rgb_last is not None:
            temporal_coherence = 1 - np.mean(np.abs(image_rgb - self.image_rgb_last)) / 255
            if self.image_depth_observation is not None:
                image_depth = self._cv_bridge.imgmsg_to_cv2(self.image_depth_observation, 'passthrough')
                temporal_coherence_depth = 1 - np.mean(np.abs(image_depth - self.image_depth_last)) / 255
                temporal_coherence = (temporal_coherence + temporal_coherence_depth) / 2
        else:
            temporal_coherence = 0      # no previous computation => no temporal coherence

        if self.image_depth_observation is None:
            image_depth = np.zeros(self.depth_image_size)
            self.image_depth_observation = self._cv_bridge.cv2_to_imgmsg(image_depth, 'passthrough')

        with self._risk_lock:
            risk_value = self._risk_value
        if risk_value is None:
            risk_value = self.max_risk_value    # unknown => high

        return {
            'rtt': rtt,
            'throughput': throughput,
            'risk_value': risk_value,
            'temporal_coherence': temporal_coherence
        }

    def _get_reward(self, response):
        # no scene graph => +0
        if response.scene_graph.sg_data == '':
            self.latency = np.inf
            self.energy = np.inf
            return 0

        communication_latency = response.communication_latency.to_sec()
        execution_latency = response.execution_latency.to_sec()
        self.latency = communication_latency + execution_latency
        risk_value = self._observation_last['risk_value']
        temporal_coherence = self._observation_last['temporal_coherence'] if self._action == 2 else 1

        if self._action == 0:
            self.energy = self.robot_transmit_power * communication_latency
        elif self._action == 1:
            self.energy = self.robot_compute_power * execution_latency
        else:
            self.energy = 0

        # latency=0.1 s => +1 (the lower, the better... saturates at 5)
        reward_latency = self.w_latency * min(0.1 / self.latency if self.latency != 0 else 5, 5)

        # energy=0.1 W => +1 (the lower, the better... saturates at 5)
        reward_energy = self.w_energy * min(1 / self.energy if self.energy != 0 else 5, 5)

        # edge => +1 (better instance segmentation)
        reward_accuracy = 1 if self.scene_graph_from_edge else 0

        # the higher risk, the more we are interested in having good latency and accurate scene graph
        # risk_value+1 so that when risk_value=0 the robot is motivated to decide well anyway
        reward_partial = (risk_value+1) * (reward_latency + reward_accuracy) + reward_energy

        # temporal_coherence=0 => -reward_partial => reward=0
        # temporal_coherence=1 => -0 (no penalty) => reward=reward_partial
        # 0 < temporal_coherence < 1 => degree 4 trend (to be high, it must be very coherent)
        reward_temporal_coherence = (temporal_coherence**4 - 1) * reward_partial
        reward = reward_partial + reward_temporal_coherence

        return reward

    def _done(self):
        with self._collision_lock:
            if self._collision:
                return True
        if self._pick_and_place_navigator.completed_pick_and_place >= self.pick_and_place_per_episode:
            return True
        return False

    def _compute_on_robot(self):
        response = self._generate_scene_graph_on_robot(
            header=Header(stamp=rospy.Time.now()),
            image_rgb=self.image_rgb_observation,
            image_depth=self.image_depth_observation
        )
        self._scene_graph_last = response.scene_graph
        self.scene_graph_from_edge = False
        self.image_rgb_last = self._cv_bridge.imgmsg_to_cv2(self.image_rgb_observation, 'rgb8')
        self.image_depth_last = self._cv_bridge.imgmsg_to_cv2(self.image_depth_observation, 'passthrough')
        return response

    def _compute_on_edge(self):
        image_rgb = self._cv_bridge.imgmsg_to_cv2(self.image_rgb_observation, 'rgb8')
        image_depth = self._cv_bridge.imgmsg_to_cv2(self.image_depth_observation, 'passthrough')
        image_rgb = cv2.resize(image_rgb, self.network_image_size)
        image_depth = cv2.resize(image_depth, self.network_image_size)
        image_rgb = self._cv_bridge.cv2_to_imgmsg(image_rgb, 'rgb8')
        image_depth = self._cv_bridge.cv2_to_imgmsg(image_depth, 'passthrough')

        try:
            self._generate_scene_graph_on_edge.wait_for_service(timeout=0.1)
            response = self._generate_scene_graph_on_edge(
                header=Header(stamp=rospy.Time.now()),
                image_rgb=image_rgb,
                image_depth=image_depth
            )
            self._scene_graph_last = response.scene_graph
            self.scene_graph_from_edge = True
            self.image_rgb_last = self._cv_bridge.imgmsg_to_cv2(self.image_rgb_observation, 'rgb8')
            self.image_depth_last = self._cv_bridge.imgmsg_to_cv2(self.image_depth_observation, 'passthrough')
        except (rospy.ROSException, rospy.ServiceException):
            response = GenerateSceneGraphResponse()

        return response

    def _use_last_output(self):
        self._scene_graph_last.header.stamp = rospy.Time.now()
        response = GenerateSceneGraphResponse(
            communication_latency=rospy.Duration(0, 0),
            execution_latency=rospy.Duration(0, 0),
            scene_graph=self._scene_graph_last
        )
        return response

    def _save_camera_image_rgb(self, image_rgb):
        with self._image_current_lock:
            self._image_rgb_current = image_rgb

    def _save_camera_image(self, image_rgb, image_depth):
        with self._image_current_lock:
            self._image_rgb_current = image_rgb
            self._image_depth_current = image_depth

    def _save_risk_value(self, risk_value):
        with self._risk_lock:
            self._risk_value = risk_value.data

    def _save_collision(self, event):
        if event.state == 1:
            with self._collision_lock:
                self._collision = True
                rospy.logwarn('Collision')
