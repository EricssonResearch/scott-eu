from __future__ import division

import threading
import numpy as np
import cv2
import rospy
import actionlib
import message_filters
from gym import Env
from gym.spaces import Discrete, Dict, Box
from gym.utils.seeding import np_random
from tf.transformations import quaternion_from_euler
from turtlebot2i_edge import VrepRobotController, NetworkMonitor
from cv_bridge import CvBridge
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_edge.srv import GenerateSceneGraph, GenerateSceneGraphRequest, GenerateSceneGraphResponse

# TODO: parametrize image size (maybe robot and edge have different size? will see when I look for good networks)
IMAGE_SIZE = (224, 224)


class TaskOffloadingEnv(Env):
    """
    TODO: write description, see https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py
    actions: 0 -> robot, 1 -> edge, 3 -> past robot, 4 -> past edge

    observations:
    - rtt: min, avg, max, mdev
    - packet_loss (percentage [0,1])
    - throughput (bit/s uplink)
    - risk_value
    - temporal_coherence (sum of absolute differences): last robot rgb, last robot depth, last edge rgb, last edge depth
    - confidence: confidence last robot, confidence last edge (probabilities [0,1])
    """

    metadata = {'render.modes': ['human', 'ansi']}

    action_space = Discrete(4)
    observation_space = Dict(
        rtt=Box(low=0, high=np.inf, shape=(3,), dtype=np.float32),
        packet_loss=Box(low=0, high=1, shape=(1,), dtype=np.float32),
        throughput=Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
        risk_value=Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
        temporal_coherence=Box(low=0, high=np.inf, shape=(4,), dtype=np.float32)
        # TODO: add confidence of prediction in observation
    )

    def __init__(self, vrep_host, vrep_port, mec_server, map_x_lim, map_y_lim, robot_name, robot_compute_power,
                 robot_transmit_power, w_latency, w_energy, w_model):
        super(TaskOffloadingEnv, self).__init__()

        self.map_x_lim = map_x_lim
        self.map_y_lim = map_y_lim
        self.robot_compute_power = robot_compute_power
        self.robot_transmit_power = robot_transmit_power
        self.w_latency = w_latency
        self.w_energy = w_energy
        self.w_model = w_model

        self._vrep_robot_controller = VrepRobotController(robot_name)
        self._vrep_robot_controller.start_connection(vrep_host, vrep_port)
        self._network_monitor = NetworkMonitor(mec_server, throughput=True)
        self._cv_bridge = CvBridge()

        rospy.loginfo('Waiting for ROS action server to move base...')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
        self._check_path = rospy.ServiceProxy('move_base_node/make_plan', GetPlan)
        self._check_path.wait_for_service()

        rospy.loginfo('Waiting for ROS services to generate scene graph on robot and on edge...')
        self._generate_scene_graph_robot = rospy.ServiceProxy('robot/generate_scene_graph', GenerateSceneGraph)
        self._generate_scene_graph_edge = rospy.ServiceProxy(
            name='edge/generate_scene_graph',
            service_class=GenerateSceneGraph,
            persistent=True     # reduce overhead over the network
        )
        self._generate_scene_graph_robot.wait_for_service()
        self._generate_scene_graph_edge.wait_for_service()
        self._scene_graph_last_robot = None
        self._scene_graph_last_edge = None
        self._scene_graph_pub = rospy.Publisher('scene_graph', SceneGraph, queue_size=1)

        self._risk_value = None
        self._risk_lock = threading.Lock()              # rospy callbacks are not thread-safe
        rospy.Subscriber('risk_value', self._save_risk_value, queue_size=1)

        self._camera_image_rgb_current = None           # to use in next action (ROS message)
        self._camera_image_rgb_last = None              # most recent from topic (ROS message)
        self._camera_image_rgb_last_robot = None        # used in last action on robot (image)
        self._camera_image_rgb_last_edge = None         # used in last action on edge (image)
        self._camera_image_depth_current = None
        self._camera_image_depth_last = None
        self._camera_image_depth_last_robot = None
        self._camera_image_depth_last_edge = None
        self._camera_image_lock = threading.Lock()      # rospy callbacks are not thread-safe
        camera_rgb_sub = message_filters.Subscriber('camera/rgb/raw_image', Image)
        camera_depth_sub = message_filters.Subscriber('camera/depth/raw_image', Image)
        camera_sub = message_filters.TimeSynchronizer([camera_rgb_sub, camera_depth_sub], queue_size=1)
        camera_sub.registerCallback(self._save_camera_image)

        self._rng = None
        self._last_observation = None
        self.seed()
        self.reset()

    def step(self, action):
        assert self.action_space.contains(action)

        # TODO: ServiceException (server is too far away)

        request = GenerateSceneGraphRequest(
            header=Header(stamp=rospy.Time.now()),
            image_rgb=self._camera_image_rgb_current,
            image_depth=self._camera_image_depth_current
        )

        if action == 0:
            rospy.loginfo('Computing scene graph on robot...')
            response = self._generate_scene_graph_robot(request)
            self._scene_graph_last_robot = response.scene_graph
            self._camera_image_rgb_last_robot = self._cv_bridge.imgmsg_to_cv2(self._camera_image_rgb_current, 'rgb8')
            self._camera_image_depth_last_robot = self._cv_bridge.imgmsg_to_cv2(self._camera_image_depth_current,
                                                                                'passthrough')

        elif action == 1:
            rospy.loginfo('Computing scene graph on edge...')
            response = self._generate_scene_graph_edge(request)
            self._scene_graph_last_edge = response.scene_graph
            self._camera_image_rgb_last_edge = self._cv_bridge.imgmsg_to_cv2(self._camera_image_rgb_current, 'rgb8')
            self._camera_image_depth_last_edge = self._cv_bridge.imgmsg_to_cv2(self._camera_image_depth_current,
                                                                               'passthrough')

        elif action == 2:
            response = GenerateSceneGraphResponse(
                communication_latency=0,
                execution_latency=0,
                scene_graph=self._scene_graph_last_robot
            )

        elif action == 3:
            response = GenerateSceneGraphResponse(
                communication_latency=0,
                execution_latency=0,
                scene_graph=self._scene_graph_last_edge
            )

        else:
            raise ValueError

        self._scene_graph_pub.publish(response.scene_graph)

        observation = self._observe()
        reward = self._get_reward(action, response)
        done = self._done()
        self._last_observation = observation    # after calculating the reward

        return observation, reward, done, {}

    def reset(self):
        start = None
        goal = None
        reachable = False

        while not reachable:
            start = self._sample_pose()
            goal = self._sample_pose()
            response = self._check_path(start=start, goal=goal)
            reachable = len(response.poses) > 0
        goal = MoveBaseGoal(target_pose=goal)

        self._vrep_robot_controller.move_robot(start)
        self._move_base_client.send_goal(goal)

        self._last_observation = self._observe()
        return self._last_observation

    def render(self, mode='human'):
        if mode == 'human':
            pass    # TODO
        elif mode == 'ansi':
            pass    # TODO
        else:
            super(TaskOffloadingEnv, self).render(mode=mode)

    def close(self):
        pass    # nothing to do

    def seed(self, seed=None):
        self._rng, seed = np_random(seed)
        return [seed]

    def _sample_pose(self):
        x = np.random.uniform(self.map_x_lim[0], self.map_x_lim[1])
        y = np.random.uniform(self.map_y_lim[0], self.map_y_lim[1])
        yaw = np.random.uniform(-np.pi, np.pi)

        pose = Pose(
            position=Point(x, y, 0.063),
            orientation=Quaternion(quaternion_from_euler(0, 0, yaw))
        )
        pose_stamped = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='map'),
            pose=pose
        )
        return pose_stamped

    def _observe(self):
        rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_loss = self._network_monitor.measure_latency()
        throughput = self._network_monitor.measure_throughput()

        with self._risk_lock:
            risk_value = self._risk_value   # TODO: qua e' sbagliato, perche' magari il risk value e' pubblicato dopo che si passa da qui

        with self._camera_image_lock:
            self._camera_image_rgb_current = self._camera_image_rgb_last
            self._camera_image_depth_current = self._camera_image_depth_last

        camera_image_rgb_current = self._cv_bridge.imgmsg_to_cv2(self._camera_image_rgb_current, 'rgb8')
        camera_image_depth_current = self._cv_bridge.imgmsg_to_cv2(self._camera_image_depth_current, 'passthrough')

        camera_image_rgb_current = cv2.resize(camera_image_rgb_current, IMAGE_SIZE)
        camera_image_depth_current = cv2.resize(camera_image_depth_current, IMAGE_SIZE)

        self._camera_image_rgb_current = self._cv_bridge.cv2_to_imgmsg(camera_image_rgb_current, 'rgb8')
        self._camera_image_depth_current = self._cv_bridge.cv2_to_imgmsg(camera_image_depth_current, 'passthrough')

        temporal_coherence = (
            np.sum(camera_image_rgb_current - self._camera_image_rgb_last_robot),
            np.sum(camera_image_depth_current - self._camera_image_depth_last_robot),
            np.sum(camera_image_rgb_current - self._camera_image_rgb_last_edge),
            np.sum(camera_image_depth_current - self._camera_image_depth_last_edge)
        )

        return {
            'rtt': (rtt_min, rtt_avg, rtt_max, rtt_mdev),
            'packet_loss': packet_loss,
            'throughput': throughput,
            'risk_value': risk_value,
            'temporal_coherence': temporal_coherence,
        }

    def _get_reward(self, action, response):
        communication_latency = response.communication_latency.to_sec()
        execution_latency = response.execution_latency.to_sec()
        latency = communication_latency + execution_latency

        if action == 0:
            energy = self.robot_transmit_power * communication_latency
        elif action == 1:
            energy = self.robot_compute_power * execution_latency
        else:
            energy = 0

        # TODO: get from response
        m_ap = 0.5 if action == 0 or action == 3 else 0.8
        confidence = 0.5 if action == 0 or action == 3 else 0.8

        reward_latency = 1 / latency
        reward_energy = 1 / energy
        reward_model = m_ap + confidence

        reward = self.w_latency * reward_latency + self.w_energy * reward_energy + self.w_model * reward_model
        return reward

    def _done(self):
        return self._move_base_client.get_state() == GoalStatus.SUCCEEDED

    def _save_camera_image(self, image_rgb, image_depth):
        with self._camera_image_lock:
            self._camera_image_rgb_last = image_rgb
            self._camera_image_depth_last = image_depth

    def _save_risk_value(self, risk_value):
        with self._risk_lock:
            self._risk_value = risk_value
