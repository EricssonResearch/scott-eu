from __future__ import print_function
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
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import BumperEvent
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_edge.srv import GenerateSceneGraph, GenerateSceneGraphRequest, GenerateSceneGraphResponse

# TODO: parametrize (maybe robot and edge have different image size? will see when I look for good networks)
IMAGE_SIZE = (224, 224)
HIGH_RISK_VALUE = 5     # from risk assessment


class TaskOffloadingEnv(Env):
    """
    TODO: write description, see https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py
    actions: 0 -> robot, 1 -> edge, 3 -> past robot, 4 -> past edge

    observations:
    - rtt: min, avg, max, mdev
    - packet_loss (percentage [0,1])
    - throughput (bit/s uplink)
    - risk_value
    - temporal_incoherence (mean of absolute differences): last robot rgb, last robot depth, last edge rgb, last edge depth

    TODO: add confidence (in observation and reward)
    """

    metadata = {'render.modes': ['human', 'ansi']}

    action_space = Discrete(4)
    observation_space = Dict(
        rtt=Box(low=0, high=np.inf, shape=(4,), dtype=np.float32),
        packet_loss=Box(low=0, high=1, shape=(1,), dtype=np.float32),
        throughput=Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
        risk_value=Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
        temporal_incoherence=Box(low=0, high=np.inf, shape=(4,), dtype=np.float32)
    )

    def __init__(self, vrep_host, vrep_port, mec_server, move_base_goals, robot_name,
                 robot_compute_power, robot_transmit_power, w_latency, w_energy):
        super(TaskOffloadingEnv, self).__init__()

        self.move_base_goals = move_base_goals
        self.robot_compute_power = robot_compute_power
        self.robot_transmit_power = robot_transmit_power
        self.w_latency = w_latency
        self.w_energy = w_energy

        self._vrep_robot_controller = VrepRobotController()
        self._vrep_robot_controller.start_connection(vrep_host, vrep_port)
        self._vrep_robot_controller.load_robot(robot_name)
        self._network_monitor = NetworkMonitor(mec_server, throughput=True)
        self._cv_bridge = CvBridge()                        # for conversions ROS message <-> image

        rospy.loginfo('Waiting for ROS action server to move base...')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        self._position_goal = None
        self._position_current = None
        self._position_current_lock = threading.Lock()

        rospy.loginfo('Waiting for ROS services to generate scene graph on robot...')
        self._generate_scene_graph_robot = rospy.ServiceProxy('robot/generate_scene_graph', GenerateSceneGraph)
        self._generate_scene_graph_robot.wait_for_service()

        rospy.loginfo('Waiting for ROS services to generate scene graph on edge...')
        self._generate_scene_graph_edge = rospy.ServiceProxy('edge/generate_scene_graph', GenerateSceneGraph)
        self._generate_scene_graph_edge.wait_for_service()

        self._scene_graph_last_robot = None
        self._scene_graph_last_edge = None
        self._scene_graph_pub = rospy.Publisher('scene_graph', SceneGraph, queue_size=1)

        self._collision = False
        self._collision_lock = threading.Lock()
        rospy.Subscriber('events/bumper', BumperEvent, self._save_collision)

        self._risk_value = None
        self._risk_lock = threading.Lock()
        rospy.Subscriber('safety/risk_val', Float64, self._save_risk_value, queue_size=1)

        self._camera_image_rgb_current = None               # to use in next action (ROS message)
        self._camera_image_depth_current = None
        self._camera_image_rgb_last_robot = None            # used in last action on robot (image)
        self._camera_image_depth_last_robot = None
        self._camera_image_rgb_last_edge = None             # used in last action on edge (image)
        self._camera_image_depth_last_edge = None
        self._camera_image_rgb_last = None                  # most recent from topic (ROS message)
        self._camera_image_depth_last = None
        self._camera_image_last_lock = threading.Lock()
        camera_rgb_sub = message_filters.Subscriber('camera/rgb/raw_image', Image)
        camera_depth_sub = message_filters.Subscriber('camera/depth/raw_image', Image)
        camera_sub = message_filters.TimeSynchronizer([camera_rgb_sub, camera_depth_sub], queue_size=1)
        camera_sub.registerCallback(self._save_camera_image)

        self._rng = None
        self._last_observation = None
        self._new_observation = None
        self._action = None
        self._reward = None
        self._step = None

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
            response = self._generate_scene_graph_robot(request)
            self._scene_graph_last_robot = response.scene_graph
            self._camera_image_rgb_last_robot = self._cv_bridge.imgmsg_to_cv2(self._camera_image_rgb_current, 'rgb8')
            self._camera_image_depth_last_robot = self._cv_bridge.imgmsg_to_cv2(self._camera_image_depth_current,
                                                                                'passthrough')

        elif action == 1:
            response = self._generate_scene_graph_edge(request)
            self._scene_graph_last_edge = response.scene_graph
            self._camera_image_rgb_last_edge = self._cv_bridge.imgmsg_to_cv2(self._camera_image_rgb_current, 'rgb8')
            self._camera_image_depth_last_edge = self._cv_bridge.imgmsg_to_cv2(self._camera_image_depth_current,
                                                                               'passthrough')

        elif action == 2:
            if self._scene_graph_last_robot is not None:
                self._scene_graph_last_robot.header.stamp = rospy.Time.now()
            response = GenerateSceneGraphResponse(
                communication_latency=rospy.Duration(0, 0),
                execution_latency=rospy.Duration(0, 0),
                scene_graph=self._scene_graph_last_robot
            )

        elif action == 3:
            if self._scene_graph_last_edge is not None:
                self._scene_graph_last_edge.header.stamp = rospy.Time.now()
            response = GenerateSceneGraphResponse(
                communication_latency=rospy.Duration(0, 0),
                execution_latency=rospy.Duration(0, 0),
                scene_graph=self._scene_graph_last_edge
            )

        else:
            raise ValueError

        if response.scene_graph.sg_data != '':  # empty if action=2 or action=3 without previous computation
            self._scene_graph_pub.publish(response.scene_graph)

        self._last_observation = self._new_observation
        self._action = action
        self._step += 1
        self._new_observation = self._observe()
        self._reward = self._get_reward(response)
        done = self._done()

        return self._new_observation, self._reward, done, {}

    def reset(self):
        self._move_base_client.cancel_goal()
        self._vrep_robot_controller.reset_robot_position()

        with self._collision_lock:
            self._collision = False

        self._scene_graph_last_robot = None
        self._scene_graph_last_edge = None

        self._camera_image_rgb_current = None
        self._camera_image_depth_current = None
        self._camera_image_rgb_last_robot = np.zeros(IMAGE_SIZE + (3,))
        self._camera_image_depth_last_robot = np.zeros(IMAGE_SIZE)
        self._camera_image_rgb_last_edge = np.zeros(IMAGE_SIZE + (3,))
        self._camera_image_depth_last_edge = np.zeros(IMAGE_SIZE)
        with self._camera_image_last_lock:
            self._camera_image_rgb_last = None
            self._camera_image_depth_last = None

        idx_goal = self._rng.choice(len(self.move_base_goals))
        x, y = self.move_base_goals[idx_goal]
        z = 0.063
        yaw = self._rng.uniform(-np.pi, np.pi)

        pose = Pose(
            position=Point(x, y, z),
            orientation=Quaternion(*list(quaternion_from_euler(0, 0, yaw)))
        )
        pose_stamped = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='map'),
            pose=pose
        )
        goal = MoveBaseGoal(target_pose=pose_stamped)
        self._move_base_client.send_goal(goal, feedback_cb=self._save_position)

        self._position_goal = np.array((x, y))
        with self._position_current_lock:
            self._position_current = None

        self._new_observation = self._observe()
        self._step = 1

        rospy.loginfo('Environment reset with goal %s' % str(self.move_base_goals[idx_goal]))
        return self._new_observation

    def render(self, mode='human'):
        if mode == 'human':
            rospy.loginfo('Step = %d' % self._step)
            rospy.loginfo('Last observation = %s' % str(self._last_observation))
            rospy.loginfo('New observation = %s' % str(self._new_observation))
            rospy.loginfo('Action = %s' % str(self._action))
            rospy.loginfo('Reward = %f' % self._reward)

        elif mode == 'ansi':
            output = ('Step = %d\n' % self._step) + \
                     ('Last observation = %s\n' % self._last_observation) + \
                     ('New observation = %s\n' % self._new_observation) + \
                     ('Action = %d\n' % self._action) + \
                     ('Reward = %f' % self._reward)
            return output

        else:
            super(TaskOffloadingEnv, self).render(mode=mode)

    def close(self):
        pass    # nothing to do

    def seed(self, seed=None):
        self._rng, seed_ = np_random(seed)
        if seed is not None:
            rospy.loginfo('Seed set to %d' % seed)
        return [seed_]

    def _observe(self):
        self._camera_image_rgb_current = None
        self._camera_image_depth_current = None
        while self._camera_image_rgb_current is None or self._camera_image_depth_current is None:
            with self._camera_image_last_lock:
                self._camera_image_rgb_current = self._camera_image_rgb_last
                self._camera_image_depth_current = self._camera_image_depth_last
            if self._camera_image_rgb_current is None:
                rospy.wait_for_message('camera/rgb/raw_image', Image)
            if self._camera_image_rgb_current is None:
                rospy.wait_for_message('camera/depth/raw_image', Image)

        camera_image_rgb_current = self._cv_bridge.imgmsg_to_cv2(self._camera_image_rgb_current, 'rgb8')
        camera_image_depth_current = self._cv_bridge.imgmsg_to_cv2(self._camera_image_depth_current, 'passthrough')
        camera_image_rgb_current = cv2.resize(camera_image_rgb_current, IMAGE_SIZE)
        camera_image_depth_current = cv2.resize(camera_image_depth_current, IMAGE_SIZE)
        self._camera_image_rgb_current = self._cv_bridge.cv2_to_imgmsg(camera_image_rgb_current, 'rgb8')
        self._camera_image_depth_current = self._cv_bridge.cv2_to_imgmsg(camera_image_depth_current, 'passthrough')

        # between 0 and 1
        temporal_incoherence = (
            np.mean(np.abs(camera_image_rgb_current - self._camera_image_rgb_last_robot)) / 255,
            np.mean(np.abs(camera_image_depth_current - self._camera_image_depth_last_robot)) / 255,
            np.mean(np.abs(camera_image_rgb_current - self._camera_image_rgb_last_edge)) / 255,
            np.mean(np.abs(camera_image_depth_current - self._camera_image_depth_last_edge)) / 255
        )

        rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_loss = self._network_monitor.measure_latency()
        throughput = self._network_monitor.measure_throughput()

        with self._risk_lock:
            risk_value = self._risk_value
        if risk_value is None:
            risk_value = HIGH_RISK_VALUE        # unknown => high

        return {
            'rtt': (rtt_min, rtt_avg, rtt_max, rtt_mdev),
            'packet_loss': packet_loss,
            'throughput': throughput,
            'risk_value': risk_value,
            'temporal_incoherence': temporal_incoherence
        }

    def _get_reward(self, response):
        communication_latency = response.communication_latency.to_sec()
        execution_latency = response.execution_latency.to_sec()
        latency = communication_latency + execution_latency
        risk_value = self._last_observation['risk_value']

        if self._action == 0:
            energy = self.robot_transmit_power * communication_latency
        elif self._action == 1:
            energy = self.robot_compute_power * execution_latency
        else:
            energy = 0

        if response.scene_graph.sg_data != '':  # empty if action=2 or action=3 without previous computation
            temporal_incoherence = 1            # error causing missing scene graph, high penalty
        elif self._action == 2:
            temporal_incoherence = np.mean(self._last_observation['temporal_incoherence'][:2])
        elif self._action == 3:
            temporal_incoherence = np.mean(self._last_observation['temporal_incoherence'][2:])
        else:
            temporal_incoherence = 0

        # latency=0.1 s => +1 (the lower, the better... saturates at 5)
        reward_latency = self.w_latency * max(0.1 / latency if latency != 0 else 5, 5)
        reward_energy = self.w_energy * max(1 / energy if energy != 0 else 5, 5)

        # edge => +1 (better instance segmentation)
        reward_model = 1 if (self._action == 1 or self._action == 3) else 0

        # temporal_incoherence=1 => -10 (so high because there is +5 from latency/energy)
        # temporal_incoherence=0 => 0
        # 0 < temporal_incoherence < 1 => sigmoid trend (gets quickly high)
        reward_temporal_incoherence = - 10 * (2 / (1 + np.exp(-5*temporal_incoherence)) - 1)

        # +1 so that when risk is 0, the robot is motivated to decide well anyway
        reward = (risk_value+1) * (reward_latency + reward_energy + reward_model + reward_temporal_incoherence)

        print('risk_value: %f' % risk_value)
        print('reward_latency: %f' % reward_latency)
        print('reward_energy: %f' % reward_energy)
        print('reward_model: %f' % reward_model)
        print('reward_temporal_incoherence: %f' % reward_temporal_incoherence)
        print('reward: %f' % reward)

        return reward

    def _done(self):
        with self._collision_lock:
            if self._collision:
                return True

        # move base does not reach exactly the goal, so we allow an error margin of 0.7
        # TODO: why? see xy_goal_tolerance parameter of navigation
        with self._position_current_lock:
            if self._position_current is not None:
                return np.linalg.norm(self._position_goal - self._position_current) < 0.7

    def _save_position(self, feedback):
        with self._position_current_lock:
            x = feedback.base_position.pose.position.x
            y = feedback.base_position.pose.position.y
            self._position_current = np.array((x, y))

    def _save_camera_image(self, image_rgb, image_depth):
        with self._camera_image_last_lock:
            self._camera_image_rgb_last = image_rgb
            self._camera_image_depth_last = image_depth

    def _save_risk_value(self, risk_value):
        with self._risk_lock:
            self._risk_value = risk_value.data

    def _save_collision(self, event):
        if event.state == 1:
            with self._collision_lock:
                self._collision = True
                rospy.logwarn('Collision')
