from __future__ import division

import threading
import numpy as np
import rospy
import actionlib
import message_filters
from gym import Env
from gym.spaces import Discrete, Dict, Box
from gym.utils.seeding import np_random
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_edge.srv import GenerateSceneGraph, GenerateSceneGraphRequest


class TaskOffloadingEnv(Env):
    """
    TODO: write description, see https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py

    TODO: temporal coherence
    """

    metadata = {'render.modes': ['human', 'ansi']}

    action_space = Discrete(2)  # 0 -> robot, 1 -> edge
    observation_space = Dict(
        network=Box(low=0, high=np.inf, shape=(2,)),    # latency, loss, bandwidth
        risk=Box(low=0, high=np.inf, shape=(1,))        # risk level
    )

    def __init__(self, vrep_remote_controller, x_lim, y_lim, transmit_power, w_latency, w_energy, w_risk):
        super(TaskOffloadingEnv, self).__init__()

        self._vrep_robot_controller = vrep_remote_controller
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.transmit_power = transmit_power
        self.w_latency = w_latency
        self.w_energy = w_energy
        self.w_risk = w_risk

        self._rng = None
        self._last_observation = None
        # self._network_monitor = NetworkMonitor()

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

        self._camera_image_rgb = None
        self._camera_image_depth = None
        self._camera_image_lock = threading.Lock()      # rospy callbacks are not thread-safe
        self._scene_graph_pub = rospy.Publisher('scene_graph', SceneGraph, queue_size=1)
        camera_rgb_sub = message_filters.Subscriber('camera/rgb/raw_image', Image)
        camera_depth_sub = message_filters.Subscriber('camera/depth/raw_image', Image)
        camera_sub = message_filters.TimeSynchronizer([camera_rgb_sub, camera_depth_sub], queue_size=1)
        camera_sub.registerCallback(self._save_camera_image)

        self.seed()
        self.reset()

    def step(self, action):
        assert self.action_space.contains(action)

        # TODO: ServiceException (server is too far away)

        request = GenerateSceneGraphRequest(
            header=Header(stamp=rospy.Time.now()),
            image_rgb=self._camera_image_rgb,
            image_depth=self._camera_image_depth
        )

        with self._camera_image_lock:
            if action == 0:
                rospy.loginfo('Computing scene graph on robot...')
                response = self._generate_scene_graph_robot(request)
            elif action == 1:
                rospy.loginfo('Computing scene graph on edge...')
                response = self._generate_scene_graph_edge(request)

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
        pass

    def seed(self, seed=None):
        self._rng, seed = np_random(seed)
        return [seed]

    def _save_camera_image(self, image_rgb, image_depth):
        with self._camera_image_lock:
            self._camera_image_rgb = image_rgb
            self._camera_image_depth = image_depth

    def _sample_pose(self):
        x = np.random.uniform(self.x_lim[0], self.x_lim[1])
        y = np.random.uniform(self.y_lim[0], self.y_lim[1])
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
        # latency, loss, bandwidth = self._network_monitor.monitor()

        # TODO

        return 0

    def _get_reward(self, action, response):
        # TODO: add accuracy of the model or confidence of prediction (??)
        # TOOD: add actions for temporal coherence

        communication_latency = response.communication_latency.secs + 1e-9 * response.communication_latency.nsecs
        execution_latency = response.execution_latency.secs + 1e-9 * response.execution_latency.nsecs
        latency = communication_latency + execution_latency

        energy_consumption = self.transmit_power * communication_latency

        # TODO
        reward = self.w1 * 1 / latency + self.w2 * 1 / energy_consumption + self.w3 * self._last_observation.risk
        return reward

    def _done(self):
        return self._move_base_client.get_state() == GoalStatus.SUCCEEDED
