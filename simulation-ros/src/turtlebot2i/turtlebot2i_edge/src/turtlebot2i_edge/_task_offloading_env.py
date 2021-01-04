import threading
import numpy as np
import rospy
import message_filters
from gym import Env
from gym.spaces import Discrete, Dict, Box
from gym.utils.seeding import np_random
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_edge.srv import GenerateSceneGraph, GenerateSceneGraphRequest


class TaskOffloadingEnv(Env):
    metadata = {'render.modes': ['human', 'ansi']}

    action_space = Discrete(2)  # 0 -> robot, 1 -> edge
    observation_space = Dict(
        network=Box(low=0, high=np.inf, shape=(2,)),    # latency, throughput (?) TODO
        risk=Box(low=0, high=np.inf, shape=(1,))        # risk level (?)
        # TODO
    )

    def __init__(self):
        super(TaskOffloadingEnv, self).__init__()

        self._camera_image_rgb = None
        self._camera_image_depth = None
        self._camera_image_lock = threading.Lock()      # rospy callbacks are not thread-safe

        camera_rgb_sub = message_filters.Subscriber('camera/rgb/raw_image', Image)
        camera_depth_sub = message_filters.Subscriber('camera/depth/raw_image', Image)
        camera_sub = message_filters.TimeSynchronizer([camera_rgb_sub, camera_depth_sub], queue_size=1)
        camera_sub.registerCallback(self._save_camera_image)

        self._scene_graph_pub = rospy.Publisher('scene_graph', SceneGraph, queue_size=1)

        rospy.loginfo('Waiting for ROS services to generate scene graph on robot and on edge...')
        rospy.wait_for_service('robot/generate_scene_graph')
        rospy.wait_for_service('edge/generate_scene_graph')

        self._generate_scene_graph_robot = rospy.ServiceProxy(
            name='robot/generate_scene_graph',
            service_class=GenerateSceneGraph,
        )
        self._generate_scene_graph_edge = rospy.ServiceProxy(
            name='edge/generate_scene_graph',
            service_class=GenerateSceneGraph,
            persistent=True     # reduce overhead over the network
        )

        self.rng = None
        self.seed()

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

        rospy.loginfo('Communication latency = %d.%06d s' % (response.communication_latency.secs, response.communication_latency.nsecs))
        rospy.loginfo('Execution latency = %d.%06d s' % (response.execution_latency.secs, response.execution_latency.nsecs))

        # TODO
        observation = None
        reward = None
        done = False
        info = None

        return observation, reward, done, info

    def reset(self):
        pass    # TODO

    def render(self, mode='human'):
        if mode == 'human':
            pass    # TODO
        elif mode == 'ansi':
            pass    # TODO
        else:
            super(TaskOffloadingEnv, self).render(mode=mode)

    def close(self):
        pass    # TODO

    def seed(self, seed=None):
        self.rng, seed = np_random(seed)
        return [seed]

    def _save_camera_image(self, image_rgb, image_depth):
        with self._camera_image_lock:
            self._camera_image_rgb = image_rgb
            self._camera_image_depth = image_depth
