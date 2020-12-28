import threading
import numpy as np
import rospy
import message_filters
from gym import Env
from gym.spaces import Discrete, Dict, Box
from sensor_msgs.msg import Image


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

        self._image_rgb = None
        self._image_depth = None
        self._image_is_new = False
        self._image_cv = threading.Condition()          # rospy callbacks are not thread-safe

        camera_rgb_sub = message_filters.Subscriber('camera_rgb', Image)
        camera_depth_sub = message_filters.Subscriber('camera_depth', Image)
        ts = message_filters.TimeSynchronizer([camera_rgb_sub, camera_depth_sub], queue_size=1)
        ts.registerCallback(self._save_camera_image)

        # publish synchronously (queue_size=None) to compute energy consumption TODO: corretto? mi serve?
        self._robot_rgb_pub = rospy.Publisher('robot_rgb', Image)
        self._robot_depth_pub = rospy.Publisher('robot_depth', Image)
        self._edge_rgb_pub = rospy.Publisher('edge_rgb', Image)
        self._edge_depth_pub = rospy.Publisher('edge_depth', Image)

    def step(self, action):
        assert self.action_space.contains(action)

        with self._image_cv:
            self._image_cv.wait_for(lambda: self._image_is_new)
            
            if action == 0:
                self._robot_rgb_pub.publish(self._image_rgb)
                self._robot_depth_pub.publish(self._image_depth)
            elif action == 1:
                self._edge_rgb_pub.publish(self._image_rgb)
                self._edge_depth_pub.publish(self._image_depth)

            self._image_is_new = False

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
        # TODO
        return [seed]

    def _save_camera_image(self, image_rgb, image_depth):
        with self._image_cv:
            self._image_rgb = image_rgb
            self._image_depth = image_depth
            self._image_is_new = True
            self._image_cv.notify()
