from rl.core import Processor
from gym.spaces import flatten
from turtlebot2i_edge import TaskOffloadingEnv


class TaskOffloadingProcessor(Processor):
    def process_observation(self, observation):
        return flatten(TaskOffloadingEnv.observation_space, observation)
