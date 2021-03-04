import copy
from rl.core import Processor
from gym.spaces import flatten
from turtlebot2i_edge import TaskOffloadingEnv


class TaskOffloadingProcessor(Processor):
    def __init__(self, env):
        self.env = env

    def process_observation(self, observation):
        # min-max scaling
        observation = copy.deepcopy(observation)
        observation['rtt'] = observation['rtt'] / self.env.max_rtt if observation['rtt'] <= self.env.max_rtt else 1
        if observation['throughput'] > 0:
            observation['throughput'] = observation['throughput'] / self.env.bandwidth
        if observation['risk_value'] > 0:
            observation['risk_value'] /= self.env.max_risk_value

        # flatten
        observation = flatten(TaskOffloadingEnv.observation_space, observation)

        return observation
