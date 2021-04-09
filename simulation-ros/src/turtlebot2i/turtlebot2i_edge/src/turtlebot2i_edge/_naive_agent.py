import numpy as np
from rl.core import Agent


class NaiveAgent(Agent):
    def __init__(self, mode='random', **kwargs):
        super(NaiveAgent, self).__init__(**kwargs)
        self.mode = mode
        self.compiled = True

        if mode not in {'all_robot', 'all_edge', 'random'}:
            raise ValueError('Invalid mode')

    def forward(self, observation):
        if self.mode == 'all_robot':
            return 0
        elif self.mode == 'all_edge':
            return 1
        elif self.mode == 'random':
            return np.random.choice(2)

    def backward(self, reward, terminal):
        pass

    def compile(self, **kwargs):
        pass

    def load_weights(self, filepath):
        pass

    def save_weights(self, filepath, overwrite=False):
        pass

    @property
    def layers(self):
        return None
