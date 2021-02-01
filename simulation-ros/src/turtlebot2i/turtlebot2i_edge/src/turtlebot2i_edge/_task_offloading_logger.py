import numpy as np
from rl.callbacks import Callback


class TaskOffloadingLogger(Callback):
    def __init__(self, filepath, interval=100):
        super(TaskOffloadingLogger, self).__init__()
        if not filepath.endswith('.npz'):
            filepath += '.npz'
        self.filepath = filepath
        self.interval = interval
        self.history = []

    def on_episode_begin(self, episode, logs=None):
        self.history.append({
            'action': [],
            'reward': [],
            'latency': [],
            'energy': [],
            'published': []
        })

    def on_step_end(self, step, logs=None):
        history = self.history[logs['episode']]
        history['action'].append(logs['action'])
        history['reward'].append(logs['reward'])
        history['latency'].append(self.env.latency)
        history['energy'].append(self.env.energy)

        if step % self.interval == 0:
            np.savez(self.filepath, np.array(self.history))
