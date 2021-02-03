import numpy as np
from rl.callbacks import Callback


class TaskOffloadingLogger(Callback):
    def __init__(self, filepath, interval=100):
        super(TaskOffloadingLogger, self).__init__()
        if not filepath.endswith('.npz'):
            filepath += '.npz'
        self.filepath = filepath
        self.interval = interval
        self.logs = None

    def on_episode_begin(self, episode, logs=None):
        if self.logs is None:
            self.logs = []      # init here and not in the constructor to avoid warning in on_step_end()
        self.logs.append({
            'action': [],
            'reward': [],
            'latency': [],
            'energy': [],
            'risk_value': [],
            'temporal_coherence': [],
            'published': [],
            'edge_model': []
        })

    def on_step_end(self, step, logs=None):
        episode = logs['episode']
        episode_logs = self.logs[episode]
        episode_logs['action'].append(logs['action'])
        episode_logs['reward'].append(logs['reward'])
        episode_logs['latency'].append(self.env.latency)
        episode_logs['energy'].append(self.env.energy)
        episode_logs['risk_value'].append(self.env.observation['risk_value'])
        episode_logs['temporal_coherence'].append(self.env.observation['temporal_coherence'])
        episode_logs['published'].append(self.env.published)
        episode_logs['edge_model'].append(self.env.scene_graph_from_edge)

        if step % self.interval == 0:
            self._save()

    def on_episode_end(self, episode, logs=None):
        self._save()

    def _save(self):
        np.savez(self.filepath, logs=self.logs)
