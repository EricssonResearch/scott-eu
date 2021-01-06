import numpy as np
from gym.envs.registration import register
from turtlebot2i_edge._task_offloading_env import TaskOffloadingEnv
from turtlebot2i_edge._naive_agent import NaiveAgent
from turtlebot2i_edge._vrep_robot_controller import VrepRobotController
from turtlebot2i_edge._network_monitor import NetworkMonitor

register(
    id='TaskOffloading-v0',
    entry_point='turtlebot2i_edge:TaskOffloadingEnv',
    kwargs={
        'vrep_remote_controller': None,
        'x_lim': (-np.inf, np.inf),
        'y_lim': (-np.inf, np.inf),
        'transmit_power': 100e-3,
        'w_latency': 0.33,
        'w_energy': 0.33,
        'w_risk': 0.33
    }
)
