from gym.envs.registration import register
from turtlebot2i_edge.task_offloading_env import TaskOffloadingEnv

register(
    id='TaskOffloading-v0',
    entry_point='turtlebot2i_edge:TaskOffloadingEnv',
)
