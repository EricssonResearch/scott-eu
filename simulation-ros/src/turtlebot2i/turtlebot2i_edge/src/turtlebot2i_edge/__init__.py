from gym.envs.registration import register
from turtlebot2i_edge._task_offloading_env import TaskOffloadingEnv
from turtlebot2i_edge._naive_agent import NaiveAgent

register(
    id='TaskOffloading-v0',
    entry_point='turtlebot2i_edge:TaskOffloadingEnv',
)
