#!/usr/bin/env python

import gym
import rospy
import turtlebot2i_edge     # DO NOT remove, this registers gym environment

EPISODES = 1
TIME_STEPS = 1000


def main():
    rospy.init_node('task_offloading')
    env = gym.make('TaskOffloading-v0')

    for i in range(EPISODES):
        observation = env.reset()

        for t in range(TIME_STEPS):
            env.render()

            # action = env.action_space.sample()  # random
            action = 0                          # all robot
            # action = 1                          # all edge

            observation, reward, done, info = env.step(action)

            if done:
                break

    env.close()


if __name__ == '__main__':
    main()
