#!/usr/bin/env python

from __future__ import print_function

import sys
import argparse
import gym
import rospy
from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam
from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory
from turtlebot2i_edge import NaiveAgent


def get_model(env):
    model = Sequential()
    model.add(Flatten(input_shape=(1,) + env.observation_space.shape))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(env.action_space.n))
    model.add(Activation('linear'))
    print(model.summary())
    return model


def main():
    rospy.init_node('task_offloading')

    rospy.loginfo('Parsing command line arguments...')
    argv = rospy.myargv(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('agent', type=str, choices=['dqn', 'all_robot', 'all_edge', 'random'])
    parser.add_argument('mode', type=str, choices=['train', 'test'])
    args = parser.parse_args(argv[1:])
    rospy.loginfo('Agent: %s' % args.agent)
    rospy.loginfo('Mode: %s' % args.mode)

    rospy.loginfo('Initializing environment...')
    env = gym.make('TaskOffloading-v0')
    env.seed(1)

    rospy.loginfo('Initializing agent...')
    if args.agent == 'dqn':
        agent = DQNAgent(
            model=get_model(env),
            nb_actions=env.action_space.n,
            memory=SequentialMemory(limit=50000, window_length=1),
            nb_steps_warmup=10,
            target_model_update=1e-2,
            policy=BoltzmannQPolicy()
        )
        agent.compile(Adam(lr=1e-3), metrics=['mae'])
        if args.mode != 'train':
            agent.load_weights('%s_weights.h5f' % args.agent)
    else:
        agent = NaiveAgent(mode=args.agent)
        if args.mode == 'train':
            raise ValueError('Naive agents do not need to be trained')

    if args.mode == 'train':
        rospy.loginfo('Training...')
        agent.fit(
            env=env,
            nb_steps=1000,
            visualize=True,
            verbose=2
        )
        agent.save_weights('%s_weights.h5f' % args.agent, overwrite=True)
    elif args.mode == 'test':
        rospy.loginfo('Testing...')
        agent.test(
            env=env,
            nb_episodes=5,
            visualize=True
        )
    else:
        raise ValueError


if __name__ == '__main__':
    main()
