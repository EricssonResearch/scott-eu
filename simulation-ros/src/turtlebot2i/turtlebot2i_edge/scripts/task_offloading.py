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

    rospy.loginfo('Getting parameters...')
    map_x_lim = (rospy.get_param('~map/x/min'), rospy.get_param('~map/x/max'))
    map_y_lim = (rospy.get_param('~map/y/min'), rospy.get_param('~map/y/max'))
    vrep_host = rospy.get_param('~vrep/host')
    vrep_port = rospy.get_param('~vrep/port')
    mec_server = rospy.get_param('/network/mec_server/host')
    robot_name = rospy.get_param('~robot/name')
    robot_compute_power = rospy.get_param('~rl/robot/compute_power')
    robot_transmit_power = rospy.get_param('~rl/robot/transmit_power')
    w_latency = rospy.get_param('~rl/reward/w_latency')
    w_energy = rospy.get_param('~rl/reward/w_energy')
    w_model = rospy.get_param('~rl/reward/w_model')

    rospy.loginfo('Map limit for x: (%f, %f)' % (map_x_lim[0], map_x_lim[1]))
    rospy.loginfo('Map limits for y: (%f, %f)' % (map_y_lim[0], map_y_lim[1]))
    rospy.loginfo('V-REP remote API server: %s:%d' % (vrep_host, vrep_port))
    rospy.loginfo('MEC server: %s' % mec_server)
    rospy.loginfo('Robot name: %s' % robot_name)
    rospy.loginfo('Robot compute power: %f W' % robot_compute_power)
    rospy.loginfo('Robot transmit power: %f W' % robot_transmit_power)
    rospy.loginfo('Latency weight: %f' % w_latency)
    rospy.loginfo('Energy weight: %f' % w_energy)
    rospy.loginfo('Model weight: %f' % w_model)

    rospy.loginfo('Initializing environment...')
    env = gym.make(
        id='TaskOffloading-v0',
        vrep_host=vrep_host,
        vrep_port=vrep_port,
        mec_server=mec_server,
        x_lim=map_x_lim,
        y_lim=map_y_lim,
        robot_name=robot_name,
        robot_compute_power=robot_compute_power,
        robot_transmit_power=robot_transmit_power,
        w_latency=w_latency,
        w_energy=w_energy,
        w_model=w_model
    )
    env.seed(1)

    rospy.loginfo('Initializing agent...')
    filepath = '%s_weights.h5f' % args.agent
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
            rospy.loginfo('Loading weights from %s...' % filepath)
            agent.load_weights(filepath)
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
        rospy.loginfo('Saving weights to %s' % filepath)
        agent.save_weights(filepath, overwrite=True)
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
