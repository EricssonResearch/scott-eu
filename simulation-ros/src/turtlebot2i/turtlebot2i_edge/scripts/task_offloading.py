#!/usr/bin/env python

from __future__ import print_function

import os
import rospy
from gym.spaces import flatdim
from keras import Input, Model
from keras.layers import Flatten, Dense
from keras.optimizers import Adam
from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory
from rl.callbacks import ModelIntervalCheckpoint, TrainIntervalLogger, FileLogger
from turtlebot2i_edge import TaskOffloadingEnv, TaskOffloadingProcessor, NaiveAgent


def get_model(env):
    inputs = Input(shape=(1, flatdim(env.observation_space)))
    x = Flatten()(inputs)
    x = Dense(16, activation='relu')(x)
    x = Dense(16, activation='relu')(x)
    x = Dense(16, activation='relu')(x)
    outputs = Dense(env.action_space.n)(x)

    model = Model(inputs=inputs, outputs=outputs)
    model.summary()
    return model


def main():
    rospy.init_node('task_offloading')

    rospy.loginfo('Getting parameters...')
    agent = rospy.get_param('~agent')
    mode = rospy.get_param('~mode')
    vrep_host = rospy.get_param('~vrep/host')
    vrep_port = rospy.get_param('~vrep/port')
    pick_goals = rospy.get_param('/pick_and_place/goals/pick')
    place_goals = rospy.get_param('/pick_and_place/goals/place')
    robot_compute_power = rospy.get_param('~rl/robot/compute_power')
    robot_transmit_power = rospy.get_param('~rl/robot/transmit_power')
    w_latency = rospy.get_param('~rl/reward/w_latency')
    w_energy = rospy.get_param('~rl/reward/w_energy')
    models_path = rospy.get_param('~rl/models/path')

    rospy.loginfo('Agent: %s' % agent)
    rospy.loginfo('Mode: %s' % mode)
    rospy.loginfo('V-REP remote API server: %s:%d' % (vrep_host, vrep_port))
    rospy.loginfo('Goals where to pick products: %s' % str(pick_goals))
    rospy.loginfo('Goals where to place products: %s' % str(place_goals))
    rospy.loginfo('Robot compute power: %f W' % robot_compute_power)
    rospy.loginfo('Robot transmit power: %f W' % robot_transmit_power)
    rospy.loginfo('Latency weight in reward: %f' % w_latency)
    rospy.loginfo('Energy weight in reward: %f' % w_energy)
    rospy.loginfo('RL models path: %s' % models_path)

    if not os.path.exists(models_path):
        os.mkdir(models_path)
    model_path = os.path.join(models_path, '%s_weights.h5f' % agent)
    log_path = os.path.join(models_path, '%s_logs.h5f' % agent)

    rospy.loginfo('Initializing environment...')
    env = TaskOffloadingEnv(
        pick_goals=pick_goals,
        place_goals=place_goals,
        pick_and_place_per_episode=10,
        robot_compute_power=robot_compute_power,
        robot_transmit_power=robot_transmit_power,
        w_latency=w_latency,
        w_energy=w_energy,
        vrep_simulation=True,
        vrep_host=vrep_host,
        vrep_port=vrep_port,
        vrep_scene_graph_extraction=True
    )
    env.seed(1)

    rospy.loginfo('Initializing agent...')
    if agent == 'dqn':
        agent = DQNAgent(
            processor=TaskOffloadingProcessor(),
            model=get_model(env),
            nb_actions=env.action_space.n,
            memory=SequentialMemory(limit=50000, window_length=1),
            nb_steps_warmup=10,
            target_model_update=1e-2,
            policy=BoltzmannQPolicy()
        )
        agent.compile(Adam(lr=1e-3), metrics=['mae'])
        if mode != 'train':
            rospy.loginfo('Loading weights from %s...' % model_path)
            agent.load_weights(model_path)
    elif agent == 'all_robot' or 'all_edge' or 'random':
        agent = NaiveAgent(mode=agent)
        if mode == 'train':
            raise ValueError('Naive agents do not need to be trained')
    else:
        raise ValueError('Invalid agent')

    # TODO: add metrics
    if mode == 'train':
        rospy.loginfo('Training...')
        agent.fit(
            env=env,
            nb_steps=10000,
            visualize=False,
            callbacks=[ModelIntervalCheckpoint(model_path, 100), TrainIntervalLogger(10), FileLogger(log_path)],
            verbose=2
        )
        rospy.loginfo('Saving weights to %s' % model_path)
        agent.save_weights(model_path, overwrite=True)
    elif mode == 'test':
        rospy.loginfo('Testing...')
        agent.test(
            env=env,
            nb_episodes=1,
            visualize=True,
            verbose=2
        )
    else:
        raise ValueError


if __name__ == '__main__':
    main()
