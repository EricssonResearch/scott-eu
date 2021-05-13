#!/usr/bin/env python

import os
import rospy
from gym.spaces import flatdim
from keras import Input, Model
from keras.layers import Flatten, Dense
from rl.agents.dqn import DQNAgent
from rl.memory import SequentialMemory
from rl.callbacks import ModelIntervalCheckpoint, TrainIntervalLogger
from turtlebot2i_edge import TaskOffloadingEnv, TaskOffloadingProcessor, TaskOffloadingLogger, SafetyLogger, NaiveAgent


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
    seed = rospy.get_param('~env/seed')
    vrep_host = rospy.get_param('~vrep/host')
    vrep_port = rospy.get_param('~vrep/port')
    pick_goals = rospy.get_param('/pick_and_place/goals/pick')
    place_goals = rospy.get_param('/pick_and_place/goals/place')
    max_rtt = rospy.get_param('~env/observation/max_rtt')
    max_duration_throughput = rospy.get_param('~env/observation/max_duration_throughput')
    good_latency = rospy.get_param('~env/reward/good_latency')
    network_speed = rospy.get_param('~env/observation/network_speed')
    robot_compute_power = rospy.get_param('~env/reward/robot/compute_power')
    robot_transmit_power = rospy.get_param('~env/reward/robot/transmit_power')
    max_latency = rospy.get_param('~env/task/max_latency')
    max_risk_value = rospy.get_param('~env/observation/max_risk_value')
    network_image_size = rospy.get_param('~env/task/network_image_size')
    depth_image_size = rospy.get_param('~env/task/depth_image_size')
    models_path = rospy.get_param('~paths/models')
    logs_path = rospy.get_param('~paths/logs')
    network_image_size = (network_image_size['width'], network_image_size['height'])
    depth_image_size = (depth_image_size['width'], depth_image_size['height'])

    rospy.loginfo('Agent: %s' % agent)
    rospy.loginfo('Mode: %s' % mode)
    rospy.loginfo('Seed: %d' % seed)
    rospy.loginfo('V-REP remote API server: %s:%d' % (vrep_host, vrep_port))
    rospy.loginfo('Goals where to pick products: %s' % str(pick_goals))
    rospy.loginfo('Goals where to place products: %s' % str(place_goals))
    rospy.loginfo('Max RTT: %f ms' % max_rtt)
    rospy.loginfo('Max duration of throughput measurement: %f s' % max_duration_throughput)
    rospy.loginfo('Good latency (+1 in reward): %f s' % good_latency)
    rospy.loginfo('Network speed: %f Mbps' % network_speed)
    rospy.loginfo('Robot compute power: %f W' % robot_compute_power)
    rospy.loginfo('Robot transmit power: %f W' % robot_transmit_power)
    rospy.loginfo('Max latency: %f' % max_latency)
    rospy.loginfo('Max risk value: %f' % max_risk_value)
    rospy.loginfo('Image size over the network: (%d,%d)' % (network_image_size[0], network_image_size[1]))
    rospy.loginfo('Depth image size (if depth camera disabled): (%d,%d)' % (depth_image_size[0], depth_image_size[1]))
    rospy.loginfo('Models path: %s' % models_path)
    rospy.loginfo('Logs path: %s' % logs_path)

    if mode != 'train' and mode != 'test':
        raise ValueError('Invalid mode')
    training = mode == 'train'

    if not os.path.exists(models_path):
        os.mkdir(models_path)
    model_path = os.path.join(models_path, '%s.h5' % agent)
    logs_path = os.path.join(logs_path, agent, 'seed_%d' % seed, 'logs')
    if not os.path.exists(logs_path):
        os.makedirs(logs_path)
    logs_path_offloading = os.path.join(logs_path, 'logs_offloading%s' % ('_train' if training else ''))
    logs_path_safety = os.path.join(logs_path, 'logs_safety%s' % ('_train' if training else ''))

    rospy.loginfo('Initializing environment...')
    env = TaskOffloadingEnv(
        pick_and_place_per_epsiode=5,
        max_rtt=max_rtt,
        max_duration_throughput=max_duration_throughput,
        max_risk_value=max_risk_value,
        good_latency=good_latency,
        max_latency=max_latency,
        network_speed=network_speed,
        network_image_size=network_image_size,
        depth_image_size=depth_image_size,
        robot_compute_power=robot_compute_power,
        robot_transmit_power=robot_transmit_power,
        pick_goals=pick_goals,
        place_goals=place_goals,
        vrep_simulation=True,
        vrep_host=vrep_host,
        vrep_port=vrep_port,
        vrep_scene_graph_extraction=True
    )
    env.seed(seed)

    rospy.loginfo('Initializing agent...')
    if agent == 'dqn':
        agent = DQNAgent(
            processor=TaskOffloadingProcessor(env),
            model=get_model(env),
            nb_actions=env.action_space.n,
            memory=SequentialMemory(limit=50000, window_length=1),
        )
        agent.compile(optimizer='adam', metrics=['mae'])
        if not training:
            rospy.loginfo('Loading weights from %s...' % model_path)
            agent.load_weights(model_path)
    elif agent == 'all_robot' or 'all_edge' or 'random':
        agent = NaiveAgent(mode=agent)
        if training:
            raise ValueError('Naive agents do not need to be trained')
    else:
        raise ValueError('Invalid agent')

    if training:
        rospy.loginfo('Training...')
        callbacks = [
            ModelIntervalCheckpoint(model_path, interval=100),
            TrainIntervalLogger(interval=100),
            TaskOffloadingLogger(logs_path_offloading, interval=100),
            SafetyLogger(logs_path_safety, interval=100)
        ]
        agent.fit(env, nb_steps=10000, visualize=True, callbacks=callbacks, verbose=2)
        rospy.loginfo('Saving weights to %s' % model_path)
        agent.save_weights(model_path, overwrite=True)
    else:
        rospy.loginfo('Testing...')
        callbacks = [
            TaskOffloadingLogger(logs_path_offloading, interval=100),
            SafetyLogger(logs_path_safety, interval=100)
        ]
        agent.test(env, nb_episodes=1, visualize=True, callbacks=callbacks, verbose=2)


if __name__ == '__main__':
    main()
