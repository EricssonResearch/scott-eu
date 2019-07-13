#!/usr/bin/env python
# Modified from: https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning/blob/master/turtlebot3_dqn/nodes/turtlebot3_dqn_stage_4

import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32MultiArray
from rl_environment import Env
import tensorflow as tf
from keras.models import Sequential, load_model
from keras.optimizers import RMSprop
from keras.layers import Conv1D, GlobalAveragePooling1D, MaxPooling1D
from keras.layers import Dense, Dropout, Activation, Flatten
from turtlebot2i_safety.msg import SafetyRisk


class ReinforceAgent():
    def __init__(self, env, training_mode):
        self.pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
        self.dirPath = os.path.dirname(os.path.realpath(__file__))
        self.dirPath = self.dirPath.replace('scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_safety/src', 'model_and_result/dqn_')
        self.result = Float32MultiArray()

        self.load_model = False
        self.load_episode = 0#340 #0
        self.env = env
        self.stack_len = 20
        self.state_stack = deque(maxlen=self.stack_len)
        if training_mode:
            self.single_state_len = len(self.env.reset())
        else:
            self.single_state_len = len(self.env.getEmptyState())
        self.state_size = (self.stack_len, self.single_state_len)
        self.state = np.empty((self.state_size), dtype=np.float64)
        self.action_size = self.env.action_size
        self.episode_step = 6000
        self.target_update = 2000
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64
        self.memory = deque(maxlen=1000000)

        self.model = self.buildModel()
        self.target_model = self.buildModel()
        #self.graph = tf.get_default_graph()

        self.updateTargetModel()

        self.scores   = []
        self.episodes = []
        self.maxQval  = []
        self.step_list= []

        if self.load_model:
            self.model.set_weights(load_model(self.dirPath + str(self.load_episode) + ".h5").get_weights())

            with open(self.dirPath + str(self.load_episode) + '.json') as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')

            loaded_result = np.load(self.dirPath + str(self.load_episode) + '.npz')
            self.scores   = list(loaded_result['scores'])
            self.episodes = list(loaded_result['episodes'])
            self.maxQval  = list(loaded_result['maxQval'])
            self.step_list= list(loaded_result['step_list'])

        self.action = 6
        self.score = 0
        self.e = self.load_episode + 1
        self.t = 0
        self.global_step = 0
        self.param_dictionary = dict(zip(['epsilon'], [self.epsilon]))

        if training_mode:
            self.sub_risk = rospy.Subscriber('/turtlebot2i/safety/obstacles_risk', SafetyRisk, self.training_callback)
        else:
            self.sub_risk = rospy.Subscriber('/turtlebot2i/safety/obstacles_risk', SafetyRisk, self.prediction_callback)
            self.epsilon = 0.0

        self.start_time = time.time()
        self.graph = tf.get_default_graph()

    def buildModel(self):
        model = Sequential()
        dropout = 0.2

        model.add(Conv1D(32, 3, activation='relu', input_shape=self.state_size))
        model.add(Conv1D(32, 3, activation='relu'))
        #model.add(MaxPooling1D(3))
        model.add(Conv1D(32, 3, activation='relu'))
        model.add(GlobalAveragePooling1D())
        #model.add(Dense(64, activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dropout(dropout))

        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        model.summary()

        return model
    
    def getQvalue(self, reward, next_target, done):
        if done:
            return reward
        else:
            return reward + self.discount_factor * np.amax(next_target)

    def updateTargetModel(self):
        #with self.graph.as_default():
        self.target_model.set_weights(self.model.get_weights())

    def getAction(self, state):
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            return random.randrange(self.action_size)
        else:
            with self.graph.as_default():
                q_value = self.model.predict(state)
            self.q_value = q_value
            return np.argmax(q_value[0])

    def appendMemory(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def trainModel(self, target=False):
        mini_batch = random.sample(self.memory, self.batch_size)
        X_batch = np.empty((0, self.stack_len, self.single_state_len), dtype=np.float64)
        Y_batch = np.empty((0, self.action_size), dtype=np.float64)

        with self.graph.as_default():
            for i in range(self.batch_size):
                states = mini_batch[i][0]
                actions = mini_batch[i][1]
                rewards = mini_batch[i][2]
                next_states = mini_batch[i][3]
                dones = mini_batch[i][4]
                #print("states shape",np.shape(states),"|xbatch shape:",np.shape(X_batch))
                q_value = self.model.predict(states)
                self.q_value = q_value

                if target:
                    next_target = self.target_model.predict(next_states)

                else:
                    next_target = self.model.predict(next_states)

                next_q_value = self.getQvalue(rewards, next_target, dones)

                X_batch = np.append(X_batch, np.array(states.copy()), axis=0)
                Y_sample = q_value.copy()

                Y_sample[0][actions] = next_q_value
                Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

                if dones:
                    X_batch = np.append(X_batch, np.array(next_states.copy()), axis=0)
                    Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)

            self.model.fit(X_batch, Y_batch, batch_size=self.batch_size, epochs=1, verbose=0)    

    def training_callback(self, data):
        if len(self.state_stack) < self.stack_len:
            state, done = self.env.getState(data)
            self.state_stack.append(np.asarray(state))
            self.env.execute(7) #scaling to 0.4 for both wheels
            if len(self.state_stack) == self.stack_len:
                self.state = np.asarray([self.state_stack])
            if done:
                self.state_stack.clear()
                self.env.reset(data=data)
                
        else:
            #print("states shape:",np.shape(self.state))
            next_state, done = self.env.getState(data)
            reward = self.env.setReward(next_state, done, self.action)
            #next_state = np.asarray(next_state)
            self.state_stack.append(np.asarray(next_state))
            next_state_stack = np.asarray([self.state_stack])
            self.appendMemory(self.state, self.action, reward, next_state_stack, done)

            self.state = next_state_stack
            self.action = agent.getAction(self.state)
            self.env.execute(self.action)
            self.score += reward
            if len(self.memory) >= self.train_start:
                if self.global_step <= self.target_update:
                    self.trainModel()
                else:
                    self.trainModel(True)

            if self.t >= 2000:
                rospy.loginfo("Time out!!")
                done = True

            if done:
                if (self.e % 5 == 0) or (self.score > -4000):
                    self.model.save(self.dirPath + str(self.e) + '.h5')
                    with open(self.dirPath + str(self.e) + '.json', 'w') as outfile:
                        json.dump(self.param_dictionary, outfile)
                    np.savez(self.dirPath + str(self.e) + '.npz', scores=self.scores, episodes=self.episodes, maxQval=self.maxQval, step_list=self.step_list)

                #self.state = self.env.reset(data=data)
                self.env.reset(data=data)
                self.updateTargetModel()
                self.scores.append(self.score)
                self.episodes.append(self.e)
                self.maxQval.append(np.max(self.q_value))
                self.step_list.append(self.global_step)
                m, s = divmod(int(time.time() - self.start_time), 60)
                h, m = divmod(m, 60)

                if self.e>2:
                    rospy.loginfo('Ep: %d score: %.2f average_score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
                                  self.e, self.score, self.score/(self.step_list[-1]-self.step_list[-2]), len(self.memory), self.epsilon, h, m, s)
                else:
                    rospy.loginfo('Ep: %d score: %.2f average_score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
                                  self.e, self.score, self.score/self.step_list[-1], len(self.memory), self.epsilon, h, m, s)
                self.param_dictionary = dict(zip(['epsilon'], [self.epsilon]))
                #break

                self.score = 0
                self.e += 1
                self.t = 0
                self.state_stack.clear()
                if agent.epsilon > agent.epsilon_min:
                    agent.epsilon *= agent.epsilon_decay
            else:
                self.t += 1
                self.global_step += 1

    def prediction_callback(self, data):
        next_state, done = self.env.getState(data)
        self.state = np.asarray(next_state)
        self.action = agent.getAction(self.state)
        self.env.execute(self.action)

EPISODES = 3000
if __name__ == '__main__':
    try:
        rospy.init_node('training_rl_py')
        training_mode = True
        env = Env()
        agent = ReinforceAgent(env,training_mode)
        print("Program ready!")
        rospy.spin()

    except rospy.ROSInterruptException:
        env.vrep_control.shutdown()
        rospy.loginfo("Training finished.")