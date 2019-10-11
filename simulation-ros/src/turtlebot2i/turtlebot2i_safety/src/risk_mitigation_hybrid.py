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
from environment_hybrid import Env
import tensorflow as tf
from keras.models import Sequential, load_model, Model
from keras.optimizers import RMSprop
from keras.layers import Conv1D, GlobalAveragePooling1D, MaxPooling1D
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.layers import concatenate
from turtlebot2i_safety.msg import SafetyRisk


class ReinforceAgent():
    def __init__(self, env, training_mode):
        self.pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=1)
        self.dirPath = os.path.dirname(os.path.realpath(__file__))
        self.dirPath = self.dirPath.replace('scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_safety/src', 'scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_safety/src/models/hybrid_')
        self.result = Float32MultiArray()

        self.load_model = True
        self.load_episode = 2540#1671#2541 #665 #0
        self.env = env
        self.stack_len = 6
        self.distance_stack = deque(maxlen=self.stack_len)
        if training_mode:
            self.single_state_len = len(self.env.reset())
        else:
            self.single_state_len = len(self.env.getEmptyState())
        self.distance_size = self.env.n_direction
        self.nondistance_size = self.single_state_len - self.distance_size
        self.state_size = (self.distance_size, self.stack_len)
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

        self.action = 7
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

        self.time_duration_list = []

    def buildModel(self):
        dropout = 0.2

        mlp = Sequential()
        mlp.add(Dense(32, input_shape=(self.nondistance_size,), activation='relu', kernel_initializer='lecun_uniform'))
        mlp.add(Dense(16, activation='relu', kernel_initializer='lecun_uniform'))
        mlp.add(Dropout(dropout))
        #mlp.summary()

        cnn = Sequential()
        cnn.add(Conv1D(16, 2, activation='relu', input_shape=self.state_size))
        cnn.add(Conv1D(32, 2, activation='relu'))
        cnn.add(Conv1D(32, 2, activation='relu'))
        cnn.add(MaxPooling1D(3))
        cnn.add(Flatten())
        #cnn.summary()
        
        combinedInput = concatenate([cnn.output, mlp.output])
 
        x = Dense(self.action_size, kernel_initializer='lecun_uniform', activation="linear")(combinedInput)
         
        hybrid = Model(inputs=[mlp.input, cnn.input], outputs=x)
        hybrid.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        hybrid.summary()
        return hybrid
    
    def getQvalue(self, reward, next_target, done):
        if done:
            return reward
        else:
            return reward + self.discount_factor * np.amax(next_target)

    def updateTargetModel(self):
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
        X_batch_distance    = np.empty((0, self.distance_size, self.stack_len), dtype=np.float64)
        X_batch_nondistance = np.empty((0, self.nondistance_size), dtype=np.float64)
        Y_batch = np.empty((0, self.action_size), dtype=np.float64)

        with self.graph.as_default():
            for i in range(self.batch_size):
                states = mini_batch[i][0]
                actions = mini_batch[i][1]
                rewards = mini_batch[i][2]
                next_states = mini_batch[i][3]
                dones = mini_batch[i][4]
                q_value = self.model.predict(states)
                self.q_value = q_value

                if target:
                    next_target = self.target_model.predict(next_states)

                else:
                    next_target = self.model.predict(next_states)

                next_q_value = self.getQvalue(rewards, next_target, dones)

                X_batch_distance    = np.append(X_batch_distance, np.array(states[1].copy()), axis=0)
                X_batch_nondistance = np.append(X_batch_nondistance, np.array(states[0].copy()), axis=0)
                Y_sample = q_value.copy()

                Y_sample[0][actions] = next_q_value
                Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

                if dones:
                    X_batch_distance    = np.append(X_batch_distance, np.array(next_states[1].copy()), axis=0)
                    X_batch_nondistance = np.append(X_batch_nondistance, np.array(next_states[0].copy()), axis=0)
                    Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)

            self.model.fit([X_batch_nondistance, X_batch_distance], Y_batch, batch_size=self.batch_size, epochs=1, verbose=0)    

    def training_callback(self, data):
        if len(self.distance_stack) < self.stack_len:
            state, done = self.env.getState(data)
            self.distance_stack.append(np.asarray(state[:self.distance_size]))
            self.env.execute(7) #both wheels are scaled to 0.4
            if len(self.distance_stack) == self.stack_len:
                self.state = [np.array(state[self.distance_size:]).reshape(1,self.nondistance_size), np.transpose(self.distance_stack).reshape(1,self.distance_size, self.stack_len)]
            if done:
                self.distance_stack.clear()
                self.env.reset(data=data)
                
        else:
            next_state, done = self.env.getState(data)
            reward = self.env.setReward(next_state, done, self.action)
            self.distance_stack.append(np.asarray(next_state[:self.distance_size]))
            next_distance_stack = [np.array(next_state[self.distance_size:]).reshape(1,self.nondistance_size), np.transpose(self.distance_stack).reshape(1,self.distance_size,self.stack_len)]
            self.appendMemory(self.state, self.action, reward, next_distance_stack, done)

            self.state = next_distance_stack
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
                
                self.score = 0
                self.e += 1
                self.t = 0
                self.distance_stack.clear()
                if agent.epsilon > agent.epsilon_min:
                    agent.epsilon *= agent.epsilon_decay
            else:
                self.t += 1
                self.global_step += 1

    def prediction_callback(self, data):
        if len(self.distance_stack) < self.stack_len:
            state, done = self.env.getState(data)
            self.distance_stack.append(np.asarray(state[:self.distance_size]))
            self.env.execute(7) #both wheels are scaled to 0.4
            if len(self.distance_stack) == self.stack_len:
                self.state = [np.array(state[self.distance_size:]).reshape(1,self.nondistance_size), np.transpose(self.distance_stack).reshape(1,self.distance_size, self.stack_len)]
            if done:
                self.distance_stack.clear()
                self.env.reset(data=data)
                
        else:
            #time_previous = time.time()
            next_state, done = self.env.getState(data)
            self.distance_stack.append(np.asarray(next_state[:self.distance_size]))
            self.state = [np.array(next_state[self.distance_size:]).reshape(1,self.nondistance_size), np.transpose(self.distance_stack).reshape(1,self.distance_size,self.stack_len)]
            
            next_action = agent.getAction(self.state)
            #time_end = time.time()
            self.env.execute(next_action)

            # self.time_duration_list.append(time_end-time_previous)
            # if len(self.time_duration_list) == 100:
            #     np.savez('/home/turtlebot/thesis2019/duration_result/time_duration_rm_hybrid.npz', time_duration_list=self.time_duration_list)
            #     dirPath = os.path.dirname(os.path.realpath(__file__))
            #     savePath = self.dirPath.replace('scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_safety/src', 'time_duration/time_duration_rm_hybrid.npz')
            #     np.savez(savePath, time_duration_list=self.time_duration_list)
            if next_action != self.action:
                print("action:",next_action)
                self.action = next_action
            

EPISODES = 3000
if __name__ == '__main__':
    try:
        rospy.init_node('risk_mitigation_hybrid_py')
        training_mode = False
        env = Env()
        agent = ReinforceAgent(env,training_mode)
        print("Program ready!")
        rospy.spin()

    except rospy.ROSInterruptException:
        env.vrep_control.shutdown()
        rospy.loginfo("Training finished.")
