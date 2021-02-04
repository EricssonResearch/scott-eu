#!/usr/bin/env python
#modified from: https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning/blob/master/turtlebot3_dqn/nodes/turtlebot3_dqn_stage_4

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
from environment_mlp_rd import Env
from keras.models import Sequential, load_model
from keras.optimizers import RMSprop
from keras.layers import Dense, Dropout, Activation
from turtlebot2i_safety.msg import SafetyRisk
from geometry_msgs.msg import Twist, Pose

EPISODES = 3000


class ReinforceAgent():
    def __init__(self, state_size, action_size):
        self.pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
        self.dirPath = os.path.dirname(os.path.realpath(__file__))
        self.dirPath = self.dirPath.replace('scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_safety/src', 'scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_safety/src/models/mlp_')
        self.result = Float32MultiArray()

        # vself.load_model = True # Inference
        self.load_model = False # Training
        #vself.load_episode = 160 # Inference
        self.load_episode = 0 # Training
        self.state_size = state_size
        self.action_size = action_size
        self.episode_step = 6000
        self.target_update = 2000
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64
        self.memory = []
        self.n_models = 5
        for i in range(self.n_models):
            self.memory.append(deque(maxlen=1000000))

        self.models = []
        self.target_models = []
        self.scores = []
        self.episodes = []
        for i in range(self.n_models):
            self.models.append(self.buildModel())
            self.target_models.append(self.buildModel())

        self.updateTargetModel()

        self.randomActions = [0] * self.action_size
        self.actions = [0] * self.action_size
        self.scores   = []
        self.episodes = []

        if self.load_model:
            for i in range(self.n_models):
                print(self.dirPath + "_" + str(i) + "_" + str(self.load_episode) + ".h5");
                self.models[i].set_weights(load_model(self.dirPath + "_" + str(i) + "_" + str(self.load_episode) + ".h5").get_weights())

                with open(self.dirPath + "_" + str(i) + "_" + str(self.load_episode) + '.json') as outfile:
                    param = json.load(outfile)
                    self.epsilon = param.get('epsilon')

                loaded_result = np.load(self.dirPath + "_" + str(i) + "_" + str(self.load_episode) + '.npz')
                self.scores.append(list(loaded_result['scores']))
                self.episodes.append(list(loaded_result['episodes']))


    def buildModel(self):
        model = Sequential()
        dropout = 0.2

        model.add(Dense(64, input_shape=(self.state_size,), activation='relu', kernel_initializer='lecun_uniform'))

        model.add(Dense(32, activation='relu', kernel_initializer='lecun_uniform'))
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
        for i in range(self.n_models):
            self.target_models[i].set_weights(self.models[i].get_weights())


    def getAction(self, state):
        #print(str(state[-2]) + " and " + str(state[-1]))
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            act = random.randrange(self.action_size)
            # printing statistics
            self.randomActions[act] = self.randomActions[act] + 1
            with open("/home/eiucale/randActions.txt", "w") as outfile:
                for index, item in enumerate(self.randomActions):
                    if (item != 0):
                        outfile.write("{}. {}\n\n".format(index, item))
        else:
            q_value = [0] * self.action_size
            for i in range(self.n_models):
                q_value_part = self.models[i].predict(state.reshape(1, len(state)))
                q_value = q_value + q_value_part
            act = np.argmax(q_value[0])

            # printing statistics
            self.actions[act] = self.actions[act] + 1
            with open("/home/eiucale/actions.txt", "w") as outfile:
                for index, item in enumerate(self.actions):
                    if (item != 0):
                        outfile.write("{}. {}\n\n".format(index, item))
            print("Q_value_max: " + str(act))
        return act

    def appendMemory(self, state, action, rewards, next_state, done, i):
        self.memory[i].append((state, action, rewards[i], next_state, done))

    def trainModel(self, target=False):
        for j in range(self.n_models):
            mini_batch = random.sample(self.memory[j], self.batch_size)
            X_batch = np.empty((0, self.state_size), dtype=np.float64)
            Y_batch = np.empty((0, self.action_size), dtype=np.float64)

            for i in range(self.batch_size):
                states = mini_batch[i][0]
                actions = mini_batch[i][1]
                rewards = mini_batch[i][2]
                next_states = mini_batch[i][3]
                dones = mini_batch[i][4]

                q_value = self.models[j].predict(states.reshape(1, len(states)))
                self.q_value = q_value

                if target:
                    next_target = self.target_models[j].predict(next_states.reshape(1, len(next_states)))

                else:
                    next_target = self.models[j].predict(next_states.reshape(1, len(next_states)))

                next_q_value = self.getQvalue(rewards, next_target, dones)

                X_batch = np.append(X_batch, np.array([states.copy()]), axis=0)
                Y_sample = q_value.copy()

                Y_sample[0][actions] = next_q_value
                Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

                if dones:
                    X_batch = np.append(X_batch, np.array([next_states.copy()]), axis=0)
                    Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)

            self.models[j].fit(X_batch, Y_batch, batch_size=self.batch_size, epochs=1, verbose=0)



if __name__ == '__main__':
    try:
        rospy.init_node('risk_mitigation_mlp_py')
        pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
        pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
        result = Float32MultiArray()
        get_action = Float32MultiArray()


        env = Env()

        action_size = env.action_size
        state_size = 17

        agent = ReinforceAgent(state_size, action_size)
        global_step = 0
        start_time = time.time()

        print('start training')
        # training_mode = False  # Inference
        training_mode = True    # Training

        if training_mode:
            state = env.reset()
            for e in range(agent.load_episode + 1, EPISODES):
                done = False
                score = 0
                started = False
                for t in range(agent.episode_step):
                    state, done, _ = env.step(None, False, agent.n_models)
                    nearest_obstacle_distance  = min(state[:12])
                    safe_zone_radius = state[-1]
                    #print(safe_zone_radius)
                    #print(state[-2])
                    print(nearest_obstacle_distance)

                    if nearest_obstacle_distance < safe_zone_radius + 0.05:
                        started = True
                        print("Start ep")
                    else:
                        if started:
                            done = True
                            started = False
                            state, _, done = env.step(None, False, agent.n_models)
                            env.publishScaleSpeed(1.0, 1.0)
                            print("Finish ep")

                    print(started)

                    if started:
                        action = agent.getAction(state)

                        next_state, rewards, done = env.step(action, True, agent.n_models)

                        for i in range(agent.n_models):
                            agent.appendMemory(state, action, rewards, next_state, done, i)

                        if len(agent.memory[0]) >= agent.train_start:
                            if global_step <= agent.target_update:
                                agent.trainModel()
                            else:
                                agent.trainModel(True)

                        reward = sum(rewards)
                        score += reward
                        state = next_state
                        get_action.data = [env.action_list[action][0], env.action_list[action][1], score, reward]
                        pub_get_action.publish(get_action)

                        if e % 5 == 0:
                            for i in range(agent.n_models):
                                agent.models[i].save(agent.dirPath + "_" + str(i) + "_" + str(e) + '.h5')
                                with open(agent.dirPath + "_" + str(i) + "_" + str(e) + '.json', 'w') as outfile:
                                    json.dump(param_dictionary, outfile)
                                np.savez(agent.dirPath + "_" + str(i) + "_" + str(e) + '.npz', scores=agent.scores, episodes=agent.episodes)
                                with open("/home/eiucale/analysis.txt", "a+") as outfile:
                                    for element in env.analysis:
                                        for x in env.order:
                                            outfile.write("{}. {}\n".format(x, element[x]))
                                        outfile.write("\n")
                                    env.analysis = []
                                with open("/home/eiucale/stats.txt", "w") as outfile:
                                    for index, item in enumerate(env.statistics):
                                        if (item != 0):
                                            outfile.write("{}. {}\n\n".format(index, item))

                        if t >= 2000:
                            rospy.loginfo("Time out!!")
                            done = True

                    if done:
                        result.data = [score, np.max(agent.q_value)]
                        pub_result.publish(result)
                        agent.updateTargetModel()
                        agent.scores.append(score)
                        agent.episodes.append(e)
                        m, s = divmod(int(time.time() - start_time), 60)
                        h, m = divmod(m, 60)

                        rospy.loginfo('Ep: %d score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
                                      e, score, len(agent.memory), agent.epsilon, h, m, s)
                        param_keys = ['epsilon']
                        param_values = [agent.epsilon]
                        param_dictionary = dict(zip(param_keys, param_values))
                        done = False
                        global_step += 1
                        if global_step % agent.target_update == 0:
                            rospy.loginfo("UPDATE TARGET NETWORK")
                        if agent.epsilon > agent.epsilon_min:
                            agent.epsilon *= agent.epsilon_decay
                        break

        else:
            agent.epsilon = 0.0
            prev_action = 0
            time_duration_list = []

            progressive = 1
            state = env.reset()

            while True:
                data = None
                data = rospy.wait_for_message('/turtlebot2i/safety/obstacles_risk', SafetyRisk, timeout=500)
                #time_previous = time.time()
                if data == None:
                    print("no data")
                    env.publishScaleSpeed(1.0, 1.0)
                # elif len(data.risk_value) == 0:
                #     env.publishScaleSpeed(1.0, 1.0)
                # elif max(data.risk_value) == 0:
                #     env.publishScaleSpeed(1.0, 1.0)
                else:
                    state, done = env.getState(data)
                    nearest_obstacle_distance = min(state[:12])
                    safe_zone_radius = state[-1]
                    # print(safe_zone_radius)
                    # print(state[-2])
                    print(nearest_obstacle_distance)

                    if nearest_obstacle_distance < safe_zone_radius + 0.05:
                        action = agent.getAction(np.asarray(state))
                        rewards = []
                        for i in range(agent.n_models):
                            rewards.append(env.setReward(state, done, action, i))
                        element = {}
                        element["Progressive"] = progressive
                        progressive = progressive + 1
                        element["State"] = state
                        element["ObsRwd"] = rewards[0]
                        element["SpeedRWD"] = rewards[1]
                        element["GoalRWD"] = rewards[2]
                        element["CollisionRWD"] = rewards[3]
                        element["Total"] = sum(rewards)
                        element["NearestObs"] = min(state[:12])
                        element["Direction"] = np.argmin(state[:12])
                        element["LinSpeedBefore"] = state[-5]
                        element["LinSpeedAfter"] = state[-5] * env.action_list[action][0]
                        element["LinSpeedScale"] = env.action_list[action][0]
                        element["RotSpeed"] = state[-4]
                        element["RiskMax"] = state[-3]
                        element["Distance"] = env.distance
                        element["Done"] = done
                        with open("/home/eiucale/analysis_test.txt", "a+") as outfile:
                            for x, y in element.items():
                                outfile.write("{}. {}\n".format(x, y))
                            outfile.write("\n")
                    #time_end = time.time()
                        env.publishScaleSpeed(env.action_list[action][0], env.action_list[action][1])
                        if prev_action != action:
                            print("action:", action)
                            prev_action = action
                    else:
                        env.publishScaleSpeed(1.0, 1.0)

                    if done:
                        print("RESETTING")
                        env.publishScaleSpeed(1.0, 1.0)
                        env.reset()
                        env.prev_position = Pose()


                        # time_duration_list.append(time_end-time_previous)
                    # if len(time_duration_list) == 100:
                    #     dirPath = os.path.dirname(os.path.realpath(__file__))
                    #     savePath = self.dirPath.replace('scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_safety/src', 'time_duration/time_duration_rm_mlp.npz')
                    #     np.savez(savePath, time_duration_list=time_duration_list)

    except rospy.ROSInterruptException:
        env.vrep_control.shutdown()
        #vrep.simxFinish(clientID)
        rospy.loginfo("Training finished.")
        
