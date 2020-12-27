import sys


def setReward(state, done, action, i, distance):
    nearest_obstacle_distance = min(state[:12])
    # nearest_obstacle_direction = np.argmin(state[:12]) #index 0 start from right side of the robot

    # yaw_reward = 1.0
    # if (nearest_obstacle_direction <= self.n_direction/3-1):#obstacle is on the right
    #     if (action >= 10):                    #robot turns right
    #         yaw_reward = -(action-9)*risk_max/6
    # elif (nearest_obstacle_direction >= self.n_direction*2/3):#obstacle is on the left
    #     if (action <= 5):                   #robot turns left
    #         yaw_reward = -(6-action)*risk_max/6
    # else:#obstacle is in the front
    #     if (action in [6,7,8,9]):
    #         yaw_reward = -(action-5)*risk_max/4

    # distance_rate = 1.0 / max(nearest_obstacle_distance, 0.175)
    reward = 0
    clear_zone = state[-1]
    warning_zone = state[-2]
    linSpeed = state[-5]
    maxRisk = (state[-3] + 2) / 2
    #maxRisk = 1
    scaling = action[0]
    scaled_speed = linSpeed * scaling
    #scaled_speed = 0
    r_critical = 0.295

    # if nearest_obstacle_distance < self.r_critical + 0.03: #r_critical + offset
    #     reward = (distance_rate) * 10 - 100
    #     self.statistics[1] = self.statistics[1] + 1
    #     if state[-5] * scaling > 0.2:
    #         self.statistics[11] = self.statistics[11] + 1
    #         reward = - 50
    #     else:
    #         self.statistics[12] = self.statistics[12] + 1
    #         reward = reward + 20
    # elif nearest_obstacle_distance < state[-2] + 0.05: #r_warning state[-2] + offset
    #     self.statistics[2] = self.statistics[2] + 1
    #     reward =  - 30
    #     if state[-5] * scaling < 0.2 or state[-5] * scaling > 0.3:
    #         self.statistics[21] = self.statistics[21] + 1
    #         reward = reward - 15
    #     else:
    #         self.statistics[22] = self.statistics[22] + 1
    #         reward = reward + 20
    # elif nearest_obstacle_distance < state[-1]+ 0.05: #r_clear state[-1] + offset
    #     reward = - 20
    #     self.statistics[3] = self.statistics[3] + 1
    #     if state[-5] * scaling < 0.3 or state[-5] * scaling > 0.4:
    #         self.statistics[31] = self.statistics[31] + 1
    #         reward = reward - 15
    #     else:
    #         self.statistics[32] = self.statistics[32] + 1
    #         reward = reward + 20
    # else:
    #     self.statistics[4] = self.statistics[4] + 1
    #     reward = 10
    #     if state[-5] * scaling < 0.4:
    #         self.statistics[41] = self.statistics[41] + 1
    #         reward = reward - 25
    #     else:
    #         self.statistics[42] = self.statistics[42] + 1
    #         reward = reward + 20
    #
    # if self.distance2D(self.prev_position, self.position) > 0.03:
    #     self.statistics[5] = self.statistics[5] + 1
    #     reward = reward +  10
    # self.prev_position = self.position

    if i == 0:  # obstacle avoidance
#        self.statistics[1] = self.statistics[1] + 1
        if nearest_obstacle_distance <= r_critical + 0.03:  # r_critical + offset
#            self.statistics[11] = self.statistics[11] + 1
            if scaled_speed > 0.2:
#                self.statistics[111] = self.statistics[111] + 1
                reward = -20 * maxRisk * (scaled_speed + 1)
            else:
#               self.statistics[112] = self.statistics[112] + 1
                reward = +10 * maxRisk / (scaled_speed + 1)
        elif nearest_obstacle_distance <= warning_zone + 0.05:  # r_warning warning_zone + offset
#            self.statistics[12] = self.statistics[12] + 1
            if scaled_speed <= 0.2:
#                self.statistics[121] = self.statistics[121] + 1
                reward = -50 / maxRisk / (scaled_speed + 1)
            elif scaled_speed > 0.3:
#                self.statistics[122] = self.statistics[122] + 1
                reward = -7 * maxRisk * (scaled_speed + 1)
            else:
#                self.statistics[123] = self.statistics[123] + 1
                reward = +20 * maxRisk * (scaled_speed + 1)
        elif nearest_obstacle_distance <= clear_zone + 0.05:  # r_clear clear_zone + offset
#            self.statistics[13] = self.statistics[13] + 1
            if scaled_speed <= 0.3:
#                self.statistics[131] = self.statistics[131] + 1
                reward = -60 / maxRisk / (scaled_speed + 1)
            elif scaled_speed > 0.4:
#                self.statistics[132] = self.statistics[132] + 1
                reward = -5 * maxRisk * (scaled_speed + 1)
            else:
#                self.statistics[133] = self.statistics[133] + 1
                reward = +75 * (scaled_speed + 1)
        else:
#            self.statistics[14] = self.statistics[14] + 1
            if scaled_speed <= 0.4:
#                self.statistics[141] = self.statistics[141] + 1
                reward = -75 / maxRisk / (scaled_speed + 1)
            else:
#                self.statistics[142] = self.statistics[142] + 1
                reward = +75 / maxRisk * (scaled_speed + 1)

    elif i == 1:  # speed
#        self.statistics[2] = self.statistics[2] + 1
        if scaled_speed >= 0.4:
#           self.statistics[21] = self.statistics[21] + 1
            if nearest_obstacle_distance > clear_zone:
#                self.statistics[211] = self.statistics[211] + 1
                reward = +100 / maxRisk * (scaled_speed + 1)
            else:
#                self.statistics[212] = self.statistics[212] + 1
                reward = -20 * maxRisk / (scaled_speed + 1)
        elif scaled_speed >= 0.2 and scaled_speed < 0.4:
#            self.statistics[22] = self.statistics[22] + 1
            if nearest_obstacle_distance <= clear_zone and nearest_obstacle_distance > r_critical:
#                self.statistics[221] = self.statistics[221] + 1
                reward = + 25 * maxRisk * (scaled_speed + 1)
            elif nearest_obstacle_distance > clear_zone:
#                self.statistics[222] = self.statistics[222] + 1
                reward = -100 / maxRisk / (scaled_speed + 1)
            else:
#                self.statistics[223] = self.statistics[223] + 1
                reward = -15 * maxRisk * (scaled_speed + 1)
        elif scaled_speed < 0.2:
#            self.statistics[23] = self.statistics[23] + 1
            if nearest_obstacle_distance < r_critical:
#                self.statistics[231] = self.statistics[231] + 1
                reward = +100 / maxRisk / (scaled_speed + 1)
            else:
#               self.statistics[232] = self.statistics[232] + 1
                reward = -25 * maxRisk * (scaled_speed + 1)


    elif i == 2:  # getting closer to goal
#        self.statistics[3] = self.statistics[3] + 1
#        self.distance = self.distance2D(self.prev_position, self.position)
        if distance > 0.015:
            if nearest_obstacle_distance > clear_zone:
 #               self.statistics[31] = self.statistics[31] + 1
                reward = +200 / maxRisk * (scaled_speed + 1)
            else:
 #               self.statistics[32] = self.statistics[32] + 1
                reward = -13 * maxRisk / (scaled_speed + 1)
#        self.prev_position = self.position

    elif i == 3:  # collision reward
#       self.statistics[4] = self.statistics[4] + 1
        if done:
#            self.statistics[41] = self.statistics[41] + 1
#            rospy.loginfo("Collision!!")
            reward = reward - 5000
#            self.publishScaleSpeed(1.0, 1.0)
#            self.reset()
 #           self.prev_position = Pose()

    # if self.get_goalbox:
    #    rospy.loginfo("Goal!!")
    #    self.respawn_goal(reset=True)
    #    self.get_goalbox = False

    return reward

def get_all_elements_by_key(collection, key):
    list = []
    for e in collection:
        list.append(e[key])
    return list

def plot_bar_chart(total_rewards, rewards_per_type, reward_types, input):
    import numpy as np
    import matplotlib.pyplot as plt

    n_groups = len(total_rewards)
    bar_width = 0.35
    fig, ax = plt.subplots()
    index = bar_width*5*np.arange(n_groups)
    stacked_rewards_per_type = np.vstack(rewards_per_type)

    for i in range(reward_types):
        print(stacked_rewards_per_type[:, i])
        plt.bar(index + i*bar_width, stacked_rewards_per_type[:, i], bar_width)

    plt.xlabel('Speed Scaling')
    plt.ylabel('Rewards')
    plt.title('Rewards per chosen scaling')
    plt.xticks(index + reward_types/2*bar_width, np.arange(n_groups))

    plt.legend()
    plt.show()

def msx(input, action_list, action_number, reward_types):
    reward_list_chosen = []
    reward_list_other = []

    other_action = action_list[action_number]
    chosen_action = [input["LinSpeedScale"], input["LinSpeedScale"]]

    if other_action[0] == chosen_action[0]:
        print("The selected action is the same as the one performed. No meaning in MSX")
        return

    for i in range(reward_types):
        chosen_reward = setReward(input["State"], input["Done"], chosen_action, i, input["Distance"])
        other_reward = setReward(input["State"], input["Done"], other_action, i, input["Distance"])
        reward_list_chosen.append(chosen_reward)
        reward_list_other.append(other_reward)

    reward_differences = np.array(reward_list_chosen) - np.array(reward_list_other)

    if np.sum(reward_differences) < 0:
        print("The selected action would actually be better than the one chosen by the alogrithm. No meaning in MSX")
        return

    d = -np.sum(reward_differences * (reward_differences < 0))

    sorted_idx = np.argsort(-reward_differences)
    sum = 0
    i = 0
    while sum < d and i < reward_types:
        sum += reward_differences[sorted_idx[i]]
        i += 1

    print("The MSX+ set is represented by the rewards: " + str(sorted_idx[:i]))

    v = sum - reward_differences[sorted_idx[i-1]]

    sum = 0
    i = reward_types - 1
    while sum < v and i >= 0:
        sum -= reward_differences[sorted_idx[i]]
        i -= 1

    print("The MSX- set is represented by the rewards: " + str(sorted_idx[:-i+1:-1]))

import matplotlib.pyplot as plt
import numpy as np

with open("/home/eiucale/compare_training.txt", "r+") as file:
    blocks = file.readlines()
n_lines_per_block = 18
i=0
data = []
action_list = [[0.0, 0.0], [0.2, 0.2], [0.4, 0.4], [0.6, 0.6], [0.7, 0.7], [0.8, 0.8], [0.9, 0.9], [1.0, 1.0], [1.1, 1.1], [1.2, 1.2], [1.3, 1.3], [1.4, 1.4]]
reward_types = 5
total_rewards = []
rewards_per_type = []
to_compare = 0

while i < len(blocks):
    lines = blocks[i:i+n_lines_per_block-1]
    block = {}
    for line in lines:
        parts = line.split(". ")
        name = parts[0]
        value = parts[1].strip("\n")
        if(name == "State"):
            value = [float(s) for s in value.strip("[]").split(',')]
        elif(name == "Done"):
            if value == "True":
                value = True
            elif value == "False":
                value = False
        else:
            value = float(value)
        block[name] = value
    data.append(block)
    i += n_lines_per_block

input = data[5000]
for action in action_list:
    reward_list = []
    for i in range(reward_types):
        reward_list.append(setReward(input["State"], input["Done"], action, i, input["Distance"]))
    rewards_per_type.append(reward_list)
    total_rewards.append(sum(reward_list))

msx(input, action_list, 10, reward_types)

print(total_rewards)

# plot speed distribution
d = get_all_elements_by_key(data, "LinSpeedAfter")
plt.hist(x=d, bins='auto')
plt.title("LinSpeedAfter " + "\nAvg: " + str(np.mean(d)))
plt.show()

# plot reward distributions and avg
d = get_all_elements_by_key(data, "ObsRWD")
plt.hist(x=d, bins='auto')
plt.title("ObsRWD" + "\nAvg: " + str(np.mean(d)))
plt.show()

d = get_all_elements_by_key(data, "SpeedRWD")
plt.hist(x=d, bins='auto')
plt.title("SpeedRWD" + "\nAvg: " + str(np.mean(d)))
plt.show()

d = get_all_elements_by_key(data, "GoalRWD")
plt.hist(x=d, bins='auto')
plt.title("GoalRWD" + "\nAvg: " + str(np.mean(d)))
plt.show()

d = get_all_elements_by_key(data, "CollisionRWD")
plt.hist(x=d, bins='auto')
plt.title("CollisionRWD" + "\nAvg: " + str(np.mean(d)))
plt.show()

d = get_all_elements_by_key(data, "DirectionRWD")
plt.hist(x=d, bins='auto')
plt.title("DirectionRWD" + "\nAvg: " + str(np.mean(d)))
plt.show()

plot_bar_chart(total_rewards, rewards_per_type, reward_types, input)
