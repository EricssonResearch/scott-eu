def setReward(state, done, action, i, distance, direction):
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
    rotSpeed = state[-4]
    maxRisk = (state[-3] + 2) / 3
    scaling = action[0]
    scaled_speed = linSpeed * scaling
    scaled_speed_factor = scaled_speed
    r_critical = 0.295

    if i == 0:  # obstacle avoidance
        if nearest_obstacle_distance <= r_critical:
            if scaled_speed > 0.2:
                reward = -20 * maxRisk * (scaled_speed_factor + 1)
            else:
                reward = +20 * maxRisk / (scaled_speed_factor + 1)
        elif nearest_obstacle_distance <= warning_zone:
            if scaled_speed <= 0.2:
                reward = -50 / maxRisk / (scaled_speed_factor + 1)
            elif scaled_speed > 0.25:
                reward = -7 * maxRisk * (scaled_speed_factor + 1)
            else:
                reward = +20 * maxRisk * (scaled_speed_factor + 1)
        elif nearest_obstacle_distance <= clear_zone:
            if scaled_speed <= 0.25:
                reward = -65 / maxRisk / (scaled_speed_factor + 1)
            elif scaled_speed > 0.35:
                reward = -5 * maxRisk * (scaled_speed_factor + 1)
            else:
                reward = +60 * (scaled_speed_factor + 1)
        else:
            if scaled_speed <= 0.35:
                reward = -75 / maxRisk / (scaled_speed_factor + 1)
            else:
                reward = +75 / maxRisk * (scaled_speed_factor + 1)

    elif i == 1:  # speed
        if scaled_speed >= 0.35:
            if nearest_obstacle_distance > clear_zone:
                reward = +65 / maxRisk * (scaled_speed_factor + 1)
            else:
                reward = -20 * maxRisk / (scaled_speed_factor + 1)
        elif scaled_speed >= 0.2 and scaled_speed < 0.35:
            if nearest_obstacle_distance <= clear_zone and nearest_obstacle_distance > r_critical:
                reward = + 25 * maxRisk * (scaled_speed_factor + 1)
            elif nearest_obstacle_distance > clear_zone:
                reward = -65 / maxRisk / (scaled_speed_factor + 1)
            else:
                reward = -15 * maxRisk * (scaled_speed_factor + 1)
        elif scaled_speed < 0.2:
            if nearest_obstacle_distance < r_critical:
                reward = +50 / maxRisk / (scaled_speed_factor + 1)
            else:
                reward = -25 * maxRisk * (scaled_speed_factor + 1)


    elif i == 2:  # getting closer to goal
        if distance > 0.013:
            if nearest_obstacle_distance > clear_zone:
                reward = +75 / maxRisk * (scaled_speed_factor + 1)
            else:
                reward = -50 * maxRisk / (scaled_speed_factor + 1)
        elif distance >= 0.003 and distance < 0.013:
            if nearest_obstacle_distance > r_critical and nearest_obstacle_distance < clear_zone:
                reward = +50 * maxRisk / (scaled_speed_factor + 1)
            elif nearest_obstacle_distance < r_critical:
                reward = -50 * maxRisk * (scaled_speed_factor + 1)
            elif nearest_obstacle_distance > clear_zone:
                reward = -75 / maxRisk / (scaled_speed_factor + 1)
        elif distance < 0.003:
            if nearest_obstacle_distance < r_critical:
                reward = +50 * maxRisk / (scaled_speed_factor + 1)
            else:
                reward = -75 / maxRisk * (scaled_speed_factor + 1)

    elif i == 3:  # collision reward
        if done:
            reward = reward - 5000

    elif i == 4:  # direction reward
        if direction <= 3:  # obstacle on the left
            if rotSpeed > 0.10:
                reward = +75 * maxRisk * (scaled_speed_factor + 1)
            else:
                reward = -60 * maxRisk * (scaled_speed_factor + 1)
        elif direction > 3 or direction <= 7:  # obstacle in the center
            if rotSpeed < 0.10 or rotSpeed > -0.10:
                reward = -60 * maxRisk * (scaled_speed_factor + 1)
            else:
                reward = +75 * maxRisk * (scaled_speed_factor + 1)
        elif direction > 7:  # obstacle on the right
            if rotSpeed < -0.10:
                reward = +75 * maxRisk * (scaled_speed_factor + 1)
            else:
                reward = -60 * maxRisk * (scaled_speed_factor + 1)

    return reward

def get_all_elements_by_key(collection, key):
    list = []
    for e in collection:
        list.append(e[key])
    return list

def plot_bar_chart(total_rewards, rewards_per_type, reward_types, input_data):
    import numpy as np
    import matplotlib.pyplot as plt

    reward_types_names = np.array(["Obstacle", "Speed", "Goal", "Collision", "Direction"])
    speed_scaling_list = np.array([0.0, 0.2, 0.4, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4])

    n_groups = len(total_rewards)
    bar_width = 0.99
    #fig, ax = plt.subplots()
    index = bar_width*(reward_types+10)*np.arange(n_groups)
    stacked_rewards_per_type = np.vstack(rewards_per_type)

    plt.figure(figsize=(20, 10))

    for i in range(reward_types):
        #print(stacked_rewards_per_type[:, i])
        plt.bar(index + i*bar_width, stacked_rewards_per_type[:, i], bar_width, label=reward_types_names[i])

    plt.xlabel('Speed Scaling (Red=Chosen Action)')
    plt.ylabel('Rewards')
    plt.title('Rewards per chosen scaling')
    plt.xticks(index + reward_types/2*bar_width, speed_scaling_list)

    idx = np.where(speed_scaling_list == input_data["LinSpeedScale"])[0][0]
    plt.gca().get_xticklabels()[idx].set_color("red")

    plt.legend()
    plt.show()

def msx(input_data, action_list, action_number, reward_types):
    reward_list_chosen = []
    reward_list_other = []
    reward_types_names = np.array(["Obstacle", "Speed", "Goal", "Collision", "Direction"])

    other_action = action_list[action_number]
    chosen_action = [input_data["LinSpeedScale"], input_data["LinSpeedScale"]]

    if other_action[0] == chosen_action[0]:
        print("The selected action is the same as the one performed. No meaning in MSX")
        return

    for i in range(reward_types):
        chosen_reward = setReward(input_data["State"], input_data["Done"], chosen_action, i, input_data["Distance"], input_data["Direction"])
        other_reward = setReward(input_data["State"], input_data["Done"], other_action, i, input_data["Distance"], input_data["Direction"])
        reward_list_chosen.append(chosen_reward)
        reward_list_other.append(other_reward)

    reward_differences = np.array(reward_list_chosen) - np.array(reward_list_other)

    if np.sum(reward_differences) < 0:
        print("The selected action would actually be better than the one chosen by the alogrithm. No meaning in MSX")
        return

    d = -np.sum(reward_differences * (reward_differences < 0))

    if d == 0:
        print("The action by the algorithm is better than the one selected for each of the reward types. No meaning in MSX")
        return

    sorted_idx = np.argsort(-reward_differences)
    sum = 0
    i = 0
    while sum < d and i < reward_types:
        sum += reward_differences[sorted_idx[i]]
        i += 1

    msx_plus = reward_types_names[sorted_idx[:i]]

    print("The MSX+ set is represented by the rewards: " + ",".join(msx_plus))

    v = sum - reward_differences[sorted_idx[i-1]]

    sum = 0
    i = reward_types - 1
    while sum < v and i >= 0:
        sum -= reward_differences[sorted_idx[i]]
        i -= 1

    msx_minus = reward_types_names[sorted_idx[:-reward_types+i:-1]]

    print("The MSX- set is represented by the rewards: " + ",".join(msx_minus))

import matplotlib.pyplot as plt
import numpy as np

with open("/home/eiucale/compare_training.txt", "r+") as file:
    blocks = file.readlines()
n_lines_per_block = 18
i=0
data = []
action_list = [[0.0, 0.0], [0.2, 0.2], [0.4, 0.4], [0.6, 0.6], [0.7, 0.7], [0.8, 0.8], [0.9, 0.9], [1.0, 1.0], [1.1, 1.1], [1.2, 1.2], [1.3, 1.3], [1.4, 1.4]]
speed_scaling_list = np.array([0.0, 0.2, 0.4, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4])
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

# truncate results
# data = data[:35000]

progressive = int(input("Insert the progessive number of the axction to analyze (max: "+ str(len(data)) +"): "))
if(progressive > len(data) or progressive < 0):
    print("This progressive number doesn't belong to any action.")
    exit()
input_data = data[progressive]

for action in action_list:
    reward_list = []
    for i in range(reward_types):
        reward_list.append(setReward(input_data["State"], input_data["Done"], action, i, input_data["Distance"], input_data["Direction"]))
    rewards_per_type.append(reward_list)
    total_rewards.append(sum(reward_list))

action_to_compare = float(input("Speed scaling: " + str(input_data["LinSpeedScale"]) + "\nSelect the speed scaling to compare for the minimal sufficient explaination.\n"
                                                                             "Options are: [0.0, 0.2, 0.4, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4]: "))
idx = np.where(speed_scaling_list == action_to_compare)[0]
if(len(idx)==0):
    print("You did not select a valid option. MSX will not be performed.")
else:
    msx(input_data, action_list, idx[0], reward_types)

#print(total_rewards)

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

plot_bar_chart(total_rewards, rewards_per_type, reward_types, input_data)
