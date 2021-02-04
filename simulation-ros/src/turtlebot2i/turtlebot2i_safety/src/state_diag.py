from collections import defaultdict
import numpy as np
from sklearn.preprocessing import normalize
from copy import copy, deepcopy
import json
import hs as hayes_shah
from markovchain import MarkovChain

def convert_block(block):
    clear_zone_radius = 0.944  # at max: 0.944 0.640
    warning_zone_radius = 0.622  # at max: 0.622 0.460
    critical_zone_radius = 0.295

    distNO = block["NearestObs"]
    speed_scale = block["LinSpeedScale"]
    speed = block["LinSpeedBefore"]

    # convert distance
    if distNO <= critical_zone_radius:
        block["NearestObsC"] = "Critical"
    elif distNO > critical_zone_radius and distNO <= warning_zone_radius:
        block["NearestObsC"] = "Warning"
    elif distNO > warning_zone_radius:
        block["NearestObsC"] = "Safe"

    # convert speed
    if speed <= 0.2:
        block["LinSpeedBeforeC"] = "Slow"
    elif speed > 0.2 and speed <= 0.4:
        block["LinSpeedBeforeC"] = "Medium"
    elif speed > 0.4:
        block["LinSpeedBeforeC"] = "Fast"

    # convert speed scale
    if speed_scale == 0:
        block["LinSpeedScaleC"] = "stop"
    elif speed_scale < 0.8:
        block["LinSpeedScaleC"] = "slow down"
    elif speed_scale >= 0.8 and speed_scale < 1.2:
        block["LinSpeedScaleC"] = "keep the same speed"
    elif speed_scale >= 1.2:
        block["LinSpeedScaleC"] = "speed up"

    return block

def nested_dict(n, type):
    if n == 1:
        return defaultdict(type)
    else:
        return defaultdict(lambda: nested_dict(n-1, type))

def print_edge_list(M):
    n = np.shape(M)[0]

    for i in range(n):
        for j in range(n):
            if M[i][j] > 30:
                print(str(i) + " " + str(j) + " " + str(M[i][j]))

with open("/home/eiucale/analysis_test.txt", "r+") as file:
    blocks = file.readlines()
n_lines_per_block = 17
i=0
data = []
action_list = [[0.0, 0.0], [0.2, 0.2], [0.4, 0.4], [0.6, 0.6], [0.7, 0.7], [0.8, 0.8], [0.9, 0.9], [1.0, 1.0], [1.1, 1.1], [1.2, 1.2], [1.3, 1.3], [1.4, 1.4]]
speed_scaling_list = np.array([0.0, 0.2, 0.4, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4])
speed_categories = ["Slow", "Medium", "Fast"]
action_categories = ["stop", "slow down", "keep the same speed", "speed up"]
distNO_categories = ["Critical", "Warning", "Safe"]
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
    block = convert_block(block)
    data.append(block)
    i += n_lines_per_block

#data = data[:45000]
#data = data[20000:]

stats_actions_converted = nested_dict(3, int)
stats_actions_converted_action_changed = nested_dict(3, int)
stats_actions_converted_state_changed = nested_dict(3, int)
probs_actions = nested_dict(3, float)

dist_prev = "None"
speed_prev = "None"
action_prev = "None"

for d in data:
    dist = d["NearestObsC"]
    speed = d["LinSpeedBeforeC"]
    action = d["LinSpeedScaleC"]

    stats_actions_converted[dist][speed][action] += 1

    # only if action changes
    if(action != action_prev):
        stats_actions_converted_action_changed[dist][speed][action] += 1

    if(dist != dist_prev or speed != speed_prev):
        stats_actions_converted_state_changed[dist_prev][speed_prev][action_prev] += 1

    dist_prev = dist
    speed_prev = speed
    action_prev = action

print("ALL ACTIONS")
for key_dist in distNO_categories:
    print(key_dist)
    l1 = stats_actions_converted[key_dist]
    for key_speed in speed_categories:
        print("\t"+key_speed)
        l2 = l1[key_speed]
        sum_values = sum(l2.values())
        for key_action in action_categories:
            value = l2[key_action]
            probability = round(value/sum_values*100, 2)
            probs_actions[key_dist][key_speed][key_action] = probability/100
            print("\t\t"+key_action+": "+str(value) + " (" + str(probability)+ "%)")

# print("ACTION CHANGED")
# for key_dist in distNO_categories:
#     print(key_dist)
#     l1 = stats_actions_converted_action_changed[key_dist]
#     for key_speed in speed_categories:
#         print("\t"+key_speed)
#         l2 = l1[key_speed]
#         sum_values = sum(l2.values())
#         for key_action in action_categories:
#             value = l2[key_action]
#             probability = round(value/sum_values*100, 2)
#             probs_actions[key_dist][key_speed][key_action] = probability/100
#             print("\t\t"+key_action+": "+str(value) + " (" + str(probability)+ "%)")
#
# print("STATE CHANGED")
# for key_dist in distNO_categories:
#     print(key_dist)
#     l1 = stats_actions_converted_state_changed[key_dist]
#     for key_speed in speed_categories:
#         print("\t"+key_speed)
#         l2 = l1[key_speed]
#         sum_values = sum(l2.values())
#         for key_action in action_categories:
#             value = l2[key_action]
#             probability = round(value/sum_values*100, 2)
#             probs_actions[key_dist][key_speed][key_action] = probability/100
#             print("\t\t"+key_action+": "+str(value) + " (" + str(probability)+ "%)")

transition_matrix = nested_dict(4, int)

for i in range(len(data) - 1):
    d1 = data[i]
    d2 = data[i+1]

    transition_matrix[d1["NearestObsC"]][d1["LinSpeedBeforeC"]][d2["NearestObsC"]][d2["LinSpeedBeforeC"]] += 1

matrix = np.zeros(shape=(len(distNO_categories)*len(speed_categories), len(distNO_categories)*len(speed_categories)))

i = 0
for dc1 in distNO_categories:
    for sc1 in speed_categories:
        j=0
        for dc2 in distNO_categories:
            for sc2 in speed_categories:
                matrix[i][j] = transition_matrix[dc1][sc1][dc2][sc2]
                j+=1
        i+=1

matrix_zero_diag = deepcopy(matrix)

for i in range(len(distNO_categories)*len(speed_categories)):
    matrix_zero_diag[i][i] = 0

matrix_norm = normalize(matrix, norm='l1') * 100
matrix_norm_zero_diag = normalize(matrix_zero_diag, norm='l1') * 100

print_edge_list(matrix_norm_zero_diag)

# print(np.sum(matrix_norm, axis=1))
# print(np.sum(matrix_norm_zero_diag, axis=1))

states = []
actions = []

for dist in distNO_categories:
    for speed in speed_categories:
        dict = {}
        dict["distNO"] = dist
        dict["speed"] = speed
        states.append(dict)
        act_dict = {}
        for action in action_categories:
            act_dict[action] = probs_actions[dist][speed][action]
        actions.append(act_dict)

isObstacleCritical = {
    'true': 'the nearest obstacle is in the critical zone',
    'false': 'the nearest obstacle is not in the critical zone',
    'verify': lambda s : s['distNO'] == 'Critical'
}

isObstacleWarning = {
    'true': 'the nearest obstacle is in the warning zone',
    'false': 'the nearest obstacle is not in the warning zone',
    'verify': lambda s : s['distNO'] == 'Warning'
}

isObstacleSafe = {
    'true': 'the nearest obstacle is in the safe zone',
    'false': 'the nearest obstacle is not in the safe zone',
    'verify': lambda s : s['distNO'] == 'Safe'
}

isRobotFast = {
    'true': 'I am fast',
    'false': 'I am not fast',
    'verify': lambda s : s['speed'] == 'Fast'
}

isRobotAtMediumSpeed = {
    'true': 'I am at medium speed',
    'false': 'I am not at medium speed',
    'verify': lambda s : s['speed'] == 'Medium'
}

isRobotSlow = {
    'true': 'I am slow',
    'false': 'I am not slow',
    'verify': lambda s : s['speed'] == 'Slow'
}

predicates = [isObstacleCritical, isObstacleWarning, isObstacleSafe, isRobotSlow, isRobotAtMediumSpeed, isRobotFast]

# Use the states, actions and predicates to construct the explainer
explainer = hayes_shah.Explainer(states, actions, predicates)


#-------------------------------- Algorithm 1 ---------------------------------#
# print('Algorithm 1:')
# Let's ask the robot when it goes straight, or when it switches lanes. From the
# action list above, we can determine the (non-)target states.

# When does the robot move forward?
# targetStates = [states[0], states[1]]
# nonTargetStates = [states[2], states[3]]
# print('I move forward when {}'.format(explainer.getLanguage(targetStates, nonTargetStates)))


#-------------------------------- Algorithm 2 ---------------------------------#
print('\nAlgorithm 2:')
# For algorithm 1 we had to determine the target states and non-target states
# ourselves. However, we can also simply ask the robot when it decides to do
# a certain action.
action_idx = int(input("Which action do you want to analyze? Select a number:\n1 - Stop\n2 - Slow Down\n3 - Keep the same speed\n4 - Speed up\n"))
[targetStates, nonTargetStates] = explainer.getStateRegion(action_categories[action_idx-1])
print('Target states: {}'.format(json.dumps(targetStates)))
print('Non-target states: {}'.format(json.dumps(nonTargetStates)))
print('I ' + action_categories[action_idx-1] + ' when {}'.format(explainer.getLanguage(targetStates, nonTargetStates)))


#-------------------------------- Algorithm 3 ---------------------------------#
print('\nAlgorithm 3:')
# We ask the robot why it does not perform a certain action in a state. It
# answers by comparing the current states to the other states.
progressive = int(input("Insert the progessive number of the axction to analyze (max: "+ str(len(data)) +"): "))
distNO_idx = distNO_categories.index(data[progressive]["NearestObsC"])
speed_idx = speed_categories.index(data[progressive]["LinSpeedBeforeC"])
state_idx = distNO_idx * 3 + speed_idx
print("Distance Nearest Obstacle: " + data[progressive]["NearestObsC"])
print("Speed: " + data[progressive]["LinSpeedBeforeC"])
str = "The action selected from the algorithm is " + data[progressive]["LinSpeedScaleC"].upper() + "\n"
action_idx = int(input(str + "Which action do you want to compare to the one cosen by the algorithm? Select a number:\n1 - Stop\n2 - Slow Down\n3 - Keep the same speed\n4 - Speed up\n"))
explainer.getBehavioralDivergences(states[state_idx], action_categories[action_idx-1])


#-------------------------------- Algorithm 4 ---------------------------------#
print('\nAlgorithm 4:')
# We now ask the robot what it would do given that certain predicates are true
# or false. It answers by comparing the current states to the other states.
# The predicates are for now encoded using 1,0 and -. 1 means the predicate must
# be true, 0 means the predicate must be false, and - means it does not matter
# what the predicate is.

# What would the ET [robot] do when the target is close?
str1 = list("---")
str2 = list("---")
distance_idx = int(input("Select the distance of the obstacle. Select a number:\n0 - Don't select\n1 - Critical\n2 - Warning\n3 - Safe\n"))
speed_idx = int(input("Select the speed of the robot. Select a number:\n0 - Don't select\n1 - Slow\n2 - Medium Speed\n3 - Fast\n"))
str = "What will you do when the obstacle is at "
if distance_idx != 0:
    str += distNO_categories[distance_idx-1].lower() + " distance from the nearest obstacle"
    if speed_idx != 0:
        str += "and is at "
if speed_idx != 0:
    str += speed_categories[speed_idx - 1].lower() + " speed"
str += "?"
print(str)
if distance_idx > 0:
    str1[distance_idx-1] = "1"
if speed_idx > 0:
    str2[speed_idx-1] = "1"
descriptions = explainer.getSituationalBehaviour("".join(str1) + "".join(str2), 10)
print(descriptions)

assert np.sum(matrix) == len(data)-1
