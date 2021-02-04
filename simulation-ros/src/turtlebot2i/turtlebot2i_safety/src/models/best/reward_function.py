if i == 0:  # obstacle reward
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

elif i == 1:  # speed reward
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


elif i == 2:  # goal reward
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
    if done: # collision happened
        reward = reward - 5000

elif i == 4:  # direction reward
    if direction <= 3:  # obstacle on the left
        if rotSpeed > 0.10: # turning clockwise
            reward = +75 * maxRisk * (scaled_speed_factor + 1)
        else:
            reward = -60 * maxRisk * (scaled_speed_factor + 1)
    elif direction > 3 or direction <= 7:  # obstacle in the center
        if rotSpeed < 0.10 or rotSpeed > -0.10: # basically not rotating
            reward = -60 * maxRisk * (scaled_speed_factor + 1)
        else:
            reward = +75 * maxRisk * (scaled_speed_factor + 1)
    elif direction > 7:  # obstacle on the right
        if rotSpeed < -0.10: # turning counter-clockwise
            reward = +75 * maxRisk * (scaled_speed_factor + 1)
        else:
            reward = -60 * maxRisk * (scaled_speed_factor + 1)
