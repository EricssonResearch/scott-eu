#!/usr/bin/env python

"""
    Edited from navigation.py in turtlebot2i_navigation module
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
import math
from std_msgs.msg import Float64
from turtlebot2i_safety.msg import SafetyZone

def init_var():
    #Here we initialize the global variables.
    global time_start, prev_pos, init_pos, travelled_distance, mean_distance_to_goal, goal
    goal = MoveBaseGoal()
    time_start = rospy.get_time()
    init_pos = geometry_msgs.msg.Point()
    prev_pos = geometry_msgs.msg.Point()
    curr_pos = geometry_msgs.msg.Point()
    travelled_distance = 0.0 #the less the better
    mean_distance_to_goal = 0.0 #the less the better

    global n_sensors, mean_obs_distance, min_obs_distance, collision_flag, n_collision, risk_mean
    n_sensors = 675
    mean_obs_distance = 0.0 #SM1
    min_obs_distance  = [1000.0] * n_sensors #SM2
    collision_flag = False
    n_collision = 0
    risk_mean = -1.0

    global r_warning, r_critical, obstacle_zone, prev_obstacle_zone, clear_zone, warning_zone, critical_zone, warning_duration, critical_duration
    r_warning  = 0.0
    r_critical = 0.0
    obstacle_zone, prev_obstacle_zone = 0, 0
    clear_zone, warning_zone, critical_zone = 0,1,2
    warning_duration  = 0.0
    critical_duration = 0.0
    

def movebase_client():
    global goal
    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #client = actionlib.SimpleActionClient('turtlebot2i_0/move_base', MoveBaseAction)
    client = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
    client.wait_for_server()

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 2.2 #2.0
    goal.target_pose.pose.position.y = 0.8 #4.0
    goal.target_pose.pose.position.z = 0.063 #1.34851861
    #goal.target_pose.pose.orientation.w = 1.0

    #copied from navi_goal_talker
    orientation=geometry_msgs.msg.Quaternion()
    yaw  =-90*math.pi/180 #unit: from deg. to rad.
    orientation=quaternion_from_euler(0,0,yaw)#(roll, pitch,yaw) # return an array
    goal.target_pose.pose.orientation.x=0.0
    goal.target_pose.pose.orientation.y=0.0
    goal.target_pose.pose.orientation.z=orientation[2]
    goal.target_pose.pose.orientation.w=orientation[3]

    client.send_goal(goal, active_cb=init_subscription, done_cb=finish_callback)
    print("Goal position is sent! waiting the robot to finish....")
    wait = client.wait_for_result(timeout=rospy.Duration(600.0)) #timeout in seconds
    if not wait:
        rospy.logerr("Action server not available or timeout!")
        rospy.signal_shutdown("Action server not available!")
    #else:
    #    state = client.get_state()
    #    if state == actionlib.GoalStatus.SUCCEEDED:
    #        rospy.loginfo("Goal succeeded!")
        #return client.get_result()

def distance2D(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

def init_subscription():
    time_start = rospy.get_time()
    rospy.Subscriber('/turtlebot2i/sensors/global_pose', geometry_msgs.msg.PoseStamped, update_pose_callback)
    rospy.Subscriber('/turtlebot2i/lidar/scan_data_collection', LaserScan, lidar_callback)
    rospy.Subscriber('/turtlebot2i/safety/risk_val', Float64, risk_callback)
    rospy.Subscriber('/turtlebot2i/safety/safety_zone', SafetyZone, safety_zone_callback)

def finish_callback(state_data, data2):
    global warning_duration, critical_duration
    if state_data == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached!")
    time_finish = rospy.get_time()
    duration = time_finish - time_start
    if obstacle_zone == warning_zone:
        warning_time_end  = rospy.get_time()
        warning_duration += (warning_time_end-warning_time_start)
    elif obstacle_zone == critical_zone:
        critical_time_end = rospy.get_time()
        critical_duration += (critical_time_end - critical_time_start)
    safe_duration = duration-(warning_duration+critical_duration)
    rospy.loginfo("1. Time execution        : %1.4f seconds (less is better)",duration)
    rospy.loginfo(" a) Safe (green) zone    : %1.4f s (%.1f%%) (more is better)",safe_duration,     100*safe_duration/duration)
    rospy.loginfo(" b) Warning (yellow) zone: %1.4f s (%.1f%%) (more is better)",warning_duration,  100*warning_duration/duration)
    rospy.loginfo(" c) Critical (red )zone  : %1.4f s (%.1f%%) (more is better)",critical_duration, 100*critical_duration/duration)
    rospy.loginfo("2. Travelled distance    : %1.4f m       (less is better)",travelled_distance)
    rospy.loginfo("3. Average speed         : %1.4f m/s     (more is better)",travelled_distance/duration)
    rospy.loginfo("4. Mean distance to goal : %1.4f m       (less is better)",mean_distance_to_goal)
    rospy.loginfo("5. SM1 mean distance to obstacle         : %1.4f m (more is better)",mean_obs_distance)
    rospy.loginfo("6. SM2 mean minimum distance to obstacle : %1.4f m (more is better)",sum(min_obs_distance)/n_sensors)
    rospy.loginfo("7. Minimum distance to obstacle          : %1.4f m (more is better)",min(min_obs_distance))
    rospy.loginfo("8. Number of collision during operation  : %d      (less is better)",n_collision)
    rospy.loginfo("9. Risk mean(average) value              : %1.4f   (less is better)",risk_mean)


def update_pose_callback(data):
    global prev_pos, init_pos, travelled_distance, mean_distance_to_goal
    if prev_pos == init_pos:
        prev_pos = data.pose.position
        mean_distance_to_goal = distance2D(prev_pos, goal.target_pose.pose.position)
    else:
        curr_pos              = data.pose.position
        travelled_distance   += distance2D(curr_pos, prev_pos)
        mean_distance_to_goal = (mean_distance_to_goal + distance2D(curr_pos, goal.target_pose.pose.position))/2.0
        prev_pos              = data.pose.position

def lidar_callback(data):
    global n_sensors, mean_obs_distance, min_obs_distance, collision_flag, n_collision
    global r_warning, r_critical, obstacle_zone, prev_obstacle_zone, clear_zone, warning_zone, critical_zone
    global warning_duration, critical_duration
    sensor_reads = data.ranges[4:679]
    #print(sensor_reads[0], sensor_reads[-1])
    #print("Lidar callback", len(sensor_reads), min(sensor_reads), max(sensor_reads))
    if mean_obs_distance == 0.0:
        mean_obs_distance = sum(sensor_reads)/n_sensors
    else:
        mean_obs_distance = (mean_obs_distance + sum(sensor_reads)/n_sensors) / 2.0

    for i in range(n_sensors):
        min_obs_distance[i] = min(min_obs_distance[i], sensor_reads[i])

    min_dist_to_obstacle = min(sensor_reads)
    if min_dist_to_obstacle == data.range_min and not collision_flag:
        print("collision happen!")
        collision_flag = True
        n_collision += 1
    elif min_dist_to_obstacle > data.range_min and collision_flag:
        collision_flag = False

    prev_obstacle_zone = obstacle_zone
    if min_dist_to_obstacle > r_warning:
        obstacle_zone = clear_zone
    elif min_dist_to_obstacle > r_critical:
        obstacle_zone = warning_zone
    else:
        obstacle_zone = critical_zone

    if obstacle_zone!=prev_obstacle_zone:
        if obstacle_zone == warning_zone:
            warning_time_start = rospy.get_time()
        elif obstacle_zone == critical_zone:
            critical_time_start = rospy.get_time()

        if prev_obstacle_zone == warning_zone:
            warning_time_end  = rospy.get_time()
            warning_duration += (warning_time_end-warning_time_start)
        elif prev_obstacle_zone == critical_zone:
            critical_time_end = rospy.get_time()
            critical_duration += (critical_time_end - critical_time_start)


def risk_callback(risk_value):
    global risk_mean
    if risk_mean == -1.0:
        risk_mean = risk_value.data
    else:
        risk_mean = (risk_mean + risk_value.data) / 2.0

def safety_zone_callback(data):
    global r_warning, r_critical
    r_warning  = data.warning_zone_radius
    r_critical = data.critical_zone_radius

if __name__ == '__main__':
    try:
        rospy.init_node('test_run_py')
        init_var()
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
