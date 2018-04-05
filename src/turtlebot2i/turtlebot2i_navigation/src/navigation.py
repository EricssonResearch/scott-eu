#!/usr/bin/env python

""" move_base_square.py - Version 0.1 2012-01-10
    Command a robot to move in a square using move_base actions..
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
#from visualization_msgs.msg import Marker
#from math import radians, pi

class MoveBaseRobot():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
            
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the move_base action server
        #self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base = actionlib.SimpleActionClient("turtlebot2i/move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        # Intialize the waypoint goal
        goal = MoveBaseGoal()
        
        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'map'
        
        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set the goal pose to the i-th waypoint
        goal.target_pose.pose = Pose()
        goal.target_pose.pose.position.x = 2.0
        goal.target_pose.pose.position.y = 1.0
        #goal.target_pose.pose.position.z = 1.0
        #goal.target_pose.pose.position.z = 1.34852790833
        goal.target_pose.pose.position.z = 1.34852790833
        
        # Start the robot moving toward the goal
        rospy.loginfo("Before move")
        self.move(goal)
        rospy.loginfo("After move")
        
    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)
            
            # Allow 1 minute to get there
            # Change the timeout
            #finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
            finished_within_time = self.move_base.wait_for_result() 
            
            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")

# Supposed not to be necessary
#    def init_markers(self):
#        # Set up our waypoint markers
#        marker_scale = 0.2
#        marker_lifetime = 0 # 0 is forever
#        marker_ns = 'waypoints'
#        marker_id = 0
#        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
#        
#        # Define a marker publisher.
#        self.marker_pub = rospy.Publisher('waypoint_markers', Marker)
#        
#        # Initialize the marker points list.
#        self.markers = Marker()
#        self.markers.ns = marker_ns
#        self.markers.id = marker_id
#        self.markers.type = Marker.SPHERE_LIST
#        self.markers.action = Marker.ADD
#        self.markers.lifetime = rospy.Duration(marker_lifetime)
#        self.markers.scale.x = marker_scale
#        self.markers.scale.y = marker_scale
#        self.markers.color.r = marker_color['r']
#        self.markers.color.g = marker_color['g']
#        self.markers.color.b = marker_color['b']
#        self.markers.color.a = marker_color['a']
#        
#        self.markers.header.frame_id = 'map'
#        self.markers.header.stamp = rospy.Time.now()
#        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseRobot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
