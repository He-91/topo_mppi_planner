#!/usr/bin/env python3

"""
Script to demonstrate the new topological and MPPI planning algorithms in EGO-Planner
Usage: rosrun ego_planner demo_new_algorithms.py
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt

class NewAlgorithmsDemoNode:
    def __init__(self):
        rospy.init_node('new_algorithms_demo')
        
        # Subscribers for visualization
        self.topo_paths_sub = rospy.Subscriber('/topo_paths', MarkerArray, self.topo_paths_callback)
        self.astar_paths_sub = rospy.Subscriber('/a_star_list', MarkerArray, self.astar_paths_callback)
        
        # Publishers for goal setting
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        rospy.loginfo("Demo node initialized. Subscribed to:")
        rospy.loginfo("  - /topo_paths (topological search results)")
        rospy.loginfo("  - /a_star_list (A* search visualization)")
        
    def topo_paths_callback(self, msg):
        """Process topological paths visualization"""
        rospy.loginfo(f"Received {len(msg.markers)} topological path markers")
        for i, marker in enumerate(msg.markers):
            if marker.type == marker.LINE_STRIP:
                rospy.loginfo(f"Path {i}: {len(marker.points)} waypoints, color=({marker.color.r:.2f}, {marker.color.g:.2f}, {marker.color.b:.2f})")
                
    def astar_paths_callback(self, msg):
        """Process A* paths visualization"""
        rospy.loginfo(f"Received {len(msg.markers)} A* path markers")
        
    def publish_test_goal(self, x, y, z):
        """Publish a test goal for planning"""
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        rospy.loginfo(f"Published goal: ({x}, {y}, {z})")

if __name__ == '__main__':
    try:
        demo = NewAlgorithmsDemoNode()
        
        # Publish some test goals
        rate = rospy.Rate(0.5)  # 0.5 Hz
        goals = [(5, 0, 1), (5, 5, 1), (0, 5, 1), (-5, 0, 1)]
        
        rospy.loginfo("Starting goal publishing sequence...")
        for goal in goals:
            demo.publish_test_goal(*goal)
            rate.sleep()
            
        rospy.loginfo("Demo complete. Keeping node alive for visualization...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass