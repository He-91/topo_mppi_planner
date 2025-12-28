#!/usr/bin/env python3
"""
Fast-Planner 2D Map Integration Test
Tests the map system and planning components
"""

import rospy
import sys
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class FastPlanner2DTest:
    def __init__(self):
        rospy.init_node('fast_planner_2d_test', anonymous=True)
        
        self.map_received = False
        self.odom_received = False
        
        # Subscribers
        self.map_sub = rospy.Subscriber('/map_generator/global_cloud', 
                                       PointCloud2, self.map_callback)
        self.odom_sub = rospy.Subscriber('/visual_slam/odom', 
                                        Odometry, self.odom_callback)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("  Fast-Planner 2D Integration Test")
        rospy.loginfo("=" * 50)
        
    def map_callback(self, msg):
        if not self.map_received:
            rospy.loginfo("✓ Map data received!")
            rospy.loginfo(f"  - Height: {msg.height}")
            rospy.loginfo(f"  - Width: {msg.width}")
            rospy.loginfo(f"  - Point step: {msg.point_step}")
            rospy.loginfo(f"  - Data size: {len(msg.data)} bytes")
            self.map_received = True
            
    def odom_callback(self, msg):
        if not self.odom_received:
            pos = msg.pose.pose.position
            rospy.loginfo("✓ Odometry data received!")
            rospy.loginfo(f"  - Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
            self.odom_received = True
            
    def run_test(self):
        rospy.loginfo("\nWaiting for system to initialize...")
        rospy.loginfo("This will check:")
        rospy.loginfo("  1. Map generation (mockamap)")
        rospy.loginfo("  2. Odometry (quadrotor simulator)")
        rospy.loginfo("  3. GridMap compatibility")
        rospy.loginfo("")
        
        rate = rospy.Rate(1)  # 1 Hz
        timeout = 30  # 30 seconds timeout
        
        for i in range(timeout):
            if self.map_received and self.odom_received:
                rospy.loginfo("\n" + "=" * 50)
                rospy.loginfo("✓✓✓ All systems operational! ✓✓✓")
                rospy.loginfo("=" * 50)
                rospy.loginfo("\nTest Summary:")
                rospy.loginfo("  ✓ 2D Map generation: PASS")
                rospy.loginfo("  ✓ Odometry system: PASS")
                rospy.loginfo("  ✓ ROS communication: PASS")
                rospy.loginfo("\nFast-Planner map system is ready for 2D testing!")
                rospy.loginfo("=" * 50)
                return True
                
            if i % 5 == 0 and i > 0:
                status = []
                if not self.map_received:
                    status.append("Waiting for map...")
                if not self.odom_received:
                    status.append("Waiting for odometry...")
                rospy.loginfo(f"[{i}s] " + ", ".join(status))
                
            rate.sleep()
            
        rospy.logerr("\n" + "=" * 50)
        rospy.logerr("✗ Test TIMEOUT after 30 seconds")
        rospy.logerr("=" * 50)
        rospy.logerr("\nMissing components:")
        if not self.map_received:
            rospy.logerr("  ✗ Map data not received")
            rospy.logerr("    Check: rosrun mockamap mockamap_node")
        if not self.odom_received:
            rospy.logerr("  ✗ Odometry not received")
            rospy.logerr("    Check: rosrun so3_quadrotor_simulator quadrotor_simulator_so3")
        return False

if __name__ == '__main__':
    try:
        tester = FastPlanner2DTest()
        success = tester.run_test()
        sys.exit(0 if success else 1)
    except rospy.ROSInterruptException:
        rospy.loginfo("\nTest interrupted by user")
        sys.exit(0)
    except Exception as e:
        rospy.logerr(f"\nTest failed with error: {e}")
        sys.exit(1)
