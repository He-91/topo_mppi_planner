/**
 * @file goal_publisher.cpp
 * @brief Publishes a goal waypoint after a configurable delay,
 *        then triggers trajectory start for benchmark automation.
 *
 * This replaces manual rviz "2D Nav Goal" clicks with automated goal publishing.
 *
 * For DDO/EGO/Fast-Planner: publishes PoseStamped to /move_base_simple/goal
 *   + PoseStamped trigger to /traj_start_trigger.
 * For TGK-Planner: publishes quadrotor_msgs::PositionCommand to /goal
 *   + PoseStamped trigger to /traj_start_trigger.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

class GoalPublisher {
public:
  GoalPublisher(ros::NodeHandle& nh) {
    nh.param("goal_x", goal_x_, 19.0);
    nh.param("goal_y", goal_y_, 0.0);
    nh.param("goal_z", goal_z_, 1.0);
    nh.param("delay_sec", delay_sec_, 3.0);
    nh.param("trigger_delay_sec", trigger_delay_, 1.0);
    nh.param("method_name", method_name_, std::string("ddo"));

    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1);
    // TGK-Planner subscribes to /goal with PositionCommand type
    tgk_goal_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/goal", 1);
    odom_sub_ = nh.subscribe("/visual_slam/odom", 1, &GoalPublisher::odomCallback, this);

    published_ = false;
    odom_received_ = false;
  }

  void spin() {
    ros::Rate rate(10);
    while (ros::ok()) {
      ros::spinOnce();

      if (!published_ && odom_received_) {
        double elapsed = (ros::Time::now() - first_odom_time_).toSec();
        if (elapsed >= delay_sec_) {
          publishGoal();
          published_ = true;

          // Wait a bit then send trigger
          ros::Duration(trigger_delay_).sleep();
          publishTrigger();
          ROS_INFO("[GoalPublisher] Goal and trigger sent. Exiting loop.");
          // Stay alive for benchmark_node to subscribe
          ros::Duration(2.0).sleep();
          return;
        }
      }
      rate.sleep();
    }
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (!odom_received_) {
      odom_received_ = true;
      first_odom_time_ = ros::Time::now();
      ROS_INFO("[GoalPublisher] Odometry received, will publish goal in %.1fs (method=%s)",
               delay_sec_, method_name_.c_str());
    }
  }

  void publishGoal() {
    // Standard goal for DDO/EGO/Fast
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;
    goal.pose.position.z = goal_z_;
    goal.pose.orientation.w = 1.0;
    goal_pub_.publish(goal);

    // TGK-specific goal: PositionCommand on /goal
    if (method_name_ == "tgk") {
      quadrotor_msgs::PositionCommand tgk_goal;
      tgk_goal.header.stamp = ros::Time::now();
      tgk_goal.header.frame_id = "world";
      tgk_goal.position.x = goal_x_;
      tgk_goal.position.y = goal_y_;
      tgk_goal.position.z = goal_z_;
      tgk_goal.velocity.x = 0.0;
      tgk_goal.velocity.y = 0.0;
      tgk_goal.velocity.z = 0.0;
      tgk_goal.acceleration.x = 0.0;
      tgk_goal.acceleration.y = 0.0;
      tgk_goal.acceleration.z = 0.0;
      tgk_goal_pub_.publish(tgk_goal);
      ROS_INFO("[GoalPublisher] Published TGK goal (PositionCommand) to /goal: (%.1f, %.1f, %.1f)",
               goal_x_, goal_y_, goal_z_);
    }

    ROS_INFO("[GoalPublisher] Published goal: (%.1f, %.1f, %.1f)",
             goal_x_, goal_y_, goal_z_);
  }

  void publishTrigger() {
    geometry_msgs::PoseStamped trigger;
    trigger.header.stamp = ros::Time::now();
    trigger.header.frame_id = "world";
    trigger.pose.orientation.w = 1.0;

    trigger_pub_.publish(trigger);
    ROS_INFO("[GoalPublisher] Published trajectory start trigger.");
  }

  ros::Publisher goal_pub_, trigger_pub_, tgk_goal_pub_;
  ros::Subscriber odom_sub_;

  std::string method_name_;
  double goal_x_, goal_y_, goal_z_;
  double delay_sec_, trigger_delay_;
  bool published_, odom_received_;
  ros::Time first_odom_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle nh("~");

  GoalPublisher gp(nh);
  gp.spin();

  return 0;
}
