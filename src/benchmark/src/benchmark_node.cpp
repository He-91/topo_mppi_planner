/**
 * @file benchmark_node.cpp
 * @brief Unified benchmark metrics collector for planner comparison.
 *
 * Subscribes to odometry, point cloud, and position command topics to compute:
 *   - Success / failure (reached goal within timeout)
 *   - Travel time (s)
 *   - Trajectory length (m)
 *   - Minimum obstacle distance (m)
 *   - Collision count (distance < threshold)
 *   - Average / max velocity (m/s)
 *   - Average / max acceleration (m/s^2)
 *   - Trajectory smoothness (integral of jerk^2)
 *   - Planning computation time (from planner topic, if available)
 *
 * Results are written to a CSV file for later analysis.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <ddo_planner/DynamicObstacles.h>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <fstream>
#include <iomanip>
#include <algorithm>
#include <deque>
#include <mutex>
#include <numeric>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

class BenchmarkNode {
public:
  BenchmarkNode(ros::NodeHandle& nh) {
    // Parameters
    nh.param("benchmark/method_name", method_name_, std::string("unknown"));
    nh.param("benchmark/scenario_name", scenario_name_, std::string("unknown"));
    nh.param("benchmark/run_id", run_id_, 0);
    nh.param("benchmark/timeout", timeout_, 120.0);
    nh.param("benchmark/goal_reach_threshold", goal_threshold_, 1.0);
    nh.param("benchmark/min_success_traj_length", min_success_traj_length_, 0.0);
    nh.param("benchmark/collision_threshold", collision_threshold_, 0.3);
    nh.param("benchmark/collision_exit_threshold", collision_exit_threshold_,
             collision_threshold_ * 1.25);
    nh.param("benchmark/safety_margin", safety_margin_, 0.65);
    nh.param("benchmark/auto_start", auto_start_, false);
    nh.param("benchmark/output_dir", output_dir_, std::string("results"));
    nh.param("benchmark/output_file", output_file_,
             std::string("benchmark_results.csv"));
    nh.param("benchmark/quality_output_file", quality_output_file_,
             std::string("benchmark_quality_metrics.csv"));
    nh.param("benchmark/odom_topic", odom_topic_, std::string("/visual_slam/odom"));
    nh.param("benchmark/cloud_topic", cloud_topic_, std::string("/pcl_render_node/cloud"));
    nh.param("benchmark/trigger_topic", trigger_topic_, std::string("/traj_start_trigger"));
    nh.param("benchmark/dynamic_obstacle_topic", dynamic_obstacle_topic_,
             std::string("/dynamic_obstacles/state"));
    nh.param("benchmark/dynamic_safety_margin", dynamic_safety_margin_, 0.65);
    nh.param("benchmark/max_valid_odom_speed", max_valid_odom_speed_, 8.0);
    nh.param("benchmark/max_valid_odom_acc", max_valid_odom_acc_, 30.0);
    nh.param("benchmark/max_valid_odom_jerk", max_valid_odom_jerk_, 500.0);

    // Goal position
    nh.param("benchmark/goal_x", goal_.x(), 19.0);
    nh.param("benchmark/goal_y", goal_.y(), 0.0);
    nh.param("benchmark/goal_z", goal_.z(), 1.0);
    nh.param("benchmark/checkpoint_reach_threshold",
             checkpoint_reach_threshold_, goal_threshold_);
    int checkpoint_count = 0;
    nh.param("benchmark/checkpoint_count", checkpoint_count, 0);
    checkpoint_count = std::max(0, std::min(checkpoint_count, 8));
    route_checkpoints_.reserve(checkpoint_count);
    for (int i = 0; i < checkpoint_count; ++i) {
      Eigen::Vector3d checkpoint;
      nh.param("benchmark/checkpoint" + std::to_string(i) + "_x",
               checkpoint.x(), 0.0);
      nh.param("benchmark/checkpoint" + std::to_string(i) + "_y",
               checkpoint.y(), 0.0);
      nh.param("benchmark/checkpoint" + std::to_string(i) + "_z",
               checkpoint.z(), goal_.z());
      route_checkpoints_.push_back(checkpoint);
    }

    // Subscribers
    odom_sub_ = nh.subscribe(odom_topic_, 50,
                             &BenchmarkNode::odomCallback, this);
    cloud_sub_ = nh.subscribe(cloud_topic_, 5,
                              &BenchmarkNode::cloudCallback, this);
    trigger_sub_ = nh.subscribe(trigger_topic_, 1,
                                &BenchmarkNode::triggerCallback, this);
    dynamic_sub_ = nh.subscribe(dynamic_obstacle_topic_, 10,
                                &BenchmarkNode::dynamicObstacleCallback, this);

    // State
    started_ = false;
    finished_ = false;
    success_ = false;
    in_collision_ = false;
    collision_count_ = 0;
    collision_sample_count_ = 0;
    below_margin_sample_count_ = 0;
    cloud_sample_count_ = 0;
    dynamic_collision_count_ = 0;
    dynamic_collision_sample_count_ = 0;
    dynamic_below_margin_sample_count_ = 0;
    dynamic_sample_count_ = 0;
    total_length_ = 0.0;
    min_obs_dist_ = 1e6;
    min_dynamic_clearance_ = 1e6;
    time_below_margin_ = 0.0;
    dynamic_time_below_margin_ = 0.0;
	    jerk_integral_ = 0.0;
    min_z_ = std::numeric_limits<double>::infinity();
    max_z_ = -std::numeric_limits<double>::infinity();
    low_altitude_sample_count_ = 0;

    ROS_INFO("[Benchmark] Method=%s Scenario=%s Run=%d Goal=(%.1f,%.1f,%.1f)",
             method_name_.c_str(), scenario_name_.c_str(), run_id_,
             goal_.x(), goal_.y(), goal_.z());
    ROS_INFO("[Benchmark] Topics: odom=%s cloud=%s trigger=%s auto_start=%s",
             odom_topic_.c_str(), cloud_topic_.c_str(), trigger_topic_.c_str(),
             auto_start_ ? "true" : "false");
    ROS_INFO("[Benchmark] Dynamic obstacle topic: %s",
             dynamic_obstacle_topic_.c_str());
    ROS_INFO("[Benchmark] Route checkpoints: %zu threshold=%.2fm",
             route_checkpoints_.size(), checkpoint_reach_threshold_);

    if (auto_start_) {
      started_ = true;
      start_time_ = ros::Time::now();
      ROS_INFO("[Benchmark] Auto-start enabled, starting metrics collection.");
    }
  }

  void spin() {
    ros::Rate rate(50);
    while (ros::ok() && !finished_) {
      ros::spinOnce();

      if (started_ && !finished_) {
        double elapsed = (ros::Time::now() - start_time_).toSec();

        // Check goal reached
        if (last_pos_valid_) {
          double dist_to_goal = (last_pos_ - goal_).norm();
          if (dist_to_goal < goal_threshold_ &&
              total_length_ >= min_success_traj_length_ &&
              routeCheckpointsComplete()) {
            finished_ = true;
            success_ = true;
            end_time_ = ros::Time::now();
            ROS_INFO("[Benchmark] Goal reached! Time=%.2fs Length=%.2fm",
                     elapsed, total_length_);
          }
        }

        // Check timeout
        if (!finished_ && elapsed > timeout_) {
          finished_ = true;
          success_ = false;
          end_time_ = ros::Time::now();
          ROS_WARN("[Benchmark] Timeout after %.1fs!", timeout_);
        }
      }

      rate.sleep();
    }

    // Write results
    writeResults();
  }

private:
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (!started_) {
      started_ = true;
      start_time_ = ros::Time::now();
      ROS_INFO("[Benchmark] Trigger received, starting metrics collection.");
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    Eigen::Vector3d pos(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    Eigen::Vector3d vel(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    double t = msg->header.stamp.toSec();

    if (!started_) {
      last_pos_ = pos;
      last_pos_time_ = t;
      last_pos_valid_ = true;
      return;
    }

    double dt_pos = 0.0;
    double seg = 0.0;
    bool position_motion_valid = false;
    if (last_pos_valid_) {
      dt_pos = t - last_pos_time_;
      seg = (pos - last_pos_).norm();
      const double position_speed =
          (dt_pos > 1e-4) ? seg / dt_pos : std::numeric_limits<double>::infinity();
      position_motion_valid =
          dt_pos > 1e-4 && dt_pos < 0.2 && seg < 2.0 &&
          std::isfinite(position_speed) &&
          position_speed <= max_valid_odom_speed_;
      if (position_motion_valid) {
        total_length_ += seg;
      }
    }

    const double speed = vel.norm();
    const bool velocity_valid =
        std::isfinite(speed) && speed <= max_valid_odom_speed_;
    if (velocity_valid) {
      velocities_.push_back(speed);
    }
    min_z_ = std::min(min_z_, pos.z());
    max_z_ = std::max(max_z_, pos.z());
    if (pos.z() < 0.65) {
      low_altitude_sample_count_++;
    }
    updateRouteCheckpointProgress(pos);

    if (last_vel_valid_ && velocity_valid) {
      double dt = t - last_vel_time_;
      if (dt > 1e-4 && dt < 0.1) {
        Eigen::Vector3d acc = (vel - last_vel_) / dt;
        double acc_norm = acc.norm();
        if (std::isfinite(acc_norm) && acc_norm <= max_valid_odom_acc_) {
          accelerations_.push_back(acc_norm);

          if (last_acc_valid_) {
            Eigen::Vector3d jerk = (acc - last_acc_) / dt;
            const double jerk_norm = jerk.norm();
            if (std::isfinite(jerk_norm) && jerk_norm <= max_valid_odom_jerk_) {
              jerk_integral_ += jerk.squaredNorm() * dt;
            }
            last_acc_ = acc;
          } else {
            last_acc_ = acc;
            last_acc_valid_ = true;
          }
        } else {
          last_acc_valid_ = false;
        }
      }
    } else {
      last_acc_valid_ = false;
    }

    last_pos_ = pos;
    last_pos_time_ = t;
    last_pos_valid_ = true;
    if (velocity_valid) {
      last_vel_ = vel;
      last_vel_time_ = t;
      last_vel_valid_ = true;
    } else {
      last_vel_valid_ = false;
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!started_ || !last_pos_valid_) return;

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;
    cloud_sample_count_++;

    // Build KD-tree and find nearest obstacle
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    pcl::PointXYZ search_pt;
    search_pt.x = last_pos_.x();
    search_pt.y = last_pos_.y();
    search_pt.z = last_pos_.z();

    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (kdtree.nearestKSearch(search_pt, 1, indices, distances) > 0) {
      double dist = std::sqrt(distances[0]);
      if (dist < min_obs_dist_) {
        min_obs_dist_ = dist;
      }

      const ros::Time now = ros::Time::now();
      const ros::Time stamp = msg->header.stamp.isZero() ? now : msg->header.stamp;
      if (last_cloud_stamp_valid_) {
        double dt = (stamp - last_cloud_stamp_).toSec();
        if (dt <= 0.0) {
          dt = (now - last_cloud_wall_stamp_).toSec();
        }
        if (dt > 0.0 && dt < 1.0 && dist < safety_margin_) {
          time_below_margin_ += dt;
        }
      }
      last_cloud_stamp_ = stamp;
      last_cloud_wall_stamp_ = now;
      last_cloud_stamp_valid_ = true;

      if (dist < safety_margin_) {
        below_margin_sample_count_++;
      }
      if (dist < collision_threshold_) {
        collision_sample_count_++;
        if (!in_collision_) {
          collision_count_++;
          in_collision_ = true;
        }
      } else if (dist > collision_exit_threshold_) {
        in_collision_ = false;
      }
    }
  }

  void updateRouteCheckpointProgress(const Eigen::Vector3d& pos) {
    if (next_checkpoint_idx_ >= route_checkpoints_.size()) {
      return;
    }
    const double dist = (pos - route_checkpoints_[next_checkpoint_idx_]).norm();
    if (dist <= checkpoint_reach_threshold_) {
      ROS_INFO("[Benchmark] Route checkpoint %zu/%zu reached (dist=%.2fm)",
               next_checkpoint_idx_ + 1, route_checkpoints_.size(), dist);
      ++next_checkpoint_idx_;
    }
  }

  bool routeCheckpointsComplete() const {
    return next_checkpoint_idx_ >= route_checkpoints_.size();
  }

  double distanceToDynamicCylinder(const Eigen::Vector3d& pos,
                                   const ddo_planner::DynamicObstacle& obs) const {
    const double dx = pos.x() - obs.position.x;
    const double dy = pos.y() - obs.position.y;
    const double radial_out = std::sqrt(dx * dx + dy * dy) - std::max(0.0, obs.radius);
    const double half_height = std::max(0.0, obs.height) * 0.5;
    const double vertical_out = std::fabs(pos.z() - obs.position.z) - half_height;

    if (radial_out <= 0.0 && vertical_out <= 0.0) {
      return std::max(radial_out, vertical_out);
    }
    const double clamped_radial = std::max(0.0, radial_out);
    const double clamped_vertical = std::max(0.0, vertical_out);
    return std::sqrt(clamped_radial * clamped_radial +
                     clamped_vertical * clamped_vertical);
  }

  void dynamicObstacleCallback(const ddo_planner::DynamicObstaclesConstPtr& msg) {
    if (!started_ || !last_pos_valid_ || msg->obstacles.empty()) return;

    double min_clearance = 1e6;
    for (const auto& obs : msg->obstacles) {
      min_clearance = std::min(min_clearance,
                               distanceToDynamicCylinder(last_pos_, obs));
    }

    if (min_clearance >= 1e5) return;
    dynamic_sample_count_++;
    min_dynamic_clearance_ = std::min(min_dynamic_clearance_, min_clearance);

    const ros::Time now = ros::Time::now();
    const ros::Time stamp = msg->header.stamp.isZero() ? now : msg->header.stamp;
    if (last_dynamic_stamp_valid_) {
      double dt = (stamp - last_dynamic_stamp_).toSec();
      if (dt <= 0.0) {
        dt = (now - last_dynamic_wall_stamp_).toSec();
      }
      if (dt > 0.0 && dt < 1.0 && min_clearance < dynamic_safety_margin_) {
        dynamic_time_below_margin_ += dt;
      }
    }
    last_dynamic_stamp_ = stamp;
    last_dynamic_wall_stamp_ = now;
    last_dynamic_stamp_valid_ = true;

    if (min_clearance < dynamic_safety_margin_) {
      dynamic_below_margin_sample_count_++;
    }
    if (min_clearance < collision_threshold_) {
      dynamic_collision_sample_count_++;
      if (!in_dynamic_collision_) {
        dynamic_collision_count_++;
        in_dynamic_collision_ = true;
      }
    } else if (min_clearance > collision_exit_threshold_) {
      in_dynamic_collision_ = false;
    }
  }

  void writeResults() {
    if (started_ && end_time_.isZero()) {
      end_time_ = ros::Time::now();
    }
    double travel_time = started_ ? (end_time_ - start_time_).toSec() : 0.0;
    if (!success_ && travel_time <= 0.0) {
      travel_time = timeout_;
    }

    // Compute velocity stats
    double avg_vel = 0.0, max_vel = 0.0;
    for (double v : velocities_) {
      avg_vel += v;
      if (v > max_vel) max_vel = v;
    }
    if (!velocities_.empty()) avg_vel /= velocities_.size();

    // Compute acceleration stats
    double avg_acc = 0.0, max_acc = 0.0;
    for (double a : accelerations_) {
      avg_acc += a;
      if (a > max_acc) max_acc = a;
    }
    if (!accelerations_.empty()) avg_acc /= accelerations_.size();

    // Output file
    mkdir(output_dir_.c_str(), 0775);
    std::string filename = output_dir_ + "/" + output_file_;
    bool file_exists = false;
    {
      std::ifstream check(filename);
      file_exists = check.good();
    }

    std::ofstream ofs(filename, std::ios::app);
    if (!file_exists) {
      ofs << "method,scenario,run_id,success,travel_time_s,traj_length_m,"
          << "min_obs_dist_m,collision_count,collision_sample_count,"
          << "time_below_margin_s,below_margin_sample_count,cloud_samples,"
          << "min_static_clearance_m,static_collision_count,"
          << "static_collision_sample_count,static_time_below_margin_s,"
          << "static_below_margin_sample_count,static_samples,"
          << "min_dynamic_clearance_m,dynamic_collision_count,"
          << "dynamic_collision_sample_count,dynamic_time_below_margin_s,"
          << "dynamic_below_margin_sample_count,dynamic_samples,"
          << "avg_vel_ms,max_vel_ms,avg_acc_ms2,max_acc_ms2,jerk_integral,"
          << "smoothness_cost,collision_threshold_m,safety_margin_m,"
          << "dynamic_safety_margin_m"
          << std::endl;
    }

    // Smoothness cost = jerk_integral / traj_length (normalized)
    double smoothness = (total_length_ > 0.1) ? jerk_integral_ / total_length_ : 0.0;
    const double min_obs_dist_out = (cloud_sample_count_ > 0) ? min_obs_dist_ : -1.0;
    const double min_dynamic_clearance_out =
        (dynamic_sample_count_ > 0) ? min_dynamic_clearance_ : -1.0;

    ofs << std::fixed << std::setprecision(4)
        << method_name_ << ","
        << scenario_name_ << ","
        << run_id_ << ","
        << (success_ ? 1 : 0) << ","
        << travel_time << ","
        << total_length_ << ","
        << min_obs_dist_out << ","
        << collision_count_ << ","
        << collision_sample_count_ << ","
        << time_below_margin_ << ","
        << below_margin_sample_count_ << ","
        << cloud_sample_count_ << ","
        << min_obs_dist_out << ","
        << collision_count_ << ","
        << collision_sample_count_ << ","
        << time_below_margin_ << ","
        << below_margin_sample_count_ << ","
        << cloud_sample_count_ << ","
        << min_dynamic_clearance_out << ","
        << dynamic_collision_count_ << ","
        << dynamic_collision_sample_count_ << ","
        << dynamic_time_below_margin_ << ","
        << dynamic_below_margin_sample_count_ << ","
        << dynamic_sample_count_ << ","
        << avg_vel << ","
        << max_vel << ","
        << avg_acc << ","
        << max_acc << ","
        << jerk_integral_ << ","
        << smoothness << ","
        << collision_threshold_ << ","
        << safety_margin_ << ","
        << dynamic_safety_margin_
        << std::endl;
    ofs.close();

    ROS_INFO("[Benchmark] Results written to %s", filename.c_str());
    ROS_INFO("[Benchmark] === Summary ===");
    ROS_INFO("  Method:       %s", method_name_.c_str());
    ROS_INFO("  Scenario:     %s", scenario_name_.c_str());
    ROS_INFO("  Run ID:       %d", run_id_);
    ROS_INFO("  Success:      %s", success_ ? "YES" : "NO");
    ROS_INFO("  Travel time:  %.2f s", travel_time);
    ROS_INFO("  Traj length:  %.2f m", total_length_);
    ROS_INFO("  Min obs dist: %.3f m", min_obs_dist_out);
    ROS_INFO("  Collisions:   %d events (%d samples)",
             collision_count_, collision_sample_count_);
    ROS_INFO("  Below margin: %.3f s (%d samples, margin=%.2fm)",
             time_below_margin_, below_margin_sample_count_, safety_margin_);
    ROS_INFO("  Dynamic clearance: %.3f m, collisions: %d events (%d samples)",
             min_dynamic_clearance_out,
             dynamic_collision_count_, dynamic_collision_sample_count_);
    ROS_INFO("  Dynamic below margin: %.3f s (%d samples, margin=%.2fm)",
             dynamic_time_below_margin_, dynamic_below_margin_sample_count_,
             dynamic_safety_margin_);
    ROS_INFO("  Avg velocity: %.2f m/s", avg_vel);
    ROS_INFO("  Max velocity: %.2f m/s", max_vel);
	    ROS_INFO("  Smoothness:   %.4f", smoothness);
    writeQualityResults(travel_time, avg_vel, max_vel, avg_acc, max_acc, smoothness);
	  }

  double percentile(std::vector<double> values, double q) const {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double clamped_q = std::min(1.0, std::max(0.0, q));
    const size_t idx =
        std::min(values.size() - 1,
                 static_cast<size_t>(std::round(clamped_q * (values.size() - 1))));
    return values[idx];
  }

  void writeQualityResults(double travel_time,
                           double avg_vel,
                           double max_vel,
                           double avg_acc,
                           double max_acc,
                           double smoothness) {
    mkdir(output_dir_.c_str(), 0775);
    const std::string filename = output_dir_ + "/" + quality_output_file_;
    bool file_exists = false;
    {
      std::ifstream check(filename);
      file_exists = check.good();
    }

    std::ofstream ofs(filename, std::ios::app);
    if (!file_exists) {
      ofs << "method,scenario,run_id,success,travel_time_s,traj_length_m,"
          << "avg_vel_ms,max_vel_ms,p95_vel_ms,p99_vel_ms,"
          << "avg_acc_ms2,max_acc_ms2,p95_acc_ms2,p99_acc_ms2,"
          << "jerk_integral,smoothness_cost,min_z_m,max_z_m,"
          << "low_altitude_sample_count,odom_samples"
          << std::endl;
    }

    const double min_z_out = std::isfinite(min_z_) ? min_z_ : 0.0;
    const double max_z_out = std::isfinite(max_z_) ? max_z_ : 0.0;
    ofs << std::fixed << std::setprecision(4)
        << method_name_ << ","
        << scenario_name_ << ","
        << run_id_ << ","
        << (success_ ? 1 : 0) << ","
        << travel_time << ","
        << total_length_ << ","
        << avg_vel << ","
        << max_vel << ","
        << percentile(velocities_, 0.95) << ","
        << percentile(velocities_, 0.99) << ","
        << avg_acc << ","
        << max_acc << ","
        << percentile(accelerations_, 0.95) << ","
        << percentile(accelerations_, 0.99) << ","
        << jerk_integral_ << ","
        << smoothness << ","
        << min_z_out << ","
        << max_z_out << ","
        << low_altitude_sample_count_ << ","
        << velocities_.size()
        << std::endl;
    ofs.close();
    ROS_INFO("[Benchmark] Quality metrics written to %s", filename.c_str());
  }

  // ROS
  ros::Subscriber odom_sub_, cloud_sub_, trigger_sub_, dynamic_sub_;

  // Parameters
	  std::string method_name_, scenario_name_, output_dir_, output_file_, quality_output_file_;
  std::string odom_topic_, cloud_topic_, trigger_topic_, dynamic_obstacle_topic_;
  int run_id_;
  double timeout_, goal_threshold_, collision_threshold_, collision_exit_threshold_;
  double safety_margin_, dynamic_safety_margin_;
  double max_valid_odom_speed_, max_valid_odom_acc_, max_valid_odom_jerk_;
  double min_success_traj_length_;
  double checkpoint_reach_threshold_;
  Eigen::Vector3d goal_;
  std::vector<Eigen::Vector3d> route_checkpoints_;
  size_t next_checkpoint_idx_ = 0;

  // State
  bool started_, finished_, success_, auto_start_, in_collision_;
  bool in_dynamic_collision_ = false;
  ros::Time start_time_, end_time_, last_cloud_stamp_, last_dynamic_stamp_;
  ros::Time last_cloud_wall_stamp_, last_dynamic_wall_stamp_;
  bool last_cloud_stamp_valid_ = false;
  bool last_dynamic_stamp_valid_ = false;
  bool last_pos_valid_ = false, last_vel_valid_ = false, last_acc_valid_ = false;
  Eigen::Vector3d last_pos_, last_vel_, last_acc_;
  double last_pos_time_ = 0.0;
  double last_vel_time_ = 0.0;

  // Metrics
  double total_length_, min_obs_dist_, min_dynamic_clearance_;
	  double time_below_margin_, dynamic_time_below_margin_, jerk_integral_;
  double min_z_, max_z_;
	  int collision_count_, collision_sample_count_, below_margin_sample_count_;
	  int cloud_sample_count_, dynamic_collision_count_, dynamic_collision_sample_count_;
	  int dynamic_below_margin_sample_count_, dynamic_sample_count_;
  int low_altitude_sample_count_;
  std::vector<double> velocities_, accelerations_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "benchmark_node");
  ros::NodeHandle nh("~");

  BenchmarkNode node(nh);
  node.spin();

  return 0;
}
