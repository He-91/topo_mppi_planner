/**
 * @file cloud_merger.cpp
 * @brief 点云融合节点 - 合并静态和动态障碍物点云
 * @date 2025-10-29
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudMerger {
private:
  ros::NodeHandle nh_;
  ros::Subscriber static_sub_;
  ros::Subscriber dynamic_sub_;
  ros::Publisher merged_pub_;
  ros::Timer publish_timer_;
  
  pcl::PointCloud<pcl::PointXYZ> static_cloud_;
  pcl::PointCloud<pcl::PointXYZ> dynamic_cloud_;
  
  bool has_static_;
  bool has_dynamic_;
  double publish_rate_;
  
public:
  CloudMerger(ros::NodeHandle& nh) : nh_(nh), has_static_(false), has_dynamic_(false) {
    // 获取参数
    nh_.param("publish_rate", publish_rate_, 20.0);
    
    // 订阅器
    static_sub_ = nh_.subscribe("static_cloud", 10, &CloudMerger::staticCloudCallback, this);
    dynamic_sub_ = nh_.subscribe("dynamic_cloud", 10, &CloudMerger::dynamicCloudCallback, this);
    
    // 发布器
    merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merged_cloud", 10);
    
    // 定时发布
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                     &CloudMerger::publishCallback, this);
    
    ROS_INFO("========================================");
    ROS_INFO("🔗 点云融合节点已启动");
    ROS_INFO("   发布频率: %.1f Hz", publish_rate_);
    ROS_INFO("========================================");
  }
  
  void staticCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::fromROSMsg(*msg, static_cloud_);
    has_static_ = true;
  }
  
  void dynamicCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::fromROSMsg(*msg, dynamic_cloud_);
    has_dynamic_ = true;
  }
  
  void publishCallback(const ros::TimerEvent& event) {
    if (!has_static_ && !has_dynamic_) {
      return;
    }
    
    // 合并点云
    pcl::PointCloud<pcl::PointXYZ> merged_cloud;
    
    if (has_static_) {
      merged_cloud += static_cloud_;
    }
    
    if (has_dynamic_) {
      merged_cloud += dynamic_cloud_;
    }
    
    // 发布
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(merged_cloud, output);
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    merged_pub_.publish(output);
    
    if (has_static_ && has_dynamic_) {
      ROS_DEBUG("融合点云: 静态 %zu + 动态 %zu = 总计 %zu", 
               static_cloud_.size(), dynamic_cloud_.size(), merged_cloud.size());
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_merger");
  ros::NodeHandle nh("~");
  
  CloudMerger merger(nh);
  
  ros::spin();
  return 0;
}
