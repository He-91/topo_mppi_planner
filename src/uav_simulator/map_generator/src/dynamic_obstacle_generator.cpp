/**
 * @file dynamic_obstacle_generator.cpp
 * @brief 动态障碍物生成器 - 支持多种运动模式
 * @date 2025-10-29
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ddo_planner/DynamicObstacles.h>
#include <Eigen/Eigen>
#include <random>
#include <vector>

using namespace std;

// 动态障碍物运动模式
enum MotionType {
  LINEAR,           // 直线运动
  CIRCULAR,         // 圆周运动
  PENDULUM,         // 钟摆运动
  RANDOM_WALK,      // 随机游走
  STATIONARY        // 静止（用于对比测试）
};

// 动态障碍物结构
struct DynamicObstacle {
  int id;
  MotionType motion_type;
  Eigen::Vector3d position;      // 当前位置
  Eigen::Vector3d velocity;      // 当前速度
  Eigen::Vector3d start_pos;     // 起始位置（用于某些运动模式）
  double radius;                 // 障碍物半径
  double height;                 // 障碍物高度
  double speed;                  // 运动速度
  double time_offset;            // 时间偏移（用于相位差）
  
  // 圆周运动参数
  Eigen::Vector3d circle_center;
  double circle_radius;
  
  // 钟摆运动参数
  double pendulum_amplitude;
  Eigen::Vector3d pendulum_axis;  // 摆动轴方向
  
  // 随机游走参数
  Eigen::Vector3d random_target;
  double change_target_time;
  
  // CERLAB-inspired: 速度预测增强
  std::vector<Eigen::Vector3d> position_history;  // 历史位置用于速度估计
  Eigen::Vector3d estimated_velocity;             // 估计速度（从历史计算）
  std::vector<Eigen::Vector3d> predicted_trajectory; // 预测轨迹（0.5-2s）
  static const int HISTORY_SIZE = 10;             // 保持10帧历史（~0.33s @ 30Hz）
};

class DynamicObstacleGenerator {
private:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher predicted_path_pub_;  // NEW: 预测轨迹发布器
  ros::Publisher dynamic_obstacles_pub_;  // P0: 结构化动态障碍物数据 (供MPPI规划器消费)
  ros::Timer update_timer_;
  
  vector<DynamicObstacle> obstacles_;
  double resolution_;
  double update_rate_;
  double start_time_;
  double prediction_horizon_;
  double prediction_dt_;
  int prediction_steps_;
  
  // 地图边界
  double map_x_min_, map_x_max_;
  double map_y_min_, map_y_max_;
  double map_z_min_, map_z_max_;
  
  // 随机数生成器
  std::default_random_engine rng_;
  std::uniform_real_distribution<double> uniform_dist_;
  
public:
  DynamicObstacleGenerator(ros::NodeHandle& nh) : nh_(nh), uniform_dist_(-1.0, 1.0) {
    // 获取参数
    nh_.param("dynamic_obstacles/resolution", resolution_, 0.1);
    nh_.param("dynamic_obstacles/update_rate", update_rate_, 20.0);
    nh_.param("dynamic_obstacles/prediction_horizon", prediction_horizon_, 2.0);
    nh_.param("dynamic_obstacles/prediction_dt", prediction_dt_, 0.1);
    prediction_dt_ = std::max(0.02, prediction_dt_);
    prediction_horizon_ = std::max(prediction_dt_, prediction_horizon_);
    prediction_steps_ = std::max(1, static_cast<int>(std::ceil(prediction_horizon_ / prediction_dt_)));
    
    // 地图边界
    double map_x_size, map_y_size, map_z_size;
    nh_.param("map/x_size", map_x_size, 40.0);
    nh_.param("map/y_size", map_y_size, 20.0);
    nh_.param("map/z_size", map_z_size, 5.0);
    
    map_x_min_ = -map_x_size / 2.0;
    map_x_max_ = map_x_size / 2.0;
    map_y_min_ = -map_y_size / 2.0;
    map_y_max_ = map_y_size / 2.0;
    map_z_min_ = 0.0;
    map_z_max_ = map_z_size;
    
    // 发布器
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_obstacles/cloud", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles/markers", 10);
    velocity_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles/velocities", 10);
    predicted_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles/predicted_paths", 10); // NEW
    dynamic_obstacles_pub_ = nh_.advertise<ddo_planner::DynamicObstacles>("/dynamic_obstacles/state", 10); // P0: 结构化数据
    
    // 初始化随机数生成器
    rng_.seed(std::random_device{}());
    
    // 生成动态障碍物
    generateObstacles();
    
    // 启动更新定时器
    start_time_ = ros::Time::now().toSec();
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), 
                                    &DynamicObstacleGenerator::updateCallback, this);
    
    ROS_INFO("========================================");
    ROS_INFO("动态障碍物生成器已启动");
    ROS_INFO("   障碍物数量: %zu", obstacles_.size());
    ROS_INFO("   更新频率: %.1f Hz", update_rate_);
    ROS_INFO("   分辨率: %.2f m", resolution_);
    ROS_INFO("   预测窗口: %.2fs / %.2fs = %d steps",
             prediction_horizon_, prediction_dt_, prediction_steps_);
    ROS_INFO("========================================");
  }
  
  void generateObstacles() {
    int num_linear, num_circular, num_pendulum, num_random;
    nh_.param("dynamic_obstacles/num_linear", num_linear, 6);
    nh_.param("dynamic_obstacles/num_circular", num_circular, 4);
    nh_.param("dynamic_obstacles/num_pendulum", num_pendulum, 4);
    nh_.param("dynamic_obstacles/num_random", num_random, 2);
    
    // 速度缩放因子 (通过 launch 参数覆盖, 默认1.0 = 原始速度)
    double speed_scale;
    nh_.param("dynamic_obstacles/speed_scale", speed_scale, 1.0);
    ROS_INFO("  速度缩放因子: %.2f", speed_scale);
    
    // 布局模式: 0=原始(全地图散布), 1=航线集中(障碍物沿航线区域分布)
    int layout_mode;
    nh_.param("dynamic_obstacles/layout_mode", layout_mode, 0);
    const char* mode_names[] = {"全地图散布", "航线集中"};
    ROS_INFO("  布局模式: %d (%s)", layout_mode, layout_mode <= 1 ? mode_names[layout_mode] : "未知");
    
    int id = 0;
    
    if (layout_mode == 1) {
      // ======= 模式1: 航线集中布局 (Env4) =======
      // 设计思路: 所有障碍物分布在航线必经之路上
      // 航线: 起点(-16,0) → (16,0) → (0,7) → (-16,0)
      // 段1: x轴中间横穿 (y≈0, x从-16到16)
      // 段2: 右上对角线 (16,0)→(0,7)
      // 段3: 左上对角线 (0,7)→(-16,0)
      generateObstaclesOnRoute(id, speed_scale);
    } else {
      // ======= 模式0: 原始全地图散布布局 =======
      generateObstaclesDefault(id, num_linear, num_circular, num_pendulum, num_random, speed_scale);
    }
    
    ROS_INFO("生成动态障碍物:");
    ROS_INFO("  总数: %zu", obstacles_.size());
    for (const auto& obs : obstacles_) {
      const char* type_str = "UNKNOWN";
      switch (obs.motion_type) {
        case LINEAR: type_str = "LINEAR"; break;
        case CIRCULAR: type_str = "CIRCULAR"; break;
        case PENDULUM: type_str = "PENDULUM"; break;
        case RANDOM_WALK: type_str = "RANDOM_WALK"; break;
        case STATIONARY: type_str = "STATIONARY"; break;
      }
      ROS_INFO("  [%d] %s @ (%.1f, %.1f) speed=%.2f", obs.id, type_str, 
               obs.start_pos.x(), obs.start_pos.y(), obs.speed);
    }
  }
  
  /**
   * @brief 创建并初始化一个障碍物的公共字段
   */
  DynamicObstacle makeBaseObs(int& id, MotionType type, double radius, double height, double speed, double time_offset) {
    DynamicObstacle obs;
    obs.id = id++;
    obs.motion_type = type;
    obs.radius = radius;
    obs.height = height;
    obs.speed = speed;
    obs.time_offset = time_offset;
    obs.estimated_velocity = Eigen::Vector3d::Zero();
    obs.position_history.clear();
    obs.predicted_trajectory.clear();
    return obs;
  }
  
  /**
   * @brief 模式1: 障碍物沿航线区域集中分布
   * 航线: (-16,0) → (16,0) → (0,7) → (-16,0)
   * 所有障碍物的运动范围与航线交叉，无人机必须面对
   */
  void generateObstaclesOnRoute(int& id, double speed_scale) {
    // === 段1: x轴横穿区域 (y≈[-3,3], x从-14到14) ===
    // 4个垂直横穿线性障碍物 — 在航线上垂直来回运动
    {
      double x_positions[] = {-10.0, -3.0, 5.0, 12.0};
      for (int i = 0; i < 4; i++) {
        DynamicObstacle obs = makeBaseObs(id, LINEAR, 0.5, 3.8, (0.5 + 0.15 * i) * speed_scale, i * 1.5);
        obs.start_pos = Eigen::Vector3d(x_positions[i], -4.0, 0.0);
        obs.velocity = Eigen::Vector3d(0, obs.speed, 0);  // Y方向往返
        obs.position = obs.start_pos;
        obs.estimated_velocity = obs.velocity;
        obstacles_.push_back(obs);
      }
    }
    
    // 2个水平线性障碍物 — 沿航线方向迎面运动
    {
      // 从右侧向左运动，迎面撞无人机
      DynamicObstacle obs1 = makeBaseObs(id, LINEAR, 0.5, 3.8, 0.8 * speed_scale, 0.0);
      obs1.start_pos = Eigen::Vector3d(8.0, 0.0, 0.0);
      obs1.velocity = Eigen::Vector3d(-obs1.speed, 0, 0);
      obs1.position = obs1.start_pos;
      obs1.estimated_velocity = obs1.velocity;
      obstacles_.push_back(obs1);
      
      // 从左侧向右运动
      DynamicObstacle obs2 = makeBaseObs(id, LINEAR, 0.5, 3.8, 0.6 * speed_scale, 3.0);
      obs2.start_pos = Eigen::Vector3d(-6.0, 1.0, 0.0);
      obs2.velocity = Eigen::Vector3d(obs2.speed, 0, 0);
      obs2.position = obs2.start_pos;
      obs2.estimated_velocity = obs2.velocity;
      obstacles_.push_back(obs2);
    }
    
    // === 段2: 右上对角区域 (16,0)→(0,7) 附近 ===
    // 2个圆周运动 — 圆心在对角航线上
    {
      DynamicObstacle obs1 = makeBaseObs(id, CIRCULAR, 0.45, 3.5, 0.3 * speed_scale, 0.0);
      obs1.circle_center = Eigen::Vector3d(10.0, 2.5, 0.0);
      obs1.circle_radius = 3.0;
      obs1.start_pos = obs1.circle_center;
      obs1.position = obs1.circle_center + Eigen::Vector3d(obs1.circle_radius, 0, 0);
      obstacles_.push_back(obs1);
      
      DynamicObstacle obs2 = makeBaseObs(id, CIRCULAR, 0.45, 3.5, 0.35 * speed_scale, M_PI);
      obs2.circle_center = Eigen::Vector3d(4.0, 5.0, 0.0);
      obs2.circle_radius = 2.5;
      obs2.start_pos = obs2.circle_center;
      obs2.position = obs2.circle_center + Eigen::Vector3d(obs2.circle_radius, 0, 0);
      obstacles_.push_back(obs2);
    }
    
    // 2个钟摆 — 在对角航线上摆动
    {
      DynamicObstacle obs1 = makeBaseObs(id, PENDULUM, 0.45, 3.6, 0.4 * speed_scale, 0.0);
      obs1.start_pos = Eigen::Vector3d(12.0, 1.5, 0.0);
      obs1.pendulum_axis = Eigen::Vector3d(0, 1, 0);  // Y轴摆动，横穿航线
      obs1.pendulum_amplitude = 3.5;
      obs1.position = obs1.start_pos;
      obstacles_.push_back(obs1);
      
      DynamicObstacle obs2 = makeBaseObs(id, PENDULUM, 0.45, 3.6, 0.45 * speed_scale, 2.0);
      obs2.start_pos = Eigen::Vector3d(6.0, 4.0, 0.0);
      obs2.pendulum_axis = Eigen::Vector3d(-0.7, 0.7, 0);  // 对角摆动
      obs2.pendulum_amplitude = 3.0;
      obs2.position = obs2.start_pos;
      obstacles_.push_back(obs2);
    }
    
    // === 段3: 左上对角区域 (0,7)→(-16,0) 附近 ===
    // 2个圆周运动
    {
      DynamicObstacle obs1 = makeBaseObs(id, CIRCULAR, 0.45, 3.5, 0.25 * speed_scale, 0.5);
      obs1.circle_center = Eigen::Vector3d(-5.0, 4.5, 0.0);
      obs1.circle_radius = 3.0;
      obs1.start_pos = obs1.circle_center;
      obs1.position = obs1.circle_center + Eigen::Vector3d(obs1.circle_radius, 0, 0);
      obstacles_.push_back(obs1);
      
      DynamicObstacle obs2 = makeBaseObs(id, CIRCULAR, 0.45, 3.5, 0.3 * speed_scale, M_PI * 0.5);
      obs2.circle_center = Eigen::Vector3d(-11.0, 2.0, 0.0);
      obs2.circle_radius = 2.5;
      obs2.start_pos = obs2.circle_center;
      obs2.position = obs2.circle_center + Eigen::Vector3d(obs2.circle_radius, 0, 0);
      obstacles_.push_back(obs2);
    }
    
    // 2个钟摆 — 在回程航线上
    {
      DynamicObstacle obs1 = makeBaseObs(id, PENDULUM, 0.45, 3.6, 0.35 * speed_scale, 1.0);
      obs1.start_pos = Eigen::Vector3d(-8.0, 3.0, 0.0);
      obs1.pendulum_axis = Eigen::Vector3d(0.7, 0.7, 0);
      obs1.pendulum_amplitude = 3.5;
      obs1.position = obs1.start_pos;
      obstacles_.push_back(obs1);
      
      DynamicObstacle obs2 = makeBaseObs(id, PENDULUM, 0.45, 3.6, 0.5 * speed_scale, 0.0);
      obs2.start_pos = Eigen::Vector3d(-13.0, 1.0, 0.0);
      obs2.pendulum_axis = Eigen::Vector3d(0, 1, 0);
      obs2.pendulum_amplitude = 3.0;
      obs2.position = obs2.start_pos;
      obstacles_.push_back(obs2);
    }
    
    // === 中心区域: 随机游走 ===
    // 4个随机游走 — 在地图中心活动，覆盖多段航线交叉区域
    {
      double start_positions[][2] = {{-2.0, 1.0}, {3.0, -1.0}, {-7.0, 2.0}, {8.0, 3.0}};
      for (int i = 0; i < 4; i++) {
        DynamicObstacle obs = makeBaseObs(id, RANDOM_WALK, 0.5, 3.7, 0.5 * speed_scale, 0.0);
        obs.start_pos = Eigen::Vector3d(start_positions[i][0], start_positions[i][1], 0.0);
        obs.position = obs.start_pos;
        obs.random_target = generateRandomTarget();
        obs.change_target_time = 0;
        obstacles_.push_back(obs);
      }
    }
  }
  
  /**
   * @brief 模式0: 原始全地图散布布局 (向后兼容)
   */
  void generateObstaclesDefault(int& id, int num_linear, int num_circular, int num_pendulum, int num_random, double speed_scale) {
    double map_width = map_x_max_ - map_x_min_;
    double map_height = map_y_max_ - map_y_min_;
    
    // 1. 直线运动障碍物 - 分布在地图不同区域
    for (int i = 0; i < num_linear; i++) {
      DynamicObstacle obs;
      obs.id = id++;
      obs.motion_type = LINEAR;
      obs.radius = 0.5;
      obs.height = 3.8;  // 高度提升: 2.5 -> 3.8m
      obs.speed = (0.3 + 0.05 * i) * speed_scale;  // 速度: 0.3-0.55 m/s × scale
      obs.time_offset = i * 2.0;  // 时间偏移
      
      // 均匀分布在地图不同区域
      if (i == 0) {
        // 左上区域 - 水平向右运动
        obs.start_pos = Eigen::Vector3d(map_x_min_ + 3, 
                                        map_y_max_ - 5, 
                                        0.0);
        obs.velocity = Eigen::Vector3d(obs.speed, 0, 0);
      } else if (i == 1) {
        // 中心区域 - 垂直向上运动
        obs.start_pos = Eigen::Vector3d((map_x_min_ + map_x_max_) * 0.5, 
                                        map_y_min_ + 3, 
                                        0.0);
        obs.velocity = Eigen::Vector3d(0, obs.speed, 0);
      } else {
        // 右下区域 - 对角线运动
        obs.start_pos = Eigen::Vector3d(map_x_max_ - 5, 
                                        map_y_min_ + 3, 
                                        0.0);
        obs.velocity = Eigen::Vector3d(-obs.speed * 0.7, obs.speed * 0.7, 0);
      }
      obs.position = obs.start_pos;
      // NEW: 初始化速度预测相关字段
      obs.estimated_velocity = obs.velocity;
      obs.position_history.clear();
      obs.predicted_trajectory.clear();
      
      obstacles_.push_back(obs);
    }
    
    // 2. 圆周运动障碍物 - 分散在地图左右两侧
    for (int i = 0; i < num_circular; i++) {
      DynamicObstacle obs;
      obs.id = id++;
      obs.motion_type = CIRCULAR;
      obs.radius = 0.4;
      obs.height = 3.5;  // 高度提升: 2.0 -> 3.5m
      obs.speed = (0.15 + 0.05 * i) * speed_scale;  // 角速度 × scale
      obs.time_offset = i * M_PI;  // 相位差
      
      // 圆心位置 - 左右分布
      if (i == 0) {
        // 左侧区域
        obs.circle_center = Eigen::Vector3d(map_x_min_ + map_width * 0.25, 
                                           map_y_min_ + map_height * 0.5, 
                                           0.0);
      } else {
        // 右侧区域
        obs.circle_center = Eigen::Vector3d(map_x_min_ + map_width * 0.75, 
                                           map_y_min_ + map_height * 0.5, 
                                           0.0);
      }
      obs.circle_radius = 3.0 + i * 0.5;
      obs.start_pos = obs.circle_center;
      obs.position = obs.circle_center + Eigen::Vector3d(obs.circle_radius, 0, 0);
      // NEW: 初始化速度预测相关字段
      obs.estimated_velocity = Eigen::Vector3d::Zero();
      obs.position_history.clear();
      obs.predicted_trajectory.clear();
      
      obstacles_.push_back(obs);
    }
    
    // 3. 钟摆运动障碍物 - 分布在地图上下区域
    for (int i = 0; i < num_pendulum; i++) {
      DynamicObstacle obs;
      obs.id = id++;
      obs.motion_type = PENDULUM;
      obs.radius = 0.45;
      obs.height = 3.6;  // 高度提升: 2.2 -> 3.6m
      obs.speed = (0.25 + 0.05 * i) * speed_scale;  // 摆动频率 × scale
      obs.time_offset = i * 1.5;
      
      if (i == 0) {
        // 上部区域 - X轴摆动
        obs.start_pos = Eigen::Vector3d((map_x_min_ + map_x_max_) * 0.5, 
                                        map_y_max_ - 8, 
                                        0.0);
        obs.pendulum_axis = Eigen::Vector3d(1, 0, 0);
      } else {
        // 下部区域 - Y轴摆动
        obs.start_pos = Eigen::Vector3d(map_x_min_ + map_width * 0.65, 
                                        map_y_min_ + 8, 
                                        0.0);
        obs.pendulum_axis = Eigen::Vector3d(0, 1, 0);
      }
      obs.pendulum_amplitude = 4.0;
      obs.position = obs.start_pos;
      // NEW: 初始化速度预测相关字段
      obs.estimated_velocity = Eigen::Vector3d::Zero();
      obs.position_history.clear();
      obs.predicted_trajectory.clear();
      
      obstacles_.push_back(obs);
    }
    
    // 4. 随机游走障碍物
    for (int i = 0; i < num_random; i++) {
      DynamicObstacle obs;
      obs.id = id++;
      obs.motion_type = RANDOM_WALK;
      obs.radius = 0.5;
      obs.height = 3.7;  // 高度提升: 2.3 -> 3.7m
      obs.speed = 0.35 * speed_scale;  // 速度 × scale
      obs.time_offset = 0;
      
      obs.start_pos = Eigen::Vector3d(
        (map_x_min_ + map_x_max_) / 2.0,
        (map_y_min_ + map_y_max_) / 2.0,
        0.0
      );
      obs.position = obs.start_pos;
      obs.random_target = generateRandomTarget();
      obs.change_target_time = 0;
      // NEW: 初始化速度预测相关字段
      obs.estimated_velocity = Eigen::Vector3d::Zero();
      obs.position_history.clear();
      obs.predicted_trajectory.clear();
      
      obstacles_.push_back(obs);
    }
  }  // end generateObstaclesDefault
  
  Eigen::Vector3d generateRandomTarget() {
    return Eigen::Vector3d(
      map_x_min_ + (map_x_max_ - map_x_min_) * (uniform_dist_(rng_) + 1.0) / 2.0,
      map_y_min_ + (map_y_max_ - map_y_min_) * (uniform_dist_(rng_) + 1.0) / 2.0,
      1.5
    );
  }
  
  void updateCallback(const ros::TimerEvent& event) {
    double current_time = ros::Time::now().toSec() - start_time_;
    
    // 更新每个障碍物的位置
    for (auto& obs : obstacles_) {
      updateObstaclePosition(obs, current_time);
      // NEW: 更新位置历史和估计速度
      updatePositionHistory(obs);
      estimateVelocity(obs);
      predictTrajectory(obs);
    }
    
    // 发布点云
    publishPointCloud();
    
    // 发布可视化标记
    publishMarkers();
    
    // 发布速度向量
    publishVelocities();
    
    // NEW: 发布预测轨迹
    publishPredictedPaths();
    
    // P0: 发布结构化动态障碍物数据 (供MPPI规划器消费)
    publishDynamicObstacleState();
  }
  
  void updateObstaclePosition(DynamicObstacle& obs, double t) {
    t += obs.time_offset;
    
    switch (obs.motion_type) {
      case LINEAR: {
        // 往返直线运动
        Eigen::Vector3d direction = obs.velocity.normalized();
        double distance = obs.speed * t;
        
        // 计算边界
        double max_dist;
        if (fabs(direction.x()) > 0.5) {
          max_dist = map_x_max_ - obs.start_pos.x() - obs.radius - 1.0;
        } else {
          max_dist = map_y_max_ - obs.start_pos.y() - obs.radius - 1.0;
        }
        
        // 往返运动
        double period = 2.0 * max_dist / obs.speed;
        double phase = fmod(distance, period);
        if (phase > max_dist) {
          phase = 2.0 * max_dist - phase;
        }
        
        obs.position = obs.start_pos + direction * phase;
        obs.velocity = direction * obs.speed * (phase < max_dist ? 1.0 : -1.0);
        break;
      }
      
      case CIRCULAR: {
        // 圆周运动
        double angle = obs.speed * t;
        obs.position = obs.circle_center + Eigen::Vector3d(
          obs.circle_radius * cos(angle),
          obs.circle_radius * sin(angle),
          0
        );
        obs.velocity = Eigen::Vector3d(
          -obs.circle_radius * obs.speed * sin(angle),
          obs.circle_radius * obs.speed * cos(angle),
          0
        );
        break;
      }
      
      case PENDULUM: {
        // 钟摆运动
        double offset = obs.pendulum_amplitude * sin(obs.speed * t);
        obs.position = obs.start_pos + obs.pendulum_axis * offset;
        obs.velocity = obs.pendulum_axis * (obs.pendulum_amplitude * obs.speed * cos(obs.speed * t));
        break;
      }
      
      case RANDOM_WALK: {
        // 随机游走
        Eigen::Vector3d direction = (obs.random_target - obs.position);
        double dist = direction.norm();
        
        if (dist < 0.5 || t > obs.change_target_time + 5.0) {
          obs.random_target = generateRandomTarget();
          obs.change_target_time = t;
        }
        
        if (dist > 0.1) {
          direction.normalize();
          obs.velocity = direction * obs.speed;
          obs.position += obs.velocity * (1.0 / update_rate_);
        }
        break;
      }
      
      case STATIONARY:
        obs.velocity.setZero();
        break;
    }
    
    // 边界检查
    obs.position.x() = std::max(map_x_min_ + obs.radius, std::min(map_x_max_ - obs.radius, obs.position.x()));
    obs.position.y() = std::max(map_y_min_ + obs.radius, std::min(map_y_max_ - obs.radius, obs.position.y()));
    // 保持z=0不变，让障碍物中心在地面，一半在地上一半在地下
    obs.position.z() = 0.0;
  }
  
  void publishPointCloud() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    for (const auto& obs : obstacles_) {
      // 生成圆柱形障碍物点云
      int num_theta = ceil(2.0 * M_PI * obs.radius / resolution_);
      int num_height = ceil(obs.height / resolution_);
      
      for (int h = 0; h < num_height; h++) {
        double z = h * resolution_ - obs.height / 2.0;  // 从-height/2开始，中心在position.z
        for (int theta_idx = 0; theta_idx < num_theta; theta_idx++) {
          double theta = 2.0 * M_PI * theta_idx / num_theta;
          
          // 圆柱表面
          pcl::PointXYZ pt;
          pt.x = obs.position.x() + obs.radius * cos(theta);
          pt.y = obs.position.y() + obs.radius * sin(theta);
          pt.z = obs.position.z() + z;
          cloud.points.push_back(pt);
          
          // 填充内部（密集点云）
          int num_r = ceil(obs.radius / resolution_);
          for (int r_idx = 0; r_idx < num_r; r_idx++) {
            double r = r_idx * resolution_;
            pcl::PointXYZ pt_inner;
            pt_inner.x = obs.position.x() + r * cos(theta);
            pt_inner.y = obs.position.y() + r * sin(theta);
            pt_inner.z = obs.position.z() + z;
            cloud.points.push_back(pt_inner);
          }
        }
      }
    }
    
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pub_.publish(cloud_msg);
  }
  
  void publishMarkers() {
    visualization_msgs::MarkerArray marker_array;
    
    for (const auto& obs : obstacles_) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "dynamic_obstacles";
      marker.id = obs.id;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      
      marker.pose.position.x = obs.position.x();
      marker.pose.position.y = obs.position.y();
      marker.pose.position.z = obs.position.z();  // 直接使用position.z，不再加height/2
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = obs.radius * 2.0;
      marker.scale.y = obs.radius * 2.0;
      marker.scale.z = obs.height;
      
      // 根据运动类型设置颜色
      switch (obs.motion_type) {
        case LINEAR:
          marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
          break;
        case CIRCULAR:
          marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
          break;
        case PENDULUM:
          marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
          break;
        case RANDOM_WALK:
          marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
          break;
        default:
          marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5;
      }
      marker.color.a = 0.7;
      marker.lifetime = ros::Duration(0.2);
      
      marker_array.markers.push_back(marker);
    }
    
    marker_pub_.publish(marker_array);
  }
  
  void publishVelocities() {
    visualization_msgs::MarkerArray vel_array;
    
    for (const auto& obs : obstacles_) {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "obstacle_velocities";
      arrow.id = obs.id;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      
      geometry_msgs::Point start, end;
      start.x = obs.position.x();
      start.y = obs.position.y();
      start.z = obs.position.z() + obs.height / 2.0;  // 箭头从障碍物顶部开始
      
      end.x = start.x + obs.velocity.x();
      end.y = start.y + obs.velocity.y();
      end.z = start.z + obs.velocity.z();
      
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      
      arrow.scale.x = 0.1;  // 箭头轴直径
      arrow.scale.y = 0.2;  // 箭头头部直径
      
      arrow.color.r = 1.0;
      arrow.color.g = 1.0;
      arrow.color.g = 1.0;
      arrow.color.a = 1.0;
      arrow.lifetime = ros::Duration(0.2);
      
      vel_array.markers.push_back(arrow);
    }
    
    velocity_pub_.publish(vel_array);
  }
  
  // NEW: CERLAB-inspired velocity prediction functions
  
  /**
   * @brief 更新障碍物的位置历史
   * @param obs 动态障碍物引用
   */
  void updatePositionHistory(DynamicObstacle& obs) {
    obs.position_history.push_back(obs.position);
    // 保持历史大小在HISTORY_SIZE以内
    if (obs.position_history.size() > obs.HISTORY_SIZE) {
      obs.position_history.erase(obs.position_history.begin());
    }
  }
  
  /**
   * @brief 从位置历史估计速度（类似CERLAB的DODT算法）
   * @param obs 动态障碍物引用
   */
  void estimateVelocity(DynamicObstacle& obs) {
    if (obs.position_history.size() < 2) {
      obs.estimated_velocity = obs.velocity;  // 使用真实速度作为初始值
      return;
    }
    
    // 使用最近N帧的线性回归来估计速度（减少噪声）
    int window_size = std::min(5, (int)obs.position_history.size());
    Eigen::Vector3d velocity_sum = Eigen::Vector3d::Zero();
    int count = 0;
    
    for (size_t i = obs.position_history.size() - window_size; i < obs.position_history.size() - 1; ++i) {
      Eigen::Vector3d vel = (obs.position_history[i + 1] - obs.position_history[i]) * update_rate_;
      velocity_sum += vel;
      count++;
    }
    
    if (count > 0) {
      obs.estimated_velocity = velocity_sum / count;
    } else {
      obs.estimated_velocity = obs.velocity;
    }
  }
  
  /**
   * @brief 预测障碍物未来轨迹
   * @param obs 动态障碍物引用
   */
  void predictTrajectory(DynamicObstacle& obs) {
    obs.predicted_trajectory.clear();

    Eigen::Vector3d predicted_pos = obs.position;
    Eigen::Vector3d predicted_vel = obs.estimated_velocity;
    
    // 根据运动类型选择预测模型
    for (int step = 0; step < prediction_steps_; ++step) {
      double future_time = step * prediction_dt_;
      
      switch (obs.motion_type) {
        case LINEAR:
        case RANDOM_WALK: {
          // 恒速模型（Constant Velocity Model）
          predicted_pos = obs.position + predicted_vel * future_time;
          break;
        }
        case CIRCULAR: {
          // 圆周运动预测（使用角速度）
          double angular_vel = obs.speed / obs.circle_radius;
          double current_angle = atan2(obs.position.y() - obs.circle_center.y(),
                                      obs.position.x() - obs.circle_center.x());
          double future_angle = current_angle + angular_vel * future_time;
          
          predicted_pos.x() = obs.circle_center.x() + obs.circle_radius * cos(future_angle);
          predicted_pos.y() = obs.circle_center.y() + obs.circle_radius * sin(future_angle);
          predicted_pos.z() = obs.circle_center.z();
          break;
        }
        case PENDULUM: {
          // 简化的周期运动预测
          double t = (ros::Time::now().toSec() - start_time_ + obs.time_offset + future_time);
          predicted_pos = obs.start_pos + obs.pendulum_axis * (obs.pendulum_amplitude * sin(obs.speed * t));
          break;
        }
        case STATIONARY: {
          predicted_pos = obs.position;
          break;
        }
      }
      
      obs.predicted_trajectory.push_back(predicted_pos);
    }
  }
  
  /**
   * @brief P0: 发布结构化动态障碍物状态 (供MPPI规划器消费)
   * 包含每个障碍物的位置、速度、半径和预测轨迹
   */
  void publishDynamicObstacleState() {
    ddo_planner::DynamicObstacles msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.prediction_horizon = prediction_horizon_;
    msg.prediction_dt = prediction_dt_;
    msg.prediction_steps = prediction_steps_;
    
    for (const auto& obs : obstacles_) {
      ddo_planner::DynamicObstacle obs_msg;
      obs_msg.id = obs.id;
      
      // Current position
      obs_msg.position.x = obs.position.x();
      obs_msg.position.y = obs.position.y();
      obs_msg.position.z = obs.position.z();
      
      // Estimated velocity
      obs_msg.velocity.x = obs.estimated_velocity.x();
      obs_msg.velocity.y = obs.estimated_velocity.y();
      obs_msg.velocity.z = obs.estimated_velocity.z();
      
      // Geometry
      obs_msg.radius = obs.radius;
      obs_msg.height = obs.height;
      
      // Predicted positions (already computed in predictTrajectory())
      for (const auto& pred_pos : obs.predicted_trajectory) {
        geometry_msgs::Point p;
        p.x = pred_pos.x();
        p.y = pred_pos.y();
        p.z = pred_pos.z();
        obs_msg.predicted_positions.push_back(p);
      }
      
      msg.obstacles.push_back(obs_msg);
    }
    
    dynamic_obstacles_pub_.publish(msg);
  }
  
  /**
   * @brief 发布预测轨迹可视化
   */
  void publishPredictedPaths() {
    visualization_msgs::MarkerArray path_array;
    
    for (const auto& obs : obstacles_) {
      if (obs.predicted_trajectory.empty()) continue;
      
      // 为每个障碍物创建轨迹线
      visualization_msgs::Marker trajectory_line;
      trajectory_line.header.frame_id = "world";
      trajectory_line.header.stamp = ros::Time::now();
      trajectory_line.ns = "predicted_trajectories";
      trajectory_line.id = obs.id;
      trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
      trajectory_line.action = visualization_msgs::Marker::ADD;
      trajectory_line.pose.orientation.w = 1.0;
      
      trajectory_line.scale.x = 0.05;  // 线宽
      
      // 颜色：从绿色（近）到红色（远）渐变
      for (size_t i = 0; i < obs.predicted_trajectory.size(); ++i) {
        geometry_msgs::Point p;
        p.x = obs.predicted_trajectory[i].x();
        p.y = obs.predicted_trajectory[i].y();
        p.z = obs.predicted_trajectory[i].z() + obs.height / 2.0;
        trajectory_line.points.push_back(p);
        
        std_msgs::ColorRGBA color;
        float ratio = (float)i / obs.predicted_trajectory.size();
        color.r = ratio;           // 红色分量随时间增加
        color.g = 1.0 - ratio;     // 绿色分量随时间减少
        color.b = 0.3;
        color.a = 0.7;
        trajectory_line.colors.push_back(color);
      }
      
      trajectory_line.lifetime = ros::Duration(0.2);
      path_array.markers.push_back(trajectory_line);
    }
    
    predicted_path_pub_.publish(path_array);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_obstacle_generator");
  ros::NodeHandle nh("~");
  
  DynamicObstacleGenerator generator(nh);
  
  ros::spin();
  return 0;
}
