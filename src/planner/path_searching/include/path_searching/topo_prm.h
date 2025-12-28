#ifndef _TOPO_PRM_H_
#define _TOPO_PRM_H_

#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <vector>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// B-spline可视化移到planner_manager处理,避免循环依赖

namespace ego_planner {

struct TopoPath {
    std::vector<Eigen::Vector3d> path;
    double cost;
    int path_id;
    
    TopoPath() : cost(0.0), path_id(-1) {}
    TopoPath(const std::vector<Eigen::Vector3d>& p, double c, int id) 
        : path(p), cost(c), path_id(id) {}
};

// 🚀 NEW: Graph node for PRM-based topology planning (Fast-Planner style)
struct GraphNode {
    Eigen::Vector3d pos;
    int id;
    std::vector<GraphNode*> neighbors;
    
    GraphNode() : id(-1) {}
    GraphNode(const Eigen::Vector3d& p, int node_id) : pos(p), id(node_id) {}
};

class TopoPRM {
private:
    GridMap::Ptr grid_map_;
    ros::Publisher topo_paths_pub_;           // 拓扑路径折线可视化
    std::string frame_id_;
    
    // Parameters
    double step_size_;
    double search_radius_;
    int max_sample_num_;
    double collision_check_resolution_;
    
    // 🚀 Fast-Planner PRM parameters
    int max_raw_paths_;           // 最大原始路径数 (Fast-Planner: 300)
    int reserve_num_;             // 保留的最短路径数 (Fast-Planner: 6)
    double clearance_;            // 节点最小安全距离 (Fast-Planner: 0.3m)
    double sample_inflate_x_;     // 🔧 NEW: X轴采样膨胀 (Fast-Planner: 1.0m)
    double sample_inflate_y_;     // 🔧 NEW: Y轴采样膨胀 (Fast-Planner: 3.5m)
    double sample_inflate_z_;     // 🔧 NEW: Z轴采样膨胀 (Fast-Planner: 1.0m)
    double ratio_to_short_;       // 相对最短路径的长度比率阈值 (Fast-Planner: 5.5)
    int discretize_points_num_;   // 拓扑去重时的离散化点数
    double max_sample_time_;      // 🔧 NEW: 最大采样时间 (Fast-Planner: 0.005s)
    
    // 🔧 NEW: Fast-Planner采样所需变量
    std::random_device rd_;
    std::default_random_engine eng_;
    std::uniform_real_distribution<double> rand_pos_;
    Eigen::Vector3d sample_r_;       // 采样范围 (半轴长度)
    Eigen::Vector3d translation_;    // 矩形盒子中心
    Eigen::Matrix3d rotation_;       // 坐标变换矩阵
    
    // 🚀 NEW: PRM graph data structures
    std::vector<GraphNode*> graph_nodes_;
    std::vector<std::vector<Eigen::Vector3d>> raw_paths_;
    std::vector<std::vector<Eigen::Vector3d>> short_paths_;  // 🔧 NEW: shortcut优化后的路径
    
    // 🔧 NEW: DFS timeout control (Fast-Planner)
    std::chrono::steady_clock::time_point dfs_start_time_;
    bool dfs_timeout_flag_;
    const double MAX_DFS_TIME_MS = 50.0;  // 50ms timeout
    
    // 🔧 NEW: Shortcut parameters (Fast-Planner)
    int short_cut_num_;        // shortcut迭代次数 (Fast-Planner: 5)
    bool parallel_shortcut_;   // 是否并行处理shortcut (Fast-Planner: false)
    
    // Shared utility functions
    bool isPathValid(const std::vector<Eigen::Vector3d>& path);
    bool isLineCollisionFree(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    bool isPointFree(const Eigen::Vector3d& pt, double min_clearance);
    
    // 🔧 Fast-Planner: Guard/Connector机制
    std::vector<GraphNode*> findVisibleGuards(const Eigen::Vector3d& pt);
    bool needConnection(GraphNode* g1, GraphNode* g2, const Eigen::Vector3d& pt);
    void pruneGraph();
    void clearGraph();
    
    // 🔧 Fast-Planner: DFS搜索
    std::vector<std::vector<Eigen::Vector3d>> searchMultiplePaths(GraphNode* start_node,
                                                                   GraphNode* goal_node);
    void depthFirstSearch(std::vector<GraphNode*>& visited, GraphNode* goal_node);
    
    // 🔧 Fast-Planner: 拓扑去重
    bool sameTopoPath(const std::vector<Eigen::Vector3d>& path1,
                      const std::vector<Eigen::Vector3d>& path2);
    std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num);
    std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path);  // 🔧 NEW: 重载版本
    std::vector<std::vector<Eigen::Vector3d>> discretizePaths(std::vector<std::vector<Eigen::Vector3d>>& paths);
    std::vector<std::vector<Eigen::Vector3d>> pruneEquivalentPaths(
        const std::vector<std::vector<Eigen::Vector3d>>& paths);
    
    // 🔧 NEW: Fast-Planner shortcut优化
    void shortcutPaths();
    void shortcutPath(std::vector<Eigen::Vector3d> path, int path_id, int iter_num = 1);
    std::vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);
    Eigen::Vector3d getOrthoPoint(const std::vector<Eigen::Vector3d>& path);
    
    // 辅助函数
    int shortestPathIndex(const std::vector<std::vector<Eigen::Vector3d>>& paths);
    double pathLength(const std::vector<Eigen::Vector3d>& path);
    std::vector<std::vector<Eigen::Vector3d>> selectShortPaths(
        const std::vector<std::vector<Eigen::Vector3d>>& paths);
    
    // Legacy functions removed (2025-11-12) - no longer needed after parameter optimization
    // Removed: findTopoPathsLegacy, generateAlternativePath, generateCircularPath, 
    //          generateVerticalPath, generateTangentPoints, estimateObstacleSize
    
    // Cost calculation
    double calculatePathCost(const std::vector<Eigen::Vector3d>& path);
    double calculateSmoothnessCost(const std::vector<Eigen::Vector3d>& path);
    double calculateObstacleCost(const std::vector<Eigen::Vector3d>& path);
    
    // Visualization
    void visualizeTopoPaths(const std::vector<TopoPath>& paths);
    // B-spline平滑可视化移到planner_manager,避免循环依赖
    void publishPath(const std::vector<Eigen::Vector3d>& path, int id, 
                    double r, double g, double b, double scale = 0.1);

public:
    typedef std::shared_ptr<TopoPRM> Ptr;
    
    TopoPRM();
    ~TopoPRM();
    
    void init(ros::NodeHandle& nh, GridMap::Ptr grid_map);
    
    // Main interface
    bool searchTopoPaths(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                        std::vector<TopoPath>& topo_paths);
    
    TopoPath selectBestPath(const std::vector<TopoPath>& paths);
    
    // Parameters
    void setStepSize(double step_size) { step_size_ = step_size; }
    void setSearchRadius(double radius) { search_radius_ = radius; }
    void setMaxSampleNum(int num) { max_sample_num_ = num; }
};

} // namespace ego_planner

#endif