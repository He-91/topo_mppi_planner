#include "ddo_path_searching/topo_prm.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <chrono>    //  P0: 添加时间测量支持
#include <thread>    //  NEW: parallel shortcut需要
#include <sstream>   //  可视化: 字符串格式化
#include <iomanip>   //  可视化: 数字格式化
#include <array>

using namespace std;
using namespace Eigen;

namespace ego_planner {

TopoPRM::TopoPRM() 
    : step_size_(0.2), search_radius_(5.0), max_sample_num_(1000), 
      collision_check_resolution_(0.2),  
      max_raw_paths_(300),
      reserve_num_(4),
      clearance_(0.3),
      sample_inflate_x_(1.0),
      sample_inflate_y_(5.0),
      sample_inflate_z_(1.0),
      ratio_to_short_(5.5),
      discretize_points_num_(25),
      max_sample_time_(0.005),
      adaptive_retry_num_(2),
      retry_sample_time_scale_(2.0),
      retry_lateral_scale_(1.35),
      retry_clearance_decay_(0.80),
      enable_deterministic_fallback_(true),
      fallback_vertical_on_empty_(false),
      fallback_vertical_when_partial_(false),
      fallback_center_vertical_when_partial_(false),
      fallback_lateral_candidates_(3),
      fallback_vertical_candidates_(3),
      fallback_vertical_step_(0.55),
      fallback_max_vertical_offset_(1.65),
      grid_seed_enabled_(false),
      grid_seed_resolution_(0.4),
      grid_seed_max_paths_(4),
      grid_seed_lateral_scale_(1.0),
      grid_seed_clearance_(0.16),
      grid_seed_clearance_cost_weight_(0.35),
      grid_seed_reuse_penalty_(8.0),
      clearance_aware_selection_(false),
      selection_smooth_weight_(2.0),
      selection_obstacle_weight_(5.0),
      eng_(rd_()),
      rand_pos_(-1.0, 1.0),
      short_cut_num_(5),
      parallel_shortcut_(false) {
}

TopoPRM::~TopoPRM() {
    clearGraph();
}

void TopoPRM::clearGraph() {
    for (auto node : graph_nodes_) {
        delete node;
    }
    graph_nodes_.clear();
    raw_paths_.clear();
    short_paths_.clear();  //  NEW: 清空shortcut路径
}

void TopoPRM::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    topo_paths_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_paths", 10);
    //  /topo_paths_smooth由planner_manager发布,避免循环依赖
    
    // Get frame_id from node parameter, default to "world" if not set
    nh.param("grid_map/frame_id", frame_id_, std::string("world"));
    
    //  Fast-Planner: 初始化随机数生成器
    eng_ = std::default_random_engine(rd_());
    rand_pos_ = std::uniform_real_distribution<double>(-1.0, 1.0);
    
    //  Read topology parameters
    nh.param("topo_prm/max_topo_paths", reserve_num_, reserve_num_);
    nh.param("topo_prm/short_cut_num", short_cut_num_, 5);
    nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, false);
    nh.param("topo_prm/max_sample_time", max_sample_time_, max_sample_time_);
    nh.param("topo_prm/clearance", clearance_, clearance_);
    nh.param("topo_prm/sample_inflate_x", sample_inflate_x_, sample_inflate_x_);
    nh.param("topo_prm/sample_inflate_y", sample_inflate_y_, sample_inflate_y_);
    nh.param("topo_prm/sample_inflate_z", sample_inflate_z_, sample_inflate_z_);
    nh.param("topo_prm/adaptive_retry_num", adaptive_retry_num_, adaptive_retry_num_);
    nh.param("topo_prm/retry_sample_time_scale", retry_sample_time_scale_, retry_sample_time_scale_);
    nh.param("topo_prm/retry_lateral_scale", retry_lateral_scale_, retry_lateral_scale_);
    nh.param("topo_prm/retry_clearance_decay", retry_clearance_decay_, retry_clearance_decay_);
    nh.param("topo_prm/enable_deterministic_fallback", enable_deterministic_fallback_, enable_deterministic_fallback_);
    nh.param("topo_prm/fallback_vertical_on_empty", fallback_vertical_on_empty_, fallback_vertical_on_empty_);
    nh.param("topo_prm/fallback_vertical_when_partial", fallback_vertical_when_partial_, fallback_vertical_when_partial_);
    nh.param("topo_prm/fallback_center_vertical_when_partial", fallback_center_vertical_when_partial_, fallback_center_vertical_when_partial_);
    nh.param("topo_prm/fallback_lateral_candidates", fallback_lateral_candidates_, fallback_lateral_candidates_);
    nh.param("topo_prm/fallback_vertical_candidates", fallback_vertical_candidates_, fallback_vertical_candidates_);
    nh.param("topo_prm/fallback_vertical_step", fallback_vertical_step_, fallback_vertical_step_);
    nh.param("topo_prm/fallback_max_vertical_offset", fallback_max_vertical_offset_, fallback_max_vertical_offset_);
    nh.param("topo_prm/grid_seed_enabled", grid_seed_enabled_, grid_seed_enabled_);
    nh.param("topo_prm/grid_seed_resolution", grid_seed_resolution_, grid_seed_resolution_);
    nh.param("topo_prm/grid_seed_max_paths", grid_seed_max_paths_, grid_seed_max_paths_);
    nh.param("topo_prm/grid_seed_lateral_scale", grid_seed_lateral_scale_, grid_seed_lateral_scale_);
    nh.param("topo_prm/grid_seed_clearance", grid_seed_clearance_, grid_seed_clearance_);
    nh.param("topo_prm/grid_seed_clearance_cost_weight",
             grid_seed_clearance_cost_weight_,
             grid_seed_clearance_cost_weight_);
    nh.param("topo_prm/grid_seed_reuse_penalty", grid_seed_reuse_penalty_, grid_seed_reuse_penalty_);
    nh.param("topo_prm/clearance_aware_selection", clearance_aware_selection_, clearance_aware_selection_);
    nh.param("topo_prm/selection_smooth_weight", selection_smooth_weight_, selection_smooth_weight_);
    nh.param("topo_prm/selection_obstacle_weight", selection_obstacle_weight_, selection_obstacle_weight_);

    if (reserve_num_ < 1) {
        ROS_WARN("[TopoPRM] topo_prm/max_topo_paths=%d is invalid, using 1", reserve_num_);
        reserve_num_ = 1;
    } else if (reserve_num_ > 10) {
        ROS_WARN("[TopoPRM] topo_prm/max_topo_paths=%d is too high for real-time MPPI, clamping to 10", reserve_num_);
        reserve_num_ = 10;
    }
    adaptive_retry_num_ = std::max(0, std::min(adaptive_retry_num_, 5));
    retry_sample_time_scale_ = std::max(1.0, retry_sample_time_scale_);
    retry_lateral_scale_ = std::max(1.0, retry_lateral_scale_);
    retry_clearance_decay_ = std::min(1.0, std::max(0.2, retry_clearance_decay_));
    fallback_lateral_candidates_ = std::max(1, std::min(fallback_lateral_candidates_, 8));
    fallback_vertical_candidates_ = std::max(0, std::min(fallback_vertical_candidates_, 6));
    fallback_vertical_step_ = std::max(0.1, fallback_vertical_step_);
    fallback_max_vertical_offset_ = std::max(0.0, fallback_max_vertical_offset_);
    grid_seed_resolution_ = std::min(1.0, std::max(0.2, grid_seed_resolution_));
    grid_seed_max_paths_ = std::max(1, std::min(grid_seed_max_paths_, 8));
    grid_seed_lateral_scale_ = std::max(0.5, grid_seed_lateral_scale_);
    grid_seed_clearance_ = std::max(0.05, grid_seed_clearance_);
    grid_seed_clearance_cost_weight_ = std::max(0.0, grid_seed_clearance_cost_weight_);
    grid_seed_reuse_penalty_ = std::max(0.0, grid_seed_reuse_penalty_);
    selection_smooth_weight_ = std::max(0.0, selection_smooth_weight_);
    selection_obstacle_weight_ = std::max(0.0, selection_obstacle_weight_);
    
    ROS_INFO("[TopoPRM] ");
    ROS_INFO("[TopoPRM]  FAST-PLANNER TOPO-PRM CONFIG:");
    ROS_INFO("[TopoPRM]    Sampling: X=%.1fm, Y=%.1fm, Z=%.1fm (anisotropic)", 
             sample_inflate_x_, sample_inflate_y_, sample_inflate_z_);
    ROS_INFO("[TopoPRM]    Graph: clearance=%.2fm", clearance_);
    ROS_INFO("[TopoPRM]    Search: max_raw=%d, ratio=%.1f", max_raw_paths_, ratio_to_short_);
    ROS_INFO("[TopoPRM]    Retry: attempts=%d sample_time_scale=%.2f lateral_scale=%.2f clearance_decay=%.2f fallback=%s",
             adaptive_retry_num_ + 1, retry_sample_time_scale_, retry_lateral_scale_,
             retry_clearance_decay_, enable_deterministic_fallback_ ? "ON" : "OFF");
    ROS_INFO("[TopoPRM]    Fallback: lateral=%d vertical=%d step=%.2f max_dz=%.2f vertical_on_empty=%s vertical_when_partial=%s center_vertical=%s",
             fallback_lateral_candidates_, fallback_vertical_candidates_,
             fallback_vertical_step_, fallback_max_vertical_offset_,
             fallback_vertical_on_empty_ ? "YES" : "NO",
             fallback_vertical_when_partial_ ? "YES" : "NO",
             fallback_center_vertical_when_partial_ ? "YES" : "NO");
    ROS_INFO("[TopoPRM]    Grid corridor seeds: %s res=%.2f max_paths=%d lateral_scale=%.2f clearance=%.2f clear_cost_w=%.2f reuse_penalty=%.1f",
             grid_seed_enabled_ ? "ON" : "OFF",
             grid_seed_resolution_, grid_seed_max_paths_,
             grid_seed_lateral_scale_, grid_seed_clearance_,
             grid_seed_clearance_cost_weight_,
             grid_seed_reuse_penalty_);
    ROS_INFO("[TopoPRM]    Clearance-aware selection: %s smooth_w=%.1f obstacle_w=%.1f",
             clearance_aware_selection_ ? "ON" : "OFF",
             selection_smooth_weight_, selection_obstacle_weight_);
    ROS_INFO("[TopoPRM]    Shortcut: iter=%d, parallel=%s", short_cut_num_, parallel_shortcut_ ? "YES" : "NO");
    ROS_INFO("[TopoPRM]    Reserve paths: %d (topo_prm/max_topo_paths)", reserve_num_);
    ROS_INFO("[TopoPRM]    Visualize: /topo_paths (raw polylines)");
    ROS_INFO("[TopoPRM]               /topo_paths_smooth (B-spline, published by ddo_planner)");
    ROS_INFO("[TopoPRM] ");
}

bool TopoPRM::searchTopoPaths(const Vector3d& start, const Vector3d& goal,
                             vector<TopoPath>& topo_paths) {
    topo_paths.clear();
    ROS_INFO("[TopoPRM]  Fast-Planner PRM: [%.2f,%.2f,%.2f] → [%.2f,%.2f,%.2f]", 
             start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());

    std::vector<std::vector<Vector3d>> accumulated_raw_paths;
    if (grid_seed_enabled_) {
        auto grid_seed_paths = generateGridCorridorSeeds(start, goal);
        if (!grid_seed_paths.empty()) {
            ROS_INFO("[TopoPRM] Grid corridor seed added %zu guide path(s)",
                     grid_seed_paths.size());
            accumulated_raw_paths.insert(accumulated_raw_paths.end(),
                                         grid_seed_paths.begin(), grid_seed_paths.end());
        }
    }

    for (int attempt = 0; attempt <= adaptive_retry_num_; ++attempt) {
        const double sample_time_limit =
            max_sample_time_ * std::pow(retry_sample_time_scale_, attempt);
        const double lateral_scale =
            std::pow(retry_lateral_scale_, attempt);
        const double clearance =
            std::max(0.10, clearance_ * std::pow(retry_clearance_decay_, attempt));

        std::vector<std::vector<Vector3d>> attempt_paths;
        if (runPRMAttempt(start, goal, sample_time_limit, lateral_scale, clearance, attempt_paths)) {
            accumulated_raw_paths.insert(accumulated_raw_paths.end(),
                                         attempt_paths.begin(), attempt_paths.end());
        }

        ROS_INFO("[TopoPRM] Attempt %d/%d: raw_total=%zu",
                 attempt + 1, adaptive_retry_num_ + 1, accumulated_raw_paths.size());
        if (accumulated_raw_paths.size() >= static_cast<size_t>(reserve_num_)) {
            break;
        }
    }

    if (enable_deterministic_fallback_ &&
        accumulated_raw_paths.size() < static_cast<size_t>(reserve_num_)) {
        const bool allow_vertical_escape =
            (accumulated_raw_paths.empty() && fallback_vertical_on_empty_) ||
            fallback_vertical_when_partial_;
        auto fallback_paths = generateDeterministicFallbackPaths(
            start, goal, std::pow(retry_lateral_scale_, adaptive_retry_num_),
            clearance_, allow_vertical_escape);
        if (!fallback_paths.empty()) {
            ROS_INFO("[TopoPRM] Deterministic fallback added %zu guide path(s), vertical_escape=%s",
                     fallback_paths.size(), allow_vertical_escape ? "YES" : "NO");
            accumulated_raw_paths.insert(accumulated_raw_paths.end(),
                                         fallback_paths.begin(), fallback_paths.end());
        }
    }

    if (accumulated_raw_paths.empty()) {
        ROS_WARN("[TopoPRM] No paths found after adaptive PRM and deterministic fallback");
        clearGraph();
        return false;
    }

    raw_paths_ = accumulated_raw_paths;
    processRawPaths(topo_paths);
    clearGraph();
    return !topo_paths.empty();
}

bool TopoPRM::runPRMAttempt(const Vector3d& start, const Vector3d& goal,
                            double sample_time_limit,
                            double lateral_scale,
                            double clearance,
                            vector<vector<Vector3d>>& raw_paths) {
    raw_paths.clear();
    clearGraph();

    const Vector3d start_goal = goal - start;
    const double start_goal_norm = start_goal.norm();
    if (start_goal_norm < 1e-3) {
        return false;
    }

    //  STEP 1: 设置采样区域 (Fast-Planner矩形盒子)
    sample_r_(0) = 0.5 * start_goal_norm + sample_inflate_x_;
    sample_r_(1) = sample_inflate_y_ * lateral_scale;
    sample_r_(2) = sample_inflate_z_;

    translation_ = 0.5 * (start + goal);

    // 坐标变换矩阵. Horizontal fallback avoids NaN when goal direction is near vertical.
    Vector3d xtf = start_goal.normalized();
    Vector3d z_world(0, 0, 1);
    Vector3d ytf = z_world.cross(xtf);
    if (ytf.norm() < 1e-3) {
        ytf = Vector3d(0, 1, 0);
    } else {
        ytf.normalize();
    }
    Vector3d ztf = xtf.cross(ytf).normalized();

    rotation_.col(0) = xtf;
    rotation_.col(1) = ytf;
    rotation_.col(2) = ztf;

    ROS_INFO("[TopoPRM] Sampling region: X=%.2fm, Y=%.2fm, Z=%.2fm, clearance=%.2fm, time=%.1fms",
             sample_r_(0), sample_r_(1), sample_r_(2), clearance, sample_time_limit * 1000.0);

    //  STEP 2: 初始化start/goal为Guard节点 (Fast-Planner)
    GraphNode* start_node = new GraphNode(start, 0);
    GraphNode* goal_node = new GraphNode(goal, 1);
    graph_nodes_.push_back(start_node);
    graph_nodes_.push_back(goal_node);
    
    int node_id = 1;
    int sample_num = 0;
    double sample_time = 0.0;
    ros::Time t1, t2;
    
    //  STEP 3: Fast-Planner采样循环 (Guard/Connector机制)
    while (sample_time < sample_time_limit && sample_num < max_sample_num_) {
        t1 = ros::Time::now();
        
        // 采样点
        Vector3d pt;
        pt(0) = rand_pos_(eng_) * sample_r_(0);
        pt(1) = rand_pos_(eng_) * sample_r_(1);
        pt(2) = rand_pos_(eng_) * sample_r_(2);
        pt = rotation_ * pt + translation_;
        
        ++sample_num;
        
        // 检查clearance
        double dist = grid_map_->getDistance(pt);
        if (dist <= clearance) {
            sample_time += (ros::Time::now() - t1).toSec();
            continue;
        }
        
        //  Fast-Planner核心: 找可见的Guard节点
        vector<GraphNode*> visib_guards = findVisibleGuards(pt);
        
        if (visib_guards.size() == 0) {
            // 看不到任何Guard → 这个点本身成为新Guard
            GraphNode* guard = new GraphNode(pt, ++node_id);
            graph_nodes_.push_back(guard);
            
        } else if (visib_guards.size() == 2) {
            // 看到恰好2个Guard → 检查是否需要新连接
            bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
            if (!need_connect) {
                sample_time += (ros::Time::now() - t1).toSec();
                continue;
            }
            
            // 需要连接 → 创建Connector节点
            GraphNode* connector = new GraphNode(pt, ++node_id);
            graph_nodes_.push_back(connector);
            
            // 双向连接两个Guard
            visib_guards[0]->neighbors.push_back(connector);
            visib_guards[1]->neighbors.push_back(connector);
            connector->neighbors.push_back(visib_guards[0]);
            connector->neighbors.push_back(visib_guards[1]);
        }
        // 如果看到1个或3+个Guard → 跳过(Fast-Planner策略)
        
        sample_time += (ros::Time::now() - t1).toSec();
    }
    
    ROS_INFO("[TopoPRM] Sampling done: %d samples, %zu nodes, time=%.3fs", 
             sample_num, graph_nodes_.size(), sample_time);
    
    if (graph_nodes_.size() < 3) {
        ROS_WARN("[TopoPRM] Too few nodes: %zu < 3", graph_nodes_.size());
        return false;
    }
    
    //  STEP 4: 剪枝孤立节点 (Fast-Planner)
    pruneGraph();
    
    if (graph_nodes_.size() < 3) {
        ROS_WARN("[TopoPRM] Too few nodes after pruning: %zu < 3", graph_nodes_.size());
        return false;
    }
    
    //  STEP 4: DFS搜索多条路径
    raw_paths_.clear();
    dfs_start_time_ = std::chrono::steady_clock::now();
    dfs_timeout_flag_ = false;
    vector<GraphNode*> visited;
    visited.push_back(start_node);
    depthFirstSearch(visited, goal_node);
    
    if (dfs_timeout_flag_) {
        ROS_WARN("[TopoPRM] DFS timeout (50ms), found %zu paths before cutoff", raw_paths_.size());
    } else {
        ROS_INFO("[TopoPRM] DFS search: found %zu raw paths", raw_paths_.size());
    }
    
    if (raw_paths_.empty()) {
        ROS_INFO("[TopoPRM] PRM attempt produced 0 raw paths");
        return false;
    }

    raw_paths = raw_paths_;
    return true;
}

void TopoPRM::processRawPaths(vector<TopoPath>& topo_paths) {
    topo_paths.clear();

    //  STEP 5: Shortcut路径优化 (Fast-Planner)
    auto t_shortcut = std::chrono::high_resolution_clock::now();
    shortcutPaths();
    auto dt_shortcut = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - t_shortcut).count();
    ROS_INFO("[TopoPRM] Shortcut: %zu paths, time=%ldms", short_paths_.size(), dt_shortcut);
    
    //  STEP 6: Topological deduplication (using shortcut paths)
    vector<vector<Vector3d>> unique_paths = pruneEquivalentPaths(short_paths_);
    ROS_INFO("[TopoPRM] Dedup: %zu -> %zu", short_paths_.size(), unique_paths.size());
    
    //  STEP 7: Select shortest paths
    vector<vector<Vector3d>> selected_paths = selectShortPaths(unique_paths);
    ROS_INFO("[TopoPRM] Final selection: %zu paths", selected_paths.size());
    
    // 转换为TopoPath
    for (size_t i = 0; i < selected_paths.size(); ++i) {
        double cost = calculatePathCost(selected_paths[i]);
        topo_paths.emplace_back(selected_paths[i], cost, i);
    }
    
    // 可视化
    visualizeTopoPaths(topo_paths);
}

// ============================================================================
//  LEGACY TOPOLOGICAL PLANNING CODE - COMMENTED OUT FOR TESTING
// ============================================================================
// This entire section (findTopoPaths and 4 path generators) is disabled
// during TGK system validation. Will be permanently removed after testing.
// Backup: topo_prm.cpp.backup_before_legacy_removal
// ============================================================================


// ============================================================================
//  FAST-PLANNER PRM IMPLEMENTATION (Week 1-4)
// ============================================================================

// ============================================================================
// Week 1: 椭球自由空间采样
// ============================================================================
//  REMOVED: 椭球采样函数 (已改用Fast-Planner的矩形盒子采样)
// sampleFreeSpaceInEllipsoid() 和 sampleBoundaryLayer() 已废弃
// ============================================================================

vector<vector<Vector3d>> TopoPRM::generateDeterministicFallbackPaths(
    const Vector3d& start, const Vector3d& goal, double lateral_scale, double clearance,
    bool allow_vertical_escape) {
    vector<vector<Vector3d>> paths;
    const Vector3d start_goal = goal - start;
    const double dist = start_goal.norm();
    if (dist < 1e-3) {
        return paths;
    }

    const Vector3d dir = start_goal.normalized();
    Vector3d horizontal_dir(start_goal.x(), start_goal.y(), 0.0);
    if (horizontal_dir.norm() < 1e-3) {
        horizontal_dir = Vector3d(1, 0, 0);
    } else {
        horizontal_dir.normalize();
    }

    Vector3d lateral = Vector3d(0, 0, 1).cross(horizontal_dir);
    if (lateral.norm() < 1e-3) {
        lateral = Vector3d(0, 1, 0);
    } else {
        lateral.normalize();
    }

    auto tryAppendPath = [&](const vector<Vector3d>& candidate, double min_clearance) -> bool {
        for (const auto& pt : candidate) {
            if (!isPointFree(pt, min_clearance)) {
                return false;
            }
        }
        if (!isPathValid(candidate)) {
            return false;
        }
        paths.push_back(candidate);
        return paths.size() >= static_cast<size_t>(reserve_num_);
    };

    const Vector3d mid = 0.5 * (start + goal);
    const double max_offset = std::max(1.0, sample_inflate_y_ * lateral_scale);
    vector<double> offsets;
    if (allow_vertical_escape && fallback_center_vertical_when_partial_) {
        offsets.push_back(0.0);
    }
    for (int i = 1; i <= fallback_lateral_candidates_; ++i) {
        const double offset = max_offset * static_cast<double>(i) /
                              static_cast<double>(fallback_lateral_candidates_);
        offsets.push_back(offset);
        offsets.push_back(-offset);
    }

    vector<double> vertical_offsets;
    vertical_offsets.push_back(0.0);
    if (allow_vertical_escape) {
        for (int i = 1; i <= fallback_vertical_candidates_; ++i) {
            const double dz = std::min(fallback_max_vertical_offset_,
                                       fallback_vertical_step_ * static_cast<double>(i));
            if (dz > 1e-3 &&
                std::find(vertical_offsets.begin(), vertical_offsets.end(), dz) == vertical_offsets.end()) {
                vertical_offsets.push_back(dz);
            }
        }
    }

    const double min_clearance = std::max(0.08, clearance * 0.75);
    const double near_start_len = std::min(0.8, dist * 0.18);
    const double near_goal_len = std::min(0.8, dist * 0.18);
    const Vector3d near_start = start + dir * near_start_len;
    const Vector3d near_goal = goal - dir * near_goal_len;

    for (double offset : offsets) {
        const Vector3d side = lateral * offset;
        for (double dz : vertical_offsets) {
            const Vector3d up(0.0, 0.0, dz);
            vector<vector<Vector3d>> candidates;
            candidates.push_back({start, mid + side + up, goal});
            candidates.push_back({start,
                                  start + dir * (0.33 * dist) + side + up,
                                  start + dir * (0.66 * dist) + side + up,
                                  goal});

            if (dz > 1e-3) {
                candidates.push_back({start,
                                      near_start + up,
                                      mid + side + up,
                                      near_goal + up,
                                      goal});
                candidates.push_back({start,
                                      start + dir * (0.25 * dist) + side + up,
                                      start + dir * (0.50 * dist) + side + up,
                                      start + dir * (0.75 * dist) + side + up,
                                      goal});
            }

            for (const auto& candidate : candidates) {
                if (tryAppendPath(candidate, min_clearance)) {
                    return paths;
                }
            }
        }
    }

    return paths;
}

vector<vector<Vector3d>> TopoPRM::generateGridCorridorSeeds(
    const Vector3d& start, const Vector3d& goal) {
    vector<vector<Vector3d>> paths;
    if (!grid_map_) {
        return paths;
    }

    const Vector3d start_goal = goal - start;
    const double dist = start_goal.norm();
    if (dist < 1e-3) {
        return paths;
    }

    Vector3d x_axis = start_goal.normalized();
    Vector3d z_world(0, 0, 1);
    Vector3d y_axis = z_world.cross(x_axis);
    if (y_axis.norm() < 1e-3) {
        y_axis = Vector3d(0, 1, 0);
    } else {
        y_axis.normalize();
    }

    const double res = grid_seed_resolution_;
    const double x_margin = std::max(1.0, sample_inflate_x_);
    const double y_extent = std::max(1.0, sample_inflate_y_ * grid_seed_lateral_scale_);
    const int nx = std::max(3, static_cast<int>(std::ceil((dist + 2.0 * x_margin) / res)) + 1);
    const int ny = std::max(3, static_cast<int>(std::ceil((2.0 * y_extent) / res)) + 1);
    const double x_min = -x_margin;
    const double y_min = -y_extent;

    auto index = [ny](int ix, int iy) { return ix * ny + iy; };
    auto inGrid = [nx, ny](int ix, int iy) {
        return ix >= 0 && ix < nx && iy >= 0 && iy < ny;
    };
    auto gridToWorld = [&](int ix, int iy) {
        const double x = x_min + static_cast<double>(ix) * res;
        const double y = y_min + static_cast<double>(iy) * res;
        const double alpha = std::min(1.0, std::max(0.0, x / std::max(1e-3, dist)));
        const double z = start.z() * (1.0 - alpha) + goal.z() * alpha;
        return start + x_axis * x + y_axis * y + Vector3d(0.0, 0.0, z - start.z());
    };

    auto nearestCell = [&](const Vector3d& p) {
        const Vector3d rel = p - start;
        const double x = rel.dot(x_axis);
        const double y = rel.dot(y_axis);
        int ix = static_cast<int>(std::round((x - x_min) / res));
        int iy = static_cast<int>(std::round((y - y_min) / res));
        ix = std::min(nx - 1, std::max(0, ix));
        iy = std::min(ny - 1, std::max(0, iy));
        return std::pair<int, int>(ix, iy);
    };

    std::vector<float> clearance(static_cast<size_t>(nx * ny), -1.0f);
    std::vector<unsigned char> free_cell(static_cast<size_t>(nx * ny), 0);
    for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
            const Vector3d p = gridToWorld(ix, iy);
            const int id = index(ix, iy);
            if (!grid_map_->isInMap(p) || grid_map_->getInflateOccupancy(p)) {
                continue;
            }
            const double d = grid_map_->getDistance(p);
            if (d <= grid_seed_clearance_) {
                continue;
            }
            clearance[id] = static_cast<float>(std::min(5.0, std::max(0.0, d)));
            free_cell[id] = 1;
        }
    }

    const auto start_cell = nearestCell(start);
    const auto goal_cell = nearestCell(goal);
    const int sx = start_cell.first;
    const int sy = start_cell.second;
    const int gx = goal_cell.first;
    const int gy = goal_cell.second;
    free_cell[index(sx, sy)] = 1;
    free_cell[index(gx, gy)] = 1;
    clearance[index(sx, sy)] = std::max(clearance[index(sx, sy)], static_cast<float>(grid_seed_clearance_ + 0.1));
    clearance[index(gx, gy)] = std::max(clearance[index(gx, gy)], static_cast<float>(grid_seed_clearance_ + 0.1));

    std::unordered_set<int> used_cells;
    const int max_paths = std::min(grid_seed_max_paths_, reserve_num_);
    const std::array<std::pair<int, int>, 8> dirs = {{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    }};

    auto compressPath = [&](const vector<int>& cell_path) {
        vector<Vector3d> raw;
        raw.reserve(cell_path.size() + 2);
        raw.push_back(start);
        for (size_t i = 1; i + 1 < cell_path.size(); ++i) {
            const int id = cell_path[i];
            const int ix = id / ny;
            const int iy = id % ny;
            const Vector3d p = gridToWorld(ix, iy);
            if ((p - raw.back()).norm() > 0.2) {
                raw.push_back(p);
            }
        }
        raw.push_back(goal);

        vector<Vector3d> simplified;
        simplified.reserve(raw.size());
        size_t anchor = 0;
        simplified.push_back(raw.front());
        while (anchor + 1 < raw.size()) {
            size_t best = anchor + 1;
            for (size_t j = raw.size() - 1; j > anchor + 1; --j) {
                if (isLineCollisionFree(raw[anchor], raw[j])) {
                    best = j;
                    break;
                }
            }
            if ((raw[best] - simplified.back()).norm() > 0.1) {
                simplified.push_back(raw[best]);
            }
            anchor = best;
        }
        return simplified;
    };

    for (int path_iter = 0; path_iter < max_paths; ++path_iter) {
        struct Node {
            int id;
            double f;
            double g;
            bool operator<(const Node& other) const { return f > other.f; }
        };

        const int start_id = index(sx, sy);
        const int goal_id = index(gx, gy);
        std::priority_queue<Node> open;
        std::vector<double> g_score(static_cast<size_t>(nx * ny), std::numeric_limits<double>::infinity());
        std::vector<int> parent(static_cast<size_t>(nx * ny), -1);
        std::vector<unsigned char> closed(static_cast<size_t>(nx * ny), 0);

        auto heuristic = [&](int id) {
            const int ix = id / ny;
            const int iy = id % ny;
            const double dx = static_cast<double>(ix - gx) * res;
            const double dy = static_cast<double>(iy - gy) * res;
            return std::hypot(dx, dy);
        };

        g_score[start_id] = 0.0;
        open.push({start_id, heuristic(start_id), 0.0});

        bool found = false;
        while (!open.empty()) {
            const Node cur = open.top();
            open.pop();
            if (closed[cur.id]) {
                continue;
            }
            closed[cur.id] = 1;
            if (cur.id == goal_id) {
                found = true;
                break;
            }

            const int ix = cur.id / ny;
            const int iy = cur.id % ny;
            for (const auto& dxy : dirs) {
                const int nx_i = ix + dxy.first;
                const int ny_i = iy + dxy.second;
                if (!inGrid(nx_i, ny_i)) {
                    continue;
                }
                const int nid = index(nx_i, ny_i);
                if (!free_cell[nid] || closed[nid]) {
                    continue;
                }
                const double step = (dxy.first != 0 && dxy.second != 0) ? res * std::sqrt(2.0) : res;
                const double c = std::max(0.05, static_cast<double>(clearance[nid]));
                const double clearance_cost = grid_seed_clearance_cost_weight_ / c;
                const double reuse_cost = used_cells.count(nid) ? grid_seed_reuse_penalty_ : 0.0;
                const double tentative = cur.g + step + clearance_cost + reuse_cost;
                if (tentative < g_score[nid]) {
                    g_score[nid] = tentative;
                    parent[nid] = cur.id;
                    open.push({nid, tentative + heuristic(nid), tentative});
                }
            }
        }

        if (!found) {
            break;
        }

        vector<int> cell_path;
        for (int id = goal_id; id >= 0; id = parent[id]) {
            cell_path.push_back(id);
            if (id == start_id) {
                break;
            }
        }
        if (cell_path.empty() || cell_path.back() != start_id) {
            break;
        }
        std::reverse(cell_path.begin(), cell_path.end());

        for (int id : cell_path) {
            used_cells.insert(id);
        }

        vector<Vector3d> candidate = compressPath(cell_path);
        if (candidate.size() >= 2 && isPathValid(candidate)) {
            bool duplicate = false;
            for (const auto& existing : paths) {
                if (sameTopoPath(candidate, existing)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                paths.push_back(candidate);
            }
        }
    }

    ROS_INFO("[TopoPRM] Grid corridor seed search: grid=%dx%d, paths=%zu",
             nx, ny, paths.size());
    return paths;
}

bool TopoPRM::isPathValid(const vector<Vector3d>& path) {
    if (path.size() < 2) {
        return false;
    }
    for (size_t i = 1; i < path.size(); ++i) {
        if (!isLineCollisionFree(path[i - 1], path[i])) {
            return false;
        }
    }
    return true;
}

bool TopoPRM::isPointFree(const Vector3d& pt, double min_clearance) {
    // 检查是否在地图范围内
    if (!grid_map_->isInMap(pt)) {
        return false;
    }
    
    // 检查距离障碍物的距离
    double dist = grid_map_->getDistance(pt);
    return dist > min_clearance;
}

// ============================================================================
// Fast-Planner辅助函数: 找可见的Guard节点
// ============================================================================
vector<GraphNode*> TopoPRM::findVisibleGuards(const Vector3d& pt) {
    vector<GraphNode*> visib_guards;
    int visib_num = 0;
    
    // 遍历所有节点,找可见的Guard (只检查id<=某个阈值的节点作为Guard候选)
    // Fast-Planner策略: Guard是start/goal或者之前被标记为Guard的节点
    for (size_t i = 0; i < graph_nodes_.size(); ++i) {
        // 检查可见性
        if (isLineCollisionFree(pt, graph_nodes_[i]->pos)) {
            visib_guards.push_back(graph_nodes_[i]);
            ++visib_num;
            if (visib_num >= 3) break;  // 最多找3个就停止
        }
    }
    
    return visib_guards;
}

// ============================================================================
// Fast-Planner辅助函数: 检查是否需要新连接
// ============================================================================
bool TopoPRM::needConnection(GraphNode* g1, GraphNode* g2, const Vector3d& pt) {
    // 路径1: g1 → pt → g2 (新连接)
    vector<Vector3d> path1(3);
    path1[0] = g1->pos;
    path1[1] = pt;
    path1[2] = g2->pos;
    
    // 检查g1和g2是否已经通过其他Connector连接
    vector<Vector3d> path2(3);
    path2[0] = g1->pos;
    path2[2] = g2->pos;
    
    for (auto nb1 : g1->neighbors) {
        for (auto nb2 : g2->neighbors) {
            if (nb1->id == nb2->id) {
                // 找到公共邻居 → 已经有连接
                path2[1] = nb1->pos;
                
                // 检查拓扑等价性
                bool same_topo = sameTopoPath(path1, path2);
                if (same_topo) {
                    // 如果新路径更短,更新Connector位置
                    if (pathLength(path1) < pathLength(path2)) {
                        nb1->pos = pt;  // Fast-Planner会更新位置!
                    }
                    return false;  // 不需要新连接
                }
            }
        }
    }
    
    return true;  // 需要新连接
}

// ============================================================================
// Fast-Planner: 剪枝孤立节点
// ============================================================================
void TopoPRM::pruneGraph() {
    if (graph_nodes_.size() <= 2) return;
    
    int pruned = 0;
    bool changed = true;
    
    while (changed && graph_nodes_.size() > 2) {
        changed = false;
        
        for (size_t i = 2; i < graph_nodes_.size(); ) {  // 跳过start(0)和goal(1)
            if (graph_nodes_[i]->neighbors.size() <= 1) {
                // 度数<=1的节点需要删除
                GraphNode* to_remove = graph_nodes_[i];
                
                // 从所有邻居中删除这个节点
                for (auto neighbor : to_remove->neighbors) {
                    neighbor->neighbors.erase(
                        std::remove(neighbor->neighbors.begin(), neighbor->neighbors.end(), to_remove),
                        neighbor->neighbors.end()
                    );
                }
                
                // 从图中删除
                delete to_remove;
                graph_nodes_.erase(graph_nodes_.begin() + i);
                pruned++;
                changed = true;
                // 不增加i,因为后面的元素前移了
            } else {
                ++i;
            }
        }
    }
    
    ROS_INFO("[TopoPRM] Pruning: removed %d isolated nodes, remaining %zu", pruned, graph_nodes_.size());
}

// ============================================================================
// DFS多路径搜索
// ============================================================================
// DFS多路径搜索 (Fast-Planner风格)
// ============================================================================
vector<vector<Vector3d>> TopoPRM::searchMultiplePaths(GraphNode* start_node, 
                                                      GraphNode* goal_node) {
    raw_paths_.clear();
    dfs_start_time_ = std::chrono::steady_clock::now();
    dfs_timeout_flag_ = false;
    
    vector<GraphNode*> visited;
    visited.push_back(start_node);
    
    depthFirstSearch(visited, goal_node);
    
    ROS_INFO("[TopoPRM] DFS search: found %zu raw paths", raw_paths_.size());
    
    return raw_paths_;
}

void TopoPRM::depthFirstSearch(vector<GraphNode*>& visited, GraphNode* goal_node) {
    // DFS timeout check: 50ms to prevent combinatorial explosion
    auto elapsed = std::chrono::steady_clock::now() - dfs_start_time_;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > MAX_DFS_TIME_MS) {
        dfs_timeout_flag_ = true;
        return;
    }

    // Fast-Planner版本: 简洁高效
    GraphNode* current = visited.back();
    
    //  第一步: 检查当前节点的邻居中是否有goal
    for (auto neighbor : current->neighbors) {
        if (neighbor->id == goal_node->id) {
            // 找到目标!构造路径
            vector<Vector3d> path;
            for (auto node : visited) {
                path.push_back(node->pos);
            }
            path.push_back(neighbor->pos);
            
            raw_paths_.push_back(path);
            
            if (raw_paths_.size() >= (size_t)max_raw_paths_) {
                return;
            }
            
            break;  // Fast-Planner: 找到goal就break,不继续搜索其他goal邻居
        }
    }
    
    //  第二步: 递归搜索非goal的邻居
    for (auto neighbor : current->neighbors) {
        // 跳过goal节点
        if (neighbor->id == goal_node->id) continue;
        
        // 跳过已访问节点
        bool already_visited = false;
        for (auto v : visited) {
            if (v->id == neighbor->id) {
                already_visited = true;
                break;
            }
        }
        if (already_visited) continue;
        
        // 递归搜索
        visited.push_back(neighbor);
        depthFirstSearch(visited, goal_node);
        
        if (raw_paths_.size() >= (size_t)max_raw_paths_ || dfs_timeout_flag_) {
            return;
        }
        
        visited.pop_back();
    }
}

// ============================================================================
// Week 4: 拓扑等价性去重
// ============================================================================
vector<vector<Vector3d>> TopoPRM::pruneEquivalentPaths(
    const vector<vector<Vector3d>>& paths) {
    
    if (paths.empty()) return paths;
    
    vector<vector<Vector3d>> unique_paths;
    unique_paths.push_back(paths[0]);
    
    for (size_t i = 1; i < paths.size(); ++i) {
        bool is_unique = true;
        
        for (const auto& existing : unique_paths) {
            if (sameTopoPath(paths[i], existing)) {
                is_unique = false;
                break;
            }
        }
        
        if (is_unique) {
            unique_paths.push_back(paths[i]);
        }
    }
    
    ROS_DEBUG("[TopoPRM] Dedup: %zu -> %zu paths", paths.size(), unique_paths.size());
    
    return unique_paths;
}

bool TopoPRM::sameTopoPath(const vector<Vector3d>& path1, 
                           const vector<Vector3d>& path2) {
    // � Fast-Planner方法: 离散化后逐点检查可见性
    // 计算路径长度
    double len1 = pathLength(path1);
    double len2 = pathLength(path2);
    double max_len = std::max(len1, len2);
    
    // 根据分辨率计算离散化点数
    double resolution = grid_map_->getResolution();
    int pt_num = std::ceil(max_len / resolution);
    
    // 离散化两条路径
    vector<Vector3d> pts1 = discretizePath(path1, pt_num);
    vector<Vector3d> pts2 = discretizePath(path2, pt_num);
    
    // 逐点检查对应点之间的可见性
    for (int i = 0; i < pt_num; ++i) {
        if (!isLineCollisionFree(pts1[i], pts2[i])) {
            return false;  // 如果对应点之间有障碍物,说明拓扑不同
        }
    }
    
    return true;  // 所有对应点都可见,说明拓扑相同
}

vector<Vector3d> TopoPRM::discretizePath(const vector<Vector3d>& path, int pt_num) {
    //  Fast-Planner版本
    vector<double> len_list;
    len_list.push_back(0.0);
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double inc_l = (path[i + 1] - path[i]).norm();
        len_list.push_back(inc_l + len_list[i]);
    }
    
    // 沿路径计算pt_num个点
    double len_total = len_list.back();
    double dl = len_total / double(pt_num - 1);
    
    vector<Vector3d> dis_path;
    for (int i = 0; i < pt_num; ++i) {
        double cur_l = double(i) * dl;
        
        // 找到cur_l所在的段
        int idx = -1;
        for (size_t j = 0; j < len_list.size() - 1; ++j) {
            if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
                idx = j;
                break;
            }
        }
        
        if (idx == -1) {
            // 边界情况
            if (cur_l < len_list[0]) idx = 0;
            else idx = len_list.size() - 2;
        }
        
        // 插值
        double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
        Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
        dis_path.push_back(inter_pt);
    }
    
    return dis_path;
}

// ============================================================================
// 辅助函数: 路径选择 (Fast-Planner版本)
// ============================================================================
vector<vector<Vector3d>> TopoPRM::selectShortPaths(
    const vector<vector<Vector3d>>& paths) {
    
    if (paths.empty()) return paths;
    
    // Keep the shortest path as a speed anchor, then optionally prefer safer
    // non-shortest alternatives for the remaining topology modes.
    vector<vector<Vector3d>> short_paths;
    vector<vector<Vector3d>> remaining_paths = paths;  // 拷贝一份用于修改
    double min_len = 0.0;
    auto selectionScore = [&](const vector<Vector3d>& path) {
        return pathLength(path) +
               selection_smooth_weight_ * calculateSmoothnessCost(path) +
               selection_obstacle_weight_ * calculateObstacleCost(path);
    };
    
    for (int i = 0; i < reserve_num_ && !remaining_paths.empty(); ++i) {
        int path_id = shortestPathIndex(remaining_paths);
        if (clearance_aware_selection_ && i > 0 && min_len > 1e-3) {
            path_id = -1;
            double best_score = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < remaining_paths.size(); ++j) {
                const double rat = pathLength(remaining_paths[j]) / min_len;
                if (rat >= ratio_to_short_) {
                    continue;
                }
                const double score = selectionScore(remaining_paths[j]);
                if (score < best_score) {
                    best_score = score;
                    path_id = static_cast<int>(j);
                }
            }
            if (path_id < 0) {
                break;
            }
        }
        
        if (i == 0) {
            // 第一条路径(最短)
            short_paths.push_back(remaining_paths[path_id]);
            min_len = pathLength(remaining_paths[path_id]);
            remaining_paths.erase(remaining_paths.begin() + path_id);
        } else {
            // 后续路径:检查长度比率
            double rat = pathLength(remaining_paths[path_id]) / min_len;
            if (rat < ratio_to_short_) {
                short_paths.push_back(remaining_paths[path_id]);
                remaining_paths.erase(remaining_paths.begin() + path_id);
            } else {
                break;  // 太长了,停止
            }
        }
    }
    
    ROS_INFO("[TopoPRM] Path selection: %zu -> %zu (min_len=%.2f, ratio=%.1f, clearance_aware=%s)",
             paths.size(), short_paths.size(), min_len, ratio_to_short_,
             clearance_aware_selection_ ? "YES" : "NO");
    
    return short_paths;
}

int TopoPRM::shortestPathIndex(const vector<vector<Vector3d>>& paths) {
    //  Fast-Planner版本
    int short_id = -1;
    double min_len = 100000000.0;
    
    for (size_t i = 0; i < paths.size(); ++i) {
        double len = pathLength(paths[i]);
        if (len < min_len) {
            short_id = i;
            min_len = len;
        }
    }
    
    return short_id;
}

double TopoPRM::pathLength(const vector<Vector3d>& path) {
    double length = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        length += (path[i+1] - path[i]).norm();
    }
    return length;
}

// ============================================================================
// Utility functions implementation
// ============================================================================

bool TopoPRM::isLineCollisionFree(const Vector3d& start, const Vector3d& end) {
    Vector3d dir = end - start;
    double dist = dir.norm();
    if (dist < 1e-6) return true;
    
    dir.normalize();
    
    for (double t = 0; t <= dist; t += collision_check_resolution_) {
        Vector3d point = start + t * dir;
        if (grid_map_->getInflateOccupancy(point)) {
            return false;
        }
    }
    return true;
}

double TopoPRM::calculatePathCost(const vector<Vector3d>& path) {
    if (path.size() < 2) return std::numeric_limits<double>::max();
    
    double length_cost = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        length_cost += (path[i + 1] - path[i]).norm();
    }
    
    double smoothness_cost = calculateSmoothnessCost(path);
    double obstacle_cost = calculateObstacleCost(path);
    
    return length_cost + 2.0 * smoothness_cost + 5.0 * obstacle_cost;
}

double TopoPRM::calculateSmoothnessCost(const vector<Vector3d>& path) {
    if (path.size() < 3) return 0.0;
    
    double smoothness_cost = 0.0;
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Vector3d v1 = (path[i] - path[i - 1]).normalized();
        Vector3d v2 = (path[i + 1] - path[i]).normalized();
        double angle = acos(std::max(-1.0, std::min(1.0, v1.dot(v2))));
        smoothness_cost += angle;
    }
    return smoothness_cost;
}

double TopoPRM::calculateObstacleCost(const vector<Vector3d>& path) {
    // BUG #2 FIX: Replace O(N * (2R/step)^3) brute-force search with O(N) ESDF query.
    // Old code used 3 nested loops over [-search_radius_, +search_radius_] with step_size_,
    // which was 51^3 = 132,651 occupancy checks PER path point (search_radius_=5.0, step_size_=0.2).
    // ESDF already stores the exact nearest-obstacle distance — use it directly.
    double obstacle_cost = 0.0;
    
    for (const auto& point : path) {
        double dist = grid_map_->getDistance(point);
        
        // Skip invalid ESDF values (e.g., outside map or uninitialized)
        if (dist < -1000.0) {
            obstacle_cost += 10.0;  // Penalize unknown regions
            continue;
        }
        
        if (dist < clearance_) {
            // Inverse-distance cost, consistent with original intent
            obstacle_cost += 1.0 / (std::max(dist, 0.01) + 0.1);
        }
    }
    
    return obstacle_cost;
}

TopoPath TopoPRM::selectBestPath(const vector<TopoPath>& paths) {
    if (paths.empty()) {
        return TopoPath();
    }
    
    // Return the path with minimum cost
    auto best_it = std::min_element(paths.begin(), paths.end(),
        [](const TopoPath& a, const TopoPath& b) {
            return a.cost < b.cost;
        });
    
    return *best_it;
}

// ============================================================================
//  Visualization Functions (Fast-Planner style)
// ============================================================================

//  Fast-Planner: 渐变色生成 (HSV → RGB)
Eigen::Vector4d getTopoPathColor(double h, double alpha = 1.0) {
    if (h < 0.0 || h > 1.0) {
        h = 0.0;
    }

    double lambda;
    Eigen::Vector4d color1, color2;
    
    if (h >= -1e-4 && h < 1.0 / 6) {
        lambda = (h - 0.0) * 6;
        color1 = Eigen::Vector4d(1, 0, 0, 1);  // Red
        color2 = Eigen::Vector4d(1, 0, 1, 1);  // Magenta
    } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
        lambda = (h - 1.0 / 6) * 6;
        color1 = Eigen::Vector4d(1, 0, 1, 1);  // Magenta
        color2 = Eigen::Vector4d(0, 0, 1, 1);  // Blue
    } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
        lambda = (h - 2.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 0, 1, 1);  // Blue
        color2 = Eigen::Vector4d(0, 1, 1, 1);  // Cyan
    } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
        lambda = (h - 3.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 1, 1, 1);  // Cyan
        color2 = Eigen::Vector4d(0, 1, 0, 1);  // Green
    } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
        lambda = (h - 4.0 / 6) * 6;
        color1 = Eigen::Vector4d(0, 1, 0, 1);  // Green
        color2 = Eigen::Vector4d(1, 1, 0, 1);  // Yellow
    } else if (h >= 5.0 / 6 && h <= 1.0 + 1e-4) {
        lambda = (h - 5.0 / 6) * 6;
        color1 = Eigen::Vector4d(1, 1, 0, 1);  // Yellow
        color2 = Eigen::Vector4d(1, 0, 0, 1);  // Red
    } else {
        lambda = 0.0;
        color1 = Eigen::Vector4d(0, 0, 0, 1);
        color2 = Eigen::Vector4d(0, 0, 0, 1);
    }

    Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
    fcolor(3) = alpha;

    return fcolor;
}

//  Fast-Planner: 拓扑路径可视化 (Phase2 = selected paths)
void TopoPRM::visualizeTopoPaths(const vector<TopoPath>& paths) {
    if (paths.empty()) {
        ROS_WARN("[TopoPRM] No paths to visualize");
        return;
    }
    
    ROS_INFO("[TopoPRM]  Visualizing %zu topo paths (Fast-Planner style)", paths.size());
    
    visualization_msgs::MarkerArray marker_array;
    
    // 清除旧的marker
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    //  Fast-Planner: 使用LINE_LIST + 渐变色可视化每条路径
    for (size_t i = 0; i < paths.size(); ++i) {
        const auto& path = paths[i].path;
        if (path.size() < 2) continue;
        
        // 创建LINE_LIST marker
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = frame_id_;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "topo_paths_phase2";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;  // Fast-Planner uses LINE_LIST
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        
        //  Fast-Planner: 渐变色
        Eigen::Vector4d color = getTopoPathColor(double(i) / paths.size(), 0.8);
        line_marker.color.r = color(0);
        line_marker.color.g = color(1);
        line_marker.color.b = color(2);
        line_marker.color.a = color(3);
        
        line_marker.scale.x = 0.08;  // Fast-Planner line width
        
        // LINE_LIST: 每条线段需要两个点
        for (size_t j = 0; j < path.size() - 1; ++j) {
            geometry_msgs::Point p1, p2;
            p1.x = path[j].x();
            p1.y = path[j].y();
            p1.z = path[j].z();
            
            p2.x = path[j+1].x();
            p2.y = path[j+1].y();
            p2.z = path[j+1].z();
            
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
        }
        
        marker_array.markers.push_back(line_marker);
        
        //  Fast-Planner: 路径节点可视化 (球体)
        visualization_msgs::Marker sphere_marker;
        sphere_marker.header.frame_id = frame_id_;
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.ns = "topo_path_nodes";
        sphere_marker.id = 100 + i;
        sphere_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        sphere_marker.action = visualization_msgs::Marker::ADD;
        sphere_marker.pose.orientation.w = 1.0;
        
        sphere_marker.color.r = color(0);
        sphere_marker.color.g = color(1);
        sphere_marker.color.b = color(2);
        sphere_marker.color.a = 0.6;
        
        sphere_marker.scale.x = 0.15;
        sphere_marker.scale.y = 0.15;
        sphere_marker.scale.z = 0.15;
        
        for (const auto& pt : path) {
            geometry_msgs::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            sphere_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(sphere_marker);
    }
    
    topo_paths_pub_.publish(marker_array);
    ROS_INFO("[TopoPRM]  Published %zu topo paths to /topo_paths", paths.size());
}

//  B-spline平滑可视化函数已移除,避免path_searching ↔ bspline_opt循环依赖
// /topo_paths_smooth由plan_manage发布 (plan_manage已依赖两者,无循环依赖)

// 
//  NEW: Fast-Planner Shortcut Path Optimization
// 

void TopoPRM::shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num) {
    vector<Eigen::Vector3d> short_path = path;
    vector<Eigen::Vector3d> last_path;

    for (int k = 0; k < iter_num; ++k) {
        last_path = short_path;

        vector<Eigen::Vector3d> dis_path = discretizePath(short_path);

        if (dis_path.size() < 2) {
            short_paths_[path_id] = dis_path;
            return;
        }

        /* visibility path shortening */
        Eigen::Vector3d colli_pt, grad, dir, push_dir;
        short_path.clear();
        short_path.push_back(dis_path.front());
        
        for (size_t i = 1; i < dis_path.size(); ++i) {  // 修复类型比较警告
            // 尝试从当前点直接连到dis_path[i]
            if (isLineCollisionFree(short_path.back(), dis_path[i])) {
                continue;  // 可见,跳过中间点
            }

            // 碰撞点处理: 获取EDT梯度,将碰撞点推离障碍物
            // 注意: Fast-Planner使用edt_environment_->evaluateEDTWithGrad()
            // 我们使用GridMap的getDistanceWithGrad()
            if (grid_map_->getInflateOccupancy(dis_path[i])) {
                // 碰撞点,获取梯度
                grid_map_->getDistanceWithGrad(dis_path[i], grad);
                if (grad.norm() > 1e-3) {
                    grad.normalize();
                    dir = (dis_path[i] - short_path.back()).normalized();
                    push_dir = grad - grad.dot(dir) * dir;  // 正交分量
                    push_dir.normalize();
                    colli_pt = dis_path[i] + collision_check_resolution_ * push_dir;
                } else {
                    colli_pt = dis_path[i];
                }
            } else {
                colli_pt = dis_path[i];
            }
            
            short_path.push_back(colli_pt);
        }
        short_path.push_back(dis_path.back());

        /* break if no shortcut */
        double len1 = pathLength(last_path);
        double len2 = pathLength(short_path);
        if (len2 > len1) {
            // 没有缩短,恢复上一次结果
            short_path = last_path;
            break;
        }
    }

    short_paths_[path_id] = short_path;
}

void TopoPRM::shortcutPaths() {
    short_paths_.resize(raw_paths_.size());

    if (parallel_shortcut_) {
        vector<std::thread> short_threads;
        for (size_t i = 0; i < raw_paths_.size(); ++i) {
            short_threads.push_back(std::thread(&TopoPRM::shortcutPath, this, raw_paths_[i], i, 1));
        }
        for (size_t i = 0; i < raw_paths_.size(); ++i) {
            short_threads[i].join();
        }
    } else {
        for (size_t i = 0; i < raw_paths_.size(); ++i) {
            shortcutPath(raw_paths_[i], i, short_cut_num_);
        }
    }
}

vector<Eigen::Vector3d> TopoPRM::discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2) {
    Eigen::Vector3d dir = p2 - p1;
    double len = dir.norm();
    int seg_num = ceil(len / collision_check_resolution_);

    vector<Eigen::Vector3d> line_pts;
    if (seg_num <= 0) {
        return line_pts;
    }

    for (int i = 0; i <= seg_num; ++i) {
        line_pts.push_back(p1 + dir * double(i) / double(seg_num));
    }

    return line_pts;
}

vector<Eigen::Vector3d> TopoPRM::discretizePath(const vector<Eigen::Vector3d>& path) {
    vector<Eigen::Vector3d> dis_path, segment;

    if (path.size() < 2) {
        ROS_ERROR("[TopoPRM] discretizePath: path too short!");
        return dis_path;
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        segment = discretizeLine(path[i], path[i + 1]);

        if (segment.size() < 1) continue;

        dis_path.insert(dis_path.end(), segment.begin(), segment.end());
        if (i != path.size() - 2) {
            dis_path.pop_back();  // 移除重复点
        }
    }
    return dis_path;
}

vector<vector<Eigen::Vector3d>> TopoPRM::discretizePaths(vector<vector<Eigen::Vector3d>>& paths) {
    vector<vector<Eigen::Vector3d>> dis_paths;
    vector<Eigen::Vector3d> dis_path;

    for (size_t i = 0; i < paths.size(); ++i) {
        dis_path = discretizePath(paths[i]);
        if (dis_path.size() > 0) {
            dis_paths.push_back(dis_path);
        }
    }

    return dis_paths;
}

Eigen::Vector3d TopoPRM::getOrthoPoint(const vector<Eigen::Vector3d>& path) {
    Eigen::Vector3d x1 = path.front();
    Eigen::Vector3d x2 = path.back();

    Eigen::Vector3d dir = (x2 - x1).normalized();
    Eigen::Vector3d mid = 0.5 * (x1 + x2);

    // 找到离直线最远的点
    double max_dist = 0.0;
    Eigen::Vector3d ortho_pt = mid;
    
    for (const auto& pt : path) {
        Eigen::Vector3d v = pt - x1;
        double proj_len = v.dot(dir);
        Eigen::Vector3d proj_pt = x1 + proj_len * dir;
        double dist = (pt - proj_pt).norm();
        
        if (dist > max_dist) {
            max_dist = dist;
            ortho_pt = pt;
        }
    }

    return ortho_pt;
}

} // namespace ego_planner
