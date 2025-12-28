#include "path_searching/topo_prm.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <chrono>    // 🚀 P0: 添加时间测量支持
#include <thread>    // 🔧 NEW: parallel shortcut需要
#include <sstream>   // 🎨 可视化: 字符串格式化
#include <iomanip>   // 🎨 可视化: 数字格式化

using namespace std;
using namespace Eigen;

namespace ego_planner {

TopoPRM::TopoPRM() 
    : step_size_(0.2), search_radius_(5.0), max_sample_num_(1000), 
      collision_check_resolution_(0.2),  
      max_raw_paths_(300),              // 🔧 Fast-Planner: 300条原始路径
      reserve_num_(6),                  // 保留6条最短路径 (与FP一致)
      clearance_(0.3),                  // 🔧 Fast-Planner: 0.3m
      sample_inflate_x_(1.0),           // 🔧 NEW: X轴采样范围 (Fast-Planner)
      sample_inflate_y_(3.5),           // 🔧 NEW: Y轴采样范围 (Fast-Planner)
      sample_inflate_z_(1.0),           // 🔧 NEW: Z轴采样范围 (Fast-Planner)
      ratio_to_short_(5.5),             // 🔧 Fast-Planner: 5.5倍最短路径
      discretize_points_num_(25),       // 拓扑去重离散化25点
      max_sample_time_(0.005),          // 🔧 NEW: 最大采样时间5ms
      short_cut_num_(5),                // 🔧 NEW: shortcut迭代次数
      parallel_shortcut_(false),        // 🔧 NEW: 默认不并行处理
      eng_(rd_()),                       // 🔧 NEW: 随机数引擎
      rand_pos_(-1.0, 1.0) {            // 🔧 NEW: [-1,1]均匀分布
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
    short_paths_.clear();  // 🔧 NEW: 清空shortcut路径
}

void TopoPRM::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    topo_paths_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_paths", 10);
    // 🔧 /topo_paths_smooth由planner_manager发布,避免循环依赖
    
    // Get frame_id from node parameter, default to "world" if not set
    nh.param("grid_map/frame_id", frame_id_, std::string("world"));
    
    // 🔧 Fast-Planner: 初始化随机数生成器
    eng_ = std::default_random_engine(rd_());
    rand_pos_ = std::uniform_real_distribution<double>(-1.0, 1.0);
    
    // 🔧 NEW: 读取shortcut参数
    nh.param("topo_prm/short_cut_num", short_cut_num_, 5);
    nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, false);
    
    ROS_INFO("[TopoPRM] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ROS_INFO("[TopoPRM] 🚀 FAST-PLANNER TOPO-PRM CONFIG:");
    ROS_INFO("[TopoPRM]   📊 采样: X=%.1fm, Y=%.1fm, Z=%.1fm (各向异性)", 
             sample_inflate_x_, sample_inflate_y_, sample_inflate_z_);
    ROS_INFO("[TopoPRM]   🕸️  图构建: clearance=%.2fm", clearance_);
    ROS_INFO("[TopoPRM]   🔍 搜索: max_raw=%d, ratio=%.1f", max_raw_paths_, ratio_to_short_);
    ROS_INFO("[TopoPRM]   ✂️  Shortcut: iter=%d, parallel=%s", short_cut_num_, parallel_shortcut_ ? "YES" : "NO");
    ROS_INFO("[TopoPRM]   🎯 保留路径: %d条", reserve_num_);
    ROS_INFO("[TopoPRM]   🎨 可视化: /topo_paths (原始折线)");
    ROS_INFO("[TopoPRM]                 /topo_paths_smooth (B-spline,由plan_manage发布)");
    ROS_INFO("[TopoPRM] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

bool TopoPRM::searchTopoPaths(const Vector3d& start, const Vector3d& goal,
                             vector<TopoPath>& topo_paths) {
    topo_paths.clear();
    clearGraph();
    
    ROS_INFO("[TopoPRM] 🚀 Fast-Planner PRM: [%.2f,%.2f,%.2f] → [%.2f,%.2f,%.2f]", 
             start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());
    
    // 🔧 STEP 1: 设置采样区域 (Fast-Planner矩形盒子)
    sample_r_(0) = 0.5 * (goal - start).norm() + sample_inflate_x_;
    sample_r_(1) = sample_inflate_y_;
    sample_r_(2) = sample_inflate_z_;
    
    translation_ = 0.5 * (start + goal);
    
    // 坐标变换矩阵
    Vector3d xtf, ytf, ztf, downward(0, 0, -1);
    xtf = (goal - translation_).normalized();
    ytf = xtf.cross(downward).normalized();
    ztf = xtf.cross(ytf);
    
    rotation_.col(0) = xtf;
    rotation_.col(1) = ytf;
    rotation_.col(2) = ztf;
    
    ROS_INFO("[TopoPRM] 采样区域: X=%.2fm, Y=%.2fm, Z=%.2fm", 
             sample_r_(0), sample_r_(1), sample_r_(2));
    
    // 🔧 STEP 2: 初始化start/goal为Guard节点 (Fast-Planner)
    GraphNode* start_node = new GraphNode(start, 0);
    GraphNode* goal_node = new GraphNode(goal, 1);
    graph_nodes_.push_back(start_node);
    graph_nodes_.push_back(goal_node);
    
    int node_id = 1;
    int sample_num = 0;
    double sample_time = 0.0;
    ros::Time t1, t2;
    
    // 🔧 STEP 3: Fast-Planner采样循环 (Guard/Connector机制)
    while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
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
        if (dist <= clearance_) {
            sample_time += (ros::Time::now() - t1).toSec();
            continue;
        }
        
        // 🔧 Fast-Planner核心: 找可见的Guard节点
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
    
    ROS_INFO("[TopoPRM] 采样完成: %d个采样, %zu个节点, 耗时%.3fs", 
             sample_num, graph_nodes_.size(), sample_time);
    
    if (graph_nodes_.size() < 3) {
        ROS_WARN("[TopoPRM] 节点太少: %zu < 3", graph_nodes_.size());
        clearGraph();
        return false;
    }
    
    // 🔧 STEP 4: 剪枝孤立节点 (Fast-Planner)
    pruneGraph();
    
    if (graph_nodes_.size() < 3) {
        ROS_WARN("[TopoPRM] 剪枝后节点太少: %zu < 3", graph_nodes_.size());
        clearGraph();
        return false;
    }
    
    // 🔧 STEP 4: DFS搜索多条路径
    raw_paths_.clear();
    vector<GraphNode*> visited;
    visited.push_back(start_node);
    depthFirstSearch(visited, goal_node);
    
    ROS_INFO("[TopoPRM] DFS搜索: 找到%zu条原始路径", raw_paths_.size());
    
    if (raw_paths_.empty()) {
        ROS_WARN("[TopoPRM] 未找到路径");
        clearGraph();
        return false;
    }
    
    // 🔧 STEP 5: Shortcut路径优化 (Fast-Planner)
    auto t_shortcut = std::chrono::high_resolution_clock::now();
    shortcutPaths();
    auto dt_shortcut = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - t_shortcut).count();
    ROS_INFO("[TopoPRM] Shortcut优化: %zu条路径, 耗时%ldms", short_paths_.size(), dt_shortcut);
    
    // 🔧 STEP 6: 拓扑去重 (使用shortcut后的路径)
    vector<vector<Vector3d>> unique_paths = pruneEquivalentPaths(short_paths_);
    ROS_INFO("[TopoPRM] 拓扑去重: %zu → %zu", short_paths_.size(), unique_paths.size());
    
    // 🔧 STEP 7: 选择最短路径
    vector<vector<Vector3d>> selected_paths = selectShortPaths(unique_paths);
    ROS_INFO("[TopoPRM] 最终选择: %zu条路径", selected_paths.size());
    
    // 转换为TopoPath
    for (size_t i = 0; i < selected_paths.size(); ++i) {
        double cost = calculatePathCost(selected_paths[i]);
        topo_paths.emplace_back(selected_paths[i], cost, i);
    }
    
    // 可视化
    visualizeTopoPaths(topo_paths);
    
    clearGraph();
    return !topo_paths.empty();
}

// ============================================================================
// 🔧 LEGACY TOPOLOGICAL PLANNING CODE - COMMENTED OUT FOR TESTING
// ============================================================================
// This entire section (findTopoPaths and 4 path generators) is disabled
// during TGK system validation. Will be permanently removed after testing.
// Backup: topo_prm.cpp.backup_before_legacy_removal
// ============================================================================


// ============================================================================
// 🚀 FAST-PLANNER PRM IMPLEMENTATION (Week 1-4)
// ============================================================================

// ============================================================================
// Week 1: 椭球自由空间采样
// ============================================================================
// 🔧 REMOVED: 椭球采样函数 (已改用Fast-Planner的矩形盒子采样)
// sampleFreeSpaceInEllipsoid() 和 sampleBoundaryLayer() 已废弃
// ============================================================================

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
    
    ROS_INFO("[TopoPRM] 剪枝: 删除%d个孤立节点, 剩余%zu个节点", pruned, graph_nodes_.size());
}

// ============================================================================
// DFS多路径搜索
// ============================================================================
// DFS多路径搜索 (Fast-Planner风格)
// ============================================================================
vector<vector<Vector3d>> TopoPRM::searchMultiplePaths(GraphNode* start_node, 
                                                      GraphNode* goal_node) {
    raw_paths_.clear();
    
    vector<GraphNode*> visited;
    visited.push_back(start_node);
    
    depthFirstSearch(visited, goal_node);
    
    ROS_INFO("[TopoPRM] DFS搜索: 找到%zu条原始路径", raw_paths_.size());
    
    return raw_paths_;
}

void TopoPRM::depthFirstSearch(vector<GraphNode*>& visited, GraphNode* goal_node) {
    // � Fast-Planner版本: 简洁高效
    GraphNode* current = visited.back();
    
    // 🔧 第一步: 检查当前节点的邻居中是否有goal
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
    
    // 🔧 第二步: 递归搜索非goal的邻居
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
        
        if (raw_paths_.size() >= (size_t)max_raw_paths_) {
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
    
    ROS_DEBUG("[TopoPRM] 拓扑去重: %zu → %zu 路径", paths.size(), unique_paths.size());
    
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
    // 🔧 Fast-Planner版本
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
    
    // 🔧 Fast-Planner策略: 迭代选择最短路径
    vector<vector<Vector3d>> short_paths;
    vector<vector<Vector3d>> remaining_paths = paths;  // 拷贝一份用于修改
    double min_len = 0.0;
    
    for (int i = 0; i < reserve_num_ && !remaining_paths.empty(); ++i) {
        int path_id = shortestPathIndex(remaining_paths);
        
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
    
    ROS_INFO("[TopoPRM] 路径选择: %zu → %zu (min_len=%.2f, ratio=%.1f)", 
             paths.size(), short_paths.size(), min_len, ratio_to_short_);
    
    return short_paths;
}

int TopoPRM::shortestPathIndex(const vector<vector<Vector3d>>& paths) {
    // 🔧 Fast-Planner版本
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
    double obstacle_cost = 0.0;
    
    for (const auto& point : path) {
        // Check distance to nearest obstacle
        double min_dist = std::numeric_limits<double>::max();
        
        // Sample around the point to find nearest obstacle
        for (double dx = -search_radius_; dx <= search_radius_; dx += step_size_) {
            for (double dy = -search_radius_; dy <= search_radius_; dy += step_size_) {
                for (double dz = -search_radius_; dz <= search_radius_; dz += step_size_) {
                    Vector3d sample = point + Vector3d(dx, dy, dz);
                    if (grid_map_->getInflateOccupancy(sample)) {
                        double dist = Vector3d(dx, dy, dz).norm();
                        min_dist = std::min(min_dist, dist);
                    }
                }
            }
        }
        
        if (min_dist < search_radius_) {
            obstacle_cost += 1.0 / (min_dist + 0.1);
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
// 🎨 Visualization Functions (Fast-Planner style)
// ============================================================================

// 🔧 Fast-Planner: 渐变色生成 (HSV → RGB)
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

// 🔧 Fast-Planner: 拓扑路径可视化 (Phase2 = selected paths)
void TopoPRM::visualizeTopoPaths(const vector<TopoPath>& paths) {
    if (paths.empty()) {
        ROS_WARN("[TopoPRM] No paths to visualize");
        return;
    }
    
    ROS_INFO("[TopoPRM] 🎨 Visualizing %zu topo paths (Fast-Planner style)", paths.size());
    
    visualization_msgs::MarkerArray marker_array;
    
    // 清除旧的marker
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // 🔧 Fast-Planner: 使用LINE_LIST + 渐变色可视化每条路径
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
        
        // 🔧 Fast-Planner: 渐变色
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
        
        // 🔧 Fast-Planner: 路径节点可视化 (球体)
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
    ROS_INFO("[TopoPRM] 🎨 Published %zu topo paths to /topo_paths", paths.size());
}

// 🔧 B-spline平滑可视化函数已移除,避免path_searching ↔ bspline_opt循环依赖
// /topo_paths_smooth由plan_manage发布 (plan_manage已依赖两者,无循环依赖)

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔧 NEW: Fast-Planner Shortcut Path Optimization
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

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
        double dist;  // 用于getDistanceWithGrad
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
                dist = grid_map_->getDistanceWithGrad(dis_path[i], grad);
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
        for (int i = 0; i < raw_paths_.size(); ++i) {
            short_threads.push_back(std::thread(&TopoPRM::shortcutPath, this, raw_paths_[i], i, 1));
        }
        for (int i = 0; i < raw_paths_.size(); ++i) {
            short_threads[i].join();
        }
    } else {
        for (int i = 0; i < raw_paths_.size(); ++i) {
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

    for (int i = 0; i < path.size() - 1; ++i) {
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

    for (int i = 0; i < paths.size(); ++i) {
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
