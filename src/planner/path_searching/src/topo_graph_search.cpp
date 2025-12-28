#include "path_searching/topo_graph_search.h"
#include <algorithm>
#include <cmath>
#include <map>

using namespace std;
using namespace Eigen;

namespace ego_planner {

TopoGraphSearch::TopoGraphSearch()
    : max_search_nodes_(1000),
      connection_radius_(20.0),  // 🎯 CRITICAL FIX: 增加到 20m 匹配 TGK 动态半径
      path_pruning_threshold_(0.5),
      max_topo_paths_(5),
      node_counter_(0) {
}

TopoGraphSearch::~TopoGraphSearch() {
}

void TopoGraphSearch::init(GridMap::Ptr grid_map, BiasSampler::Ptr bias_sampler) {
    grid_map_ = grid_map;
    bias_sampler_ = bias_sampler;
    
    ROS_INFO("[TopoGraphSearch] Initialized");
}

bool TopoGraphSearch::searchTopoPaths(const Vector3d& start,
                                     const Vector3d& goal,
                                     vector<vector<Vector3d>>& paths) {
    paths.clear();
    
    ROS_INFO("[TopoGraphSearch] Searching multiple topological paths");
    
    // Build search graph
    if (!buildSearchGraph(start, goal)) {
        ROS_WARN("[TopoGraphSearch] Failed to build search graph");
        return false;
    }
    
    // Extract multiple topologically distinct paths
    extractMultiplePaths(start, goal, paths);
    
    if (paths.empty()) {
        ROS_WARN("[TopoGraphSearch] No valid paths found");
        return false;
    }
    
    ROS_INFO("[TopoGraphSearch] Found %zu topological paths", paths.size());
    return true;
}

bool TopoGraphSearch::searchSinglePath(const Vector3d& start,
                                      const Vector3d& goal,
                                      vector<Vector3d>& path) {
    path.clear();
    
    // Build search graph
    if (!buildSearchGraph(start, goal)) {
        ROS_WARN("[TopoGraphSearch] Failed to build search graph");
        return false;
    }
    
    // Run A* search
    if (!astarSearch(start, goal, path)) {
        ROS_WARN("[TopoGraphSearch] A* search failed");
        return false;
    }
    
    // Smooth path
    smoothPath(path);
    
    ROS_INFO("[TopoGraphSearch] Found path with %zu waypoints", path.size());
    return true;
}

bool TopoGraphSearch::buildSearchGraph(const Vector3d& start, const Vector3d& goal) {
    node_pool_.clear();
    node_counter_ = 0;
    
    // Get topological key points from bias sampler
    vector<Vector3d> key_points = bias_sampler_->getTopoKeyPoints(start, goal);
    
    ROS_INFO("[TopoGraphSearch] Building graph with %zu key points", key_points.size());
    
    // If no key points found and direct path is free, just connect start-goal
    if (key_points.empty() && isPathFree(start, goal)) {
        ROS_INFO("[TopoGraphSearch] Direct path is free, using simple connection");
        // Still build graph with just start and goal for consistency
    }
    
    // Add start node
    TopoNode start_node;
    start_node.pos = start;
    start_node.node_id = node_counter_++;
    start_node.topo_class = 0;
    node_pool_.push_back(start_node);
    
    // Add key point nodes
    for (const auto& pt : key_points) {
        TopoNode node;
        node.pos = pt;
        node.node_id = node_counter_++;
        node.topo_class = -1;  // Will be set by sampler if needed
        node_pool_.push_back(node);
    }
    
    // Add goal node
    TopoNode goal_node;
    goal_node.pos = goal;
    goal_node.node_id = node_counter_++;
    goal_node.topo_class = 0;
    node_pool_.push_back(goal_node);
    
    // 🚀 BRIDGE NODE ALGORITHM: 修复图不连通问题
    // 目标: TGK成功率 85% → 95%
    // 策略: 检测孤岛节点,在起点-终点连线上插入桥接节点
    
    if (node_pool_.size() >= 2) {
        // 检查起点能否直接看到终点
        if (!isPathFree(start, goal)) {
            ROS_DEBUG("[TopoGraphSearch] Direct path blocked, checking graph connectivity");
            
            // 简单连通性检查: 起点能否通过现有节点到达终点
            // 统计起点和终点的可见节点数量
            int start_connections = 0;
            int goal_connections = 0;
            
            for (size_t i = 1; i < node_pool_.size() - 1; ++i) {
                if (canConnect(start, node_pool_[i].pos)) {
                    start_connections++;
                }
                if (canConnect(node_pool_[i].pos, goal)) {
                    goal_connections++;
                }
            }
            
            ROS_DEBUG("[TopoGraphSearch] Connectivity check: start→%d nodes, goal←%d nodes",
                     start_connections, goal_connections);
            
            // 🔧 改进: 降低桥接节点触发阈值,在连接稀疏时也触发 (< 3个连接)
            if (start_connections < 3 || goal_connections < 3) {
                ROS_WARN("[TopoGraphSearch] 🔧 Graph connectivity issue detected (start:%d, goal:%d), adding bridge nodes",
                        start_connections, goal_connections);
                
                // 在起点-终点连线上尝试插入桥接节点
                Vector3d dir = (goal - start).normalized();
                double dist = (goal - start).norm();
                
                // 尝试在1/4, 1/2, 3/4位置插入桥接节点
                vector<double> bridge_ratios = {0.25, 0.5, 0.75};
                int bridges_added = 0;
                
                for (double ratio : bridge_ratios) {
                    Vector3d bridge_pos = start + ratio * dist * dir;
                    
                    // 检查桥接位置是否无碰撞
                    if (!grid_map_->getInflateOccupancy(bridge_pos)) {
                        // 检查桥接节点是否能改善连通性
                        bool helpful = false;
                        
                        // 桥接节点应该能连接起点或终点
                        if (isPathFree(start, bridge_pos) || isPathFree(bridge_pos, goal)) {
                            helpful = true;
                        }
                        
                        if (helpful) {
                            TopoNode bridge_node;
                            bridge_node.pos = bridge_pos;
                            bridge_node.node_id = node_counter_++;
                            bridge_node.topo_class = -2;  // 标记为桥接节点
                            
                            // 插入到goal节点之前
                            node_pool_.insert(node_pool_.end() - 1, bridge_node);
                            bridges_added++;
                            
                            ROS_INFO("[TopoGraphSearch] ✅ Added bridge node at [%.2f, %.2f, %.2f]",
                                   bridge_pos.x(), bridge_pos.y(), bridge_pos.z());
                        }
                    }
                }
                
                if (bridges_added > 0) {
                    ROS_INFO("[TopoGraphSearch] 🔧 Added %d bridge nodes to improve connectivity",
                            bridges_added);
                }
            }
        }
    }
    
    // 🚀 CORRIDOR ID LABELING: 为每个节点标记所属走廊
    // 目标: 实现拓扑感知的K-shortest paths
    // 策略: 根据节点相对起点-终点连线的侧向偏移量分类
    
    if (node_pool_.size() >= 2) {
        Vector3d sg_dir = (goal - start).normalized();
        Vector3d lateral_dir = Vector3d(-sg_dir.y(), sg_dir.x(), 0.0).normalized();  // 垂直方向
        Vector3d midpoint = (start + goal) / 2.0;
        
        // 🔧 FIX 1: 修复走廊分类 - 不强制起点/终点,使用实际侧向偏移量
        for (size_t i = 0; i < node_pool_.size(); ++i) {
            Vector3d to_node = node_pool_[i].pos - midpoint;
            double lateral_offset = to_node.dot(lateral_dir);
            
            // 🔧 FIX 2: 根据侧向偏移量分配走廊ID (对齐BiasedSampler的±0/±5/±10m)
            // 分类边界: -7.5, -2.5, +2.5, +7.5 (走廊中心±2.5m)
            if (lateral_offset < -7.5) {
                node_pool_[i].corridor_id = -10;
            } else if (lateral_offset < -2.5) {
                node_pool_[i].corridor_id = -5;
            } else if (lateral_offset < 2.5) {
                node_pool_[i].corridor_id = 0;  // 主走廊
            } else if (lateral_offset < 7.5) {
                node_pool_[i].corridor_id = 5;
            } else {
                node_pool_[i].corridor_id = 10;
            }
            
            ROS_DEBUG("[TopoGraphSearch] Node %zu: corridor_id=%d, lateral_offset=%.2f",
                     i, node_pool_[i].corridor_id, lateral_offset);
        }
        
        // 统计每个走廊的节点数
        map<int, int> corridor_counts;
        for (const auto& node : node_pool_) {
            corridor_counts[node.corridor_id]++;
        }
        
        ROS_INFO("[TopoGraphSearch] 🗺️ Corridor distribution (all nodes):");
        for (const auto& pair : corridor_counts) {
            ROS_INFO("  Corridor %+3d: %2d nodes", pair.first, pair.second);
        }
    }
    
    ROS_INFO("[TopoGraphSearch] Graph built with %zu nodes", node_pool_.size());
    return node_pool_.size() >= 2;
}

bool TopoGraphSearch::astarSearch(const Vector3d& start,
                                  const Vector3d& goal,
                                  vector<Vector3d>& path,
                                  vector<int>* path_node_ids) {
    if (node_pool_.size() < 2) {
        ROS_WARN("[TopoGraphSearch] Node pool too small: %zu", node_pool_.size());
        return false;
    }
    
    // Priority queue for A*
    priority_queue<TopoNode, vector<TopoNode>, TopoNodeComparator> open_set;
    
    // Visited set
    vector<bool> closed_set(node_pool_.size(), false);
    vector<TopoNode> node_states = node_pool_;
    
    // Initialize all nodes with infinite cost
    for (size_t i = 0; i < node_states.size(); ++i) {
        node_states[i].g_cost = std::numeric_limits<double>::max();
        node_states[i].parent_id = -1;
    }
    
    // Initialize start node
    int start_id = 0;  // First node is start
    node_states[start_id].g_cost = 0.0;
    node_states[start_id].h_cost = heuristic(node_states[start_id].pos, goal);
    node_states[start_id].parent_id = -1;
    open_set.push(node_states[start_id]);
    
    int goal_id = node_pool_.size() - 1;  // Last node is goal
    
    ROS_INFO("[TopoGraphSearch] A* search: %zu nodes, start_id=%d, goal_id=%d", 
             node_pool_.size(), start_id, goal_id);
    
    int iter = 0;
    int connections_tested = 0;
    while (!open_set.empty() && iter < max_search_nodes_) {
        iter++;
        
        // Get node with lowest f_cost
        TopoNode current = open_set.top();
        open_set.pop();
        
        int current_id = current.node_id;
        
        // Check if already visited
        if (closed_set[current_id]) {
            continue;
        }
        
        closed_set[current_id] = true;
        
        // Check if reached goal
        if (current_id == goal_id) {
            // Reconstruct path
            path.clear();
            int id = goal_id;
            vector<int> path_node_ids_temp;  // 临时记录路径节点ID
            while (id != -1) {
                path.push_back(node_states[id].pos);
                path_node_ids_temp.push_back(id);
                id = node_states[id].parent_id;
            }
            reverse(path.begin(), path.end());
            reverse(path_node_ids_temp.begin(), path_node_ids_temp.end());
            
            // � OPTIMIZATION: 输出节点ID序列
            if (path_node_ids != nullptr) {
                *path_node_ids = path_node_ids_temp;
            }
            
            // �🔧 FIX 9: 验证路径是否经过被阻塞的节点(调试用)
            bool path_uses_blocked = false;
            for (int node_id : path_node_ids_temp) {
                if (node_pool_[node_id].is_blocked) {
                    path_uses_blocked = true;
                    ROS_ERROR("[TopoGraphSearch] ❌ BUG: Path uses blocked node %d (corridor %+d)!",
                              node_id, node_pool_[node_id].corridor_id);
                }
            }
            
            if (!path_uses_blocked) {
                ROS_INFO("[TopoGraphSearch] ✅ A* found valid path in %d iterations (no blocked nodes used)", iter);
            }
            
            return true;
        }
        
        // Expand neighbors
        for (size_t i = 0; i < node_pool_.size(); ++i) {
            if (closed_set[i]) continue;
            if (i == static_cast<size_t>(current_id)) continue;  // Skip self
            
            // 🔧 CRITICAL FIX: 跳过被阻塞的节点 (K-shortest paths)
            // 这样阻塞节点后,A*会自动寻找绕过的路径
            if (node_pool_[i].is_blocked) {
                ROS_DEBUG("[TopoGraphSearch] Skipping blocked node %zu (corridor %+d)", 
                         i, node_pool_[i].corridor_id);
                continue;
            }
            
            // Check if can connect
            connections_tested++;
            if (!canConnect(node_states[current_id].pos, node_pool_[i].pos)) {
                continue;
            }
            
            // Calculate tentative g_cost
            double tentative_g = node_states[current_id].g_cost + 
                                edgeCost(node_states[current_id].pos, node_pool_[i].pos);
            
            // Update if this path is better
            if (tentative_g < node_states[i].g_cost) {
                node_states[i].g_cost = tentative_g;
                node_states[i].h_cost = heuristic(node_states[i].pos, goal);
                node_states[i].parent_id = current_id;
                open_set.push(node_states[i]);
            }
        }
    }
    
    ROS_WARN("[TopoGraphSearch] A* failed to find path after %d iterations, tested %d connections", 
             iter, connections_tested);
    ROS_WARN("[TopoGraphSearch] Graph has %zu nodes, start can see goal: %s",
             node_pool_.size(), 
             isPathFree(node_states[start_id].pos, node_states[goal_id].pos) ? "YES" : "NO");
    return false;
}

void TopoGraphSearch::extractMultiplePaths(const Vector3d& start,
                                          const Vector3d& goal,
                                          vector<vector<Vector3d>>& paths) {
    paths.clear();
    
    // Step 1: Find first path using A*
    vector<Vector3d> first_path;
    vector<int> first_path_node_ids;
    if (!astarSearch(start, goal, first_path, &first_path_node_ids)) {
        ROS_WARN("[TopoGraphSearch] A* failed to find first path!");
        
        // Safety fallback: try direct line
        if (isPathFree(start, goal)) {
            ROS_INFO("[TopoGraphSearch] Using direct line path");
            vector<Vector3d> direct_path = {start, goal};
            paths.push_back(direct_path);
            return;
        }
        
        ROS_ERROR("[TopoGraphSearch] Direct path also blocked!");
        return;
    }
    
    // Success: found first path
    smoothPath(first_path);
    paths.push_back(first_path);
    vector<vector<int>> all_path_node_ids;
    all_path_node_ids.push_back(first_path_node_ids);
    
    ROS_INFO("[TopoGraphSearch] Path 1: %zu waypoints", first_path.size());
    ROS_INFO("[TopoGraphSearch] Path 1 uses %zu nodes", first_path_node_ids.size());
    
    // Step 2: Generate alternative paths by blocking used nodes
    int max_attempts = 10;
    int target_paths = max_topo_paths_;  // 3-5 paths
    
    // 🎯 STEP 2/5: Progressive blocking strategy
    // Instead of blocking ALL nodes at once (causes disconnect),
    // gradually increase blocking to maintain graph connectivity
    
    for (int attempt = 1; attempt < max_attempts && paths.size() < static_cast<size_t>(target_paths); attempt++) {
        // Reset all blocked flags
        for (size_t i = 0; i < node_pool_.size(); i++) {
            node_pool_[i].is_blocked = false;
        }
        
        // 🔧 Progressive blocking: only block most important nodes first
        // Attempt 1: Block 1 node per previous path
        // Attempt 2: Block 2 nodes per previous path
        // Attempt 3+: Block all nodes
        int nodes_to_block_per_path = std::min(attempt, 3);
        
        set<int> blocked_nodes;
        for (const auto& prev_path_nodes : all_path_node_ids) {
            // Extract middle nodes only (skip start/goal)
            vector<int> middle_nodes;
            for (size_t j = 1; j < prev_path_nodes.size() - 1; j++) {
                middle_nodes.push_back(prev_path_nodes[j]);
            }
            
            // Block first N middle nodes (most important for topology)
            int block_count = 0;
            for (int node_id : middle_nodes) {
                if (block_count >= nodes_to_block_per_path) break;
                
                blocked_nodes.insert(node_id);
                node_pool_[node_id].is_blocked = true;
                block_count++;
            }
        }
        
        ROS_INFO("[TopoGraphSearch] Attempt %d: Blocked %zu/%zu nodes (progressive: %d per path)",
                 attempt, blocked_nodes.size(), node_pool_.size() - 2, nodes_to_block_per_path);
        
        // Try A* with blocked nodes
        vector<Vector3d> alt_path;
        vector<int> alt_path_node_ids;
        
        if (!astarSearch(start, goal, alt_path, &alt_path_node_ids)) {
            ROS_WARN("[TopoGraphSearch] Attempt %d: A* failed (graph disconnected)", attempt);
            continue;
        }
        
        // 🎯 STEP 4/5: Use corridor sequence for topological difference check
        // Old method: Compare node IDs (complex, false negatives)
        // New method: Compare corridor sequences (simple, intuitive)
        // 
        // Example:
        //   Path 1: [0]           → Direct line
        //   Path 2: [-5, 0]       → Left detour
        //   Path 3: [+5, 0]       → Right detour
        //   All are different topologies!
        
        bool is_different = true;
        
        // Extract corridor sequence for alternative path
        vector<int> alt_corridor_seq;
        for (int node_id : alt_path_node_ids) {
            if (node_id >= 0 && node_id < static_cast<int>(node_pool_.size())) {
                int corridor_id = node_pool_[node_id].corridor_id;
                // Only add if different from last corridor (compress sequence)
                if (alt_corridor_seq.empty() || alt_corridor_seq.back() != corridor_id) {
                    alt_corridor_seq.push_back(corridor_id);
                }
            }
        }
        
        // Compare with existing paths
        for (size_t i = 0; i < all_path_node_ids.size(); i++) {
            const vector<int>& existing_nodes = all_path_node_ids[i];
            
            // Extract corridor sequence for existing path
            vector<int> existing_corridor_seq;
            for (int node_id : existing_nodes) {
                if (node_id >= 0 && node_id < static_cast<int>(node_pool_.size())) {
                    int corridor_id = node_pool_[node_id].corridor_id;
                    if (existing_corridor_seq.empty() || existing_corridor_seq.back() != corridor_id) {
                        existing_corridor_seq.push_back(corridor_id);
                    }
                }
            }
            
            // Simple comparison: different corridor sequence = different topology
            bool same_sequence = (alt_corridor_seq == existing_corridor_seq);
            
            if (same_sequence) {
                is_different = false;
                ROS_INFO("[TopoGraphSearch] ❌ Rejected: same corridor sequence as path %zu", i+1);
                
                // Debug: print corridor sequences
                string alt_seq_str = "[";
                for (size_t k = 0; k < alt_corridor_seq.size(); k++) {
                    alt_seq_str += std::to_string(alt_corridor_seq[k]);
                    if (k < alt_corridor_seq.size() - 1) alt_seq_str += ", ";
                }
                alt_seq_str += "]";
                
                string existing_seq_str = "[";
                for (size_t k = 0; k < existing_corridor_seq.size(); k++) {
                    existing_seq_str += std::to_string(existing_corridor_seq[k]);
                    if (k < existing_corridor_seq.size() - 1) existing_seq_str += ", ";
                }
                existing_seq_str += "]";
                
                ROS_INFO("[TopoGraphSearch]   Alt corridor: %s", alt_seq_str.c_str());
                ROS_INFO("[TopoGraphSearch]   Path %zu corridor: %s", i+1, existing_seq_str.c_str());
                break;
            }
        }
        
        if (is_different) {
            smoothPath(alt_path);
            paths.push_back(alt_path);
            all_path_node_ids.push_back(alt_path_node_ids);
            
            // Print accepted corridor sequence
            string corridor_str = "[";
            vector<int> accepted_corridor_seq;
            for (int node_id : alt_path_node_ids) {
                if (node_id >= 0 && node_id < static_cast<int>(node_pool_.size())) {
                    int cid = node_pool_[node_id].corridor_id;
                    if (accepted_corridor_seq.empty() || accepted_corridor_seq.back() != cid) {
                        accepted_corridor_seq.push_back(cid);
                    }
                }
            }
            for (size_t k = 0; k < accepted_corridor_seq.size(); k++) {
                corridor_str += std::to_string(accepted_corridor_seq[k]);
                if (k < accepted_corridor_seq.size() - 1) corridor_str += ", ";
            }
            corridor_str += "]";
            
            ROS_INFO("[TopoGraphSearch] ✅ Path %zu accepted: %zu waypoints, %zu nodes, corridor %s",
                     paths.size(), alt_path.size(), alt_path_node_ids.size(), corridor_str.c_str());
        }
    }
    
    ROS_INFO("[TopoGraphSearch] Generated %zu topological paths", paths.size());
}

double TopoGraphSearch::heuristic(const Vector3d& pos, const Vector3d& goal) {
    // Euclidean distance
    return (goal - pos).norm();
}

double TopoGraphSearch::edgeCost(const Vector3d& from, const Vector3d& to) {
    double dist = (to - from).norm();
    
    // Add penalty for proximity to obstacles
    // 🔧 Phase 4: Use getDistanceWithGrad (our ESDF API)
    Vector3d mid = (from + to) / 2.0;
    Vector3d edt_grad;
    double edt_dist = grid_map_->getDistanceWithGrad(mid, edt_grad);
    
    // 🔧 Phase 4.5.1.10: Filter abnormal ESDF values (unobserved regions)
    // ESDF returns 10000.0m for unobserved regions → treat as safe
    if (edt_dist > 100.0) {
        return dist;  // No penalty for unobserved regions (conservative)
    }
    
    // Normal region: penalize proximity to obstacles
    // Use 1.0m threshold (more lenient than 0.5m) for better path diversity
    double obs_penalty = 0.0;
    if (edt_dist < 1.0) {
        obs_penalty = (1.0 - edt_dist) * 2.0;
    }
    
    return dist + obs_penalty;
}

bool TopoGraphSearch::canConnect(const Vector3d& from, const Vector3d& to) {
    double dist = (to - from).norm();
    
    // 🎯 REAL FIX: Use FIXED radius instead of dynamic formula
    // Previous approach was WRONG:
    //   - Dynamic radius based on start-goal distance
    //   - But corner points are NOT on start-goal line
    //   - Result: Graph always disconnected
    // 
    // Correct approach (TGK paper):
    //   - Fixed connection radius 8-10m
    //   - Simple and effective
    
    double CONNECTION_RADIUS = 8.0;  // Fixed 8m radius
    
    // Increase radius when blocking nodes to maintain connectivity
    if (node_pool_.size() >= 2) {
        int blocked_count = 0;
        for (const auto& node : node_pool_) {
            if (node.is_blocked) blocked_count++;
        }
        
        if (blocked_count > 0) {
            // Increase by 30% when blocking nodes
            CONNECTION_RADIUS = 8.0 * 1.3;  // = 10.4m
        }
    }
    
    if (dist > CONNECTION_RADIUS) {
        return false;
    }
    
    // Check collision-free path
    return isPathFree(from, to);
}

bool TopoGraphSearch::isPathFree(const Vector3d& from, const Vector3d& to) {
    Vector3d dir = to - from;
    double dist = dir.norm();
    
    if (dist < 1e-6) {
        return true;  // Same point
    }
    
    dir.normalize();
    
    // � CRITICAL FIX: 放宽路径检查,匹配原始 TGK-Planner
    // 原 TGK 使用动力学约束检查而非密集几何检查
    // 降低步长要求: 0.08/0.12m → 0.3/0.5m
    double step = (dist < 3.0) ? 0.3 : 0.5;
    int num_checks = static_cast<int>(dist / step);
    if (num_checks < 1) num_checks = 1;
    
    for (int i = 0; i <= num_checks; ++i) {
        Vector3d check_pt = from + (dist * i / num_checks) * dir;
        if (grid_map_->getInflateOccupancy(check_pt)) {
            return false;
        }
    }
    
    return true;
}

void TopoGraphSearch::smoothPath(vector<Vector3d>& path) {
    if (path.size() <= 2) {
        return;
    }
    
    // Remove redundant waypoints using line-of-sight check
    vector<Vector3d> smoothed_path;
    smoothed_path.push_back(path[0]);
    
    size_t current_idx = 0;
    while (current_idx < path.size() - 1) {
        // Try to connect to farthest visible point
        for (int i = path.size() - 1; i > static_cast<int>(current_idx); --i) {
            if (isPathFree(path[current_idx], path[i])) {
                smoothed_path.push_back(path[i]);
                current_idx = i;
                break;
            }
        }
        
        // If no line-of-sight, move to next point
        if (current_idx < path.size() - 1 && 
            (smoothed_path.back() - path[current_idx]).norm() < 0.01) {
            current_idx++;
            smoothed_path.push_back(path[current_idx]);
        }
    }
    
    path = smoothed_path;
    
    ROS_INFO("[TopoGraphSearch] Path smoothed: %zu waypoints", path.size());
}

bool TopoGraphSearch::arePathsSimilar(const vector<Vector3d>& path1,
                                     const vector<Vector3d>& path2) {
    // Simple similarity check based on average distance
    if (path1.empty() || path2.empty()) {
        return false;
    }
    
    // Sample points along both paths and compare
    int num_samples = 10;
    double total_dist = 0.0;
    
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        
        size_t idx1 = static_cast<size_t>(t * (path1.size() - 1));
        size_t idx2 = static_cast<size_t>(t * (path2.size() - 1));
        
        total_dist += (path1[idx1] - path2[idx2]).norm();
    }
    
    double avg_dist = total_dist / num_samples;
    
    return avg_dist < path_pruning_threshold_;
}

double TopoGraphSearch::calculatePathSimilarity(const vector<Vector3d>& path1,
                                                const vector<Vector3d>& path2) {
    // 🚀 IMPROVED: 使用Hausdorff距离计算路径相似度
    // 🔧 FIX 8: 排除起点/终点，只比较中间waypoint的拓扑差异
    // 目标: 当起点/终点相同时，避免它们过度影响相似度
    
    if (path1.empty() || path2.empty()) {
        return 0.0;
    }
    
    // 🔧 关键改进: 只比较中间waypoint (排除起点/终点)
    vector<Vector3d> middle_path1, middle_path2;
    
    if (path1.size() > 2) {
        for (size_t i = 1; i < path1.size() - 1; ++i) {
            middle_path1.push_back(path1[i]);
        }
    }
    
    if (path2.size() > 2) {
        for (size_t i = 1; i < path2.size() - 1; ++i) {
            middle_path2.push_back(path2[i]);
        }
    }
    
    // 如果没有中间waypoint (只有起点终点), 使用完整路径
    const vector<Vector3d>& compare_path1 = middle_path1.empty() ? path1 : middle_path1;
    const vector<Vector3d>& compare_path2 = middle_path2.empty() ? path2 : middle_path2;
    
    // 计算路径长度作为归一化基准
    double path1_length = 0.0;
    for (size_t i = 0; i < path1.size() - 1; ++i) {
        path1_length += (path1[i + 1] - path1[i]).norm();
    }
    
    if (path1_length < 1e-6) {
        return 1.0;  // 退化路径
    }
    
    // 双向Hausdorff距离 (只在中间waypoint之间计算)
    double max_dist_1to2 = 0.0;
    for (const auto& p1 : compare_path1) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& p2 : compare_path2) {
            double dist = (p1 - p2).norm();
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist > max_dist_1to2) {
            max_dist_1to2 = min_dist;
        }
    }
    
    // 反向: path2到path1
    double max_dist_2to1 = 0.0;
    for (const auto& p2 : compare_path2) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& p1 : compare_path1) {
            double dist = (p2 - p1).norm();
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist > max_dist_2to1) {
            max_dist_2to1 = min_dist;
        }
    }
    
    // 双向Hausdorff距离 = max(单向距离)
    double hausdorff_dist = std::max(max_dist_1to2, max_dist_2to1);
    
    // 归一化到[0,1]
    // hausdorff_dist = 0 → similarity = 1.0 (完全相同)
    // hausdorff_dist ≥ path_length → similarity = 0.0 (完全不同)
    double similarity = 1.0 - std::min(1.0, hausdorff_dist / path1_length);
    
    ROS_DEBUG("[TopoGraphSearch] Similarity calculation: middle waypoints only (%zu vs %zu points)",
             compare_path1.size(), compare_path2.size());
    
    return similarity;
}

} // namespace ego_planner
