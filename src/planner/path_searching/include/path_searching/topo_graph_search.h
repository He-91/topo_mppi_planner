#ifndef _TOPO_GRAPH_SEARCH_H_
#define _TOPO_GRAPH_SEARCH_H_

#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <queue>
#include <unordered_map>
#include <plan_env/grid_map.h>
#include "path_searching/bias_sampler.h"

namespace ego_planner {

/**
 * @brief Graph node for topological path search
 */
struct TopoNode {
    Eigen::Vector3d pos;
    double g_cost;          // Cost from start
    double h_cost;          // Heuristic to goal
    int parent_id;          // Parent node ID
    int node_id;            // Unique node ID
    int topo_class;         // Topological class
    bool is_blocked;        // 🚀 NEW: 标记节点是否被阻塞 (用于K-shortest paths)
    int corridor_id;        // 🚀 NEW: 走廊ID (-10, -5, 0, +5, +10) - 用于拓扑感知的K-shortest
    
    TopoNode() : g_cost(0), h_cost(0), parent_id(-1), node_id(-1), topo_class(-1), is_blocked(false), corridor_id(0) {}
    
    double f_cost() const { return g_cost + h_cost; }
};

/**
 * @brief Priority queue comparator for A*
 */
struct TopoNodeComparator {
    bool operator()(const TopoNode& a, const TopoNode& b) const {
        return a.f_cost() > b.f_cost();  // Min-heap
    }
};

/**
 * @brief Topological Graph Search using A*
 * Simplified geometric search connecting topological key points
 * No dynamics, no time allocation - pure geometry
 */
class TopoGraphSearch {
public:
    typedef std::shared_ptr<TopoGraphSearch> Ptr;
    
    TopoGraphSearch();
    ~TopoGraphSearch();
    
    /**
     * @brief Initialize with grid map and bias sampler
     */
    void init(GridMap::Ptr grid_map, BiasSampler::Ptr bias_sampler);
    
    /**
     * @brief Search topological paths from start to goal
     * @param start Start position
     * @param goal Goal position
     * @param paths Output: multiple topological paths
     * @return Success flag
     */
    bool searchTopoPaths(const Eigen::Vector3d& start,
                        const Eigen::Vector3d& goal,
                        std::vector<std::vector<Eigen::Vector3d>>& paths);
    
    /**
     * @brief Search a single best path (for quick planning)
     */
    bool searchSinglePath(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& goal,
                         std::vector<Eigen::Vector3d>& path);
    
    // Parameter setters
    void setMaxSearchNodes(int num) { max_search_nodes_ = num; }
    void setConnectionRadius(double radius) { connection_radius_ = radius; }
    void setPathPruningThreshold(double threshold) { path_pruning_threshold_ = threshold; }
    void setMaxTopoPaths(int num) { max_topo_paths_ = num; }  // 🚀 NEW: Set maximum topological paths
    
private:
    GridMap::Ptr grid_map_;
    BiasSampler::Ptr bias_sampler_;
    
    // Parameters
    int max_search_nodes_;
    double connection_radius_;      // Max distance for connecting nodes
    double path_pruning_threshold_; // Remove similar paths
    int max_topo_paths_;           // Maximum topological paths to return
    
    // Search state
    std::vector<TopoNode> node_pool_;
    int node_counter_;
    
    /**
     * @brief Build search graph from start to goal
     * Creates nodes at key topological points and connects them
     */
    bool buildSearchGraph(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& goal);
    
    /**
     * @brief A* search on the constructed graph
     * @param path_node_ids Optional output: node IDs along the path (for corridor extraction)
     */
    bool astarSearch(const Eigen::Vector3d& start,
                    const Eigen::Vector3d& goal,
                    std::vector<Eigen::Vector3d>& path,
                    std::vector<int>* path_node_ids = nullptr);
    
    /**
     * @brief Extract multiple topologically distinct paths
     */
    void extractMultiplePaths(const Eigen::Vector3d& start,
                             const Eigen::Vector3d& goal,
                             std::vector<std::vector<Eigen::Vector3d>>& paths);
    
    /**
     * @brief Heuristic function for A*
     */
    double heuristic(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal);
    
    /**
     * @brief Cost function for edge connection
     */
    double edgeCost(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    
    /**
     * @brief Check if two nodes can be connected
     */
    bool canConnect(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    
    /**
     * @brief Check if path is collision-free
     */
    bool isPathFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    
    /**
     * @brief Smooth path by removing redundant waypoints
     */
    void smoothPath(std::vector<Eigen::Vector3d>& path);
    
    /**
     * @brief Check if two paths are topologically similar
     */
    bool arePathsSimilar(const std::vector<Eigen::Vector3d>& path1,
                        const std::vector<Eigen::Vector3d>& path2);
    
    /**
     * @brief Calculate path similarity score (0.0 = different, 1.0 = identical)
     * 🚀 NEW: For improved K-shortest paths algorithm
     */
    double calculatePathSimilarity(const std::vector<Eigen::Vector3d>& path1,
                                   const std::vector<Eigen::Vector3d>& path2);
};

} // namespace ego_planner

#endif
