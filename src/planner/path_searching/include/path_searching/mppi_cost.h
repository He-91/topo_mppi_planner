#ifndef _MPPI_COST_H_
#define _MPPI_COST_H_

#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <memory>
#include <cmath>
#include <limits>

namespace ego_planner {

/**
 * @brief Cost function for MPPI trajectory optimization
 * Inspired by MPPI-Generic's cost interface with separated running and terminal costs
 */
class MPPICost {
public:
    MPPICost() 
        : w_obstacle_(100.0), w_dynamic_(1.5), w_smoothness_(5.0),  // ✅ Phase 2: 降低权重避免cost爆炸
          w_goal_(15.0), w_velocity_(10.0), w_path_guidance_(30.0),  // 关键: goal 50→15, vel 20→10
          safe_distance_(0.3), desired_velocity_(2.0) {
    }
    
    void setGridMap(GridMap::Ptr grid_map) { grid_map_ = grid_map; }
    
    /**
     * @brief Compute running cost at timestep t
     * @param position Current position
     * @param velocity Current velocity
     * @param acceleration Current control (acceleration)
     * @param goal_pos Goal position
     * @param timestep Current timestep
     * @param path_waypoint Optional guidance from topo path
     * @return Running cost value
     */
    double computeRunningCost(const Eigen::Vector3d& position,
                             const Eigen::Vector3d& velocity,
                             const Eigen::Vector3d& acceleration,
                             const Eigen::Vector3d& goal_pos,
                             int timestep,
                             const Eigen::Vector3d* path_waypoint = nullptr) const {
        double cost = 0.0;
        
        // 1. Obstacle cost (EDT-based) - accumulated per timestep
        cost += computeObstacleCost(position);
        
        // 2. Dynamic obstacle cost - accumulated per timestep
        cost += computeDynamicObstacleCost(position, velocity, timestep);
        
        // 3. Smoothness cost (control effort) - accumulated per timestep
        cost += w_smoothness_ * acceleration.squaredNorm();
        
        // 4. Path guidance cost (if provided) - accumulated per timestep
        if (path_waypoint != nullptr) {
            double dist_to_path = (position - *path_waypoint).norm();
            cost += w_path_guidance_ * dist_to_path * dist_to_path;
        }
        
        // ✅ FIX: Remove per-timestep velocity/goal costs to avoid 30x amplification
        // These should only be evaluated once (in terminal cost or aggregated)
        // Old code computed them once for entire trajectory, not per-step!
        
        return cost;
    }
    
    /**
     * @brief Compute terminal cost at final state
     * @param final_position Final position
     * @param final_velocity Final velocity
     * @param goal_pos Goal position
     * @param goal_vel Goal velocity
     * @return Terminal cost value
     */
    double computeTerminalCost(const Eigen::Vector3d& final_position,
                              const Eigen::Vector3d& final_velocity,
                              const Eigen::Vector3d& goal_pos,
                              const Eigen::Vector3d& goal_vel) const {
        double cost = 0.0;
        
        // Strong penalty for not reaching goal position
        double pos_error = (final_position - goal_pos).norm();
        cost += w_goal_ * pos_error * pos_error;
        
        // Velocity matching at goal
        double vel_error = (final_velocity - goal_vel).norm();
        cost += w_velocity_ * vel_error * vel_error;
        
        return cost;
    }
    
    /**
     * @brief Compute obstacle cost using EDT distance field
     */
    double computeObstacleCost(const Eigen::Vector3d& position) const {
        if (!grid_map_) {
            return 0.0;
        }
        
        // Query EDT distance
        double dist = grid_map_->getDistance(position);
        
        // 🔧 FIX: Check for ESDF invalid values (like -10000.0)
        if (dist < -1000.0) {
            // ESDF not initialized at this position, treat as unknown (moderate penalty)
            return w_obstacle_ * 10.0;
        }
        
        // 🔧 FIX: Cap maximum penalty to avoid 100000+ costs
        if (dist < 0.0) {
            // Inside obstacle - large but bounded penalty
            double penetration = std::min(-dist, 2.0);  // Cap penetration at 2m
            return w_obstacle_ * 50.0 * (1.0 + penetration * penetration);
        }
        
        // Exponential cost near obstacles
        if (dist < safe_distance_) {
            double penetration = safe_distance_ - dist;
            return w_obstacle_ * penetration * penetration;
        }
        
        return 0.0;
    }
    
    /**
     * @brief Compute dynamic obstacle cost (time-aware)
     */
    double computeDynamicObstacleCost(const Eigen::Vector3d& position,
                                     const Eigen::Vector3d& velocity,
                                     int timestep) const {
        if (!grid_map_) {
            return 0.0;
        }
        
        // Check if position will collide with predicted dynamic obstacles
        // This requires time-synchronized EDT query
        // For now, use static check with higher weight
        double dist = grid_map_->getDistance(position);
        
        // 🔧 FIX: Check for ESDF invalid values
        if (dist < -1000.0) {
            return w_dynamic_ * w_obstacle_ * 10.0;
        }
        
        // 🔧 FIX: Cap maximum penalty
        if (dist < 0.0) {
            double penetration = std::min(-dist, 2.0);  // Cap at 2m
            return w_dynamic_ * w_obstacle_ * 50.0 * (1.0 + penetration * penetration);
        }
        
        if (dist < safe_distance_) {
            double penetration = safe_distance_ - dist;
            return w_dynamic_ * w_obstacle_ * penetration * penetration;
        }
        
        return 0.0;
    }
    
    // Setters for cost weights
    void setObstacleWeight(double w) { w_obstacle_ = w; }
    void setDynamicWeight(double w) { w_dynamic_ = w; }
    void setSmoothnessWeight(double w) { w_smoothness_ = w; }
    void setGoalWeight(double w) { w_goal_ = w; }
    void setVelocityWeight(double w) { w_velocity_ = w; }
    void setPathGuidanceWeight(double w) { w_path_guidance_ = w; }
    void setSafeDistance(double d) { safe_distance_ = d; }
    void setDesiredVelocity(double v) { desired_velocity_ = v; }
    
    // Getters
    double getObstacleWeight() const { return w_obstacle_; }
    double getDynamicWeight() const { return w_dynamic_; }
    double getSmoothnessWeight() const { return w_smoothness_; }
    double getGoalWeight() const { return w_goal_; }
    double getVelocityWeight() const { return w_velocity_; }
    double getPathGuidanceWeight() const { return w_path_guidance_; }
    double getSafeDistance() const { return safe_distance_; }
    double getDesiredVelocity() const { return desired_velocity_; }
    
private:
    GridMap::Ptr grid_map_;
    
    // Cost weights
    double w_obstacle_;
    double w_dynamic_;
    double w_smoothness_;
    double w_goal_;
    double w_velocity_;
    double w_path_guidance_;
    
    // Parameters
    double safe_distance_;
    double desired_velocity_;
};

} // namespace ego_planner

#endif // _MPPI_COST_H_
