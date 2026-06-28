#ifndef _MPPI_COST_H_
#define _MPPI_COST_H_

#include <Eigen/Eigen>
#include <ddo_plan_env/grid_map.h>
#include <memory>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

namespace ego_planner {

/**
 * @brief Cost function for MPPI trajectory optimization
 * Inspired by MPPI-Generic's cost interface with separated running and terminal costs
 *  Now supports actual dynamic obstacle positions with time-synchronized prediction
 */
class MPPICost {
public:
    //  Dynamic obstacle data structure
    struct DynamicObstacleData {
        std::vector<Eigen::Vector3d> current_positions;  // [num_obstacles]
        std::vector<Eigen::Vector3d> velocities;         // [num_obstacles]
        std::vector<float> radii;                        // [num_obstacles]
        std::vector<float> heights;                      // [num_obstacles]
        std::vector<std::vector<Eigen::Vector3d>> predicted_positions;  // [num_obstacles][horizon]
        int num_obstacles = 0;
        int prediction_horizon = 0;
        double prediction_dt = 0.1;
        bool valid = false;
    };

    MPPICost() 
        : w_obstacle_(100.0), w_dynamic_(30.0), w_smoothness_(5.0),
          w_goal_(15.0), w_velocity_(10.0), w_path_guidance_(30.0),
          // TUNED v10: safe_distance 0.5→0.55 to match dist0=0.65 layering.
          // MPPI safe_dist should be slightly less than B-spline dist0 for smooth handoff.
          // Layer: Topo clearance=0.2 < MPPI safe=0.55 < B-spline dist0=0.65
          safe_distance_(0.55), desired_velocity_(2.0),
          dynamic_safe_distance_(0.8),
          dynamic_collision_distance_(0.0),
          near_collision_distance_(0.15), near_collision_weight_(0.0) {
    }
    
    void setGridMap(GridMap::Ptr grid_map) { grid_map_ = grid_map; }
    
    /**
     * @brief  Set dynamic obstacle data for time-synchronized cost computation
     */
    void setDynamicObstacles(const std::vector<Eigen::Vector3d>& positions,
                            const std::vector<Eigen::Vector3d>& velocities,
                            const std::vector<float>& radii,
                            const std::vector<float>& heights = {}) {
        dynamic_data_.current_positions = positions;
        dynamic_data_.velocities = velocities;
        dynamic_data_.radii = radii;
        dynamic_data_.heights = heights;
        dynamic_data_.num_obstacles = positions.size();
        dynamic_data_.valid = !positions.empty();
    }
    
    /**
     * @brief  Set full prediction data (positions at each future timestep)
     */
    void setDynamicObstaclePredictions(const std::vector<std::vector<Eigen::Vector3d>>& predicted_positions,
                                      int horizon, double dt) {
        dynamic_data_.predicted_positions = predicted_positions;
        dynamic_data_.prediction_horizon = horizon;
        dynamic_data_.prediction_dt = dt;
    }
    
    /**
     * @brief Compute running cost at continuous rollout time.
     */
    double computeRunningCost(const Eigen::Vector3d& position,
                             const Eigen::Vector3d& velocity,
                             const Eigen::Vector3d& acceleration,
                             const Eigen::Vector3d& goal_pos,
                             double query_time,
                             const Eigen::Vector3d* path_waypoint = nullptr) const {
        double cost = 0.0;
        
        // 1. Obstacle cost (EDT-based) - accumulated per timestep
        cost += computeObstacleCost(position);
        
        // 2. Dynamic obstacle cost -  Now uses actual predicted positions
        cost += computeDynamicObstacleCost(position, velocity, query_time);
        
        // 3. Smoothness cost (control effort) - accumulated per timestep
        cost += w_smoothness_ * acceleration.squaredNorm();
        
        // 4. Path guidance cost (if provided) - accumulated per timestep
        if (path_waypoint != nullptr) {
            double dist_to_path = (position - *path_waypoint).norm();
            cost += w_path_guidance_ * dist_to_path * dist_to_path;
        }
        
        return cost;
    }
    
    /**
     * @brief Compute terminal cost at final state
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
        
        //  FIX: Check for ESDF invalid values (like -10000.0)
        if (dist < -1000.0) {
            return w_obstacle_ * 10.0;
        }
        
        //  FIX: Cap maximum penalty to avoid 100000+ costs
        if (dist < 0.0) {
            double penetration = std::min(-dist, 2.0);
            return w_obstacle_ * 50.0 * (1.0 + penetration * penetration);
        }
        
        // Exponential cost near obstacles
        if (dist < safe_distance_) {
            double penetration = safe_distance_ - dist;
            double cost = w_obstacle_ * penetration * penetration;
            if (dist < near_collision_distance_) {
                double near_penetration = near_collision_distance_ - dist;
                cost += w_obstacle_ * near_collision_weight_ *
                        near_penetration * near_penetration;
            }
            return cost;
        }
        
        return 0.0;
    }
    
    /**
     * @brief  Compute dynamic obstacle cost using actual predicted positions (time-synchronized)
     * Replaces the FAKE implementation that just re-queried static EDT
     */
    double computeDynamicObstacleCost(const Eigen::Vector3d& position,
                                     const Eigen::Vector3d& velocity,
                                     double query_time) const {
        if (!dynamic_data_.valid || dynamic_data_.num_obstacles == 0) {
            return 0.0;  // No dynamic obstacles → zero cost
        }
        
        double total_cost = 0.0;
        
        for (int i = 0; i < dynamic_data_.num_obstacles; ++i) {
            //  Get predicted obstacle position at this rollout time.
            Eigen::Vector3d obs_pos;
            const double t_query = std::max(0.0, query_time);
            if (!dynamic_data_.predicted_positions.empty() && 
                i < (int)dynamic_data_.predicted_positions.size() &&
                !dynamic_data_.predicted_positions[i].empty()) {
                const auto& pred = dynamic_data_.predicted_positions[i];
                const double pred_dt = std::max(1e-4, dynamic_data_.prediction_dt);
                const double fidx = t_query / pred_dt;
                const int idx0 = std::max(0, std::min((int)std::floor(fidx), (int)pred.size() - 1));
                const int idx1 = std::max(0, std::min(idx0 + 1, (int)pred.size() - 1));
                const double alpha = std::min(1.0, std::max(0.0, fidx - (double)idx0));
                obs_pos = pred[idx0] * (1.0 - alpha) + pred[idx1] * alpha;
            } else {
                // Fallback: constant velocity prediction
                obs_pos = dynamic_data_.current_positions[i] + dynamic_data_.velocities[i] * t_query;
            }
            
            const double obs_radius =
                (i < (int)dynamic_data_.radii.size()) ? dynamic_data_.radii[i] : 0.0;
            const double obs_height =
                (i < (int)dynamic_data_.heights.size()) ? dynamic_data_.heights[i] : 0.0;
            const double dx = position.x() - obs_pos.x();
            const double dy = position.y() - obs_pos.y();
            const double radial_out = std::sqrt(dx * dx + dy * dy) - std::max(0.0, obs_radius);
            const double half_height = std::max(0.0, obs_height) * 0.5;
            const double vertical_out = std::fabs(position.z() - obs_pos.z()) - half_height;
            double dist = 0.0;
            if (radial_out <= 0.0 && vertical_out <= 0.0) {
                dist = std::max(radial_out, vertical_out);
            } else {
                const double clamped_radial = std::max(0.0, radial_out);
                const double clamped_vertical = std::max(0.0, vertical_out);
                dist = std::sqrt(clamped_radial * clamped_radial +
                                 clamped_vertical * clamped_vertical);
            }
            
            // Critical dynamic zone penalty. Keep this finite for parity with
            // the GPU implementation: dense dynamic scenes may have all
            // rollouts briefly enter the critical band, and hard infinity can
            // starve the optimizer.
            if (dist < dynamic_collision_distance_) {
                double penetration =
                    std::min(dynamic_collision_distance_ - dist, 2.0);
                total_cost += w_dynamic_ * 50.0 * (1.0 + penetration * penetration);
            } else if (dist < dynamic_safe_distance_) {
                // Within safety margin - exponential penalty
                double penetration = dynamic_safe_distance_ - dist;
                total_cost += w_dynamic_ * penetration * penetration;
            }
            
            //  Relative velocity penalty: extra cost for approaching obstacles
            if (dist < dynamic_safe_distance_ * 2.0) {
                Eigen::Vector3d relative_pos = position - obs_pos;
                Eigen::Vector3d obs_vel = (i < (int)dynamic_data_.velocities.size()) ? 
                    dynamic_data_.velocities[i] : Eigen::Vector3d::Zero();
                Eigen::Vector3d relative_vel = velocity - obs_vel;
                
                // Closing speed (positive = approaching)
                double closing_speed = -relative_vel.dot(relative_pos.normalized());
                if (closing_speed > 0.0 && dist < dynamic_safe_distance_) {
                    // Penalize approaching behavior
                    total_cost += w_dynamic_ * 0.5 * closing_speed * closing_speed;
                }
            }
        }
        
        return total_cost;
    }
    
    // Setters for cost weights
    void setObstacleWeight(double w) { w_obstacle_ = w; }
    void setDynamicWeight(double w) { w_dynamic_ = w; }
    void setSmoothnessWeight(double w) { w_smoothness_ = w; }
    void setGoalWeight(double w) { w_goal_ = w; }
    void setVelocityWeight(double w) { w_velocity_ = w; }
    void setPathGuidanceWeight(double w) { w_path_guidance_ = w; }
    void setSafeDistance(double d) { safe_distance_ = d; }
    void setDynamicSafeDistance(double d) { dynamic_safe_distance_ = d; }
    void setDynamicCollisionDistance(double d) { dynamic_collision_distance_ = std::max(0.0, d); }
    void setDesiredVelocity(double v) { desired_velocity_ = v; }
    void setNearCollisionDistance(double d) { near_collision_distance_ = std::max(0.0, d); }
    void setNearCollisionWeight(double w) { near_collision_weight_ = std::max(0.0, w); }
    
    // Getters
    double getObstacleWeight() const { return w_obstacle_; }
    double getDynamicWeight() const { return w_dynamic_; }
    double getSmoothnessWeight() const { return w_smoothness_; }
    double getGoalWeight() const { return w_goal_; }
    double getVelocityWeight() const { return w_velocity_; }
    double getPathGuidanceWeight() const { return w_path_guidance_; }
    double getSafeDistance() const { return safe_distance_; }
    double getDynamicSafeDistance() const { return dynamic_safe_distance_; }
    double getDynamicCollisionDistance() const { return dynamic_collision_distance_; }
    double getDesiredVelocity() const { return desired_velocity_; }
    double getNearCollisionDistance() const { return near_collision_distance_; }
    double getNearCollisionWeight() const { return near_collision_weight_; }
    
    //  Check if dynamic obstacles are set
    bool hasDynamicObstacles() const { return dynamic_data_.valid; }
    int getNumDynamicObstacles() const { return dynamic_data_.num_obstacles; }
    
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
    double dynamic_safe_distance_;  //  Safety margin for dynamic obstacles
    double dynamic_collision_distance_;
    double desired_velocity_;
    double near_collision_distance_;
    double near_collision_weight_;
    
    //  Dynamic obstacle data
    DynamicObstacleData dynamic_data_;
};

} // namespace ego_planner

#endif // _MPPI_COST_H_
