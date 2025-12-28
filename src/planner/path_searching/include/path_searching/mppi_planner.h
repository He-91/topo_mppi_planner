#ifndef _MPPI_PLANNER_H_
#define _MPPI_PLANNER_H_

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <path_searching/mppi_dynamics.h>
#include <path_searching/mppi_cost.h>
#include <path_searching/mppi_sampling.h>
#include <vector>
#include <random>
#include <memory>
#include <chrono>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ego_planner {

// 🚀 Forward declaration for GPU planner (avoid including CUDA headers in .h)
#ifdef USE_GPU_MPPI
class MPPIGPUPlanner;
#endif

struct MPPITrajectory {
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> velocities;
    std::vector<Eigen::Vector3d> accelerations;
    double cost;
    double weight;
    
    MPPITrajectory() : cost(0.0), weight(0.0) {}
    
    void resize(int size) {
        positions.resize(size);
        velocities.resize(size);
        accelerations.resize(size);
    }
    
    int size() const { return positions.size(); }
};

class MPPIPlanner {
private:
    GridMap::Ptr grid_map_;
    
    // Visualization
    ros::Publisher mppi_trajectories_pub_;
    ros::Publisher optimal_trajectory_pub_;
    std::string frame_id_;
    
    // ✨ NEW: Modularized components (inspired by MPPI-Generic)
    SimplifiedQuadrotorDynamics dynamics_;
    MPPICost cost_;
    MPPISampling sampling_;
    
#ifdef USE_GPU_MPPI
    // 🚀 GPU acceleration (use unique_ptr to avoid including CUDA headers in .h)
    std::unique_ptr<MPPIGPUPlanner> gpu_planner_;
    bool use_gpu_;
    bool force_cpu_;  // 🔧 Force CPU even if GPU available (避免多实例冲突)
#endif
    
    // MPPI parameters
    int num_samples_;          // Number of rollout samples (default: 1000)
    int num_samples_min_;      // Minimum samples for adaptive sampling (default: 1000, upgraded from 500)
    int num_samples_max_;      // Maximum samples for adaptive sampling (default: 2000)
    bool use_adaptive_sampling_; // Enable adaptive sampling based on environment complexity
    int horizon_steps_;        // Planning horizon steps
    double dt_;                // Time step
    double lambda_;            // Temperature parameter for importance sampling
    
    // 🚀 NEW: Iterative MPPI parameters (Phase 2)
    int num_iterations_;       // Number of MPPI iterations (default: 1, set to 3 for iterative)
    bool use_iterative_mppi_;  // Enable iterative optimization
    
    // 🚀 Control distribution for iterative MPPI (μ, Σ)
    struct ControlDistribution {
        std::vector<Eigen::Vector3d> mean_control;      // μ: mean control sequence (horizon_steps_)
        std::vector<Eigen::Vector3d> std_control;       // σ: std dev control sequence (horizon_steps_)
        
        void resize(int horizon) {
            mean_control.resize(horizon, Eigen::Vector3d::Zero());
            std_control.resize(horizon, Eigen::Vector3d::Ones());  // Initialize to 1.0
        }
    };
    ControlDistribution control_distribution_;
    
    // Performance monitoring
    struct TimingStats {
        double rollout_time = 0.0;
        double cost_time = 0.0;
        double weight_time = 0.0;
        double average_time = 0.0;
        double total_time = 0.0;
        int num_samples = 0;
    };
    TimingStats last_timing_;
    
    // Random number generator (for backward compatibility)
    std::mt19937 generator_;
    std::normal_distribution<double> normal_dist_;
    
    // Helper functions
    void rolloutTrajectory(const Eigen::Vector3d& start_pos,
                          const Eigen::Vector3d& start_vel,
                          const Eigen::Vector3d& goal_pos,
                          const Eigen::Vector3d& goal_vel,
                          MPPITrajectory& trajectory);
    
    void rolloutTrajectory(const Eigen::Vector3d& start_pos,
                          const Eigen::Vector3d& start_vel,
                          const Eigen::Vector3d& goal_pos,
                          const Eigen::Vector3d& goal_vel,
                          const std::vector<Eigen::Vector3d>& guide_path,
                          MPPITrajectory& trajectory);
    
    // 🚀 Thread-safe rollout functions for OpenMP parallel execution
    void rolloutTrajectory(const Eigen::Vector3d& start_pos,
                          const Eigen::Vector3d& start_vel,
                          const Eigen::Vector3d& goal_pos,
                          const Eigen::Vector3d& goal_vel,
                          MPPITrajectory& trajectory,
                          std::mt19937& local_gen,
                          std::normal_distribution<double>& local_dist);
    
    void rolloutTrajectory(const Eigen::Vector3d& start_pos,
                          const Eigen::Vector3d& start_vel,
                          const Eigen::Vector3d& goal_pos,
                          const Eigen::Vector3d& goal_vel,
                          const std::vector<Eigen::Vector3d>& guide_path,
                          MPPITrajectory& trajectory,
                          std::mt19937& local_gen,
                          std::normal_distribution<double>& local_dist);
    
    double calculateTrajectoryCost(const MPPITrajectory& trajectory,
                                  const Eigen::Vector3d& goal_pos,
                                  const Eigen::Vector3d& goal_vel);
    
    double obstacleCost(const Eigen::Vector3d& position);
    double obstacleCost(const Eigen::Vector3d& position, double dist);  // ✅ Phase 3: ESDF-based cost
    double smoothnessCost(const MPPITrajectory& trajectory);
    double goalCost(const MPPITrajectory& trajectory,
                   const Eigen::Vector3d& goal_pos,
                   const Eigen::Vector3d& goal_vel);
    double velocityCost(const MPPITrajectory& trajectory,
                       const Eigen::Vector3d& desired_vel);
    
    void constrainDynamics(Eigen::Vector3d& velocity, Eigen::Vector3d& acceleration);
    
    MPPITrajectory weightedAverage(const std::vector<MPPITrajectory>& trajectories);
    
    // 🚀 NEW: Iterative MPPI functions
    void updateControlDistribution(const std::vector<MPPITrajectory>& trajectories,
                                   const std::vector<Eigen::Vector3d>& control_samples,
                                   ControlDistribution& new_distribution);
    
    void sampleControlFromDistribution(const ControlDistribution& distribution,
                                      std::vector<Eigen::Vector3d>& control_sequence,
                                      std::mt19937& local_gen,
                                      std::normal_distribution<double>& local_dist);
    
    // Adaptive sampling
    int computeAdaptiveSamples(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& goal_pos);
    
    // Visualization functions
    void visualizeTrajectories(const std::vector<MPPITrajectory>& trajectories);
    void visualizeOptimalTrajectory(const MPPITrajectory& trajectory);

public:
    typedef std::shared_ptr<MPPIPlanner> Ptr;
    
    MPPIPlanner();
    ~MPPIPlanner();
    
    void init(ros::NodeHandle& nh, GridMap::Ptr grid_map);
    void setForceCPU(bool force) { 
#ifdef USE_GPU_MPPI
force_cpu_ = force;
#endif
}  // Force CPU mode
    
    // Main planning interface
    bool planTrajectory(const Eigen::Vector3d& start_pos,
                       const Eigen::Vector3d& start_vel,
                       const Eigen::Vector3d& goal_pos,
                       const Eigen::Vector3d& goal_vel,
                       MPPITrajectory& optimal_trajectory);
    
    // Overload: with initial path guidance (for topological paths)
    bool planTrajectory(const Eigen::Vector3d& start_pos,
                       const Eigen::Vector3d& start_vel,
                       const Eigen::Vector3d& goal_pos,
                       const Eigen::Vector3d& goal_vel,
                       const std::vector<Eigen::Vector3d>& initial_path,
                       MPPITrajectory& optimal_trajectory);
    
    // Local path planning interface (for replacing A* in B-spline optimizer)
    bool planLocalPath(const Eigen::Vector3d& start_pos,
                      const Eigen::Vector3d& goal_pos,
                      std::vector<Eigen::Vector3d>& path_points);
    
    // Parameter setters
    void setNumSamples(int num_samples) { num_samples_ = num_samples; }
    void setHorizonSteps(int steps) { horizon_steps_ = steps; }
    void setTimeStep(double dt) { dt_ = dt; }
    void setTemperature(double lambda) { lambda_ = lambda; }
    void setNoiseParameters(double sigma_pos, double sigma_vel, double sigma_acc) {
        sampling_.setSigma(sigma_acc);
    }
    void setCostWeights(double w_obs, double w_smooth, double w_goal, double w_vel) {
        cost_.setObstacleWeight(w_obs);
        cost_.setSmoothnessWeight(w_smooth);
        cost_.setGoalWeight(w_goal);
        cost_.setVelocityWeight(w_vel);
    }
    void setVehicleLimits(double max_vel, double max_acc) {
        dynamics_.setMaxVelocity(max_vel);
        dynamics_.setMaxAcceleration(max_acc);
    }
    
    // Getters
    int getHorizonSteps() const { return horizon_steps_; }
    double getTimeStep() const { return dt_; }
    const TimingStats& getLastTiming() const { return last_timing_; }
    
    // ✨ NEW: Get modularized components
    SimplifiedQuadrotorDynamics& getDynamics() { return dynamics_; }
    MPPICost& getCost() { return cost_; }
    MPPISampling& getSampling() { return sampling_; }
};

} // namespace ego_planner

#endif