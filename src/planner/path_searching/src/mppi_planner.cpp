#include "path_searching/mppi_planner.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <omp.h>

#ifdef USE_GPU_MPPI
#include <path_searching/mppi_gpu_planner.h>
#endif

using namespace std;
using namespace Eigen;

namespace ego_planner {

MPPIPlanner::MPPIPlanner() 
    : num_samples_(500), num_samples_min_(300), num_samples_max_(800),  // 🔧 降低样本数防止OOM
      use_adaptive_sampling_(true), horizon_steps_(15), dt_(0.1), lambda_(1.0),  // 🔧 降低horizon防止内存爆炸
      num_iterations_(1), use_iterative_mppi_(false),  // 🚀 Phase 2: 迭代优化默认关闭
      generator_(std::random_device{}()), normal_dist_(0.0, 1.0)
#ifdef USE_GPU_MPPI
    , use_gpu_(true), force_cpu_(false)  // 🔥 Initialize force_cpu flag
#endif
{
    
    // ✨ Initialize modularized components
    dynamics_.setMaxVelocity(3.0);
    dynamics_.setMaxAcceleration(3.0);
    
    // ✅ Phase 2优化: 降低代价权重,避免cost爆炸
    cost_.setObstacleWeight(100.0);    // 200→100: 降低障碍物惩罚
    cost_.setDynamicWeight(1.5);
    cost_.setSmoothnessWeight(5.0);    // 10→5: 降低平滑性惩罚
    cost_.setGoalWeight(15.0);         // 50→15: 关键修复! 降低目标距离代价
    cost_.setVelocityWeight(10.0);     // 20→10: 降低速度惩罚
    cost_.setSafeDistance(0.3);
    cost_.setDesiredVelocity(2.0);
    
    sampling_.setSigma(1.0);
    sampling_.setUseColoredNoise(true);  // ✨ Enable ColoredNoise for smoother control
    sampling_.setTemporalCorrelation(0.7);  // α=0.7 for medium temporal coherence
    sampling_.initialize();
    
    // 🚀 Initialize control distribution for iterative MPPI
    control_distribution_.resize(horizon_steps_);
}

MPPIPlanner::~MPPIPlanner() {
}

void MPPIPlanner::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    cost_.setGridMap(grid_map);  // ✨ Set grid map in cost module
    
    // 🚀 Read MPPI parameters from ROS parameter server (覆盖构造函数中的硬编码默认值)
    nh.param("mppi/horizon_steps", horizon_steps_, 20);  // 默认20步
    nh.param("mppi/dt", dt_, 0.1);                       // 默认0.1秒
    nh.param("mppi/num_samples", num_samples_, 500);
    nh.param("mppi/num_samples_min", num_samples_min_, 300);
    nh.param("mppi/num_samples_max", num_samples_max_, 800);
    nh.param("mppi/use_adaptive_sampling", use_adaptive_sampling_, true);
    nh.param("mppi/lambda", lambda_, 1.0);
    nh.param("mppi/use_iterative_mppi", use_iterative_mppi_, false);
    nh.param("mppi/num_iterations", num_iterations_, 1);
    
    // 🔒 Validate parameters
    if (horizon_steps_ < 5 || horizon_steps_ > 100) {
        ROS_WARN("[MPPI] Invalid horizon_steps=%d, using default 20", horizon_steps_);
        horizon_steps_ = 20;
    }
    if (dt_ <= 0.0 || dt_ > 1.0) {
        ROS_WARN("[MPPI] Invalid dt=%.3f, using default 0.1", dt_);
        dt_ = 0.1;
    }
    
    // 🚀 Resize control distribution with updated horizon
    control_distribution_.resize(horizon_steps_);
    
    // Initialize visualization publishers
    mppi_trajectories_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mppi_trajectories", 10);
    optimal_trajectory_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mppi_optimal_trajectory", 10);
    
    // Get frame_id from node parameter, default to "world" if not set
    nh.param("grid_map/frame_id", frame_id_, std::string("world"));
    
#ifdef USE_GPU_MPPI
    // 🚀 Initialize GPU planner
    nh.param("mppi/use_gpu", use_gpu_, false);
    ROS_INFO("[MPPI] 🔍 DEBUG: Read parameter mppi/use_gpu = %s", use_gpu_ ? "TRUE" : "FALSE");
    
    // 🔧 Override with force_cpu flag if set
    if (force_cpu_) {
        use_gpu_ = false;
        ROS_INFO("[MPPI] 🔧 Forced CPU mode (force_cpu_ flag set)");
    }
    
    if (use_gpu_) {
        gpu_planner_ = std::make_unique<MPPIGPUPlanner>();
        
        // Configure GPU planner parameters
        MPPIGPUPlanner::Params gpu_params;
        gpu_params.num_samples = num_samples_;
        gpu_params.horizon_steps = horizon_steps_;
        gpu_params.dt = dt_;
        gpu_params.lambda = lambda_;
        gpu_params.max_velocity = dynamics_.getMaxVelocity();
        gpu_params.max_acceleration = dynamics_.getMaxAcceleration();
        gpu_params.w_obstacle = cost_.getObstacleWeight();
        gpu_params.w_smoothness = cost_.getSmoothnessWeight();
        
        // 🔥 Iterative MPPI parameters
        gpu_params.use_iterative_mppi = use_iterative_mppi_;
        gpu_params.num_iterations = num_iterations_;
        
        gpu_planner_->initialize(gpu_params);
        
        ROS_INFO("[MPPI] 🚀 GPU acceleration ENABLED (Iterative=%s, Iters=%d)",
                 use_iterative_mppi_ ? "YES" : "NO", num_iterations_);
    } else {
        ROS_INFO("[MPPI] Using CPU implementation (set use_gpu=true to enable GPU)");
    }
#endif
    
    ROS_INFO("[MPPI] ✨ Initialized with modularized design (inspired by MPPI-Generic)");
    ROS_INFO("[MPPI] Samples: %d, Horizon: %d steps (%.2fs), dt: %.3f", 
             num_samples_, horizon_steps_, horizon_steps_ * dt_, dt_);
    ROS_INFO("[MPPI] Dynamics: MaxVel=%.2f m/s, MaxAcc=%.2f m/s²", 
             dynamics_.getMaxVelocity(), dynamics_.getMaxAcceleration());
    ROS_INFO("[MPPI] Cost weights: Obs=%.1f, Dyn=%.1f, Smooth=%.1f, Goal=%.1f, Vel=%.1f",
             cost_.getObstacleWeight(), cost_.getDynamicWeight(), 
             cost_.getSmoothnessWeight(), cost_.getGoalWeight(), cost_.getVelocityWeight());
    ROS_INFO("[MPPI] Sampling: Sigma=%.2f, ColoredNoise=%s, Alpha=%.2f",
             sampling_.getSigma(), sampling_.getUseColoredNoise() ? "ON" : "OFF",
             sampling_.getTemporalCorrelation());
    
    // 🚀 NEW: Iterative MPPI parameters
    nh.param("mppi/use_iterative_mppi", use_iterative_mppi_, false);
    nh.param("mppi/num_iterations", num_iterations_, 1);
    if (use_iterative_mppi_ && num_iterations_ > 1) {
        ROS_INFO("[MPPI] 🔥 Iterative MPPI ENABLED: %d iterations", num_iterations_);
    } else {
        ROS_INFO("[MPPI] Standard single-pass MPPI (set use_iterative_mppi=true for iterative mode)");
    }
    
    ROS_INFO("[MPPI] Using frame_id: %s", frame_id_.c_str());
}

bool MPPIPlanner::planTrajectory(const Vector3d& start_pos,
                                const Vector3d& start_vel,
                                const Vector3d& goal_pos,
                                const Vector3d& goal_vel,
                                MPPITrajectory& optimal_trajectory) {
    
    // 🔒 CRITICAL: Validate object state before planning
    if (horizon_steps_ <= 0 || horizon_steps_ > 100) {
        ROS_ERROR("[MPPI] ❌ INVALID STATE: horizon_steps_=%d (expected 10-30). Object not properly initialized!", 
                  horizon_steps_);
        return false;
    }
    if (!grid_map_) {
        ROS_ERROR("[MPPI] ❌ INVALID STATE: grid_map_ is null!");
        return false;
    }
    ROS_INFO("[MPPI] ✅ State check passed: horizon=%d, samples=%d, grid_map valid", 
             horizon_steps_, num_samples_);
    
#ifdef USE_GPU_MPPI
    // 🚀 Use GPU implementation if enabled
    if (use_gpu_ && gpu_planner_) {
        // 🔥 Set EDT map data to GPU before planning
        auto edt_data = grid_map_->getEDTData();
        
        // Convert double* to float* for GPU (CUDA prefers float for performance)
        // Combine positive and negative ESDF into single buffer
        size_t grid_size = edt_data.size_x * edt_data.size_y * edt_data.size_z;
        std::vector<float> edt_float(grid_size);
        
        for (size_t i = 0; i < grid_size; ++i) {
            // Use ESDF buffer directly (already has correct sign)
            double dist = edt_data.esdf_buffer[i];
            if (dist == 0.0 && edt_data.esdf_buffer_neg[i] > 0.0) {
                // Inside obstacle
                dist = -edt_data.esdf_buffer_neg[i];
            }
            edt_float[i] = static_cast<float>(dist);
        }
        
        gpu_planner_->setEDTMap(edt_float.data(),
                                edt_data.size_x, edt_data.size_y, edt_data.size_z,
                                static_cast<float>(edt_data.resolution),
                                static_cast<float>(edt_data.origin_x),
                                static_cast<float>(edt_data.origin_y),
                                static_cast<float>(edt_data.origin_z));
        
        std::vector<Eigen::Vector3d> path;
        bool success = gpu_planner_->plan(start_pos, start_vel, goal_pos, goal_vel, path);
        
        if (success && !path.empty()) {
            optimal_trajectory.resize(path.size());
            optimal_trajectory.positions = path;
            
            // Compute velocities from positions (simple finite difference)
            for (size_t i = 0; i < path.size(); ++i) {
                if (i == 0) {
                    optimal_trajectory.velocities[i] = start_vel;
                } else {
                    optimal_trajectory.velocities[i] = (path[i] - path[i-1]) / dt_;
                }
                optimal_trajectory.accelerations[i] = Eigen::Vector3d::Zero();
            }
            
            // ✅ Phase 2.5B: 获取实际cost而非硬编码为0
            optimal_trajectory.cost = gpu_planner_->getLastBestCost();
            ROS_INFO("[MPPI] 🎯 Phase 2.5B: Retrieved cost=%.2f from GPU planner", optimal_trajectory.cost);
            return true;
        }
        return false;
    }
#endif
    
    // CPU fallback or default implementation
    // ⏱️ Performance monitoring
    auto t_start = std::chrono::high_resolution_clock::now();
    
    // 🔍 Debug: Check start position validity
    if (grid_map_) {
        double start_dist = grid_map_->getDistance(start_pos);
        if (start_dist < 0.0) {
            ROS_WARN("[MPPI] Start position is in collision! dist=%.3f", start_dist);
        } else if (start_dist < 0.3) {
            ROS_WARN("[MPPI] Start position too close to obstacle! dist=%.3f", start_dist);
        }
    }
    
    // Adaptive sampling: adjust number of samples based on environment complexity
    int adaptive_samples = computeAdaptiveSamples(start_pos, goal_pos);
    last_timing_.num_samples = adaptive_samples;
    
    // 🔍 Memory monitoring: estimate memory usage
    size_t traj_memory = sizeof(MPPITrajectory) * adaptive_samples;
    size_t vec_memory = sizeof(Vector3d) * horizon_steps_ * adaptive_samples * 3;  // positions, velocities, accelerations
    size_t total_memory_mb = (traj_memory + vec_memory) / (1024 * 1024);
    ROS_INFO("[MPPI] Memory estimate: %zu MB for %d trajectories (horizon=%d)", 
             total_memory_mb, adaptive_samples, horizon_steps_);
    
    vector<MPPITrajectory> trajectories(adaptive_samples);
    double min_cost = std::numeric_limits<double>::max();
    
    // 🚀 Generate rollout trajectories in PARALLEL
    auto t_rollout_start = std::chrono::high_resolution_clock::now();
    
    #pragma omp parallel
    {
        // Thread-local random number generator for thread safety
        std::mt19937 local_gen(generator_() + omp_get_thread_num());
        std::normal_distribution<double> local_dist(0.0, 1.0);
        
        #pragma omp for reduction(min:min_cost)
        for (int i = 0; i < adaptive_samples; ++i) {
            trajectories[i].resize(horizon_steps_);
            rolloutTrajectory(start_pos, start_vel, goal_pos, goal_vel, trajectories[i], local_gen, local_dist);
            
            double cost = calculateTrajectoryCost(trajectories[i], goal_pos, goal_vel);
            trajectories[i].cost = cost;
            
            if (cost < min_cost) {
                min_cost = cost;
            }
        }
    }
    
    auto t_rollout_end = std::chrono::high_resolution_clock::now();
    last_timing_.rollout_time = std::chrono::duration<double, std::milli>(t_rollout_end - t_rollout_start).count();
    
    // 🔍 Debug: Count how many trajectories have finite cost
    int valid_count = 0;
    for (const auto& traj : trajectories) {
        if (traj.cost < std::numeric_limits<double>::max()) {
            valid_count++;
        }
    }
    ROS_INFO("[MPPI] Valid trajectories: %d/%d, min_cost=%.2f", 
             valid_count, adaptive_samples, min_cost);
    
    if (min_cost >= std::numeric_limits<double>::max()) {
        ROS_WARN("[MPPI] All trajectories have infinite cost");
        return false;
    }
    
    // ✨ Improved importance weight calculation (inspired by MPPI-Generic)
    auto t_weight_start = std::chrono::high_resolution_clock::now();
    
    double weight_sum = 0.0;
    double max_exp_arg = -1e10;  // For numerical stability
    
    // First pass: find max exp argument to avoid overflow
    for (const auto& traj : trajectories) {
        double exp_arg = -(traj.cost - min_cost) / lambda_;
        if (exp_arg > max_exp_arg) {
            max_exp_arg = exp_arg;
        }
    }
    
    // Second pass: compute weights with numerical stability
    for (auto& traj : trajectories) {
        double exp_arg = -(traj.cost - min_cost) / lambda_;
        traj.weight = std::exp(exp_arg - max_exp_arg);  // Subtract max for stability
        weight_sum += traj.weight;
        
        // NaN check (inspired by MPPI-Generic)
        if (std::isnan(traj.weight)) {
            ROS_WARN("[MPPI] NaN weight detected, setting to zero");
            traj.weight = 0.0;
        }
    }
    
    // Normalize weights
    if (weight_sum > 1e-8) {
        for (auto& traj : trajectories) {
            traj.weight /= weight_sum;
        }
    } else {
        ROS_WARN("[MPPI] Weight sum too small (%.2e), using uniform weights", weight_sum);
        for (auto& traj : trajectories) {
            traj.weight = 1.0 / adaptive_samples;
        }
    }
    
    auto t_weight_end = std::chrono::high_resolution_clock::now();
    last_timing_.weight_time = std::chrono::duration<double, std::milli>(t_weight_end - t_weight_start).count();
    
    // Compute weighted average trajectory
    auto t_average_start = std::chrono::high_resolution_clock::now();
    optimal_trajectory = weightedAverage(trajectories);
    auto t_average_end = std::chrono::high_resolution_clock::now();
    last_timing_.average_time = std::chrono::duration<double, std::milli>(t_average_end - t_average_start).count();
    
    // Visualize trajectories
    visualizeTrajectories(trajectories);
    visualizeOptimalTrajectory(optimal_trajectory);
    
    // ⏱️ Total timing
    auto t_end = std::chrono::high_resolution_clock::now();
    last_timing_.total_time = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    ROS_INFO("[MPPI] ⏱️ Timing: Total=%.2fms (Rollout=%.2fms, Weight=%.2fms, Avg=%.2fms) | Samples=%d | Cost=%.2f",
             last_timing_.total_time, last_timing_.rollout_time, last_timing_.weight_time, 
             last_timing_.average_time, adaptive_samples, optimal_trajectory.cost);
    
    return true;
}

// ✅ NEW: planTrajectory with initial path guidance
bool MPPIPlanner::planTrajectory(const Vector3d& start_pos,
                                const Vector3d& start_vel,
                                const Vector3d& goal_pos,
                                const Vector3d& goal_vel,
                                const vector<Vector3d>& initial_path,
                                MPPITrajectory& optimal_trajectory) {
    
    // 🔒 CRITICAL: Validate object state before planning
    if (horizon_steps_ <= 0 || horizon_steps_ > 100) {
        ROS_ERROR("[MPPI] ❌ INVALID STATE: horizon_steps_=%d (expected 10-30). Object not properly initialized!", 
                  horizon_steps_);
        return false;
    }
    if (!grid_map_) {
        ROS_ERROR("[MPPI] ❌ INVALID STATE: grid_map_ is null!");
        return false;
    }
    ROS_INFO("[MPPI] ✅ State check passed: horizon=%d, samples=%d, grid_map valid", 
             horizon_steps_, num_samples_);
    
#ifdef USE_GPU_MPPI
    // 🚀 Use GPU implementation if enabled
    if (use_gpu_ && gpu_planner_) {
        // 🔥 Set EDT map data (same as above)
        auto edt_data = grid_map_->getEDTData();
        size_t grid_size = edt_data.size_x * edt_data.size_y * edt_data.size_z;
        std::vector<float> edt_float(grid_size);
        
        for (size_t i = 0; i < grid_size; ++i) {
            double dist = edt_data.esdf_buffer[i];
            if (dist == 0.0 && edt_data.esdf_buffer_neg[i] > 0.0) {
                dist = -edt_data.esdf_buffer_neg[i];
            }
            edt_float[i] = static_cast<float>(dist);
        }
        
        gpu_planner_->setEDTMap(edt_float.data(),
                                edt_data.size_x, edt_data.size_y, edt_data.size_z,
                                static_cast<float>(edt_data.resolution),
                                static_cast<float>(edt_data.origin_x),
                                static_cast<float>(edt_data.origin_y),
                                static_cast<float>(edt_data.origin_z));
        
        std::vector<Eigen::Vector3d> path;
        bool success = gpu_planner_->plan(start_pos, start_vel, goal_pos, goal_vel, path);
        
        if (success && !path.empty()) {
            optimal_trajectory.resize(path.size());
            optimal_trajectory.positions = path;
            
            // Compute velocities from positions
            for (size_t i = 0; i < path.size(); ++i) {
                if (i == 0) {
                    optimal_trajectory.velocities[i] = start_vel;
                } else {
                    optimal_trajectory.velocities[i] = (path[i] - path[i-1]) / dt_;
                }
                optimal_trajectory.accelerations[i] = Eigen::Vector3d::Zero();
            }
            
            // ✅ Phase 2.5B: 获取实际cost而非硬编码为0
            optimal_trajectory.cost = gpu_planner_->getLastBestCost();
            return true;
        }
        return false;
    }
#endif
    
    // CPU fallback implementation
    // 🔍 Debug: Check start position validity
    if (grid_map_) {
        double start_dist = grid_map_->getDistance(start_pos);
        if (start_dist < 0.0) {
            ROS_WARN("[MPPI] Start position is in collision! dist=%.3f", start_dist);
        } else if (start_dist < 0.3) {
            ROS_WARN("[MPPI] Start position too close to obstacle! dist=%.3f", start_dist);
        }
    }
    
    // Adaptive sampling: adjust number of samples based on environment complexity
    int adaptive_samples = computeAdaptiveSamples(start_pos, goal_pos);
    
    // 🔍 Memory monitoring: estimate memory usage
    size_t traj_memory = sizeof(MPPITrajectory) * adaptive_samples;
    size_t vec_memory = sizeof(Vector3d) * horizon_steps_ * adaptive_samples * 3;  // positions, velocities, accelerations
    size_t total_memory_mb = (traj_memory + vec_memory) / (1024 * 1024);
    ROS_INFO("[MPPI-Guided] Memory estimate: %zu MB for %d trajectories (horizon=%d)", 
             total_memory_mb, adaptive_samples, horizon_steps_);
    
    // 🚀 ITERATIVE MPPI: Initialize control distribution
    if (use_iterative_mppi_ && num_iterations_ > 1) {
        control_distribution_.resize(horizon_steps_);
        // Initialize mean to zero, std to 1.0 (will be overridden in first iteration)
        for (int t = 0; t < horizon_steps_; ++t) {
            control_distribution_.mean_control[t].setZero();
            control_distribution_.std_control[t] = Eigen::Vector3d::Ones();
        }
    }
    
    vector<MPPITrajectory> trajectories(adaptive_samples);
    double min_cost = std::numeric_limits<double>::max();
    MPPITrajectory best_trajectory_overall;
    best_trajectory_overall.cost = std::numeric_limits<double>::max();
    
    // 🚀 ITERATIVE MPPI LOOP (Algorithm 1 from MPPI-Generic)
    int num_iters = use_iterative_mppi_ ? num_iterations_ : 1;
    for (int iter = 0; iter < num_iters; ++iter) {
        // 🔥 Temperature annealing: λ_k = λ₀ * (1 - k/K)^2
        double lambda_k = lambda_;
        if (use_iterative_mppi_ && num_iterations_ > 1) {
            double ratio = 1.0 - (double)iter / num_iterations_;
            lambda_k = lambda_ * ratio * ratio;  // Quadratic decay
            if (lambda_k < 0.1) lambda_k = 0.1;  // Minimum temperature
        }
        
        min_cost = std::numeric_limits<double>::max();
        
        // 🚀 Generate rollout trajectories guided by initial path in PARALLEL
    #pragma omp parallel
    {
        // Thread-local random number generator for thread safety
        std::mt19937 local_gen(generator_() + omp_get_thread_num());
        std::normal_distribution<double> local_dist(0.0, 1.0);
        
        #pragma omp for reduction(min:min_cost)
        for (int i = 0; i < adaptive_samples; ++i) {
            trajectories[i].resize(horizon_steps_);
            rolloutTrajectory(start_pos, start_vel, goal_pos, goal_vel, initial_path, trajectories[i], local_gen, local_dist);
            
            double cost = calculateTrajectoryCost(trajectories[i], goal_pos, goal_vel);
            trajectories[i].cost = cost;
            
            if (cost < min_cost) {
                min_cost = cost;
            }
        }
    }
    
    // 🔍 Debug: Count valid trajectories
    int valid_count = 0;
    for (const auto& traj : trajectories) {
        if (traj.cost < std::numeric_limits<double>::max()) {
            valid_count++;
        }
    }
    
    // 🚀 Improved logging for iterative MPPI
    if (use_iterative_mppi_ && num_iterations_ > 1) {
        ROS_INFO("[MPPI] Iteration %d/%d: Valid=%d/%d, min_cost=%.2f, lambda=%.2f", 
                 iter+1, num_iters, valid_count, adaptive_samples, min_cost, lambda_k);
    } else {
        ROS_INFO("[MPPI] Valid guided trajectories: %d/%d, min_cost=%.2f", 
                 valid_count, adaptive_samples, min_cost);
    }
    
    if (min_cost >= std::numeric_limits<double>::max()) {
        ROS_WARN("[MPPI] All guided trajectories have infinite cost at iteration %d", iter);
        if (iter == 0) {
            return false;  // First iteration failed completely
        } else {
            break;  // Use best from previous iterations
        }
    }
    
    // Calculate importance weights with current lambda_k
    double weight_sum = 0.0;
    for (auto& traj : trajectories) {
        traj.weight = exp(-(traj.cost - min_cost) / lambda_k);
        weight_sum += traj.weight;
    }
    
    // Normalize weights
    if (weight_sum > 1e-8) {
        for (auto& traj : trajectories) {
            traj.weight /= weight_sum;
        }
    } else {
        ROS_WARN("[MPPI] Weight sum too small, using uniform weights");
        for (auto& traj : trajectories) {
            traj.weight = 1.0 / adaptive_samples;
        }
    }
    
    // 🚀 ITERATIVE MPPI: Update control distribution for next iteration
    if (use_iterative_mppi_ && iter < num_iters - 1) {
        ControlDistribution new_distribution;
        std::vector<Eigen::Vector3d> dummy_controls;  // Not used in current implementation
        updateControlDistribution(trajectories, dummy_controls, new_distribution);
        control_distribution_ = new_distribution;
    }
    
    // Track best trajectory across all iterations
    MPPITrajectory current_best = weightedAverage(trajectories);
    if (current_best.cost < best_trajectory_overall.cost) {
        best_trajectory_overall = current_best;
    }
    
    } // End of iteration loop
    
    // Use best trajectory found across all iterations
    optimal_trajectory = best_trajectory_overall;
    
    // Visualize
    visualizeTrajectories(trajectories);
    visualizeOptimalTrajectory(optimal_trajectory);
    
    if (use_iterative_mppi_ && num_iterations_ > 1) {
        ROS_INFO("[MPPI] Guided trajectory with cost: %.3f (using %zu waypoints, %d samples, %d iterations)", 
                 optimal_trajectory.cost, initial_path.size(), adaptive_samples, num_iterations_);
    } else {
        ROS_INFO("[MPPI] Guided trajectory with cost: %.3f (using %zu waypoints, %d adaptive samples)", 
                 optimal_trajectory.cost, initial_path.size(), adaptive_samples);
    }
    return true;
}

void MPPIPlanner::rolloutTrajectory(const Vector3d& start_pos,
                                   const Vector3d& start_vel,
                                   const Vector3d& goal_pos,
                                   const Vector3d& goal_vel,
                                   MPPITrajectory& trajectory) {
    // ✨ NEW: Use modularized Dynamics
    // Initialize state
    VectorXd state(6);
    state << start_pos, start_vel;
    
    trajectory.positions[0] = start_pos;
    trajectory.velocities[0] = start_vel;
    trajectory.accelerations[0] = Vector3d::Zero();
    
    // Generate noisy control inputs and rollout dynamics
    for (int t = 1; t < horizon_steps_; ++t) {
        // Calculate nominal control towards goal
        Vector3d pos_error = goal_pos - trajectory.positions[t-1];
        Vector3d vel_error = goal_vel - trajectory.velocities[t-1];
        
        // Simple PD control for nominal trajectory
        Vector3d nominal_acc = 2.0 * pos_error + 1.0 * vel_error;
        
        // Sample noisy control using new sampling module
        Vector3d control;
        sampling_.sampleGuidedControl(nominal_acc, control);
        
        // ✨ Use new Dynamics module to enforce constraints and integrate
        dynamics_.enforceConstraints(control);
        
        VectorXd next_state;
        dynamics_.step(state, control, dt_, next_state);
        
        // Extract position and velocity from next state
        trajectory.positions[t] = next_state.head<3>();
        trajectory.velocities[t] = next_state.tail<3>();
        trajectory.accelerations[t] = control;
        
        // Update state for next iteration
        state = next_state;
    }
}

// ✅ NEW: rolloutTrajectory with path guidance
void MPPIPlanner::rolloutTrajectory(const Vector3d& start_pos,
                                   const Vector3d& start_vel,
                                   const Vector3d& goal_pos,
                                   const Vector3d& goal_vel,
                                   const vector<Vector3d>& guide_path,
                                   MPPITrajectory& trajectory) {
    // ✨ NEW: Use modularized Dynamics
    // Initialize state
    VectorXd state(6);
    state << start_pos, start_vel;
    
    trajectory.positions[0] = start_pos;
    trajectory.velocities[0] = start_vel;
    trajectory.accelerations[0] = Vector3d::Zero();
    
    // Precompute path segment lengths for interpolation
    vector<double> segment_lengths;
    double total_length = 0.0;
    if (guide_path.size() >= 2) {
        for (size_t i = 1; i < guide_path.size(); ++i) {
            double len = (guide_path[i] - guide_path[i-1]).norm();
            segment_lengths.push_back(len);
            total_length += len;
        }
    }
    
    // Generate noisy control inputs guided by path
    for (int t = 1; t < horizon_steps_; ++t) {
        // Find target point on guide path based on time
        Vector3d target_point = goal_pos;
        
        if (!guide_path.empty() && total_length > 1e-3) {
            // Interpolate along guide path based on progress ratio
            double progress_ratio = (double)t / horizon_steps_;
            double target_dist = progress_ratio * total_length;
            
            double accumulated_dist = 0.0;
            for (size_t i = 0; i < segment_lengths.size(); ++i) {
                if (accumulated_dist + segment_lengths[i] >= target_dist) {
                    // Interpolate within this segment
                    double local_ratio = (target_dist - accumulated_dist) / segment_lengths[i];
                    target_point = guide_path[i] + local_ratio * (guide_path[i+1] - guide_path[i]);
                    break;
                }
                accumulated_dist += segment_lengths[i];
            }
        }
        
        // Calculate control towards target point (path-guided PD)
        Vector3d pos_error = target_point - trajectory.positions[t-1];
        Vector3d vel_error = goal_vel - trajectory.velocities[t-1];
        
        Vector3d nominal_acc = 2.0 * pos_error + 1.0 * vel_error;
        
        // Sample noisy control using new sampling module
        Vector3d control;
        sampling_.sampleGuidedControl(nominal_acc, control);
        
        // ✨ Use new Dynamics module to enforce constraints and integrate
        dynamics_.enforceConstraints(control);
        
        VectorXd next_state;
        dynamics_.step(state, control, dt_, next_state);
        
        // Extract position and velocity from next state
        trajectory.positions[t] = next_state.head<3>();
        trajectory.velocities[t] = next_state.tail<3>();
        trajectory.accelerations[t] = control;
        
        // Update state for next iteration
        state = next_state;
    }
}

double MPPIPlanner::calculateTrajectoryCost(const MPPITrajectory& trajectory,
                                          const Vector3d& goal_pos,
                                          const Vector3d& goal_vel) {
    // ✨ NEW: Use modularized Cost module with running + terminal separation
    double total_cost = 0.0;
    int collision_count = 0;
    
    // Running costs for each timestep
    for (int t = 0; t < trajectory.size(); ++t) {
        double running_cost = cost_.computeRunningCost(
            trajectory.positions[t],
            trajectory.velocities[t],
            trajectory.accelerations[t],
            goal_pos,
            t,
            nullptr  // No path waypoint guidance in cost (already in rollout)
        );
        
        // 🔧 IMPROVED: Use graduated penalties instead of constant 1e8
        if (std::isinf(running_cost) || running_cost > 1e8) {
            collision_count++;
            // Exponential penalty: 1st collision=1e5, 2nd=2e5, 3rd=4e5...
            total_cost += 1e5 * (1 << std::min(collision_count-1, 5));  // Cap at 2^5=32
        } else {
            total_cost += running_cost;
        }
        
        // Early exit if too many collisions (>5 steps)
        if (collision_count > 5) {
            return 1e8;  // Trajectory clearly bad
        }
    }
    
    // Terminal cost at final state
    int final_idx = trajectory.size() - 1;
    double terminal_cost = cost_.computeTerminalCost(
        trajectory.positions[final_idx],
        trajectory.velocities[final_idx],
        goal_pos,
        goal_vel
    );
    
    total_cost += terminal_cost;
    
    return total_cost;
}

double MPPIPlanner::obstacleCost(const Vector3d& position) {
    // ✅ Phase 3: Use ESDF for O(1) distance query instead of O(n³) sampling
    double dist = grid_map_->getDistance(position);
    return obstacleCost(position, dist);
}

double MPPIPlanner::obstacleCost(const Vector3d& position, double dist) {
    // ✅ Phase 3: ESDF-based obstacle cost - O(1) instead of O(n³)
    // 
    // Previously: Sampled 11×11×11 = 1331 points around position (O(n³))
    // Now: Single ESDF lookup (O(1)) - ~1000x faster!
    //
    // Cost function: Exponentially increases as distance decreases
    // - dist > safety_distance: no cost (0.0)
    // - dist < safety_distance: exponential cost increase
    // - dist < 0: inside obstacle (handled in calculateTrajectoryCost)
    
    const double safety_distance = 1.0;  // Safe distance from obstacles (meters)
    const double cost_scale = 1.0;       // Cost scaling factor
    
    if (dist >= safety_distance) {
        return 0.0;  // Safe distance, no cost
    }
    
    if (dist < 0.0) {
        // Inside obstacle - return very high cost
        // (infinite cost is handled in calculateTrajectoryCost)
        return 1000.0;
    }
    
    // Exponential cost: cost = scale * exp(-dist / sigma)
    // As dist → 0, cost → infinity
    // As dist → safety_distance, cost → 0
    double normalized_dist = dist / safety_distance;
    double cost = cost_scale * std::exp(-normalized_dist * 5.0) / (dist + 0.01);
    
    return cost;
}

double MPPIPlanner::smoothnessCost(const MPPITrajectory& trajectory) {
    double cost = 0.0;
    
    // Acceleration smoothness
    for (int t = 1; t < trajectory.size(); ++t) {
        Vector3d acc_diff = trajectory.accelerations[t] - trajectory.accelerations[t-1];
        cost += acc_diff.squaredNorm();
    }
    
    // Velocity smoothness  
    for (int t = 1; t < trajectory.size(); ++t) {
        Vector3d vel_diff = trajectory.velocities[t] - trajectory.velocities[t-1];
        cost += 0.5 * vel_diff.squaredNorm();
    }
    
    return cost;
}

double MPPIPlanner::goalCost(const MPPITrajectory& trajectory,
                           const Vector3d& goal_pos,
                           const Vector3d& goal_vel) {
    // Terminal state cost
    Vector3d final_pos = trajectory.positions.back();
    Vector3d final_vel = trajectory.velocities.back();
    
    double pos_error = (final_pos - goal_pos).squaredNorm();
    double vel_error = (final_vel - goal_vel).squaredNorm();
    
    return pos_error + 0.5 * vel_error;
}

double MPPIPlanner::velocityCost(const MPPITrajectory& trajectory,
                               const Vector3d& desired_vel) {
    double cost = 0.0;
    
    for (int t = 0; t < trajectory.size(); ++t) {
        Vector3d vel_error = trajectory.velocities[t] - desired_vel;
        cost += vel_error.squaredNorm();
    }
    
    return cost / trajectory.size();
}

void MPPIPlanner::constrainDynamics(Vector3d& velocity, Vector3d& acceleration) {
    // ✨ Use modularized dynamics constraints
    double max_acc = dynamics_.getMaxAcceleration();
    double max_vel = dynamics_.getMaxVelocity();
    
    // Limit acceleration magnitude
    if (acceleration.norm() > max_acc) {
        acceleration = acceleration.normalized() * max_acc;
    }
    
    // Predict next velocity and limit if necessary
    Vector3d next_vel = velocity + acceleration * dt_;
    if (next_vel.norm() > max_vel) {
        next_vel = next_vel.normalized() * max_vel;
        acceleration = (next_vel - velocity) / dt_;
    }
}

MPPITrajectory MPPIPlanner::weightedAverage(const vector<MPPITrajectory>& trajectories) {
    MPPITrajectory avg_trajectory;
    avg_trajectory.resize(horizon_steps_);
    
    // Initialize with zeros
    for (int t = 0; t < horizon_steps_; ++t) {
        avg_trajectory.positions[t] = Vector3d::Zero();
        avg_trajectory.velocities[t] = Vector3d::Zero();
        avg_trajectory.accelerations[t] = Vector3d::Zero();
    }
    
    // Weighted average
    double total_cost = 0.0;
    for (const auto& traj : trajectories) {
        for (int t = 0; t < horizon_steps_; ++t) {
            avg_trajectory.positions[t] += traj.weight * traj.positions[t];
            avg_trajectory.velocities[t] += traj.weight * traj.velocities[t];
            avg_trajectory.accelerations[t] += traj.weight * traj.accelerations[t];
        }
        total_cost += traj.weight * traj.cost;
    }
    
    avg_trajectory.cost = total_cost;
    
    return avg_trajectory;
}

bool MPPIPlanner::planLocalPath(const Vector3d& start_pos,
                               const Vector3d& goal_pos,
                               vector<Vector3d>& path_points) {
    path_points.clear();
    
    // Use a simplified MPPI for local path planning
    Vector3d start_vel = Vector3d::Zero();
    Vector3d goal_vel = Vector3d::Zero();
    
    // Reduce samples and horizon for faster local planning
    int original_samples = num_samples_;
    int original_horizon = horizon_steps_;
    num_samples_ = 200;  // Fewer samples for speed
    horizon_steps_ = 10; // Shorter horizon for local planning
    
    MPPITrajectory local_trajectory;
    bool success = planTrajectory(start_pos, start_vel, goal_pos, goal_vel, local_trajectory);
    
    // Restore original parameters
    num_samples_ = original_samples;
    horizon_steps_ = original_horizon;
    
    if (!success || local_trajectory.positions.empty()) {
        ROS_WARN("[MPPI] Local path planning failed");
        return false;
    }
    
    // Extract path points from trajectory (subsample for efficiency)
    int step = std::max(1, (int)(local_trajectory.positions.size() / 10)); // Max 10 points
    for (size_t i = 0; i < local_trajectory.positions.size(); i += step) {
        path_points.push_back(local_trajectory.positions[i]);
    }
    
    // Always include the goal point
    if (path_points.empty() || (path_points.back() - goal_pos).norm() > 0.1) {
        path_points.push_back(goal_pos);
    }
    
    ROS_DEBUG("[MPPI] Generated local path with %zu points", path_points.size());
    return true;
}

void MPPIPlanner::visualizeTrajectories(const vector<MPPITrajectory>& trajectories) {
    if (trajectories.empty()) return;
    
    ROS_DEBUG("[MPPI] Visualizing %zu sample trajectories with frame_id: %s", trajectories.size(), frame_id_.c_str());
    
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Visualize a subset of sample trajectories (to avoid overwhelming RViz)
    int visualization_step = std::max(1, (int)(trajectories.size() / 50));  // Show at most 50 trajectories
    
    for (size_t i = 0; i < trajectories.size(); i += visualization_step) {
        if (trajectories[i].positions.empty()) continue;
        
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = frame_id_;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "mppi_sample_trajectories";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        
        // Color based on trajectory cost (red = high cost, green = low cost)
        double normalized_cost = trajectories[i].weight; // Use weight for coloring
        line_marker.color.r = 1.0 - normalized_cost;
        line_marker.color.g = normalized_cost;
        line_marker.color.b = 0.2;
        line_marker.color.a = 0.3;  // Make them semi-transparent
        line_marker.scale.x = 0.05;  // Thin lines for sample trajectories
        
        for (const auto& pos : trajectories[i].positions) {
            geometry_msgs::Point p;
            p.x = pos.x();
            p.y = pos.y();
            p.z = pos.z();
            line_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
    }
    
    mppi_trajectories_pub_.publish(marker_array);
    ROS_DEBUG("[MPPI] Published %zu sample trajectory markers", marker_array.markers.size() - 1);
}

void MPPIPlanner::visualizeOptimalTrajectory(const MPPITrajectory& trajectory) {
    if (trajectory.positions.empty()) return;
    
    ROS_DEBUG("[MPPI] Visualizing optimal trajectory with frame_id: %s", frame_id_.c_str());
    
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Optimal trajectory line
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id_;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "mppi_optimal_trajectory";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    
    // Bright orange for optimal trajectory
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.5;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;
    line_marker.scale.x = 0.15;  // Thicker line for optimal trajectory
    
    for (const auto& pos : trajectory.positions) {
        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = pos.z();
        line_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(line_marker);
    
    // Add velocity vectors as arrows (optional, show every few steps)
    int arrow_step = std::max(1, horizon_steps_ / 5);  // Show 5 arrows max
    for (int i = 0; i < trajectory.size(); i += arrow_step) {
        if (trajectory.velocities[i].norm() < 0.1) continue;  // Skip very small velocities
        
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header.frame_id = frame_id_;
        arrow_marker.header.stamp = ros::Time::now();
        arrow_marker.ns = "mppi_velocity_arrows";
        arrow_marker.id = i;
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.action = visualization_msgs::Marker::ADD;
        
        // Arrow position
        arrow_marker.pose.position.x = trajectory.positions[i].x();
        arrow_marker.pose.position.y = trajectory.positions[i].y();
        arrow_marker.pose.position.z = trajectory.positions[i].z();
        
        // Arrow orientation (pointing in velocity direction)
        Vector3d vel_normalized = trajectory.velocities[i].normalized();
        double yaw = atan2(vel_normalized.y(), vel_normalized.x());
        double pitch = asin(vel_normalized.z());
        
        arrow_marker.pose.orientation.x = 0;
        arrow_marker.pose.orientation.y = sin(pitch/2);
        arrow_marker.pose.orientation.z = sin(yaw/2) * cos(pitch/2);
        arrow_marker.pose.orientation.w = cos(yaw/2) * cos(pitch/2);
        
        // Arrow scale based on velocity magnitude
        double vel_mag = trajectory.velocities[i].norm();
        arrow_marker.scale.x = vel_mag * 0.5;  // Arrow length
        arrow_marker.scale.y = 0.05;  // Arrow width
        arrow_marker.scale.z = 0.05;  // Arrow height
        
        // Blue color for velocity arrows
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 0.3;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 0.7;
        
        marker_array.markers.push_back(arrow_marker);
    }
    
    optimal_trajectory_pub_.publish(marker_array);
    ROS_DEBUG("[MPPI] Published optimal trajectory with %zu markers", marker_array.markers.size() - 1);
}

// Adaptive sampling: compute number of samples based on environment complexity
int MPPIPlanner::computeAdaptiveSamples(const Vector3d& start_pos, const Vector3d& goal_pos) {
    // 🔒 Safety check: ensure grid_map is initialized
    if (!grid_map_) {
        ROS_WARN("[MPPI] Grid map not initialized, using default samples=%d", num_samples_);
        return num_samples_;
    }
    
    if (!use_adaptive_sampling_) {
        return num_samples_;  // Use fixed number if adaptive sampling is disabled
    }
    
    // Sample environment complexity along straight line from start to goal
    Vector3d direction = (goal_pos - start_pos).normalized();
    double distance = (goal_pos - start_pos).norm();
    
    int num_checks = std::min(20, static_cast<int>(distance / 0.5));  // Check every 0.5m, max 20 points
    if (num_checks < 5) num_checks = 5;  // At least 5 checks
    
    double avg_clearance = 0.0;
    int valid_checks = 0;
    
    for (int i = 0; i < num_checks; ++i) {
        double t = static_cast<double>(i) / (num_checks - 1);
        Vector3d check_pos = start_pos + direction * distance * t;
        
        double dist = grid_map_->getDistance(check_pos);
        if (dist >= 0.0) {  // Valid check (not inside obstacle)
            avg_clearance += dist;
            valid_checks++;
        }
    }
    
    if (valid_checks == 0) {
        // Very cluttered environment, use maximum samples
        ROS_DEBUG("[MPPI] Adaptive sampling: cluttered environment, using max samples %d", num_samples_max_);
        return num_samples_max_;
    }
    
    avg_clearance /= valid_checks;
    
    // Adaptive sampling logic:
    // - High clearance (> 3m): fewer samples needed (min_samples)
    // - Low clearance (< 1m): more samples needed (max_samples)
    // - Medium clearance: interpolate
    
    int adaptive_samples;
    if (avg_clearance >= 3.0) {
        adaptive_samples = num_samples_min_;  // Open space, use minimum
    } else if (avg_clearance <= 1.0) {
        adaptive_samples = num_samples_max_;  // Cluttered space, use maximum
    } else {
        // Linear interpolation between min and max
        double ratio = (3.0 - avg_clearance) / 2.0;  // ratio ∈ [0, 1]
        adaptive_samples = num_samples_min_ + 
                          static_cast<int>(ratio * (num_samples_max_ - num_samples_min_));
    }
    
    ROS_DEBUG("[MPPI] Adaptive sampling: avg_clearance=%.2fm, samples=%d (min=%d, max=%d)", 
              avg_clearance, adaptive_samples, num_samples_min_, num_samples_max_);
    
    return adaptive_samples;
}

// 🚀 Thread-safe rolloutTrajectory with local random generators (for OpenMP)
void MPPIPlanner::rolloutTrajectory(const Vector3d& start_pos,
                                   const Vector3d& start_vel,
                                   const Vector3d& goal_pos,
                                   const Vector3d& goal_vel,
                                   MPPITrajectory& trajectory,
                                   std::mt19937& local_gen,
                                   std::normal_distribution<double>& local_dist) {
    // ✨ NEW: Use modularized Dynamics (thread-safe version)
    VectorXd state(6);
    state << start_pos, start_vel;
    
    trajectory.positions[0] = start_pos;
    trajectory.velocities[0] = start_vel;
    trajectory.accelerations[0] = Vector3d::Zero();
    
    for (int t = 1; t < horizon_steps_; ++t) {
        // Calculate nominal control towards goal
        Vector3d pos_error = goal_pos - trajectory.positions[t-1];
        Vector3d vel_error = goal_vel - trajectory.velocities[t-1];
        
        // Simple PD control for nominal trajectory
        Vector3d nominal_acc = 2.0 * pos_error + 1.0 * vel_error;
        
        // Sample noisy control using local thread-safe generator
        Vector3d control;
        sampling_.sampleGuidedControl(nominal_acc, control, local_gen, local_dist);
        
        // Integrate using new dynamics module
        VectorXd next_state;
        dynamics_.enforceConstraints(control);
        dynamics_.step(state, control, dt_, next_state);
        
        trajectory.positions[t] = next_state.head<3>();
        trajectory.velocities[t] = next_state.tail<3>();
        trajectory.accelerations[t] = control;
        
        state = next_state;
    }
}

// 🚀 Thread-safe rolloutTrajectory with path guidance and local random generators
void MPPIPlanner::rolloutTrajectory(const Vector3d& start_pos,
                                   const Vector3d& start_vel,
                                   const Vector3d& goal_pos,
                                   const Vector3d& goal_vel,
                                   const vector<Vector3d>& guide_path,
                                   MPPITrajectory& trajectory,
                                   std::mt19937& local_gen,
                                   std::normal_distribution<double>& local_dist) {
    // ✨ NEW: Use modularized Dynamics (thread-safe version with guidance)
    VectorXd state(6);
    state << start_pos, start_vel;
    
    trajectory.positions[0] = start_pos;
    trajectory.velocities[0] = start_vel;
    trajectory.accelerations[0] = Vector3d::Zero();
    
    // 🔍 Debug: Check guide_path validity
    if (guide_path.empty()) {
        ROS_WARN_THROTTLE(1.0, "[MPPI] ❌ Guide path is EMPTY! Using goal-directed rollout");
        // Use simple goal-directed rollout instead
        for (int t = 1; t < horizon_steps_; ++t) {
            Vector3d pos_error = goal_pos - trajectory.positions[t-1];
            Vector3d vel_error = goal_vel - trajectory.velocities[t-1];
            Vector3d nominal_acc = 2.0 * pos_error + 1.0 * vel_error;
            
            Vector3d control;
            sampling_.sampleGuidedControl(nominal_acc, control, local_gen, local_dist);
            
            VectorXd next_state;
            dynamics_.enforceConstraints(control);
            dynamics_.step(state, control, dt_, next_state);
            
            trajectory.positions[t] = next_state.head<3>();
            trajectory.velocities[t] = next_state.tail<3>();
            trajectory.accelerations[t] = control;
            state = next_state;
        }
        return;
    }
    
    // Precompute path segment lengths for interpolation
    vector<double> segment_lengths;
    double total_length = 0.0;
    if (guide_path.size() >= 2) {
        for (size_t i = 1; i < guide_path.size(); ++i) {
            double len = (guide_path[i] - guide_path[i-1]).norm();
            segment_lengths.push_back(len);
            total_length += len;
        }
    }
    
    // Generate noisy control inputs guided by path
    for (int t = 1; t < horizon_steps_; ++t) {
        // Interpolate target position along guide path based on progress
        double progress = (double)t / horizon_steps_;
        double target_dist = progress * total_length;
        
        Vector3d target_pos = goal_pos;
        if (!segment_lengths.empty()) {
            double cumulative_len = 0.0;
            for (size_t i = 0; i < segment_lengths.size(); ++i) {
                if (cumulative_len + segment_lengths[i] >= target_dist) {
                    double ratio = (target_dist - cumulative_len) / segment_lengths[i];
                    target_pos = guide_path[i] + ratio * (guide_path[i+1] - guide_path[i]);
                    break;
                }
                cumulative_len += segment_lengths[i];
            }
        }
        
        // Calculate control towards interpolated target
        Vector3d pos_error = target_pos - trajectory.positions[t-1];
        Vector3d vel_error = goal_vel - trajectory.velocities[t-1];
        
        // PD control with path guidance
        Vector3d nominal_acc = 2.0 * pos_error + 1.0 * vel_error;
        
        // Sample noisy control using local thread-safe generator
        Vector3d control;
        sampling_.sampleGuidedControl(nominal_acc, control, local_gen, local_dist);
        
        // Integrate using new dynamics module
        VectorXd next_state;
        dynamics_.enforceConstraints(control);
        dynamics_.step(state, control, dt_, next_state);
        
        trajectory.positions[t] = next_state.head<3>();
        trajectory.velocities[t] = next_state.tail<3>();
        trajectory.accelerations[t] = control;
        
        state = next_state;
    }
}

// 🚀 NEW: Update control distribution based on weighted trajectory samples (MPPI-Generic Algorithm 1)
void MPPIPlanner::updateControlDistribution(const std::vector<MPPITrajectory>& trajectories,
                                           const std::vector<Eigen::Vector3d>& control_samples,
                                           ControlDistribution& new_distribution) {
    int horizon = control_distribution_.mean_control.size();
    new_distribution.resize(horizon);
    
    // Initialize to zero
    for (int t = 0; t < horizon; ++t) {
        new_distribution.mean_control[t].setZero();
        new_distribution.std_control[t].setZero();
    }
    
    // Compute weighted mean: μ_{k+1} = Σ w_i * u_i
    for (size_t i = 0; i < trajectories.size(); ++i) {
        for (int t = 0; t < horizon && t < (int)trajectories[i].accelerations.size(); ++t) {
            new_distribution.mean_control[t] += trajectories[i].weight * trajectories[i].accelerations[t];
        }
    }
    
    // Compute weighted std dev: σ_{k+1} = sqrt(Σ w_i * (u_i - μ_{k+1})^2)
    for (size_t i = 0; i < trajectories.size(); ++i) {
        for (int t = 0; t < horizon && t < (int)trajectories[i].accelerations.size(); ++t) {
            Eigen::Vector3d diff = trajectories[i].accelerations[t] - new_distribution.mean_control[t];
            new_distribution.std_control[t] += trajectories[i].weight * diff.cwiseProduct(diff);
        }
    }
    
    // Take square root to get std dev
    for (int t = 0; t < horizon; ++t) {
        new_distribution.std_control[t] = new_distribution.std_control[t].cwiseSqrt();
        
        // 🔧 Ensure minimum std dev (exploration)
        for (int dim = 0; dim < 3; ++dim) {
            if (new_distribution.std_control[t](dim) < 0.1) {
                new_distribution.std_control[t](dim) = 0.1;
            }
        }
    }
}

// 🚀 NEW: Sample control sequence from distribution
void MPPIPlanner::sampleControlFromDistribution(const ControlDistribution& distribution,
                                               std::vector<Eigen::Vector3d>& control_sequence,
                                               std::mt19937& local_gen,
                                               std::normal_distribution<double>& local_dist) {
    int horizon = distribution.mean_control.size();
    control_sequence.resize(horizon);
    
    for (int t = 0; t < horizon; ++t) {
        // Sample from N(μ_t, Σ_t)
        Eigen::Vector3d noise;
        noise.x() = local_dist(local_gen);
        noise.y() = local_dist(local_gen);
        noise.z() = local_dist(local_gen);
        
        control_sequence[t] = distribution.mean_control[t] + 
                             distribution.std_control[t].cwiseProduct(noise);
    }
}

} // namespace ego_planner
