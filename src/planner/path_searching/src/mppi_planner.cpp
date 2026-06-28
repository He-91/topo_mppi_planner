#include "ddo_path_searching/mppi_planner.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <omp.h>

#ifdef USE_GPU_MPPI
#include <ddo_path_searching/mppi_gpu_planner.h>
#endif

using namespace std;
using namespace Eigen;

namespace ego_planner {

MPPIPlanner::MPPIPlanner() 
#ifdef USE_GPU_MPPI
    : use_gpu_(true), force_cpu_(false),
#else
    :
#endif
      num_samples_(500), num_samples_min_(300), num_samples_max_(800),
      use_adaptive_sampling_(true), horizon_steps_(15), dt_(0.1), lambda_(1.0),
      num_iterations_(1), use_iterative_mppi_(false),
      use_batch_iterative_update_(false),
      gpu_random_seed_(0),
      generator_(std::random_device{}()), normal_dist_(0.0, 1.0)
{
    dynamics_.setMaxVelocity(3.0);
    dynamics_.setMaxAcceleration(3.0);
    
    // Cost weights are defined in MPPICost constructor (single source of truth)
    // Do NOT override them here to avoid inconsistency
    
    sampling_.setSigma(1.0);
    sampling_.setUseColoredNoise(true);
    sampling_.setTemporalCorrelation(0.7);
    sampling_.initialize();
    
    control_distribution_.resize(horizon_steps_);
}

MPPIPlanner::~MPPIPlanner() {
}

void MPPIPlanner::setVehicleLimits(double max_vel, double max_acc) {
    dynamics_.setMaxVelocity(max_vel);
    dynamics_.setMaxAcceleration(max_acc);
#ifdef USE_GPU_MPPI
    if (gpu_planner_) {
        const double gpu_max_vel = gpu_nominal_max_vel_ > 0.0 ? gpu_nominal_max_vel_ : max_vel;
        const double gpu_max_acc = gpu_nominal_max_acc_ > 0.0 ? gpu_nominal_max_acc_ : max_acc;
        gpu_planner_->setVehicleLimits(static_cast<float>(gpu_max_vel), static_cast<float>(gpu_max_acc));
    }
#endif
}

void MPPIPlanner::setVehicleLimitsZ(double max_vel_z, double max_acc_z) {
    dynamics_.setMaxVelocityZ(max_vel_z);
    dynamics_.setMaxAccelerationZ(max_acc_z);
#ifdef USE_GPU_MPPI
    if (gpu_planner_) {
        gpu_planner_->setVehicleLimitsZ(static_cast<float>(max_vel_z), static_cast<float>(max_acc_z));
    }
#endif
}

void MPPIPlanner::setDynamicObstacles(const std::vector<Eigen::Vector3d>& positions,
                                     const std::vector<float>& radii,
                                     const std::vector<float>& heights,
                                     int num_obstacles,
                                     int horizon,
                                     float dt) {
#ifdef USE_GPU_MPPI
    if (use_gpu_ && gpu_planner_) {
        gpu_planner_->setDynamicObstacles(positions, radii, heights, num_obstacles, horizon, dt);
        ROS_INFO_THROTTLE(2.0, "[MPPI]  Set %d dynamic obstacles on GPU (horizon=%d, dt=%.2f)", 
                         num_obstacles, horizon, dt);
    }
#endif
    // Also update CPU cost function with prediction data
    // Reshape flat positions into per-obstacle predicted positions
    std::vector<std::vector<Eigen::Vector3d>> per_obstacle_predictions(num_obstacles);
    for (int i = 0; i < num_obstacles; ++i) {
        per_obstacle_predictions[i].resize(horizon);
        for (int t = 0; t < horizon; ++t) {
            int idx = i * horizon + t;
            if (idx < (int)positions.size()) {
                per_obstacle_predictions[i][t] = positions[idx];
            }
        }
    }
    cost_.setDynamicObstaclePredictions(per_obstacle_predictions, horizon, (double)dt);
    
    // Set current positions (first timestep of each obstacle)
    std::vector<Eigen::Vector3d> current_pos(num_obstacles);
    std::vector<Eigen::Vector3d> vels(num_obstacles, Eigen::Vector3d::Zero());
    for (int i = 0; i < num_obstacles; ++i) {
        if (i * horizon < (int)positions.size()) {
            current_pos[i] = positions[i * horizon];
            // Estimate velocity from first two prediction steps
            if (horizon > 1 && (i * horizon + 1) < (int)positions.size() && dt > 0.0f) {
                vels[i] = (positions[i * horizon + 1] - positions[i * horizon]) / (double)dt;
            }
        }
    }
    cost_.setDynamicObstacles(current_pos, vels, radii, heights);
}

void MPPIPlanner::setDynamicObstaclesCurrent(const std::vector<Eigen::Vector3d>& current_positions,
                                            const std::vector<Eigen::Vector3d>& velocities,
                                            const std::vector<float>& radii) {
    cost_.setDynamicObstacles(current_positions, velocities, radii);
    ROS_INFO_THROTTLE(2.0, "[MPPI]  Set %zu dynamic obstacles on CPU cost", current_positions.size());
}

void MPPIPlanner::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    cost_.setGridMap(grid_map);
    
    // Read MPPI parameters from ROS parameter server
    nh.param("mppi/horizon_steps", horizon_steps_, 20);
    nh.param("mppi/dt", dt_, 0.1);
    nh.param("mppi/num_samples", num_samples_, 500);
    nh.param("mppi/num_samples_min", num_samples_min_, 300);
    nh.param("mppi/num_samples_max", num_samples_max_, 800);
    nh.param("mppi/use_adaptive_sampling", use_adaptive_sampling_, true);
    nh.param("mppi/lambda", lambda_, 1.0);
    nh.param("mppi/use_iterative_mppi", use_iterative_mppi_, false);
    nh.param("mppi/num_iterations", num_iterations_, 1);
    nh.param("mppi/use_batch_iterative_update", use_batch_iterative_update_, use_batch_iterative_update_);
    nh.param("mppi/gpu_random_seed", gpu_random_seed_, gpu_random_seed_);
    nh.param("mppi/gpu_nominal_max_vel", gpu_nominal_max_vel_, gpu_nominal_max_vel_);
    nh.param("mppi/gpu_nominal_max_acc", gpu_nominal_max_acc_, gpu_nominal_max_acc_);

    double w_obstacle = cost_.getObstacleWeight();
    double w_dynamic = cost_.getDynamicWeight();
    double w_smoothness = cost_.getSmoothnessWeight();
    double w_goal = cost_.getGoalWeight();
    double w_velocity = cost_.getVelocityWeight();
    double w_path_guidance = cost_.getPathGuidanceWeight();
    double safe_distance = cost_.getSafeDistance();
    double dynamic_safe_distance = cost_.getDynamicSafeDistance();
    double dynamic_collision_distance = cost_.getDynamicCollisionDistance();
    double desired_velocity = cost_.getDesiredVelocity();
    double near_collision_distance = cost_.getNearCollisionDistance();
    double near_collision_weight = cost_.getNearCollisionWeight();
    double sigma_acc = sampling_.getSigma();
    bool use_colored_noise = sampling_.getUseColoredNoise();
    double temporal_correlation = sampling_.getTemporalCorrelation();

    nh.param("mppi/w_obstacle", w_obstacle, w_obstacle);
    nh.param("mppi/w_dynamic", w_dynamic, w_dynamic);
    nh.param("mppi/w_smoothness", w_smoothness, w_smoothness);
    nh.param("mppi/w_goal", w_goal, w_goal);
    nh.param("mppi/w_velocity", w_velocity, w_velocity);
    nh.param("mppi/w_path_guidance", w_path_guidance, w_path_guidance);
    nh.param("mppi/safe_distance", safe_distance, safe_distance);
    nh.param("mppi/dynamic_safe_distance", dynamic_safe_distance, dynamic_safe_distance);
    nh.param("mppi/dynamic_collision_distance", dynamic_collision_distance, dynamic_collision_distance);
    nh.param("mppi/desired_velocity", desired_velocity, desired_velocity);
    nh.param("mppi/near_collision_distance", near_collision_distance, near_collision_distance);
    nh.param("mppi/near_collision_weight", near_collision_weight, near_collision_weight);
    nh.param("mppi/sigma_acc", sigma_acc, sigma_acc);
    nh.param("mppi/use_colored_noise", use_colored_noise, use_colored_noise);
    nh.param("mppi/temporal_correlation", temporal_correlation, temporal_correlation);

    cost_.setObstacleWeight(w_obstacle);
    cost_.setDynamicWeight(w_dynamic);
    cost_.setSmoothnessWeight(w_smoothness);
    cost_.setGoalWeight(w_goal);
    cost_.setVelocityWeight(w_velocity);
    cost_.setPathGuidanceWeight(w_path_guidance);
    cost_.setSafeDistance(safe_distance);
    cost_.setDynamicSafeDistance(dynamic_safe_distance);
    cost_.setDynamicCollisionDistance(dynamic_collision_distance);
    cost_.setDesiredVelocity(desired_velocity);
    cost_.setNearCollisionDistance(near_collision_distance);
    cost_.setNearCollisionWeight(near_collision_weight);
    sampling_.setSigma(sigma_acc);
    sampling_.setUseColoredNoise(use_colored_noise);
    sampling_.setTemporalCorrelation(temporal_correlation);
    
    //  Validate parameters
    if (horizon_steps_ < 5 || horizon_steps_ > 100) {
        ROS_WARN("[MPPI] Invalid horizon_steps=%d, using default 20", horizon_steps_);
        horizon_steps_ = 20;
    }
    if (dt_ <= 0.0 || dt_ > 1.0) {
        ROS_WARN("[MPPI] Invalid dt=%.3f, using default 0.1", dt_);
        dt_ = 0.1;
    }
    
    //  Resize control distribution with updated horizon
    control_distribution_.resize(horizon_steps_);
    
    // Initialize visualization publishers
    mppi_trajectories_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mppi_trajectories", 10);
    optimal_trajectory_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mppi_optimal_trajectory", 10);
    
    // Get frame_id from node parameter, default to "world" if not set
    nh.param("grid_map/frame_id", frame_id_, std::string("world"));
    
#ifdef USE_GPU_MPPI
    //  Initialize GPU planner
    nh.param("mppi/use_gpu", use_gpu_, false);
    ROS_INFO("[MPPI]  DEBUG: Read parameter mppi/use_gpu = %s", use_gpu_ ? "TRUE" : "FALSE");
    
    //  Override with force_cpu flag if set
    if (force_cpu_) {
        use_gpu_ = false;
        ROS_INFO("[MPPI]  Forced CPU mode (force_cpu_ flag set)");
    }
    
    if (use_gpu_) {
        gpu_planner_ = std::make_unique<MPPIGPUPlanner>();
        
        // Configure GPU planner parameters
        MPPIGPUPlanner::Params gpu_params;
        double min_z = 0.3;
        double max_z = 4.5;
        double w_height = 80.0;
        bool safety_filter_enabled = false;
        double safety_filter_min_clearance = 0.10;
        double safety_filter_preferred_clearance = 0.25;
        double safety_filter_cost_slack = 120.0;
        int safety_filter_top_k = 96;
        bool dynamic_temporal_modes_enabled = false;
        int dynamic_temporal_mode_count = 4;
        double dynamic_temporal_cautious_scale = 0.55;
        double dynamic_temporal_fast_scale = 1.15;
        int dynamic_temporal_yield_steps = 8;
        nh.param("fsm/min_z", min_z, min_z);
        nh.param("fsm/max_z", max_z, max_z);
        nh.param("mppi/w_height", w_height, w_height);
        nh.param("mppi/safety_filter_enabled", safety_filter_enabled, safety_filter_enabled);
        nh.param("mppi/safety_filter_min_clearance", safety_filter_min_clearance, safety_filter_min_clearance);
        nh.param("mppi/safety_filter_preferred_clearance", safety_filter_preferred_clearance, safety_filter_preferred_clearance);
        nh.param("mppi/safety_filter_cost_slack", safety_filter_cost_slack, safety_filter_cost_slack);
        nh.param("mppi/safety_filter_top_k", safety_filter_top_k, safety_filter_top_k);
        nh.param("mppi/dynamic_temporal_modes_enabled", dynamic_temporal_modes_enabled, dynamic_temporal_modes_enabled);
        nh.param("mppi/dynamic_temporal_mode_count", dynamic_temporal_mode_count, dynamic_temporal_mode_count);
        nh.param("mppi/dynamic_temporal_cautious_scale", dynamic_temporal_cautious_scale, dynamic_temporal_cautious_scale);
        nh.param("mppi/dynamic_temporal_fast_scale", dynamic_temporal_fast_scale, dynamic_temporal_fast_scale);
        nh.param("mppi/dynamic_temporal_yield_steps", dynamic_temporal_yield_steps, dynamic_temporal_yield_steps);
        gpu_params.num_samples = use_adaptive_sampling_
                                     ? std::max(num_samples_, num_samples_max_)
                                     : num_samples_;
        gpu_params.horizon_steps = horizon_steps_;
        gpu_params.dt = dt_;
        gpu_params.lambda = lambda_;
        gpu_params.sigma_acc = sampling_.getSigma();
        gpu_params.max_velocity = gpu_nominal_max_vel_ > 0.0
                                      ? static_cast<float>(gpu_nominal_max_vel_)
                                      : static_cast<float>(dynamics_.getMaxVelocity());
        gpu_params.max_acceleration = gpu_nominal_max_acc_ > 0.0
                                          ? static_cast<float>(gpu_nominal_max_acc_)
                                          : static_cast<float>(dynamics_.getMaxAcceleration());
        gpu_params.max_velocity_z = dynamics_.getMaxVelocityZ();
        gpu_params.max_acceleration_z = dynamics_.getMaxAccelerationZ();
        gpu_params.min_z = static_cast<float>(min_z);
        gpu_params.max_z = static_cast<float>(max_z);
        gpu_params.w_height = static_cast<float>(w_height);
        gpu_params.safety_filter_enabled = safety_filter_enabled;
        gpu_params.safety_filter_min_clearance = static_cast<float>(std::max(0.0, safety_filter_min_clearance));
        gpu_params.safety_filter_preferred_clearance = static_cast<float>(std::max(0.0, safety_filter_preferred_clearance));
        gpu_params.safety_filter_cost_slack = static_cast<float>(std::max(0.0, safety_filter_cost_slack));
        gpu_params.safety_filter_top_k = std::max(1, safety_filter_top_k);
        gpu_params.dynamic_collision_distance =
            static_cast<float>(cost_.getDynamicCollisionDistance());
        gpu_params.dynamic_temporal_modes_enabled = dynamic_temporal_modes_enabled;
        gpu_params.dynamic_temporal_mode_count = std::max(1, std::min(4, dynamic_temporal_mode_count));
        gpu_params.dynamic_temporal_cautious_scale =
            static_cast<float>(std::max(0.10, std::min(1.0, dynamic_temporal_cautious_scale)));
        gpu_params.dynamic_temporal_fast_scale =
            static_cast<float>(std::max(1.0, std::min(1.6, dynamic_temporal_fast_scale)));
        gpu_params.dynamic_temporal_yield_steps =
            std::max(1, std::min(horizon_steps_ - 1, dynamic_temporal_yield_steps));
        
        // Cost weights: use MPPICost as single source of truth
        gpu_params.w_obstacle = cost_.getObstacleWeight();
        gpu_params.w_smoothness = cost_.getSmoothnessWeight();
        gpu_params.w_goal = cost_.getGoalWeight();
        gpu_params.w_velocity = cost_.getVelocityWeight();
        gpu_params.safe_distance = cost_.getSafeDistance();
        gpu_params.near_collision_distance = cost_.getNearCollisionDistance();
        gpu_params.near_collision_weight = cost_.getNearCollisionWeight();
        gpu_params.w_dynamic = cost_.getDynamicWeight();
        gpu_params.dynamic_safe_distance = cost_.getDynamicSafeDistance();
        
        // Iterative MPPI parameters
        gpu_params.use_iterative_mppi = use_iterative_mppi_;
        gpu_params.num_iterations = num_iterations_;
        gpu_params.use_batch_iterative_update = use_batch_iterative_update_;
        gpu_params.random_seed = gpu_random_seed_ > 0
                                     ? static_cast<unsigned long long>(gpu_random_seed_)
                                     : 0ULL;
        
        gpu_planner_->initialize(gpu_params);
        
        ROS_INFO("[MPPI]  GPU acceleration ENABLED (Iterative=%s, Iters=%d)",
                 use_iterative_mppi_ ? "YES" : "NO", num_iterations_);
        ROS_INFO("[MPPI-GPU] Sample capacity=%d active_default=%d adaptive=%s",
                 gpu_params.num_samples, num_samples_,
                 use_adaptive_sampling_ ? "ON" : "OFF");
        ROS_INFO("[MPPI-GPU] Batch iterative update: %s",
                 use_batch_iterative_update_ ? "ENABLED" : "DISABLED");
        ROS_INFO("[MPPI-GPU] Random seed: %d (%s)",
                 gpu_random_seed_, gpu_random_seed_ > 0 ? "deterministic" : "wall-clock");
        ROS_INFO("[MPPI]  GPU safety filter: %s min=%.2fm preferred=%.2fm slack=%.1f top_k=%d",
                 gpu_params.safety_filter_enabled ? "ON" : "OFF",
                 gpu_params.safety_filter_min_clearance,
                 gpu_params.safety_filter_preferred_clearance,
                 gpu_params.safety_filter_cost_slack,
                 gpu_params.safety_filter_top_k);
        ROS_INFO("[MPPI]  GPU dynamic temporal modes: %s count=%d cautious=%.2f fast=%.2f yield_steps=%d",
                 gpu_params.dynamic_temporal_modes_enabled ? "ON" : "OFF",
                 gpu_params.dynamic_temporal_mode_count,
                 gpu_params.dynamic_temporal_cautious_scale,
                 gpu_params.dynamic_temporal_fast_scale,
                 gpu_params.dynamic_temporal_yield_steps);
        ROS_INFO("[MPPI]  GPU dynamic hard collision distance: %.2fm",
                 gpu_params.dynamic_collision_distance);
    } else {
        ROS_INFO("[MPPI] Using CPU implementation (set use_gpu=true to enable GPU)");
    }
    // Initialize EDT GPU cache variables
    edt_last_ptr_ = nullptr;
    edt_last_size_ = 0;
    edt_gpu_ready_ = false;
#endif
    
    ROS_INFO("[MPPI]  Initialized with modularized design (inspired by MPPI-Generic)");
    ROS_INFO("[MPPI] Samples: %d, Horizon: %d steps (%.2fs), dt: %.3f", 
             num_samples_, horizon_steps_, horizon_steps_ * dt_, dt_);
    ROS_INFO("[MPPI] Dynamics: MaxVel=%.2f m/s, MaxAcc=%.2f m/s²", 
             dynamics_.getMaxVelocity(), dynamics_.getMaxAcceleration());
#ifdef USE_GPU_MPPI
    if (use_gpu_ && gpu_planner_) {
        ROS_INFO("[MPPI] GPU nominal dynamics override: max_vel=%.2f max_acc=%.2f (-1 means synchronized)",
                 gpu_nominal_max_vel_, gpu_nominal_max_acc_);
    }
#endif
    ROS_INFO("[MPPI] Cost weights: Obs=%.1f, Dyn=%.1f, Smooth=%.1f, Goal=%.1f, Vel=%.1f",
             cost_.getObstacleWeight(), cost_.getDynamicWeight(), 
             cost_.getSmoothnessWeight(), cost_.getGoalWeight(), cost_.getVelocityWeight());
    ROS_INFO("[MPPI] Sampling: Sigma=%.2f, ColoredNoise=%s, Alpha=%.2f",
             sampling_.getSigma(), sampling_.getUseColoredNoise() ? "ON" : "OFF",
             sampling_.getTemporalCorrelation());
    
    if (use_iterative_mppi_ && num_iterations_ > 1) {
        ROS_INFO("[MPPI] Iterative MPPI: %d iterations", num_iterations_);
    }
    
    ROS_INFO("[MPPI] frame_id: %s", frame_id_.c_str());
}

// ---------------------------------------------------------------------------
// Internal helpers (extracted to eliminate code duplication)
// ---------------------------------------------------------------------------

bool MPPIPlanner::validatePlannerState() const {
    if (horizon_steps_ <= 0 || horizon_steps_ > 100) {
        ROS_ERROR("[MPPI] INVALID STATE: horizon_steps_=%d (expected 10-30). Object not properly initialized!", 
                  horizon_steps_);
        return false;
    }
    if (!grid_map_) {
        ROS_ERROR("[MPPI] INVALID STATE: grid_map_ is null!");
        return false;
    }
    ROS_INFO("[MPPI] State check passed: horizon=%d, samples=%d, grid_map valid", 
             horizon_steps_, num_samples_);
    return true;
}

#ifdef USE_GPU_MPPI
void MPPIPlanner::uploadEDTToGPU() {
    auto edt_data = grid_map_->getEDTData();
    size_t grid_size = edt_data.size_x * edt_data.size_y * edt_data.size_z;
    
    // Reuse cached float vector allocation (avoid 4.8M element reallocation every call)
    if (edt_float_cache_.size() != grid_size) {
        edt_float_cache_.resize(grid_size);
    }
    
    for (size_t i = 0; i < grid_size; ++i) {
        double dist = edt_data.esdf_buffer[i];
        if (dist == 0.0 && edt_data.esdf_buffer_neg[i] > 0.0) {
            dist = -edt_data.esdf_buffer_neg[i];
        }
        edt_float_cache_[i] = static_cast<float>(dist);
    }
    
    gpu_planner_->setEDTMap(edt_float_cache_.data(),
                            edt_data.size_x, edt_data.size_y, edt_data.size_z,
                            static_cast<float>(edt_data.resolution),
                            static_cast<float>(edt_data.origin_x),
                            static_cast<float>(edt_data.origin_y),
                            static_cast<float>(edt_data.origin_z));
    
    edt_last_ptr_ = edt_data.esdf_buffer;
    edt_last_size_ = grid_size;
}

void MPPIPlanner::prepareEDTForGPU() {
    // Skip if EDT already uploaded in this replan cycle
    if (edt_gpu_ready_) return;
    
    uploadEDTToGPU();
    edt_gpu_ready_ = true;
}

bool MPPIPlanner::gpuPlan(const Vector3d& start_pos,
                          const Vector3d& start_vel,
                          const Vector3d& goal_pos,
                          const Vector3d& goal_vel,
                          MPPITrajectory& optimal_trajectory) {
    // EDT should already be prepared by prepareEDTForGPU() before the topo loop.
    // If not (fallback), upload now.
    if (!edt_gpu_ready_) {
        prepareEDTForGPU();
    }

    const int adaptive_samples = computeAdaptiveSamples(start_pos, goal_pos);
    gpu_planner_->setActiveNumSamples(adaptive_samples);
    last_timing_.num_samples = adaptive_samples;
    
    std::vector<Eigen::Vector3d> path;
    bool success = gpu_planner_->plan(start_pos, start_vel, goal_pos, goal_vel, path);
    
    if (!success || path.empty()) {
        return false;
    }
    
    optimal_trajectory.resize(path.size());
    optimal_trajectory.positions = path;
    
    for (size_t i = 0; i < path.size(); ++i) {
        if (i == 0) {
            optimal_trajectory.velocities[i] = start_vel;
        } else {
            optimal_trajectory.velocities[i] = (path[i] - path[i-1]) / dt_;
        }
        optimal_trajectory.accelerations[i] = Eigen::Vector3d::Zero();
    }
    
    optimal_trajectory.cost = gpu_planner_->getLastBestCost();
    ROS_INFO("[MPPI] GPU plan succeeded, cost=%.2f, path_size=%zu", 
             optimal_trajectory.cost, path.size());
    return true;
}

bool MPPIPlanner::gpuPlan(const Vector3d& start_pos,
                          const Vector3d& start_vel,
                          const Vector3d& goal_pos,
                          const Vector3d& goal_vel,
                          const vector<Vector3d>& guide_path,
                          MPPITrajectory& optimal_trajectory) {
    if (!edt_gpu_ready_) {
        prepareEDTForGPU();
    }

    const int adaptive_samples = computeAdaptiveSamples(start_pos, goal_pos);
    gpu_planner_->setActiveNumSamples(adaptive_samples);
    last_timing_.num_samples = adaptive_samples;
    
    std::vector<Eigen::Vector3d> path;
    bool success = gpu_planner_->plan(start_pos, start_vel, goal_pos, goal_vel, guide_path, path);
    
    if (!success || path.empty()) {
        return false;
    }
    
    optimal_trajectory.resize(path.size());
    optimal_trajectory.positions = path;
    
    for (size_t i = 0; i < path.size(); ++i) {
        if (i == 0) {
            optimal_trajectory.velocities[i] = start_vel;
        } else {
            optimal_trajectory.velocities[i] = (path[i] - path[i-1]) / dt_;
        }
        optimal_trajectory.accelerations[i] = Eigen::Vector3d::Zero();
    }
    
    optimal_trajectory.cost = gpu_planner_->getLastBestCost();
    ROS_INFO("[MPPI] GPU guided plan succeeded, cost=%.2f, path_size=%zu, guide_waypoints=%zu",
             optimal_trajectory.cost, path.size(), guide_path.size());
    return true;
}
#endif

// ---------------------------------------------------------------------------
// planTrajectory (no guide path)
// ---------------------------------------------------------------------------

bool MPPIPlanner::planTrajectory(const Vector3d& start_pos,
                                const Vector3d& start_vel,
                                const Vector3d& goal_pos,
                                const Vector3d& goal_vel,
                                MPPITrajectory& optimal_trajectory) {
    
    if (!validatePlannerState()) return false;
    
#ifdef USE_GPU_MPPI
    if (use_gpu_ && gpu_planner_) {
        return gpuPlan(start_pos, start_vel, goal_pos, goal_vel, optimal_trajectory);
    }
#endif
    
    // CPU fallback or default implementation
    //  Performance monitoring
    auto t_start = std::chrono::high_resolution_clock::now();
    
    //  Debug: Check start position validity
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
    
    //  Memory monitoring: estimate memory usage
    size_t traj_memory = sizeof(MPPITrajectory) * adaptive_samples;
    size_t vec_memory = sizeof(Vector3d) * horizon_steps_ * adaptive_samples * 3;  // positions, velocities, accelerations
    size_t total_memory_mb = (traj_memory + vec_memory) / (1024 * 1024);
    ROS_INFO("[MPPI] Memory estimate: %zu MB for %d trajectories (horizon=%d)", 
             total_memory_mb, adaptive_samples, horizon_steps_);
    
    vector<MPPITrajectory> trajectories(adaptive_samples);
    double min_cost = std::numeric_limits<double>::max();
    
    //  Generate rollout trajectories in PARALLEL
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
    
    //  Debug: Count how many trajectories have finite cost
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
    
    //  Improved importance weight calculation (inspired by MPPI-Generic)
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
    
    //  Total timing
    auto t_end = std::chrono::high_resolution_clock::now();
    last_timing_.total_time = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    ROS_INFO("[MPPI]  Timing: Total=%.2fms (Rollout=%.2fms, Weight=%.2fms, Avg=%.2fms) | Samples=%d | Cost=%.2f",
             last_timing_.total_time, last_timing_.rollout_time, last_timing_.weight_time, 
             last_timing_.average_time, adaptive_samples, optimal_trajectory.cost);
    
    return true;
}

//  NEW: planTrajectory with initial path guidance
bool MPPIPlanner::planTrajectory(const Vector3d& start_pos,
                                const Vector3d& start_vel,
                                const Vector3d& goal_pos,
                                const Vector3d& goal_vel,
                                const vector<Vector3d>& initial_path,
                                MPPITrajectory& optimal_trajectory) {
    
    if (!validatePlannerState()) return false;
    
#ifdef USE_GPU_MPPI
    if (use_gpu_ && gpu_planner_) {
        return gpuPlan(start_pos, start_vel, goal_pos, goal_vel, initial_path, optimal_trajectory);
    }
#endif
    
    // CPU fallback implementation
    //  Debug: Check start position validity
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
    
    //  Memory monitoring: estimate memory usage
    size_t traj_memory = sizeof(MPPITrajectory) * adaptive_samples;
    size_t vec_memory = sizeof(Vector3d) * horizon_steps_ * adaptive_samples * 3;  // positions, velocities, accelerations
    size_t total_memory_mb = (traj_memory + vec_memory) / (1024 * 1024);
    ROS_INFO("[MPPI-Guided] Memory estimate: %zu MB for %d trajectories (horizon=%d)", 
             total_memory_mb, adaptive_samples, horizon_steps_);
    
    //  ITERATIVE MPPI: Initialize control distribution
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
    
    //  ITERATIVE MPPI LOOP (Algorithm 1 from MPPI-Generic)
    int num_iters = use_iterative_mppi_ ? num_iterations_ : 1;
    for (int iter = 0; iter < num_iters; ++iter) {
        //  Temperature annealing: λ_k = λ₀ * (1 - k/K)^2
        double lambda_k = lambda_;
        if (use_iterative_mppi_ && num_iterations_ > 1) {
            double ratio = 1.0 - (double)iter / num_iterations_;
            lambda_k = lambda_ * ratio * ratio;  // Quadratic decay
            if (lambda_k < 0.1) lambda_k = 0.1;  // Minimum temperature
        }
        
        min_cost = std::numeric_limits<double>::max();
        
        //  Generate rollout trajectories guided by initial path in PARALLEL
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
    
    //  Debug: Count valid trajectories
    int valid_count = 0;
    for (const auto& traj : trajectories) {
        if (traj.cost < std::numeric_limits<double>::max()) {
            valid_count++;
        }
    }
    
    //  Improved logging for iterative MPPI
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
    
    // Calculate importance weights with current lambda_k.
    // Keep the guided CPU path numerically aligned with the non-guided MPPI path.
    double weight_sum = 0.0;
    double max_exp_arg = -1e10;

    for (const auto& traj : trajectories) {
        double exp_arg = -(traj.cost - min_cost) / lambda_k;
        if (exp_arg > max_exp_arg) {
            max_exp_arg = exp_arg;
        }
    }

    for (auto& traj : trajectories) {
        double exp_arg = -(traj.cost - min_cost) / lambda_k;
        traj.weight = std::exp(exp_arg - max_exp_arg);
        if (!std::isfinite(traj.weight)) {
            ROS_WARN("[MPPI] Non-finite guided weight detected, setting to zero");
            traj.weight = 0.0;
        }
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
    
    //  ITERATIVE MPPI: Update control distribution for next iteration
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

bool MPPIPlanner::planTrajectoryBatch(const Vector3d& start_pos,
                                      const Vector3d& start_vel,
                                      const Vector3d& goal_pos,
                                      const Vector3d& goal_vel,
                                      const vector<vector<Vector3d>>& initial_paths,
                                      vector<MPPITrajectory>& optimal_trajectories) {
    optimal_trajectories.clear();
    if (initial_paths.empty()) {
        return false;
    }
    if (!validatePlannerState()) return false;

#ifdef USE_GPU_MPPI
    if (use_gpu_ && gpu_planner_) {
        if (!edt_gpu_ready_) {
            prepareEDTForGPU();
        }

        const int adaptive_samples = computeAdaptiveSamples(start_pos, goal_pos);
        gpu_planner_->setActiveNumSamples(adaptive_samples);
        last_timing_.num_samples = adaptive_samples;

        vector<vector<Vector3d>> paths;
        vector<float> costs;
        const bool success = gpu_planner_->planBatch(start_pos, start_vel, goal_pos, goal_vel,
                                                     initial_paths, paths, costs);
        optimal_trajectories.resize(paths.size());
        for (size_t i = 0; i < paths.size(); ++i) {
            optimal_trajectories[i].resize(paths[i].size());
            optimal_trajectories[i].positions = paths[i];
            optimal_trajectories[i].cost =
                i < costs.size() ? static_cast<double>(costs[i]) : std::numeric_limits<double>::max();
            for (size_t j = 0; j < paths[i].size(); ++j) {
                if (j == 0) {
                    optimal_trajectories[i].velocities[j] = start_vel;
                } else {
                    optimal_trajectories[i].velocities[j] = (paths[i][j] - paths[i][j - 1]) / dt_;
                }
                optimal_trajectories[i].accelerations[j] = Eigen::Vector3d::Zero();
            }
        }
        return success;
    }
#endif

    optimal_trajectories.resize(initial_paths.size());
    bool any_success = false;
    for (size_t i = 0; i < initial_paths.size(); ++i) {
        if (planTrajectory(start_pos, start_vel, goal_pos, goal_vel,
                           initial_paths[i], optimal_trajectories[i])) {
            any_success = true;
        }
    }
    return any_success;
}

double MPPIPlanner::calculateTrajectoryCost(const MPPITrajectory& trajectory,
                                          const Vector3d& goal_pos,
                                          const Vector3d& goal_vel) {
    //  NEW: Use modularized Cost module with running + terminal separation
    double total_cost = 0.0;
    int collision_count = 0;
    
    // Running costs for each timestep
    for (int t = 0; t < trajectory.size(); ++t) {
        double running_cost = cost_.computeRunningCost(
            trajectory.positions[t],
            trajectory.velocities[t],
            trajectory.accelerations[t],
            goal_pos,
            static_cast<double>(t) * dt_,
            nullptr  // No path waypoint guidance in cost (already in rollout)
        );
        
        //  IMPROVED: Use graduated penalties instead of constant 1e8
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
    //  Phase 3: Use ESDF for O(1) distance query instead of O(n³) sampling
    double dist = grid_map_->getDistance(position);
    return obstacleCost(position, dist);
}

double MPPIPlanner::obstacleCost(const Vector3d& position, double dist) {
    //  Phase 3: ESDF-based obstacle cost - O(1) instead of O(n³)
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
    //  Use modularized dynamics constraints
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
    //  Safety check: ensure grid_map is initialized
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

//  Thread-safe rolloutTrajectory with local random generators (for OpenMP)
void MPPIPlanner::rolloutTrajectory(const Vector3d& start_pos,
                                   const Vector3d& start_vel,
                                   const Vector3d& goal_pos,
                                   const Vector3d& goal_vel,
                                   MPPITrajectory& trajectory,
                                   std::mt19937& local_gen,
                                   std::normal_distribution<double>& local_dist) {
    //  NEW: Use modularized Dynamics (thread-safe version)
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

//  Thread-safe rolloutTrajectory with path guidance and local random generators
void MPPIPlanner::rolloutTrajectory(const Vector3d& start_pos,
                                   const Vector3d& start_vel,
                                   const Vector3d& goal_pos,
                                   const Vector3d& goal_vel,
                                   const vector<Vector3d>& guide_path,
                                   MPPITrajectory& trajectory,
                                   std::mt19937& local_gen,
                                   std::normal_distribution<double>& local_dist) {
    //  NEW: Use modularized Dynamics (thread-safe version with guidance)
    VectorXd state(6);
    state << start_pos, start_vel;
    
    trajectory.positions[0] = start_pos;
    trajectory.velocities[0] = start_vel;
    trajectory.accelerations[0] = Vector3d::Zero();
    
    //  Debug: Check guide_path validity
    if (guide_path.empty()) {
        ROS_WARN_THROTTLE(1.0, "[MPPI]  Guide path is EMPTY! Using goal-directed rollout");
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

//  NEW: Update control distribution based on weighted trajectory samples (MPPI-Generic Algorithm 1)
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
        
        //  Ensure minimum std dev (exploration)
        for (int dim = 0; dim < 3; ++dim) {
            if (new_distribution.std_control[t](dim) < 0.1) {
                new_distribution.std_control[t](dim) = 0.1;
            }
        }
    }
}

//  NEW: Sample control sequence from distribution
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
