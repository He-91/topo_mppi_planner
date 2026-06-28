#include "ddo_path_searching/mppi_gpu_planner.h"
#include "ddo_path_searching/mppi_cuda_kernel.cuh"
#include <ros/ros.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <limits>

namespace ego_planner {

MPPIGPUPlanner::MPPIGPUPlanner() 
    : initialized_(false),
      stream_(nullptr),
      own_stream_(false),
      d_initial_state_(nullptr),
      d_goal_pos_(nullptr),
      d_goal_vel_(nullptr),
      d_nominal_control_(nullptr),
      d_trajectory_costs_(nullptr),
      d_weights_(nullptr),
      d_min_cost_(nullptr),
      d_weight_sum_(nullptr),
      d_edt_buffer_(nullptr),
      edt_buffer_allocated_size_(0),
      d_dynamic_positions_(nullptr),
      d_dynamic_radii_(nullptr),
      d_dynamic_heights_(nullptr),
      num_dynamic_obstacles_(0),
      dynamic_horizon_(0),
      dynamic_dt_(0.0f),
      d_control_samples_(nullptr),
      d_updated_control_(nullptr),
      last_best_cost_(0.0f),
      d_best_trajectory_states_(nullptr),
      h_best_trajectory_states_(nullptr) {
}

MPPIGPUPlanner::~MPPIGPUPlanner() {
    if (initialized_) {
        freeGPUMemory();
        
        // Destroy stream if we own it
        if (own_stream_ && stream_ != nullptr) {
            cudaStreamDestroy(stream_);
            stream_ = nullptr;
        }
    }
}

void MPPIGPUPlanner::initialize(const Params& params, cudaStream_t stream) {
    params_ = params;
    
    // Explicitly initialize CUDA runtime
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    if (err != cudaSuccess) {
        ROS_ERROR("[MPPI-GPU] cudaGetDeviceCount failed: %s", cudaGetErrorString(err));
        throw std::runtime_error("CUDA initialization failed");
    }
    ROS_INFO("[MPPI-GPU] Found %d CUDA device(s)", device_count);
    
    err = cudaSetDevice(0);
    if (err != cudaSuccess) {
        ROS_ERROR("[MPPI-GPU] cudaSetDevice failed: %s", cudaGetErrorString(err));
        throw std::runtime_error("CUDA device selection failed");
    }
    
    // Use default stream (no cudaStreamCreate to avoid malloc issues)
    stream_ = nullptr;
    own_stream_ = false;
    ROS_INFO("[MPPI-GPU] Using default CUDA stream");
    
    // Allocate host memory (pinned for async transfer)
    h_trajectory_costs_.resize(params_.num_samples);
    h_weights_.resize(params_.num_samples);
    h_nominal_control_.resize(params_.horizon_steps * 3, 0.0f);
    if (params_.safety_filter_enabled) {
        h_control_samples_.resize(params_.num_samples * params_.horizon_steps * CONTROL_DIM);
    } else {
        h_control_samples_.clear();
    }
    
    // Allocate GPU memory
    allocateGPUMemory();
    
    initialized_ = true;
    ROS_INFO("[MPPI-GPU] Initialized with %d samples, horizon=%d", 
             params_.num_samples, params_.horizon_steps);
}

void MPPIGPUPlanner::setActiveNumSamples(int num_samples) {
    const int capacity = static_cast<int>(h_trajectory_costs_.size());
    if (capacity <= 0) {
        return;
    }

    const int clamped_samples = std::max(1, std::min(num_samples, capacity));
    if (clamped_samples != num_samples) {
        ROS_WARN_THROTTLE(2.0,
                          "[MPPI-GPU] Requested %d samples exceeds allocated capacity %d; using %d",
                          num_samples, capacity, clamped_samples);
    }

    if (params_.num_samples != clamped_samples) {
        params_.num_samples = clamped_samples;
        ROS_DEBUG("[MPPI-GPU] Active sample count set to %d (capacity=%d)",
                  params_.num_samples, capacity);
    }
}

void MPPIGPUPlanner::setVehicleLimits(float max_velocity, float max_acceleration) {
    params_.max_velocity = std::max(0.01f, max_velocity);
    params_.max_acceleration = std::max(0.01f, max_acceleration);
    ROS_INFO("[MPPI-GPU] Vehicle limits updated: max_vel=%.2f max_acc=%.2f",
             params_.max_velocity, params_.max_acceleration);
}

void MPPIGPUPlanner::setVehicleLimitsZ(float max_velocity_z, float max_acceleration_z) {
    params_.max_velocity_z = std::max(0.01f, max_velocity_z);
    params_.max_acceleration_z = std::max(0.01f, max_acceleration_z);
    ROS_INFO("[MPPI-GPU] Z limits updated: max_vel_z=%.2f max_acc_z=%.2f",
             params_.max_velocity_z, params_.max_acceleration_z);
}

void MPPIGPUPlanner::allocateGPUMemory() {
    // State and goal
    CUDA_CHECK(cudaMalloc(&d_initial_state_, STATE_DIM * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_goal_pos_, 3 * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_goal_vel_, 3 * sizeof(float)));
    
    // Control sequence
    CUDA_CHECK(cudaMalloc(&d_nominal_control_, 
                         params_.horizon_steps * CONTROL_DIM * sizeof(float)));
    
    // Trajectory costs and weights
    CUDA_CHECK(cudaMalloc(&d_trajectory_costs_, params_.num_samples * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_weights_, params_.num_samples * sizeof(float)));
    
    // Min cost and weight sum (single values)
    CUDA_CHECK(cudaMalloc(&d_min_cost_, sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_weight_sum_, sizeof(float)));
    
    // Control samples和updated control
    size_t control_samples_size = params_.num_samples * params_.horizon_steps * CONTROL_DIM * sizeof(float);
    CUDA_CHECK(cudaMalloc(&d_control_samples_, control_samples_size));
    CUDA_CHECK(cudaMalloc(&d_updated_control_, params_.horizon_steps * CONTROL_DIM * sizeof(float)));
    
    // P0改进: 分配最优轨迹状态内存 (position + velocity)
    size_t trajectory_state_size = params_.horizon_steps * 6 * sizeof(float);  // (x,y,z,vx,vy,vz) per timestep
    CUDA_CHECK(cudaMalloc(&d_best_trajectory_states_, trajectory_state_size));
    CUDA_CHECK(cudaMallocHost(&h_best_trajectory_states_, trajectory_state_size));  // Pinned memory for fast transfer
    
    ROS_INFO("[MPPI-GPU] GPU memory allocated successfully");
}

void MPPIGPUPlanner::freeGPUMemory() {
    if (d_initial_state_) cudaFree(d_initial_state_);
    if (d_goal_pos_) cudaFree(d_goal_pos_);
    if (d_goal_vel_) cudaFree(d_goal_vel_);
    if (d_nominal_control_) cudaFree(d_nominal_control_);
    if (d_trajectory_costs_) cudaFree(d_trajectory_costs_);
    if (d_weights_) cudaFree(d_weights_);
    if (d_min_cost_) cudaFree(d_min_cost_);
    if (d_weight_sum_) cudaFree(d_weight_sum_);
    
    // P0改进: 释放轨迹状态内存
    if (d_best_trajectory_states_) cudaFree(d_best_trajectory_states_);
    if (h_best_trajectory_states_) cudaFreeHost(h_best_trajectory_states_);
    
    // Free EDT buffer (linear memory instead of texture)
    if (d_edt_buffer_) {
        cudaFree(d_edt_buffer_);
        d_edt_buffer_ = nullptr;
    }
    
    // Free dynamic obstacles data
    if (d_dynamic_positions_) cudaFree(d_dynamic_positions_);
    if (d_dynamic_radii_) cudaFree(d_dynamic_radii_);
    if (d_dynamic_heights_) cudaFree(d_dynamic_heights_);
    
    if (d_control_samples_) cudaFree(d_control_samples_);
    if (d_updated_control_) cudaFree(d_updated_control_);
}

void MPPIGPUPlanner::setEDTMap(const float* edt_data, 
                               int size_x, int size_y, int size_z,
                               float resolution, 
                               float origin_x, float origin_y, float origin_z) {
    grid_size_x_ = size_x;
    grid_size_y_ = size_y;
    grid_size_z_ = size_z;
    grid_resolution_ = resolution;
    grid_origin_x_ = origin_x;
    grid_origin_y_ = origin_y;
    grid_origin_z_ = origin_z;
    
    size_t buffer_size = (size_t)size_x * size_y * size_z * sizeof(float);
    
    // Only reallocate if buffer size changed (avoids cudaFree+cudaMalloc overhead)
    if (buffer_size != edt_buffer_allocated_size_) {
        if (d_edt_buffer_) {
            cudaFree(d_edt_buffer_);
            d_edt_buffer_ = nullptr;
        }
        CUDA_CHECK(cudaMalloc(&d_edt_buffer_, buffer_size));
        edt_buffer_allocated_size_ = buffer_size;
        ROS_INFO("[MPPI-GPU] EDT buffer (re)allocated: %dx%dx%d, res=%.2f, size=%zu MB", 
                 size_x, size_y, size_z, resolution, buffer_size / (1024 * 1024));
    }
    
    // Copy EDT data to GPU (linear memory) — only memcpy, no alloc
    CUDA_CHECK(cudaMemcpy(d_edt_buffer_, edt_data, buffer_size, cudaMemcpyHostToDevice));
    if (params_.safety_filter_enabled) {
        h_edt_buffer_.assign(edt_data, edt_data + (size_t)size_x * size_y * size_z);
    } else {
        h_edt_buffer_.clear();
    }
}

void MPPIGPUPlanner::setDynamicObstacles(const std::vector<Eigen::Vector3d>& positions,
                                        const std::vector<float>& radii,
                                        const std::vector<float>& heights,
                                        int num_obstacles,
                                        int horizon,
                                        float dt) {
    num_dynamic_obstacles_ = num_obstacles;
    dynamic_horizon_ = horizon;
    dynamic_dt_ = dt;
    
    if (num_obstacles == 0 || positions.empty()) {
        // Clear dynamic obstacles
        if (d_dynamic_positions_) {
            cudaFree(d_dynamic_positions_);
            d_dynamic_positions_ = nullptr;
        }
        if (d_dynamic_radii_) {
            cudaFree(d_dynamic_radii_);
            d_dynamic_radii_ = nullptr;
        }
        if (d_dynamic_heights_) {
            cudaFree(d_dynamic_heights_);
            d_dynamic_heights_ = nullptr;
        }
        num_dynamic_obstacles_ = 0;
        return;
    }
    
    // Prepare host data in flat format
    std::vector<float3> h_positions(num_obstacles * horizon);
    for (int i = 0; i < num_obstacles; ++i) {
        for (int t = 0; t < horizon; ++t) {
            int idx = i * horizon + t;
            if (idx < (int)positions.size()) {
                h_positions[i * horizon + t] = make_float3(
                    (float)positions[idx].x(),
                    (float)positions[idx].y(),
                    (float)positions[idx].z()
                );
            } else {
                // Use last available position if not enough predictions
                int last_idx = (int)positions.size() - 1;
                h_positions[i * horizon + t] = make_float3(
                    (float)positions[last_idx].x(),
                    (float)positions[last_idx].y(),
                    (float)positions[last_idx].z()
                );
            }
        }
    }
    
    // Free old data
    if (d_dynamic_positions_) cudaFree(d_dynamic_positions_);
    if (d_dynamic_radii_) cudaFree(d_dynamic_radii_);
    if (d_dynamic_heights_) cudaFree(d_dynamic_heights_);
    
    // Allocate and copy to GPU
    size_t pos_size = num_obstacles * horizon * sizeof(float3);
    size_t radii_size = num_obstacles * sizeof(float);
    size_t heights_size = num_obstacles * sizeof(float);
    std::vector<float> h_heights(num_obstacles, 0.0f);
    for (int i = 0; i < num_obstacles && i < (int)heights.size(); ++i) {
        h_heights[i] = heights[i];
    }
    
    CUDA_CHECK(cudaMalloc(&d_dynamic_positions_, pos_size));
    CUDA_CHECK(cudaMalloc(&d_dynamic_radii_, radii_size));
    CUDA_CHECK(cudaMalloc(&d_dynamic_heights_, heights_size));
    
    CUDA_CHECK(cudaMemcpyAsync(d_dynamic_positions_, h_positions.data(), pos_size,
                              cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_dynamic_radii_, radii.data(), radii_size,
                              cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_dynamic_heights_, h_heights.data(), heights_size,
                              cudaMemcpyHostToDevice, stream_));
    
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    
    ROS_INFO("[MPPI-GPU] Dynamic obstacles updated: %d obstacles × %d timesteps (dt=%.2fs)",
             num_obstacles, horizon, dt);
}

bool MPPIGPUPlanner::plan(const Vector3d& start_pos,
                          const Vector3d& start_vel,
                          const Vector3d& goal_pos,
                          const Vector3d& goal_vel,
                          std::vector<Vector3d>& optimal_path) {
    if (!initialized_) {
        ROS_ERROR("[MPPI-GPU] Not initialized!");
        return false;
    }
    
    auto t_start = std::chrono::high_resolution_clock::now();
    
    // Prepare initial state
    float h_initial_state[STATE_DIM] = {
        (float)start_pos.x(), (float)start_pos.y(), (float)start_pos.z(),
        (float)start_vel.x(), (float)start_vel.y(), (float)start_vel.z()
    };
    
    h_goal_pos_[0] = (float)goal_pos.x();
    h_goal_pos_[1] = (float)goal_pos.y();
    h_goal_pos_[2] = (float)goal_pos.z();
    h_goal_vel_[0] = (float)goal_vel.x();
    h_goal_vel_[1] = (float)goal_vel.y();
    h_goal_vel_[2] = (float)goal_vel.z();
    
    // Copy to GPU (Async transfers with stream)
    CUDA_CHECK(cudaMemcpyAsync(d_initial_state_, h_initial_state, 
                         STATE_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_goal_pos_, h_goal_pos_, 
                         3 * sizeof(float), cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_goal_vel_, h_goal_vel_, 
                         3 * sizeof(float), cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_nominal_control_, h_nominal_control_.data(),
                         params_.horizon_steps * CONTROL_DIM * sizeof(float), 
                         cudaMemcpyHostToDevice, stream_));
    
    // Iterative MPPI Loop (on GPU)
    int num_iters = params_.use_iterative_mppi ? params_.num_iterations : 1;
    float total_rollout_ms = 0.0f;
    float total_weight_ms = 0.0f;
    float total_update_ms = 0.0f;
    
    for (int iter = 0; iter < num_iters; ++iter) {
        // 改进: 线性退火,保留30%温度 (参考MPPI-Generic)
        float lambda_annealed;
        if (num_iters == 1) {
            lambda_annealed = params_.lambda;  // 单次迭代保持原值
        } else {
            // 线性退火: 100% → 30%
            float decay_ratio = 0.7f * (float)iter / (float)(num_iters - 1);
            lambda_annealed = params_.lambda * (1.0f - decay_ratio);
            // 最低保留30%温度(而非1%),保持探索能力
            lambda_annealed = fmaxf(lambda_annealed, 0.3f * params_.lambda);
        }
        
        // Launch rollout kernel (on stream)
        auto t_rollout_start = std::chrono::high_resolution_clock::now();
        launchRolloutKernel();
        CUDA_CHECK(cudaStreamSynchronize(stream_));  // Wait for rollout completion
        auto t_rollout_end = std::chrono::high_resolution_clock::now();
        float rollout_ms = std::chrono::duration<float, std::milli>(t_rollout_end - t_rollout_start).count();
        total_rollout_ms += rollout_ms;
        
        // Compute weights (on stream, with annealed lambda)
        auto t_weight_start = std::chrono::high_resolution_clock::now();
        computeWeights(lambda_annealed);  // Pass annealed lambda
        CUDA_CHECK(cudaStreamSynchronize(stream_));  // Wait for weight computation
        auto t_weight_end = std::chrono::high_resolution_clock::now();
        float weight_ms = std::chrono::duration<float, std::milli>(t_weight_end - t_weight_start).count();
        total_weight_ms += weight_ms;
        
        // Update nominal control (MPPI核心算法, on stream)
        auto t_update_start = std::chrono::high_resolution_clock::now();
        updateNominalControl();
        CUDA_CHECK(cudaStreamSynchronize(stream_));  // Wait for control update
        auto t_update_end = std::chrono::high_resolution_clock::now();
        float update_ms = std::chrono::duration<float, std::milli>(t_update_end - t_update_start).count();
        total_update_ms += update_ms;
        
        if (params_.use_iterative_mppi && num_iters > 1) {
            ROS_INFO("[MPPI-GPU] Iter %d/%d: lambda=%.2f, rollout=%.2fms, weight=%.2fms, update=%.2fms",
                     iter + 1, num_iters, lambda_annealed, rollout_ms, weight_ms, update_ms);
        }
    }
    
    last_timing_.rollout_time_ms = total_rollout_ms;
    last_timing_.weight_time_ms = total_weight_ms;
    float update_time_ms = total_update_ms;
    
    // Copy results back (Async)
    CUDA_CHECK(cudaMemcpyAsync(h_trajectory_costs_.data(), d_trajectory_costs_,
                         params_.num_samples * sizeof(float), cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaMemcpyAsync(h_weights_.data(), d_weights_,
                         params_.num_samples * sizeof(float), cudaMemcpyDeviceToHost, stream_));
    if (params_.safety_filter_enabled) {
        CUDA_CHECK(cudaMemcpyAsync(h_control_samples_.data(), d_control_samples_,
                             h_control_samples_.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_));
    }
    
    // Wait for all async operations to complete
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    
    // Extract optimal trajectory (CPU)
    extractOptimalTrajectory(start_pos, start_vel, optimal_path);
    
    auto t_end = std::chrono::high_resolution_clock::now();
    last_timing_.total_time_ms = 
        std::chrono::duration<float, std::milli>(t_end - t_start).count();
    
    ROS_INFO("[MPPI-GPU] Timing: Total=%.2fms (Rollout=%.2fms, Weight=%.2fms, Update=%.2fms)",
             last_timing_.total_time_ms, last_timing_.rollout_time_ms, 
             last_timing_.weight_time_ms, update_time_ms);
    
    return true;
}

bool MPPIGPUPlanner::plan(const Vector3d& start_pos,
                          const Vector3d& start_vel,
                          const Vector3d& goal_pos,
                          const Vector3d& goal_vel,
                          const std::vector<Vector3d>& guide_path,
                          std::vector<Vector3d>& optimal_path) {
    setNominalControlFromGuidePath(start_pos, start_vel, goal_pos, guide_path);
    ROS_INFO("[MPPI-GPU] Guided nominal control seeded from topo path: waypoints=%zu", guide_path.size());
    return plan(start_pos, start_vel, goal_pos, goal_vel, optimal_path);
}

bool MPPIGPUPlanner::planBatch(const Vector3d& start_pos,
                               const Vector3d& start_vel,
                               const Vector3d& goal_pos,
                               const Vector3d& goal_vel,
                               const std::vector<std::vector<Vector3d>>& guide_paths,
                               std::vector<std::vector<Vector3d>>& optimal_paths,
                               std::vector<float>& costs) {
    if (!initialized_) {
        ROS_ERROR("[MPPI-GPU] Not initialized!");
        return false;
    }
    if (guide_paths.empty()) {
        return false;
    }

    const int num_topo_modes = static_cast<int>(guide_paths.size());
    const bool use_temporal_modes =
        params_.dynamic_temporal_modes_enabled && num_dynamic_obstacles_ > 0;
    const int temporal_modes =
        use_temporal_modes
            ? std::max(1, std::min(params_.dynamic_temporal_mode_count, 4))
            : 1;
    const int num_modes = num_topo_modes * temporal_modes;
    const int total_rollouts = num_modes * params_.num_samples;
    auto t_start = std::chrono::high_resolution_clock::now();

    std::vector<float> h_multi_nominal(
        static_cast<size_t>(num_modes) * params_.horizon_steps * CONTROL_DIM,
        0.0f);
    const auto clamp_control_profile = [&](std::vector<float>& controls) {
        for (int t = 0; t < params_.horizon_steps; ++t) {
            float* u = controls.data() + t * CONTROL_DIM;
            const float acc_xy = std::sqrt(u[0] * u[0] + u[1] * u[1]);
            if (acc_xy > params_.max_acceleration && acc_xy > 1e-6f) {
                const float scale = params_.max_acceleration / acc_xy;
                u[0] *= scale;
                u[1] *= scale;
            }
            if (u[2] > params_.max_acceleration_z) {
                u[2] = params_.max_acceleration_z;
            } else if (u[2] < -params_.max_acceleration_z) {
                u[2] = -params_.max_acceleration_z;
            }
        }
    };

    const auto make_temporal_nominal = [&](int temporal_mode) {
        std::vector<float> controls = h_nominal_control_;
        if (temporal_mode == 1) {
            const float scale = params_.dynamic_temporal_cautious_scale;
            for (float& u : controls) {
                u *= scale;
            }
        } else if (temporal_mode == 2) {
            const int yield_steps =
                std::max(1, std::min(params_.dynamic_temporal_yield_steps,
                                     params_.horizon_steps));
            const float inv_stop_time = 1.0f / (params_.dt * yield_steps);
            const float stop_ax = static_cast<float>(-start_vel.x()) * inv_stop_time;
            const float stop_ay = static_cast<float>(-start_vel.y()) * inv_stop_time;
            const float stop_az = static_cast<float>(-start_vel.z()) * inv_stop_time;
            for (int t = 0; t < params_.horizon_steps; ++t) {
                float* u = controls.data() + t * CONTROL_DIM;
                if (t < yield_steps) {
                    const float fade =
                        1.0f - static_cast<float>(t) /
                                   static_cast<float>(std::max(1, yield_steps));
                    u[0] = stop_ax * fade;
                    u[1] = stop_ay * fade;
                    u[2] = stop_az * fade;
                } else {
                    const int src_t = std::max(0, t - yield_steps);
                    const float resume = std::min(
                        1.0f,
                        static_cast<float>(t - yield_steps + 1) /
                            static_cast<float>(std::max(1, yield_steps)));
                    u[0] = h_nominal_control_[src_t * CONTROL_DIM + 0] * resume;
                    u[1] = h_nominal_control_[src_t * CONTROL_DIM + 1] * resume;
                    u[2] = h_nominal_control_[src_t * CONTROL_DIM + 2] * resume;
                }
            }
        } else if (temporal_mode == 3) {
            const float scale = params_.dynamic_temporal_fast_scale;
            for (float& u : controls) {
                u *= scale;
            }
        }
        clamp_control_profile(controls);
        return controls;
    };

    for (int topo_mode = 0; topo_mode < num_topo_modes; ++topo_mode) {
        setNominalControlFromGuidePath(start_pos, start_vel, goal_pos, guide_paths[topo_mode]);
        for (int temporal_mode = 0; temporal_mode < temporal_modes; ++temporal_mode) {
            const int expanded_mode = topo_mode * temporal_modes + temporal_mode;
            const std::vector<float> temporal_nominal =
                make_temporal_nominal(temporal_mode);
            std::copy(temporal_nominal.begin(), temporal_nominal.end(),
                      h_multi_nominal.begin() +
                          static_cast<size_t>(expanded_mode) *
                              params_.horizon_steps * CONTROL_DIM);
        }
    }

    float h_initial_state[STATE_DIM] = {
        static_cast<float>(start_pos.x()), static_cast<float>(start_pos.y()), static_cast<float>(start_pos.z()),
        static_cast<float>(start_vel.x()), static_cast<float>(start_vel.y()), static_cast<float>(start_vel.z())
    };
    h_goal_pos_[0] = static_cast<float>(goal_pos.x());
    h_goal_pos_[1] = static_cast<float>(goal_pos.y());
    h_goal_pos_[2] = static_cast<float>(goal_pos.z());
    h_goal_vel_[0] = static_cast<float>(goal_vel.x());
    h_goal_vel_[1] = static_cast<float>(goal_vel.y());
    h_goal_vel_[2] = static_cast<float>(goal_vel.z());

    float* d_multi_nominal = nullptr;
    float* d_multi_costs = nullptr;
    float* d_multi_controls = nullptr;
    const size_t nominal_bytes = h_multi_nominal.size() * sizeof(float);
    const size_t costs_bytes = static_cast<size_t>(total_rollouts) * sizeof(float);
    const size_t controls_count =
        static_cast<size_t>(total_rollouts) * params_.horizon_steps * CONTROL_DIM;
    const size_t controls_bytes = controls_count * sizeof(float);

    CUDA_CHECK(cudaMalloc(&d_multi_nominal, nominal_bytes));
    CUDA_CHECK(cudaMalloc(&d_multi_costs, costs_bytes));
    CUDA_CHECK(cudaMalloc(&d_multi_controls, controls_bytes));

    CUDA_CHECK(cudaMemcpyAsync(d_initial_state_, h_initial_state,
                               STATE_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_goal_pos_, h_goal_pos_,
                               3 * sizeof(float), cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_goal_vel_, h_goal_vel_,
                               3 * sizeof(float), cudaMemcpyHostToDevice, stream_));

    SimpleDynamicsGPU dynamics;
    dynamics.max_velocity = params_.max_velocity;
    dynamics.max_acceleration = params_.max_acceleration;
    dynamics.max_velocity_z = params_.max_velocity_z;
    dynamics.max_acceleration_z = params_.max_acceleration_z;
    dynamics.dt = params_.dt;

    CostFunctionGPU cost_func;
    cost_func.w_obstacle = params_.w_obstacle;
    cost_func.w_smoothness = params_.w_smoothness;
    cost_func.w_goal = params_.w_goal;
    cost_func.w_velocity = params_.w_velocity;
    cost_func.safe_distance = params_.safe_distance;
    cost_func.near_collision_distance = params_.near_collision_distance;
    cost_func.near_collision_weight = params_.near_collision_weight;
    cost_func.min_z = params_.min_z;
    cost_func.max_z = params_.max_z;
    cost_func.w_height = params_.w_height;
    cost_func.edt_buffer = d_edt_buffer_;
    cost_func.grid_size_x = grid_size_x_;
    cost_func.grid_size_y = grid_size_y_;
    cost_func.grid_size_z = grid_size_z_;
    cost_func.resolution = grid_resolution_;
    cost_func.origin_x = grid_origin_x_;
    cost_func.origin_y = grid_origin_y_;
    cost_func.origin_z = grid_origin_z_;
    cost_func.w_dynamic = params_.w_dynamic;
    cost_func.dynamic_safe_distance = params_.dynamic_safe_distance;
    cost_func.dynamic_collision_distance = params_.dynamic_collision_distance;
    cost_func.dynamic_positions = d_dynamic_positions_;
    cost_func.dynamic_radii = d_dynamic_radii_;
    cost_func.dynamic_heights = d_dynamic_heights_;
    cost_func.num_dynamic_obstacles = num_dynamic_obstacles_;
    cost_func.dynamic_horizon = dynamic_horizon_;
    cost_func.dynamic_dt = dynamic_dt_;

    std::vector<float> h_multi_costs(total_rollouts, 1e10f);
    std::vector<float> h_multi_controls(controls_count, 0.0f);
    std::vector<float> h_updated_nominal(h_multi_nominal.size(), 0.0f);
    std::vector<float> mode_expected_costs(num_modes, 1e10f);
    std::vector<float> mode_best_costs(num_modes, 1e10f);

    const int num_blocks = (total_rollouts + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    const int num_iters = (params_.use_batch_iterative_update && params_.use_iterative_mppi)
                              ? std::max(1, params_.num_iterations)
                              : 1;

    for (int iter = 0; iter < num_iters; ++iter) {
        CUDA_CHECK(cudaMemcpyAsync(d_multi_nominal, h_multi_nominal.data(),
                                   nominal_bytes, cudaMemcpyHostToDevice, stream_));

        const unsigned long long seed =
            (params_.random_seed > 0ULL
                 ? params_.random_seed + (rollout_seed_counter_++) * 1000003ULL
                 : static_cast<unsigned long long>(
                       std::chrono::system_clock::now().time_since_epoch().count())) +
            static_cast<unsigned long long>(iter) * 9176ULL;
        mppiMultiModeRolloutKernel<<<num_blocks, THREADS_PER_BLOCK, 0, stream_>>>(
            d_initial_state_,
            d_goal_pos_,
            d_goal_vel_,
            d_multi_nominal,
            dynamics,
            cost_func,
            params_.sigma_acc,
            params_.horizon_steps,
            params_.num_samples,
            num_modes,
            d_multi_costs,
            d_multi_controls,
            seed);
        CUDA_CHECK(cudaGetLastError());

        CUDA_CHECK(cudaMemcpyAsync(h_multi_costs.data(), d_multi_costs,
                                   costs_bytes, cudaMemcpyDeviceToHost, stream_));
        CUDA_CHECK(cudaMemcpyAsync(h_multi_controls.data(), d_multi_controls,
                                   controls_bytes, cudaMemcpyDeviceToHost, stream_));
        CUDA_CHECK(cudaStreamSynchronize(stream_));

        std::fill(h_updated_nominal.begin(), h_updated_nominal.end(), 0.0f);
        for (int mode = 0; mode < num_modes; ++mode) {
            const int cost_base = mode * params_.num_samples;
            float min_cost = h_multi_costs[cost_base];
            for (int sample = 1; sample < params_.num_samples; ++sample) {
                min_cost = std::min(min_cost, h_multi_costs[cost_base + sample]);
            }
            mode_best_costs[mode] = min_cost;

            double weight_sum = 0.0;
            std::vector<float> weights(params_.num_samples, 0.0f);
            const float lambda = std::max(1e-3f, params_.lambda);
            for (int sample = 0; sample < params_.num_samples; ++sample) {
                const float cost = h_multi_costs[cost_base + sample];
                float exp_arg = -(cost - min_cost) / lambda;
                exp_arg = std::max(-50.0f, std::min(50.0f, exp_arg));
                const float w = std::isfinite(cost) && cost < 1e9f ? std::exp(exp_arg) : 0.0f;
                weights[sample] = w;
                weight_sum += w;
            }

            if (weight_sum <= 1e-8) {
                int best_sample = 0;
                for (int sample = 1; sample < params_.num_samples; ++sample) {
                    if (h_multi_costs[cost_base + sample] < h_multi_costs[cost_base + best_sample]) {
                        best_sample = sample;
                    }
                }
                const size_t best_offset =
                    (static_cast<size_t>(mode) * params_.num_samples + best_sample) *
                    params_.horizon_steps * CONTROL_DIM;
                const size_t nominal_offset =
                    static_cast<size_t>(mode) * params_.horizon_steps * CONTROL_DIM;
                std::copy(h_multi_controls.begin() + best_offset,
                          h_multi_controls.begin() + best_offset +
                              params_.horizon_steps * CONTROL_DIM,
                          h_updated_nominal.begin() + nominal_offset);
                mode_expected_costs[mode] = h_multi_costs[cost_base + best_sample];
                continue;
            }

            double expected_cost = 0.0;
            const size_t nominal_offset =
                static_cast<size_t>(mode) * params_.horizon_steps * CONTROL_DIM;
            for (int sample = 0; sample < params_.num_samples; ++sample) {
                const float normalized_w = static_cast<float>(weights[sample] / weight_sum);
                expected_cost += normalized_w * h_multi_costs[cost_base + sample];
                const size_t sample_offset =
                    (static_cast<size_t>(mode) * params_.num_samples + sample) *
                    params_.horizon_steps * CONTROL_DIM;
                for (int t = 0; t < params_.horizon_steps; ++t) {
                    for (int d = 0; d < CONTROL_DIM; ++d) {
                        const size_t idx = nominal_offset + t * CONTROL_DIM + d;
                        h_updated_nominal[idx] +=
                            normalized_w * h_multi_controls[sample_offset + t * CONTROL_DIM + d];
                    }
                }
            }
            mode_expected_costs[mode] = static_cast<float>(expected_cost);
        }

        h_multi_nominal.swap(h_updated_nominal);
    }

    std::vector<std::vector<Vector3d>> expanded_paths(num_modes);
    std::vector<float> expanded_costs(num_modes, 1e10f);
    optimal_paths.assign(num_topo_modes, {});
    costs.assign(num_topo_modes, 1e10f);
    int global_best_mode = -1;
    float global_best_cost = 1e10f;

    const auto evaluate_batch_clearance = [&](int rollout_idx) -> float {
        if (!params_.safety_filter_enabled || h_edt_buffer_.empty() ||
            rollout_idx < 0 || rollout_idx >= total_rollouts) {
            return -1.0f;
        }

        const size_t base = static_cast<size_t>(rollout_idx) *
                            params_.horizon_steps * CONTROL_DIM;
        float min_clearance = -1.0f;
        (void)evaluateControlsCost(start_pos, start_vel, goal_pos, goal_vel,
                                   h_multi_controls, base, &min_clearance);
        return min_clearance;
    };

    for (int mode = 0; mode < num_modes; ++mode) {
        const int cost_base = mode * params_.num_samples;
        int raw_best_sample = 0;
        float raw_best_cost = h_multi_costs[cost_base];
        for (int sample = 1; sample < params_.num_samples; ++sample) {
            const int idx = cost_base + sample;
            if (h_multi_costs[idx] < raw_best_cost) {
                raw_best_cost = h_multi_costs[idx];
                raw_best_sample = sample;
            }
        }

        int best_sample = raw_best_sample;
        float best_cost = raw_best_cost;
        float raw_clearance = -1.0f;
        float selected_clearance = -1.0f;

        if (params_.safety_filter_enabled) {
            const int raw_idx = cost_base + raw_best_sample;
            raw_clearance = evaluate_batch_clearance(raw_idx);
            selected_clearance = raw_clearance;

            if (raw_clearance < params_.safety_filter_preferred_clearance) {
                std::vector<int> order(params_.num_samples);
                std::iota(order.begin(), order.end(), 0);
                const int top_k = std::max(1, std::min(params_.safety_filter_top_k,
                                                       params_.num_samples));
                std::partial_sort(order.begin(), order.begin() + top_k, order.end(),
                                  [&](int a, int b) {
                                      return h_multi_costs[cost_base + a] <
                                             h_multi_costs[cost_base + b];
                                  });

                const float max_eligible_cost =
                    raw_best_cost + std::max(0.0f, params_.safety_filter_cost_slack);
                int fallback_sample = raw_best_sample;
                float fallback_clearance = raw_clearance;
                float fallback_cost = raw_best_cost;

                for (int rank = 0; rank < top_k; ++rank) {
                    const int sample = order[rank];
                    const float sample_cost = h_multi_costs[cost_base + sample];
                    if (sample_cost > max_eligible_cost) {
                        break;
                    }
                    const float clearance = evaluate_batch_clearance(cost_base + sample);
                    if (clearance > fallback_clearance) {
                        fallback_clearance = clearance;
                        fallback_sample = sample;
                        fallback_cost = sample_cost;
                    }
                    if (clearance >= params_.safety_filter_preferred_clearance) {
                        best_sample = sample;
                        best_cost = sample_cost;
                        selected_clearance = clearance;
                        break;
                    }
                }

                if (best_sample == raw_best_sample &&
                    fallback_sample != raw_best_sample &&
                    fallback_clearance >= params_.safety_filter_min_clearance) {
                    best_sample = fallback_sample;
                    best_cost = fallback_cost;
                    selected_clearance = fallback_clearance;
                }
            }

            if (best_sample != raw_best_sample) {
                ROS_INFO("[MPPI-GPU] Batch safety filter mode=%d selected sample=%d cost=%.2f clearance=%.2fm (raw sample=%d cost=%.2f clearance=%.2fm)",
                         mode + 1, best_sample, best_cost, selected_clearance,
                         raw_best_sample, raw_best_cost, raw_clearance);
            }
        } else {
            selected_clearance = evaluate_batch_clearance(cost_base + best_sample);
        }

        const size_t nominal_offset =
            static_cast<size_t>(mode) * params_.horizon_steps * CONTROL_DIM;
        float nominal_clearance = -1.0f;
        const float nominal_cost =
            params_.safety_filter_enabled && !h_edt_buffer_.empty()
                ? evaluateControlsCost(start_pos, start_vel, goal_pos, goal_vel,
                                       h_multi_nominal, nominal_offset, &nominal_clearance)
                : 1e10f;
        if (std::isfinite(nominal_cost) && nominal_cost < 1e9f &&
            nominal_clearance >= 0.0f && nominal_clearance <= 5.0f) {
            const float preferred_clearance = params_.safety_filter_enabled
                                                  ? params_.safety_filter_preferred_clearance
                                                  : params_.safety_filter_min_clearance;
            const float min_clearance = params_.safety_filter_enabled
                                            ? params_.safety_filter_min_clearance
                                            : 0.0f;
            const float cost_slack = std::max(0.0f, params_.safety_filter_cost_slack);
            const bool sample_unsafe = selected_clearance >= 0.0f &&
                                       selected_clearance < preferred_clearance;
            const bool nominal_preferred = nominal_clearance >= preferred_clearance &&
                                           nominal_cost <= best_cost + 1.5f * cost_slack;
            const bool nominal_safer = nominal_clearance >= min_clearance &&
                                       nominal_clearance > selected_clearance + 0.10f &&
                                       nominal_cost <= best_cost + 3.0f * cost_slack;
            const bool nominal_cheaper = nominal_clearance >= min_clearance &&
                                         nominal_cost < best_cost;

            if (nominal_preferred || (sample_unsafe && nominal_safer) || nominal_cheaper) {
                ROS_INFO("[MPPI-GPU] Batch nominal refinement mode=%d selected nominal cost=%.2f clearance=%.2fm (sample=%d cost=%.2f clearance=%.2fm)",
                         mode + 1, nominal_cost, nominal_clearance,
                         best_sample, best_cost, selected_clearance);
                rolloutControlsOnCPU(start_pos, start_vel, h_multi_nominal,
                                     nominal_offset, expanded_paths[mode]);
                expanded_costs[mode] = nominal_cost;
                if (expanded_costs[mode] < global_best_cost) {
                    global_best_cost = expanded_costs[mode];
                    global_best_mode = mode;
                }
                continue;
            }
        }

        const size_t control_offset =
            (static_cast<size_t>(mode) * params_.num_samples + best_sample) *
            params_.horizon_steps * CONTROL_DIM;
        rolloutControlsOnCPU(start_pos, start_vel, h_multi_controls,
                             control_offset, expanded_paths[mode]);
        expanded_costs[mode] = best_cost;

        if (expanded_costs[mode] < global_best_cost) {
            global_best_cost = expanded_costs[mode];
            global_best_mode = mode;
        }
    }

    for (int topo_mode = 0; topo_mode < num_topo_modes; ++topo_mode) {
        int best_temporal_mode = -1;
        float best_temporal_cost = 1e10f;
        for (int temporal_mode = 0; temporal_mode < temporal_modes; ++temporal_mode) {
            const int expanded_mode = topo_mode * temporal_modes + temporal_mode;
            if (expanded_costs[expanded_mode] < best_temporal_cost) {
                best_temporal_cost = expanded_costs[expanded_mode];
                best_temporal_mode = temporal_mode;
            }
        }
        if (best_temporal_mode >= 0 && best_temporal_cost < 1e9f) {
            const int expanded_mode = topo_mode * temporal_modes + best_temporal_mode;
            optimal_paths[topo_mode] = expanded_paths[expanded_mode];
            costs[topo_mode] = best_temporal_cost;
            if (use_temporal_modes) {
                ROS_INFO("[MPPI-GPU] Dynamic temporal selection topo=%d temporal=%d cost=%.2f",
                         topo_mode + 1, best_temporal_mode, best_temporal_cost);
            }
        }
    }
    last_best_cost_ = global_best_cost;

    cudaFree(d_multi_nominal);
    cudaFree(d_multi_costs);
    cudaFree(d_multi_controls);

    auto t_end = std::chrono::high_resolution_clock::now();
    last_timing_.rollout_time_ms =
        std::chrono::duration<float, std::milli>(t_end - t_start).count();
    last_timing_.weight_time_ms = 0.0f;
    last_timing_.total_time_ms = last_timing_.rollout_time_ms;

    ROS_INFO("[MPPI-GPU] Batched multi-mode MPPI: topo_modes=%d temporal_modes=%d samples/mode=%d iters=%d total_rollouts=%d best_mode=%d best_cost=%.2f total=%.2fms",
             num_topo_modes, temporal_modes, params_.num_samples, num_iters, total_rollouts,
             global_best_mode + 1, global_best_cost, last_timing_.total_time_ms);
    return global_best_mode >= 0 && global_best_cost < 1e9f;
}

void MPPIGPUPlanner::setNominalControlFromGuidePath(const Vector3d& start_pos,
                                                    const Vector3d& start_vel,
                                                    const Vector3d& goal_pos,
                                                    const std::vector<Vector3d>& guide_path) {
    h_nominal_control_.assign(params_.horizon_steps * CONTROL_DIM, 0.0f);

    std::vector<Vector3d> path = guide_path;
    if (path.empty()) {
        path.push_back(start_pos);
        path.push_back(goal_pos);
    } else {
        if ((path.front() - start_pos).norm() > 1e-3) {
            path.insert(path.begin(), start_pos);
        }
        if ((path.back() - goal_pos).norm() > 1e-3) {
            path.push_back(goal_pos);
        }
    }

    if (path.size() < 2 || params_.horizon_steps <= 0 || params_.dt <= 1e-4f) {
        return;
    }

    std::vector<double> cumulative(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i) {
        cumulative[i] = cumulative[i - 1] + (path[i] - path[i - 1]).norm();
    }

    const double total_length = cumulative.back();
    if (total_length < 1e-4) {
        return;
    }

    std::vector<Vector3d> desired_pos(params_.horizon_steps + 1, goal_pos);
    for (int t = 0; t <= params_.horizon_steps; ++t) {
        const double s = total_length * static_cast<double>(t) / static_cast<double>(params_.horizon_steps);
        size_t seg = 0;
        while (seg + 1 < cumulative.size() && cumulative[seg + 1] < s) {
            ++seg;
        }

        if (seg + 1 >= path.size()) {
            desired_pos[t] = path.back();
            continue;
        }

        const double seg_len = std::max(cumulative[seg + 1] - cumulative[seg], 1e-6);
        const double ratio = std::max(0.0, std::min(1.0, (s - cumulative[seg]) / seg_len));
        desired_pos[t] = path[seg] + ratio * (path[seg + 1] - path[seg]);
    }

    Vector3d prev_vel = start_vel;
    for (int t = 0; t < params_.horizon_steps; ++t) {
        Vector3d desired_vel = (desired_pos[t + 1] - desired_pos[t]) / params_.dt;
        const double vel_norm = desired_vel.norm();
        if (vel_norm > params_.max_velocity) {
            desired_vel = desired_vel / vel_norm * params_.max_velocity;
        }
        if (std::abs(desired_vel.z()) > params_.max_velocity_z) {
            desired_vel.z() = std::copysign(params_.max_velocity_z, desired_vel.z());
        }

        Vector3d acc = (desired_vel - prev_vel) / params_.dt;
        const double acc_xy = std::hypot(acc.x(), acc.y());
        if (acc_xy > params_.max_acceleration) {
            const double scale = params_.max_acceleration / acc_xy;
            acc.x() *= scale;
            acc.y() *= scale;
        }
        if (std::abs(acc.z()) > params_.max_acceleration_z) {
            acc.z() = std::copysign(params_.max_acceleration_z, acc.z());
        }

        h_nominal_control_[t * CONTROL_DIM + 0] = static_cast<float>(acc.x());
        h_nominal_control_[t * CONTROL_DIM + 1] = static_cast<float>(acc.y());
        h_nominal_control_[t * CONTROL_DIM + 2] = static_cast<float>(acc.z());
        prev_vel = desired_vel;
    }
}

void MPPIGPUPlanner::rolloutControlsOnCPU(const Vector3d& start_pos,
                                          const Vector3d& start_vel,
                                          const std::vector<float>& controls,
                                          size_t control_offset,
                                          std::vector<Vector3d>& path) const {
    Vector3d pos = start_pos;
    Vector3d vel = start_vel;
    path.clear();
    path.reserve(params_.horizon_steps);

    for (int t = 0; t < params_.horizon_steps; ++t) {
        path.push_back(pos);
        Vector3d acc(controls[control_offset + t * CONTROL_DIM + 0],
                     controls[control_offset + t * CONTROL_DIM + 1],
                     controls[control_offset + t * CONTROL_DIM + 2]);

        pos += vel * params_.dt;
        vel += acc * params_.dt;

        const double speed_xy = std::hypot(vel.x(), vel.y());
        if (speed_xy > params_.max_velocity && speed_xy > 1e-6) {
            const double scale = params_.max_velocity / speed_xy;
            vel.x() *= scale;
            vel.y() *= scale;
        }
        if (std::abs(vel.z()) > params_.max_velocity_z) {
            vel.z() = std::copysign(params_.max_velocity_z, vel.z());
        }
    }
}

void MPPIGPUPlanner::launchRolloutKernel() {
    // Setup dynamics
    SimpleDynamicsGPU dynamics;
    dynamics.max_velocity = params_.max_velocity;
    dynamics.max_acceleration = params_.max_acceleration;
    dynamics.max_velocity_z = params_.max_velocity_z;
    dynamics.max_acceleration_z = params_.max_acceleration_z;
    dynamics.dt = params_.dt;
    
    // Setup cost function (linear EDT buffer + dynamic obstacles)
    CostFunctionGPU cost_func;
    cost_func.w_obstacle = params_.w_obstacle;
    cost_func.w_smoothness = params_.w_smoothness;
    cost_func.w_goal = params_.w_goal;
    cost_func.w_velocity = params_.w_velocity;
    cost_func.safe_distance = params_.safe_distance;
    cost_func.near_collision_distance = params_.near_collision_distance;
    cost_func.near_collision_weight = params_.near_collision_weight;
    cost_func.min_z = params_.min_z;
    cost_func.max_z = params_.max_z;
    cost_func.w_height = params_.w_height;
    cost_func.edt_buffer = d_edt_buffer_;  // Pass linear EDT buffer to kernel
    cost_func.grid_size_x = grid_size_x_;
    cost_func.grid_size_y = grid_size_y_;
    cost_func.grid_size_z = grid_size_z_;
    cost_func.resolution = grid_resolution_;
    cost_func.origin_x = grid_origin_x_;
    cost_func.origin_y = grid_origin_y_;
    cost_func.origin_z = grid_origin_z_;
    
    // Dynamic obstacles
    cost_func.w_dynamic = params_.w_dynamic;
    cost_func.dynamic_safe_distance = params_.dynamic_safe_distance;
    cost_func.dynamic_collision_distance = params_.dynamic_collision_distance;
    cost_func.dynamic_positions = d_dynamic_positions_;
    cost_func.dynamic_radii = d_dynamic_radii_;
    cost_func.dynamic_heights = d_dynamic_heights_;
    cost_func.num_dynamic_obstacles = num_dynamic_obstacles_;
    cost_func.dynamic_horizon = dynamic_horizon_;
    cost_func.dynamic_dt = dynamic_dt_;
    
    // Kernel launch configuration
    int num_blocks = (params_.num_samples + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    // Random seed
    unsigned long long seed =
        params_.random_seed > 0ULL
            ? params_.random_seed + (rollout_seed_counter_++) * 1000003ULL
            : static_cast<unsigned long long>(
                  std::chrono::system_clock::now().time_since_epoch().count());
    
    // Launch kernel on stream (添加control_samples参数)
    mppiRolloutKernel<<<num_blocks, THREADS_PER_BLOCK, 0, stream_>>>(
        d_initial_state_,
        d_goal_pos_,
        d_goal_vel_,
        d_nominal_control_,
        dynamics,
        cost_func,
        params_.sigma_acc,
        params_.horizon_steps,
        params_.num_samples,
        d_trajectory_costs_,
        d_control_samples_,  // 存储采样控制
        seed
    );
    
    CUDA_CHECK(cudaGetLastError());
}

void MPPIGPUPlanner::computeWeights(float lambda) {
    int num_blocks = (params_.num_samples + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    // Reset min_cost and weight_sum (async)
    float init_min = 1e10f;
    float init_sum = 0.0f;
    CUDA_CHECK(cudaMemcpyAsync(d_min_cost_, &init_min, sizeof(float), 
                              cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_weight_sum_, &init_sum, sizeof(float), 
                              cudaMemcpyHostToDevice, stream_));
    
    // Compute weights (on stream, with provided lambda)
    computeWeightsKernel<<<num_blocks, THREADS_PER_BLOCK, 0, stream_>>>(
        d_trajectory_costs_,
        d_weights_,
        d_min_cost_,
        lambda,  // Use provided lambda (may be annealed)
        params_.num_samples
    );
    CUDA_CHECK(cudaGetLastError());
    
    // Normalize weights (on stream)
    normalizeWeightsKernel<<<num_blocks, THREADS_PER_BLOCK, 0, stream_>>>(
        d_weights_,
        d_weight_sum_,
        params_.num_samples
    );
    CUDA_CHECK(cudaGetLastError());
}

void MPPIGPUPlanner::updateNominalControl() {
    // Grid: num_timesteps, Block: THREADS_PER_BLOCK
    int num_blocks = params_.horizon_steps;
    
    // � 加权控制更新: u_new = u_nominal + Σ(w[i] * du[i]) (on stream)
    weightedControlUpdateKernel<<<num_blocks, THREADS_PER_BLOCK, 0, stream_>>>(
        d_weights_,
        d_control_samples_,
        d_nominal_control_,
        d_updated_control_,
        params_.horizon_steps,
        params_.num_samples
    );
    CUDA_CHECK(cudaGetLastError());
    
    // Copy updated control back to nominal (for next iteration, device-to-device)
    CUDA_CHECK(cudaMemcpyAsync(d_nominal_control_, d_updated_control_,
                         params_.horizon_steps * CONTROL_DIM * sizeof(float),
                         cudaMemcpyDeviceToDevice, stream_));
    
    // Also update host-side nominal control (async)
    CUDA_CHECK(cudaMemcpyAsync(h_nominal_control_.data(), d_updated_control_,
                         params_.horizon_steps * CONTROL_DIM * sizeof(float),
                         cudaMemcpyDeviceToHost, stream_));
}

float MPPIGPUPlanner::queryEDTDistance(float x, float y, float z) const {
    if (h_edt_buffer_.empty() || grid_size_x_ <= 1 || grid_size_y_ <= 1 ||
        grid_size_z_ <= 1 || grid_resolution_ <= 1e-5f) {
        return -1.0f;
    }

    const float fx = (x - grid_origin_x_) / grid_resolution_;
    const float fy = (y - grid_origin_y_) / grid_resolution_;
    const float fz = (z - grid_origin_z_) / grid_resolution_;

    const int ix = static_cast<int>(floorf(fx));
    const int iy = static_cast<int>(floorf(fy));
    const int iz = static_cast<int>(floorf(fz));

    if (ix < 0 || ix >= grid_size_x_ - 1 ||
        iy < 0 || iy >= grid_size_y_ - 1 ||
        iz < 0 || iz >= grid_size_z_ - 1) {
        return -1.0f;
    }

    const float tx = fx - static_cast<float>(ix);
    const float ty = fy - static_cast<float>(iy);
    const float tz = fz - static_cast<float>(iz);

    const auto get_value = [&](int i, int j, int k) -> float {
        const size_t idx = static_cast<size_t>(k) * grid_size_x_ * grid_size_y_ +
                           static_cast<size_t>(j) * grid_size_x_ + i;
        return h_edt_buffer_[idx];
    };

    const float c000 = get_value(ix, iy, iz);
    const float c100 = get_value(ix + 1, iy, iz);
    const float c010 = get_value(ix, iy + 1, iz);
    const float c110 = get_value(ix + 1, iy + 1, iz);
    const float c001 = get_value(ix, iy, iz + 1);
    const float c101 = get_value(ix + 1, iy, iz + 1);
    const float c011 = get_value(ix, iy + 1, iz + 1);
    const float c111 = get_value(ix + 1, iy + 1, iz + 1);

    const float c00 = c000 * (1.0f - tx) + c100 * tx;
    const float c01 = c001 * (1.0f - tx) + c101 * tx;
    const float c10 = c010 * (1.0f - tx) + c110 * tx;
    const float c11 = c011 * (1.0f - tx) + c111 * tx;
    const float c0 = c00 * (1.0f - ty) + c10 * ty;
    const float c1 = c01 * (1.0f - ty) + c11 * ty;
    return c0 * (1.0f - tz) + c1 * tz;
}

float MPPIGPUPlanner::evaluateControlsCost(const Vector3d& start_pos,
                                           const Vector3d& start_vel,
                                           const Vector3d& goal_pos,
                                           const Vector3d& goal_vel,
                                           const std::vector<float>& controls,
                                           size_t control_offset,
                                           float* min_clearance) const {
    if (controls.empty() ||
        control_offset + static_cast<size_t>(params_.horizon_steps) * CONTROL_DIM > controls.size()) {
        if (min_clearance) *min_clearance = -1.0f;
        return 1e10f;
    }

    Vector3d pos = start_pos;
    Vector3d vel = start_vel;
    float local_min_clearance = std::numeric_limits<float>::infinity();
    double total_cost = 0.0;

    for (int t = 0; t < params_.horizon_steps; ++t) {
        if (pos.z() < params_.min_z || pos.z() > params_.max_z) {
            if (min_clearance) *min_clearance = 0.0f;
            return 1e10f;
        }

        const float dist = queryEDTDistance(static_cast<float>(pos.x()),
                                            static_cast<float>(pos.y()),
                                            static_cast<float>(pos.z()));
        if (dist < 0.0f || dist > 100.0f) {
            if (min_clearance) *min_clearance = dist;
            return 1e10f;
        }
        local_min_clearance = std::min(local_min_clearance, std::min(dist, 5.0f));

        const Vector3d acc(controls[control_offset + t * CONTROL_DIM + 0],
                           controls[control_offset + t * CONTROL_DIM + 1],
                           controls[control_offset + t * CONTROL_DIM + 2]);
        const double z_error = pos.z() - goal_pos.z();
        total_cost += params_.w_height * z_error * z_error;
        if (z_error < -0.2) {
            total_cost += 4.0 * params_.w_height * z_error * z_error;
        }
        total_cost += params_.w_smoothness * acc.squaredNorm();
        if (dist < params_.safe_distance) {
            const double penetration = params_.safe_distance - dist;
            total_cost += params_.w_obstacle * penetration * penetration;
            if (dist < params_.near_collision_distance) {
                const double near_penetration = params_.near_collision_distance - dist;
                total_cost += params_.w_obstacle * params_.near_collision_weight *
                              near_penetration * near_penetration;
            }
        }

        pos += vel * params_.dt;
        vel += acc * params_.dt;

        const double speed_xy = std::hypot(vel.x(), vel.y());
        if (speed_xy > params_.max_velocity && speed_xy > 1e-6) {
            const double scale = params_.max_velocity / speed_xy;
            vel.x() *= scale;
            vel.y() *= scale;
        }
        if (std::abs(vel.z()) > params_.max_velocity_z) {
            vel.z() = std::copysign(params_.max_velocity_z, vel.z());
        }
    }

    const float final_dist = queryEDTDistance(static_cast<float>(pos.x()),
                                              static_cast<float>(pos.y()),
                                              static_cast<float>(pos.z()));
    if (final_dist < 0.0f || final_dist > 100.0f) {
        if (min_clearance) *min_clearance = final_dist;
        return 1e10f;
    }
    local_min_clearance = std::min(local_min_clearance, std::min(final_dist, 5.0f));

    const Vector3d pos_error = pos - goal_pos;
    const Vector3d vel_error = vel - goal_vel;
    total_cost += params_.w_goal * pos_error.squaredNorm();
    total_cost += params_.w_velocity * vel_error.squaredNorm();

    if (min_clearance) {
        *min_clearance = std::isfinite(local_min_clearance) ? local_min_clearance : -1.0f;
    }
    return static_cast<float>(std::min(total_cost, 1e10));
}

float MPPIGPUPlanner::evaluateCandidateClearance(int candidate_idx,
                                                 const Vector3d& start_pos,
                                                 const Vector3d& start_vel) const {
    if (candidate_idx < 0 || candidate_idx >= params_.num_samples ||
        h_control_samples_.empty()) {
        return -1.0f;
    }

    const size_t base = static_cast<size_t>(candidate_idx) *
                        params_.horizon_steps * CONTROL_DIM;
    float min_clearance = -1.0f;
    (void)evaluateControlsCost(start_pos, start_vel,
                               Vector3d(h_goal_pos_[0], h_goal_pos_[1], h_goal_pos_[2]),
                               Vector3d(h_goal_vel_[0], h_goal_vel_[1], h_goal_vel_[2]),
                               h_control_samples_, base, &min_clearance);
    return min_clearance;
}

int MPPIGPUPlanner::selectSafetyAwareBestIndex(const Vector3d& start_pos,
                                               const Vector3d& start_vel,
                                               float* selected_clearance,
                                               float* raw_best_clearance) const {
    if (params_.num_samples <= 0) {
        if (selected_clearance) *selected_clearance = -1.0f;
        if (raw_best_clearance) *raw_best_clearance = -1.0f;
        return 0;
    }

    int best_idx = 0;
    float min_cost = h_trajectory_costs_[0];
    for (int i = 1; i < params_.num_samples; i++) {
        if (h_trajectory_costs_[i] < min_cost) {
            min_cost = h_trajectory_costs_[i];
            best_idx = i;
        }
    }

    const float raw_clearance = evaluateCandidateClearance(best_idx, start_pos, start_vel);
    if (raw_best_clearance) *raw_best_clearance = raw_clearance;

    if (!params_.safety_filter_enabled ||
        raw_clearance >= params_.safety_filter_preferred_clearance ||
        h_control_samples_.empty()) {
        if (selected_clearance) *selected_clearance = raw_clearance;
        return best_idx;
    }

    std::vector<int> order(params_.num_samples);
    std::iota(order.begin(), order.end(), 0);
    const int top_k = std::max(1, std::min(params_.safety_filter_top_k, params_.num_samples));
    std::partial_sort(order.begin(), order.begin() + top_k, order.end(),
                      [&](int a, int b) {
                          return h_trajectory_costs_[a] < h_trajectory_costs_[b];
                      });

    const float max_eligible_cost = min_cost + std::max(0.0f, params_.safety_filter_cost_slack);
    int fallback_idx = best_idx;
    float fallback_clearance = raw_clearance;

    for (int rank = 0; rank < top_k; ++rank) {
        const int idx = order[rank];
        if (h_trajectory_costs_[idx] > max_eligible_cost) {
            break;
        }
        const float clearance = evaluateCandidateClearance(idx, start_pos, start_vel);
        if (clearance > fallback_clearance) {
            fallback_clearance = clearance;
            fallback_idx = idx;
        }
        if (clearance >= params_.safety_filter_preferred_clearance) {
            if (selected_clearance) *selected_clearance = clearance;
            return idx;
        }
    }

    if (fallback_idx != best_idx && fallback_clearance >= params_.safety_filter_min_clearance) {
        if (selected_clearance) *selected_clearance = fallback_clearance;
        return fallback_idx;
    }

    if (selected_clearance) *selected_clearance = raw_clearance;
    return best_idx;
}

void MPPIGPUPlanner::extractOptimalTrajectory(const Vector3d& start_pos,
                                              const Vector3d& start_vel,
                                              std::vector<Vector3d>& path) {
    float selected_clearance = -1.0f;
    float raw_best_clearance = -1.0f;
    const int best_idx = selectSafetyAwareBestIndex(start_pos, start_vel,
                                                    &selected_clearance,
                                                    &raw_best_clearance);
    const float min_cost = h_trajectory_costs_[best_idx];

    // Phase 2.5B: 保存best cost供外部使用
    last_best_cost_ = min_cost;
    
    if (selected_clearance > raw_best_clearance + 1e-3f) {
        ROS_INFO("[MPPI-GPU] Safety filter selected idx=%d cost=%.2f weight=%.4f clearance=%.2fm (raw_clearance=%.2fm)",
                 best_idx, min_cost, h_weights_[best_idx], selected_clearance, raw_best_clearance);
    } else {
        ROS_INFO("[MPPI-GPU] Best trajectory: idx=%d, cost=%.2f, weight=%.4f, clearance=%.2fm",
                 best_idx, min_cost, h_weights_[best_idx], selected_clearance);
    }

    float nominal_clearance = -1.0f;
    const float nominal_cost =
        params_.safety_filter_enabled && !h_edt_buffer_.empty()
            ? evaluateControlsCost(start_pos, start_vel,
                                   Vector3d(h_goal_pos_[0], h_goal_pos_[1], h_goal_pos_[2]),
                                   Vector3d(h_goal_vel_[0], h_goal_vel_[1], h_goal_vel_[2]),
                                   h_nominal_control_, 0, &nominal_clearance)
            : 1e10f;
    if (std::isfinite(nominal_cost) && nominal_cost < 1e9f &&
        nominal_clearance >= 0.0f && nominal_clearance <= 5.0f) {
        const float preferred_clearance = params_.safety_filter_enabled
                                              ? params_.safety_filter_preferred_clearance
                                              : params_.safety_filter_min_clearance;
        const float min_clearance = params_.safety_filter_enabled
                                        ? params_.safety_filter_min_clearance
                                        : 0.0f;
        const float cost_slack = std::max(0.0f, params_.safety_filter_cost_slack);
        const bool sample_unsafe = selected_clearance >= 0.0f &&
                                   selected_clearance < preferred_clearance;
        const bool nominal_preferred = nominal_clearance >= preferred_clearance &&
                                       nominal_cost <= min_cost + 1.5f * cost_slack;
        const bool nominal_safer = nominal_clearance >= min_clearance &&
                                   nominal_clearance > selected_clearance + 0.10f &&
                                   nominal_cost <= min_cost + 3.0f * cost_slack;
        const bool nominal_cheaper = nominal_clearance >= min_clearance &&
                                     nominal_cost < min_cost;
        if (nominal_preferred || (sample_unsafe && nominal_safer) || nominal_cheaper) {
            ROS_INFO("[MPPI-GPU] Nominal refinement selected cost=%.2f clearance=%.2fm (sample idx=%d cost=%.2f clearance=%.2fm)",
                     nominal_cost, nominal_clearance, best_idx, min_cost, selected_clearance);
            last_best_cost_ = nominal_cost;
            rolloutControlsOnCPU(start_pos, start_vel, h_nominal_control_, 0, path);
            return;
        }
    }
    
    // P0改进: Rollout并保存完整最优轨迹 (30点)
    // 准备初始状态
    float h_initial_state[STATE_DIM];
    h_initial_state[0] = start_pos.x();
    h_initial_state[1] = start_pos.y();
    h_initial_state[2] = start_pos.z();
    h_initial_state[3] = start_vel.x();
    h_initial_state[4] = start_vel.y();
    h_initial_state[5] = start_vel.z();
    
    // 上传初始状态到GPU
    CUDA_CHECK(cudaMemcpyAsync(d_initial_state_, h_initial_state, 
                               STATE_DIM * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));
    
    // 启动kernel: rollout最优轨迹
    SimpleDynamicsGPU dynamics;
    dynamics.max_velocity = params_.max_velocity;
    dynamics.max_acceleration = params_.max_acceleration;
    dynamics.max_velocity_z = params_.max_velocity_z;
    dynamics.max_acceleration_z = params_.max_acceleration_z;
    dynamics.dt = params_.dt;
    
    int num_blocks = (params_.horizon_steps + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    extractBestTrajectoryKernel<<<num_blocks, THREADS_PER_BLOCK, 0, stream_>>>(
        best_idx,
        d_control_samples_,
        d_initial_state_,
        d_best_trajectory_states_,
        params_.num_samples,
        params_.horizon_steps,
        dynamics
    );
    CUDA_CHECK(cudaGetLastError());
    
    // 异步拷贝轨迹状态回Host
    CUDA_CHECK(cudaMemcpyAsync(h_best_trajectory_states_, d_best_trajectory_states_,
                               params_.horizon_steps * 6 * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    
    // 同步等待完成
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    
    // 构建轨迹路径 (position only)
    path.clear();
    path.reserve(params_.horizon_steps);
    
    for (int t = 0; t < params_.horizon_steps; t++) {
        Vector3d pos(
            h_best_trajectory_states_[t * 6 + 0],
            h_best_trajectory_states_[t * 6 + 1],
            h_best_trajectory_states_[t * 6 + 2]
        );
        path.push_back(pos);
    }
    
    ROS_INFO("[MPPI-GPU] P0: Extracted full %d-point optimized trajectory (cost=%.2f)", 
             params_.horizon_steps, last_best_cost_);
}

} // namespace ego_planner
