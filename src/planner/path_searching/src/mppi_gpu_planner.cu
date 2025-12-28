#include "path_searching/mppi_gpu_planner.h"
#include "path_searching/mppi_cuda_kernel.cuh"
#include <ros/ros.h>
#include <chrono>

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
      d_dynamic_positions_(nullptr),
      d_dynamic_radii_(nullptr),
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
        
        // 🚀 Destroy stream if we own it
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
    
    // Allocate GPU memory
    allocateGPUMemory();
    
    initialized_ = true;
    ROS_INFO("[MPPI-GPU] Initialized with %d samples, horizon=%d", 
             params_.num_samples, params_.horizon_steps);
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
    
    // 🔥 Control samples和updated control
    size_t control_samples_size = params_.num_samples * params_.horizon_steps * CONTROL_DIM * sizeof(float);
    CUDA_CHECK(cudaMalloc(&d_control_samples_, control_samples_size));
    CUDA_CHECK(cudaMalloc(&d_updated_control_, params_.horizon_steps * CONTROL_DIM * sizeof(float)));
    
    // 🚀 P0改进: 分配最优轨迹状态内存 (position + velocity)
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
    
    // 🚀 P0改进: 释放轨迹状态内存
    if (d_best_trajectory_states_) cudaFree(d_best_trajectory_states_);
    if (h_best_trajectory_states_) cudaFreeHost(h_best_trajectory_states_);
    
    // Free EDT buffer (linear memory instead of texture)
    if (d_edt_buffer_) {
        cudaFree(d_edt_buffer_);
        d_edt_buffer_ = nullptr;
    }
    
    // 🔥 Free dynamic obstacles data
    if (d_dynamic_positions_) cudaFree(d_dynamic_positions_);
    if (d_dynamic_radii_) cudaFree(d_dynamic_radii_);
    
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
    
    // Free old EDT buffer if exists
    if (d_edt_buffer_) {
        cudaFree(d_edt_buffer_);
        d_edt_buffer_ = nullptr;
    }
    
    // Allocate linear EDT buffer (simple cudaMalloc, no 3D texture)
    size_t buffer_size = size_x * size_y * size_z * sizeof(float);
    CUDA_CHECK(cudaMalloc(&d_edt_buffer_, buffer_size));
    
    // Copy EDT data to GPU (linear memory)
    CUDA_CHECK(cudaMemcpy(d_edt_buffer_, edt_data, buffer_size, cudaMemcpyHostToDevice));
    
    ROS_INFO("[MPPI-GPU] EDT map uploaded as linear buffer: %dx%dx%d, res=%.2f, size=%zu MB", 
             size_x, size_y, size_z, resolution, buffer_size / (1024 * 1024));
}

void MPPIGPUPlanner::setDynamicObstacles(const std::vector<Eigen::Vector3d>& positions,
                                        const std::vector<float>& radii,
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
        num_dynamic_obstacles_ = 0;
        return;
    }
    
    // 🔥 Prepare host data in flat format
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
    
    // 🔥 Free old data
    if (d_dynamic_positions_) cudaFree(d_dynamic_positions_);
    if (d_dynamic_radii_) cudaFree(d_dynamic_radii_);
    
    // 🔥 Allocate and copy to GPU
    size_t pos_size = num_obstacles * horizon * sizeof(float3);
    size_t radii_size = num_obstacles * sizeof(float);
    
    CUDA_CHECK(cudaMalloc(&d_dynamic_positions_, pos_size));
    CUDA_CHECK(cudaMalloc(&d_dynamic_radii_, radii_size));
    
    CUDA_CHECK(cudaMemcpyAsync(d_dynamic_positions_, h_positions.data(), pos_size,
                              cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_dynamic_radii_, radii.data(), radii_size,
                              cudaMemcpyHostToDevice, stream_));
    
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    
    ROS_INFO("[MPPI-GPU] 🔥 Dynamic obstacles updated: %d obstacles × %d timesteps (dt=%.2fs)",
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
    
    // 🚀 Copy to GPU (Async transfers with stream)
    CUDA_CHECK(cudaMemcpyAsync(d_initial_state_, h_initial_state, 
                         STATE_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_goal_pos_, h_goal_pos_, 
                         3 * sizeof(float), cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_goal_vel_, h_goal_vel_, 
                         3 * sizeof(float), cudaMemcpyHostToDevice, stream_));
    CUDA_CHECK(cudaMemcpyAsync(d_nominal_control_, h_nominal_control_.data(),
                         params_.horizon_steps * CONTROL_DIM * sizeof(float), 
                         cudaMemcpyHostToDevice, stream_));
    
    // 🔥 Iterative MPPI Loop (on GPU)
    int num_iters = params_.use_iterative_mppi ? params_.num_iterations : 1;
    float total_rollout_ms = 0.0f;
    float total_weight_ms = 0.0f;
    float total_update_ms = 0.0f;
    
    for (int iter = 0; iter < num_iters; ++iter) {
        // ✅ 改进: 线性退火,保留30%温度 (参考MPPI-Generic)
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
        
        // 🔥 Update nominal control (MPPI核心算法, on stream)
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
    
    // 🚀 Copy results back (Async)
    CUDA_CHECK(cudaMemcpyAsync(h_trajectory_costs_.data(), d_trajectory_costs_,
                         params_.num_samples * sizeof(float), cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaMemcpyAsync(h_weights_.data(), d_weights_,
                         params_.num_samples * sizeof(float), cudaMemcpyDeviceToHost, stream_));
    
    // Wait for all async operations to complete
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    
    // Extract optimal trajectory (CPU)
    extractOptimalTrajectory(start_pos, start_vel, optimal_path);
    
    auto t_end = std::chrono::high_resolution_clock::now();
    last_timing_.total_time_ms = 
        std::chrono::duration<float, std::milli>(t_end - t_start).count();
    
    ROS_INFO("[MPPI-GPU] ⏱️ Timing: Total=%.2fms (Rollout=%.2fms, Weight=%.2fms, Update=%.2fms)",
             last_timing_.total_time_ms, last_timing_.rollout_time_ms, 
             last_timing_.weight_time_ms, update_time_ms);
    
    return true;
}

void MPPIGPUPlanner::launchRolloutKernel() {
    // Setup dynamics
    SimpleDynamicsGPU dynamics;
    dynamics.max_velocity = params_.max_velocity;
    dynamics.max_acceleration = params_.max_acceleration;
    dynamics.dt = params_.dt;
    
    // Setup cost function (linear EDT buffer + dynamic obstacles)
    CostFunctionGPU cost_func;
    cost_func.w_obstacle = params_.w_obstacle;
    cost_func.w_smoothness = params_.w_smoothness;
    cost_func.w_goal = params_.w_goal;
    cost_func.w_velocity = params_.w_velocity;
    cost_func.safe_distance = params_.safe_distance;
    cost_func.edt_buffer = d_edt_buffer_;  // Pass linear EDT buffer to kernel
    cost_func.grid_size_x = grid_size_x_;
    cost_func.grid_size_y = grid_size_y_;
    cost_func.grid_size_z = grid_size_z_;
    cost_func.resolution = grid_resolution_;
    cost_func.origin_x = grid_origin_x_;
    cost_func.origin_y = grid_origin_y_;
    cost_func.origin_z = grid_origin_z_;
    
    // 🔥 Dynamic obstacles
    cost_func.w_dynamic = 1.0f;  // TODO: Make this configurable
    cost_func.dynamic_positions = d_dynamic_positions_;
    cost_func.dynamic_radii = d_dynamic_radii_;
    cost_func.num_dynamic_obstacles = num_dynamic_obstacles_;
    cost_func.dynamic_horizon = dynamic_horizon_;
    cost_func.dynamic_dt = dynamic_dt_;
    
    // Kernel launch configuration
    int num_blocks = (params_.num_samples + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    // Random seed
    unsigned long long seed = std::chrono::system_clock::now().time_since_epoch().count();
    
    // 🚀 Launch kernel on stream (🔥 添加control_samples参数)
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
        d_control_samples_,  // 🔥 存储采样控制
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
    
    // 🚀 Compute weights (on stream, with provided lambda)
    computeWeightsKernel<<<num_blocks, THREADS_PER_BLOCK, 0, stream_>>>(
        d_trajectory_costs_,
        d_weights_,
        d_min_cost_,
        lambda,  // Use provided lambda (may be annealed)
        params_.num_samples
    );
    CUDA_CHECK(cudaGetLastError());
    
    // 🚀 Normalize weights (on stream)
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
    
    // 🔥 Copy updated control back to nominal (for next iteration, device-to-device)
    CUDA_CHECK(cudaMemcpyAsync(d_nominal_control_, d_updated_control_,
                         params_.horizon_steps * CONTROL_DIM * sizeof(float),
                         cudaMemcpyDeviceToDevice, stream_));
    
    // Also update host-side nominal control (async)
    CUDA_CHECK(cudaMemcpyAsync(h_nominal_control_.data(), d_updated_control_,
                         params_.horizon_steps * CONTROL_DIM * sizeof(float),
                         cudaMemcpyDeviceToHost, stream_));
}

void MPPIGPUPlanner::extractOptimalTrajectory(const Vector3d& start_pos,
                                              const Vector3d& start_vel,
                                              std::vector<Vector3d>& path) {
    // Find best trajectory (minimum cost)
    int best_idx = 0;
    float min_cost = h_trajectory_costs_[0];
    for (int i = 1; i < params_.num_samples; i++) {
        if (h_trajectory_costs_[i] < min_cost) {
            min_cost = h_trajectory_costs_[i];
            best_idx = i;
        }
    }
    
    // ✅ Phase 2.5B: 保存best cost供外部使用
    last_best_cost_ = min_cost;
    
    ROS_INFO("[MPPI-GPU] Best trajectory: idx=%d, cost=%.2f, weight=%.4f", 
             best_idx, min_cost, h_weights_[best_idx]);
    
    // 🚀 P0改进: Rollout并保存完整最优轨迹 (30点)
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
    
    ROS_INFO("[MPPI-GPU] 🚀 P0: Extracted full %d-point optimized trajectory (cost=%.2f)", 
             params_.horizon_steps, last_best_cost_);
}

} // namespace ego_planner
