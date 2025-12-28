#ifndef _MPPI_CUDA_KERNEL_CUH_
#define _MPPI_CUDA_KERNEL_CUH_

#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <Eigen/Dense>

namespace ego_planner {

// CUDA错误检查宏
#define CUDA_CHECK(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            fprintf(stderr, "CUDA Error: %s:%d, code: %d, reason: %s\n", \
                    __FILE__, __LINE__, error, cudaGetErrorString(error)); \
            exit(1); \
        } \
    } while(0)

// 常量定义
#define MAX_HORIZON 50
#define STATE_DIM 6   // [px, py, pz, vx, vy, vz]
#define CONTROL_DIM 3 // [ax, ay, az]
#define THREADS_PER_BLOCK 256

/**
 * @brief Device端简化四旋翼动力学
 */
struct SimpleDynamicsGPU {
    float max_velocity;
    float max_acceleration;
    float dt;
    
    __device__ void step(const float* state, const float* control, 
                        float* next_state) const {
        // Euler integration: s[t+1] = s[t] + ds/dt * dt
        // pos_next = pos + vel * dt
        next_state[0] = state[0] + state[3] * dt;
        next_state[1] = state[1] + state[4] * dt;
        next_state[2] = state[2] + state[5] * dt;
        
        // vel_next = vel + acc * dt
        next_state[3] = state[3] + control[0] * dt;
        next_state[4] = state[4] + control[1] * dt;
        next_state[5] = state[5] + control[2] * dt;
        
        // Velocity constraints
        float speed = sqrtf(next_state[3]*next_state[3] + 
                           next_state[4]*next_state[4] + 
                           next_state[5]*next_state[5]);
        if (speed > max_velocity) {
            float scale = max_velocity / speed;
            next_state[3] *= scale;
            next_state[4] *= scale;
            next_state[5] *= scale;
        }
    }
    
    __device__ void enforceConstraints(float* control) const {
        // Acceleration magnitude constraint
        float acc_mag = sqrtf(control[0]*control[0] + 
                             control[1]*control[1] + 
                             control[2]*control[2]);
        if (acc_mag > max_acceleration) {
            float scale = max_acceleration / acc_mag;
            control[0] *= scale;
            control[1] *= scale;
            control[2] *= scale;
        }
    }
};

/**
 * @brief Device端代价函数 (🚀 Using 3D Texture Memory for EDT)
 */
struct CostFunctionGPU {
    float w_obstacle;
    float w_smoothness;
    float w_goal;
    float w_velocity;
    float safe_distance;
    
    // � FIX: Simple linear EDT buffer instead of texture (avoid malloc bug)
    float* edt_buffer;  // [size_x * size_y * size_z] linear array
    int grid_size_x, grid_size_y, grid_size_z;
    float resolution;
    float origin_x, origin_y, origin_z;
    
    // 🔥 Dynamic obstacles data
    float w_dynamic;  // Dynamic obstacle weight
    float3* dynamic_positions;  // [num_obstacles * horizon_steps] predicted positions
    float* dynamic_radii;       // [num_obstacles] obstacle radii
    int num_dynamic_obstacles;
    int dynamic_horizon;
    float dynamic_dt;  // Time resolution for dynamic predictions
    
    __device__ float computeRunningCost(const float* state, 
                                       const float* control,
                                       const float* goal_pos,
                                       int timestep) const {
        float cost = 0.0f;
        
        // 1. Smoothness cost (control effort)
        cost += w_smoothness * (control[0]*control[0] + 
                               control[1]*control[1] + 
                               control[2]*control[2]);
        
        // 2. Static obstacle cost (EDT-based)
        float dist = getDistance(state[0], state[1], state[2]);
        if (dist < 0.0f) {
            return 1e10f; // Collision
        }
        if (dist < safe_distance) {
            float penetration = safe_distance - dist;
            cost += w_obstacle * penetration * penetration;
        }
        
        // 🔥 3. Dynamic obstacle cost (time-synchronized)
        if (num_dynamic_obstacles > 0 && dynamic_positions != nullptr) {
            float dynamic_cost = computeDynamicObstacleCost(
                state[0], state[1], state[2], timestep);
            if (dynamic_cost > 1e9f) {
                return 1e10f; // Dynamic collision
            }
            cost += dynamic_cost;
        }
        
        return cost;
    }
    
    __device__ float computeTerminalCost(const float* final_state,
                                        const float* goal_pos,
                                        const float* goal_vel) const {
        float cost = 0.0f;
        
        // Position error
        float dx = final_state[0] - goal_pos[0];
        float dy = final_state[1] - goal_pos[1];
        float dz = final_state[2] - goal_pos[2];
        cost += w_goal * (dx*dx + dy*dy + dz*dz);
        
        // Velocity error
        float dvx = final_state[3] - goal_vel[0];
        float dvy = final_state[4] - goal_vel[1];
        float dvz = final_state[5] - goal_vel[2];
        cost += w_velocity * (dvx*dvx + dvy*dvy + dvz*dvz);
        
        return cost;
    }
    
    __device__ float getDistance(float x, float y, float z) const {
        // � Convert world coordinates to grid indices
        float fx = (x - origin_x) / resolution;
        float fy = (y - origin_y) / resolution;
        float fz = (z - origin_z) / resolution;
        
        // Integer grid coordinates
        int ix = (int)floorf(fx);
        int iy = (int)floorf(fy);
        int iz = (int)floorf(fz);
        
        // Boundary check
        if (ix < 0 || ix >= grid_size_x - 1 ||
            iy < 0 || iy >= grid_size_y - 1 ||
            iz < 0 || iz >= grid_size_z - 1) {
            return -1.0f; // Out of bounds = collision
        }
        
        // � Manual trilinear interpolation (since we can't use texture)
        // Get fractional part for interpolation
        float tx = fx - (float)ix;
        float ty = fy - (float)iy;
        float tz = fz - (float)iz;
        
        // Fetch 8 neighbor values
        auto get_value = [&](int i, int j, int k) -> float {
            int idx = k * (grid_size_x * grid_size_y) + j * grid_size_x + i;
            return edt_buffer[idx];
        };
        
        float c000 = get_value(ix, iy, iz);
        float c100 = get_value(ix+1, iy, iz);
        float c010 = get_value(ix, iy+1, iz);
        float c110 = get_value(ix+1, iy+1, iz);
        float c001 = get_value(ix, iy, iz+1);
        float c101 = get_value(ix+1, iy, iz+1);
        float c011 = get_value(ix, iy+1, iz+1);
        float c111 = get_value(ix+1, iy+1, iz+1);
        
        // Interpolate in x direction
        float c00 = c000 * (1.0f - tx) + c100 * tx;
        float c01 = c001 * (1.0f - tx) + c101 * tx;
        float c10 = c010 * (1.0f - tx) + c110 * tx;
        float c11 = c011 * (1.0f - tx) + c111 * tx;
        
        // Interpolate in y direction
        float c0 = c00 * (1.0f - ty) + c10 * ty;
        float c1 = c01 * (1.0f - ty) + c11 * ty;
        
        // Interpolate in z direction
        float dist = c0 * (1.0f - tz) + c1 * tz;
        
        return dist;
    }
    
    // 🔥 NEW: Compute dynamic obstacle cost (time-synchronized)
    __device__ float computeDynamicObstacleCost(float x, float y, float z, int timestep) const {
        if (num_dynamic_obstacles == 0 || dynamic_positions == nullptr) {
            return 0.0f;
        }
        
        // Clamp timestep to valid range
        int t = min(timestep, dynamic_horizon - 1);
        
        float min_dist = 1e10f;
        
        // Check distance to all dynamic obstacles at this timestep
        for (int i = 0; i < num_dynamic_obstacles; ++i) {
            // Get predicted position of obstacle i at time t
            int idx = i * dynamic_horizon + t;
            float3 obs_pos = dynamic_positions[idx];
            
            // Compute distance to obstacle center
            float dx = x - obs_pos.x;
            float dy = y - obs_pos.y;
            float dz = z - obs_pos.z;
            float dist_to_center = sqrtf(dx*dx + dy*dy + dz*dz);
            
            // Subtract obstacle radius
            float dist = dist_to_center - dynamic_radii[i];
            min_dist = fminf(min_dist, dist);
        }
        
        // Apply cost based on minimum distance
        if (min_dist < 0.0f) {
            // Collision with dynamic obstacle
            return 1e10f;
        }
        
        if (min_dist < safe_distance) {
            float penetration = safe_distance - min_dist;
            return w_dynamic * w_obstacle * penetration * penetration;
        }
        
        return 0.0f;
    }
};

/**
 * @brief MPPI Rollout Kernel (存储control samples用于后续加权更新)
 * 每个线程处理一条轨迹
 */
__global__ void mppiRolloutKernel(
    const float* initial_state,      // [STATE_DIM] 初始状态
    const float* goal_pos,            // [3] 目标位置
    const float* goal_vel,            // [3] 目标速度
    const float* nominal_control,     // [horizon * CONTROL_DIM] nominal控制序列
    SimpleDynamicsGPU dynamics,       // 动力学参数
    CostFunctionGPU cost_func,        // 代价函数参数
    float sigma_acc,                  // 采样噪声标准差
    int horizon_steps,                // 规划horizon
    int num_rollouts,                 // rollout总数
    float* trajectory_costs,          // [num_rollouts] 输出: 每条轨迹的代价
    float* control_samples,           // [num_rollouts * horizon * CONTROL_DIM] 输出: 采样的控制序列
    unsigned long long seed           // 随机种子
) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (tid >= num_rollouts) return;
    
    // 初始化随机数生成器(每个线程独立)
    curandState rand_state;
    curand_init(seed, tid, 0, &rand_state);
    
    // Thread-local state和control
    float state[STATE_DIM];
    float next_state[STATE_DIM];
    float control[CONTROL_DIM];
    
    // 初始化状态
    #pragma unroll
    for (int i = 0; i < STATE_DIM; i++) {
        state[i] = initial_state[i];
    }
    
    float total_cost = 0.0f;
    bool collision = false;
    
    // Rollout trajectory
    for (int t = 0; t < horizon_steps; t++) {
        // 采样控制: u = u_nominal + noise
        #pragma unroll
        for (int i = 0; i < CONTROL_DIM; i++) {
            float noise = curand_normal(&rand_state) * sigma_acc;
            control[i] = nominal_control[t * CONTROL_DIM + i] + noise;
        }
        
        // 约束控制
        dynamics.enforceConstraints(control);
        
        // 🔥 存储采样的控制(用于后续加权更新)
        int control_idx = tid * horizon_steps * CONTROL_DIM + t * CONTROL_DIM;
        #pragma unroll
        for (int i = 0; i < CONTROL_DIM; i++) {
            control_samples[control_idx + i] = control[i];
        }
        
        // 计算running cost (🔥 pass timestep for dynamic obstacles)
        float running_cost = cost_func.computeRunningCost(state, control, goal_pos, t);
        
        // 检查碰撞
        if (running_cost > 1e9f) {
            collision = true;
            total_cost = 1e10f;
            break;
        }
        
        total_cost += running_cost;
        
        // 状态转移
        dynamics.step(state, control, next_state);
        
        // 更新状态
        #pragma unroll
        for (int i = 0; i < STATE_DIM; i++) {
            state[i] = next_state[i];
        }
    }
    
    // 计算terminal cost
    if (!collision) {
        float terminal_cost = cost_func.computeTerminalCost(state, goal_pos, goal_vel);
        total_cost += terminal_cost;
    }
    
    // 保存代价
    trajectory_costs[tid] = total_cost;
}

/**
 * @brief 计算重要性采样权重 (GPU reduction)
 */
__global__ void computeWeightsKernel(
    const float* trajectory_costs,   // [num_rollouts]
    float* weights,                  // [num_rollouts] 输出
    float* min_cost,                 // [1] 输出最小代价
    float lambda,                    // Temperature参数
    int num_rollouts
) {
    __shared__ float shared_costs[THREADS_PER_BLOCK];
    __shared__ float shared_min;
    
    int tid = threadIdx.x;
    int gid = blockIdx.x * blockDim.x + threadIdx.x;
    
    // Load cost to shared memory
    float cost = (gid < num_rollouts) ? trajectory_costs[gid] : 1e10f;
    shared_costs[tid] = cost;
    __syncthreads();
    
    // Block-level reduction to find min
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            shared_costs[tid] = fminf(shared_costs[tid], shared_costs[tid + s]);
        }
        __syncthreads();
    }
    
    // Block min
    if (tid == 0) {
        atomicMin((int*)min_cost, __float_as_int(shared_costs[0]));
    }
    __syncthreads();
    
    // 等待global min计算完成
    __threadfence();
    
    // 计算权重: exp(-(cost - min_cost) / lambda)
    if (gid < num_rollouts) {
        float exp_arg = -(cost - (*min_cost)) / lambda;
        // 数值稳定性: 限制exp参数范围
        exp_arg = fminf(exp_arg, 50.0f);
        exp_arg = fmaxf(exp_arg, -50.0f);
        weights[gid] = expf(exp_arg);
    }
}

/**
 * @brief Normalize weights (parallel reduction for sum)
 */
__global__ void normalizeWeightsKernel(
    float* weights,
    float* weight_sum,
    int num_rollouts
) {
    __shared__ float shared_sum[THREADS_PER_BLOCK];
    
    int tid = threadIdx.x;
    int gid = blockIdx.x * blockDim.x + threadIdx.x;
    
    // Load weight
    shared_sum[tid] = (gid < num_rollouts) ? weights[gid] : 0.0f;
    __syncthreads();
    
    // Reduction sum
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            shared_sum[tid] += shared_sum[tid + s];
        }
        __syncthreads();
    }
    
    // Block sum
    if (tid == 0) {
        atomicAdd(weight_sum, shared_sum[0]);
    }
    __syncthreads();
    
    // 归一化
    __threadfence();
    if (gid < num_rollouts && (*weight_sum) > 1e-8f) {
        weights[gid] /= (*weight_sum);
    }
}

/**
 * @brief 🔥 加权控制更新Kernel - MPPI核心算法
 * u_new[t] = u_nominal[t] + Σ(w[i] * (u_sample[i][t] - u_nominal[t])) / Σ(w[i])
 *          = u_nominal[t] + Σ(w[i] * du[i][t])
 * 
 * Grid:  num_timesteps blocks
 * Block: THREADS_PER_BLOCK threads (并行处理rollouts)
 */
__global__ void weightedControlUpdateKernel(
    const float* weights,             // [num_rollouts] 归一化后的权重
    const float* control_samples,     // [num_rollouts * horizon * CONTROL_DIM] 采样控制
    const float* nominal_control,     // [horizon * CONTROL_DIM] 当前nominal
    float* updated_control,           // [horizon * CONTROL_DIM] 输出: 更新后的控制
    int horizon_steps,
    int num_rollouts
) {
    int t = blockIdx.x;  // Timestep index
    int tid = threadIdx.x;  // Thread index within block
    
    if (t >= horizon_steps) return;
    
    // Shared memory for reduction
    __shared__ float shared_weighted_du[THREADS_PER_BLOCK * CONTROL_DIM];
    
    // Initialize local weighted du
    float local_weighted_du[CONTROL_DIM];
    #pragma unroll
    for (int d = 0; d < CONTROL_DIM; d++) {
        local_weighted_du[d] = 0.0f;
    }
    
    // 每个线程处理多个rollouts (stride循环)
    for (int i = tid; i < num_rollouts; i += blockDim.x) {
        float w = weights[i];
        
        // 读取采样控制: u_sample[i][t]
        int sample_idx = i * horizon_steps * CONTROL_DIM + t * CONTROL_DIM;
        
        #pragma unroll
        for (int d = 0; d < CONTROL_DIM; d++) {
            float u_sample = control_samples[sample_idx + d];
            float u_nominal = nominal_control[t * CONTROL_DIM + d];
            float du = u_sample - u_nominal;  // Control deviation
            local_weighted_du[d] += w * du;
        }
    }
    
    // Store to shared memory
    #pragma unroll
    for (int d = 0; d < CONTROL_DIM; d++) {
        shared_weighted_du[tid * CONTROL_DIM + d] = local_weighted_du[d];
    }
    __syncthreads();
    
    // Parallel reduction (sum across threads)
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            #pragma unroll
            for (int d = 0; d < CONTROL_DIM; d++) {
                shared_weighted_du[tid * CONTROL_DIM + d] += 
                    shared_weighted_du[(tid + s) * CONTROL_DIM + d];
            }
        }
        __syncthreads();
    }
    
    // Thread 0 writes the result: u_new = u_nominal + weighted_du
    if (tid == 0) {
        #pragma unroll
        for (int d = 0; d < CONTROL_DIM; d++) {
            updated_control[t * CONTROL_DIM + d] = 
                nominal_control[t * CONTROL_DIM + d] + shared_weighted_du[d];
        }
    }
}

/**
 * @brief 🚀 P0改进: Rollout最优轨迹并保存完整状态
 * @param best_trajectory_idx 最优轨迹的索引
 * @param control_samples 控制采样 [num_samples * horizon * CONTROL_DIM]
 * @param initial_state 初始状态 [STATE_DIM]
 * @param output_states 输出: 完整轨迹状态 [horizon * 6] (x,y,z,vx,vy,vz)
 */
__global__ void extractBestTrajectoryKernel(
    int best_trajectory_idx,
    const float* control_samples,
    const float* initial_state,
    float* output_states,
    int num_samples,
    int horizon_steps,
    SimpleDynamicsGPU dynamics)
{
    int t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= horizon_steps) return;
    
    // Rollout最优控制序列
    float state[STATE_DIM];
    
    // 初始化状态
    if (t == 0) {
        #pragma unroll
        for (int i = 0; i < STATE_DIM; i++) {
            state[i] = initial_state[i];
        }
    } else {
        // 从前一步继承 (需要同步,这里简化为重新rollout)
        #pragma unroll
        for (int i = 0; i < STATE_DIM; i++) {
            state[i] = initial_state[i];
        }
        
        // Rollout到第t步
        for (int step = 0; step < t; step++) {
            const float* control = &control_samples[
                best_trajectory_idx * horizon_steps * CONTROL_DIM + 
                step * CONTROL_DIM
            ];
            
            float next_state[STATE_DIM];
            dynamics.step(state, control, next_state);
            
            #pragma unroll
            for (int i = 0; i < STATE_DIM; i++) {
                state[i] = next_state[i];
            }
        }
    }
    
    // 保存当前状态 (位置+速度)
    output_states[t * 6 + 0] = state[0];  // x
    output_states[t * 6 + 1] = state[1];  // y
    output_states[t * 6 + 2] = state[2];  // z
    output_states[t * 6 + 3] = state[3];  // vx
    output_states[t * 6 + 4] = state[4];  // vy
    output_states[t * 6 + 5] = state[5];  // vz
}

} // namespace ego_planner

#endif // _MPPI_CUDA_KERNEL_CUH_
