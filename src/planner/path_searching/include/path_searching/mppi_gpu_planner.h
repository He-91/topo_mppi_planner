#ifndef _MPPI_GPU_PLANNER_H_
#define _MPPI_GPU_PLANNER_H_

#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <cuda_runtime.h>

namespace ego_planner {

class MPPIGPUPlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Vector3d = Eigen::Vector3d;
    
    struct Params {
        int num_samples = 1000;
        int horizon_steps = 30;
        float dt = 0.1f;
        float lambda = 1.0f;
        float sigma_acc = 2.0f;
        
        // 🔥 Iterative MPPI parameters
        bool use_iterative_mppi = false;
        int num_iterations = 1;  // Number of MPPI iterations (1 = standard)
        
        // Dynamics
        float max_velocity = 3.0f;
        float max_acceleration = 3.0f;
        
        // Cost weights (✅ Phase 2优化: 降低goal权重避免cost爆炸)
        float w_obstacle = 30.0f;      // 50→30: 降低障碍物惩罚
        float w_smoothness = 2.0f;     // 3→2: 降低平滑性惩罚
        float w_goal = 15.0f;          // 50→15: 关键修复! 降低目标距离代价
        float w_velocity = 10.0f;      // 20→10: 降低速度惩罚
        float safe_distance = 0.5f;
    };
    
    MPPIGPUPlanner();
    ~MPPIGPUPlanner();
    
    /**
     * @brief 初始化GPU资源 (with optional CUDA stream)
     * @param params MPPI parameters
     * @param stream CUDA stream for async operations (nullptr = default stream)
     */
    void initialize(const Params& params, cudaStream_t stream = nullptr);
    
    /**
     * @brief 设置EDT障碍物地图
     */
    void setEDTMap(const float* edt_data, int size_x, int size_y, int size_z,
                   float resolution, float origin_x, float origin_y, float origin_z);
    
    /**
     * @brief 🔥 设置动态障碍物预测数据
     * @param positions Predicted positions [num_obstacles * horizon_steps] as (x,y,z) triplets
     * @param radii Obstacle radii [num_obstacles]
     * @param num_obstacles Number of dynamic obstacles
     * @param horizon Number of prediction timesteps
     * @param dt Time resolution between predictions
     */
    void setDynamicObstacles(const std::vector<Eigen::Vector3d>& positions,
                            const std::vector<float>& radii,
                            int num_obstacles,
                            int horizon,
                            float dt);
    
    /**
     * @brief GPU版本的MPPI规划
     */
    bool plan(const Vector3d& start_pos,
              const Vector3d& start_vel,
              const Vector3d& goal_pos,
              const Vector3d& goal_vel,
              std::vector<Vector3d>& optimal_path);
    
    /**
     * @brief 获取timing信息
     */
    struct Timing {
        float rollout_time_ms;
        float weight_time_ms;
        float total_time_ms;
    };
    Timing getLastTiming() const { return last_timing_; }
    
    /**
     * @brief 获取最优轨迹的cost
     */
    float getLastBestCost() const { return last_best_cost_; }
    
private:
    Params params_;
    Timing last_timing_;
    
    // 🚀 CUDA Stream for async operations (inspired by MPPI-Generic)
    cudaStream_t stream_;
    bool own_stream_;  // Whether we own the stream (need to destroy it)
    
    // GPU memory pointers
    float* d_initial_state_;      // Device: initial state
    float* d_goal_pos_;           // Device: goal position
    float* d_goal_vel_;           // Device: goal velocity
    float* d_nominal_control_;    // Device: nominal control sequence
    float* d_trajectory_costs_;   // Device: trajectory costs
    float* d_weights_;            // Device: importance weights
    float* d_min_cost_;           // Device: minimum cost
    float* d_weight_sum_;         // Device: sum of weights
    
    // � FIX: Simple linear EDT buffer instead of 3D texture (avoid malloc bug)
    float* d_edt_buffer_;         // Device: linear EDT buffer [size_x * size_y * size_z]
    
    // 🔥 Dynamic obstacles GPU data
    float3* d_dynamic_positions_;  // Device: [num_obstacles * horizon] predicted positions
    float* d_dynamic_radii_;       // Device: [num_obstacles] radii
    int num_dynamic_obstacles_;
    int dynamic_horizon_;
    float dynamic_dt_;
    
    float* d_control_samples_;    // 🔥 Device: sampled controls [num_samples * horizon * 3]
    float* d_updated_control_;    // 🔥 Device: updated control sequence [horizon * 3]
    
    // Host memory
    std::vector<float> h_trajectory_costs_;
    std::vector<float> h_weights_;
    std::vector<float> h_nominal_control_;
    float h_goal_pos_[3];  // 用于extractOptimalTrajectory
    float h_goal_vel_[3];
    
    // EDT grid info
    int grid_size_x_, grid_size_y_, grid_size_z_;
    float grid_resolution_;
    float grid_origin_x_, grid_origin_y_, grid_origin_z_;
    
    bool initialized_;
    
    // 最优轨迹cost (Phase 2.5B修复)
    float last_best_cost_;
    
    // 🚀 P0改进: 保存最优轨迹的完整状态
    float* d_best_trajectory_states_;  // Device: [horizon_steps * 6] (x,y,z,vx,vy,vz)
    float* h_best_trajectory_states_;  // Host pinned memory
    
    /**
     * @brief 分配GPU内存
     */
    void allocateGPUMemory();
    
    /**
     * @brief 释放GPU内存
     */
    void freeGPUMemory();
    
    /**
     * @brief 执行GPU rollout
     */
    void launchRolloutKernel();
    
    /**
     * @brief 计算重要性采样权重
     * @param lambda Temperature parameter (may be annealed in iterative MPPI)
     */
    void computeWeights(float lambda);
    
    /**
     * @brief 🔥 加权控制更新 - MPPI核心算法
     */
    void updateNominalControl();
    
    /**
     * @brief 提取最优轨迹(CPU端后处理)
     */
    void extractOptimalTrajectory(const Vector3d& start_pos,
                                  const Vector3d& start_vel,
                                  std::vector<Vector3d>& path);
};

} // namespace ego_planner

#endif // _MPPI_GPU_PLANNER_H_
