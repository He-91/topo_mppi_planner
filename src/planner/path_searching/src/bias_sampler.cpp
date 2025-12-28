#include "path_searching/bias_sampler.h"
#include <cmath>
#include <algorithm>
#include <random>

using namespace std;
using namespace Eigen;

namespace ego_planner {

BiasSampler::BiasSampler() 
    : corner_detection_radius_(3.0),
      sampling_radius_(2.0),
      resolution_(0.1),
      collision_check_res_(0.05),
      max_corner_num_(60) {  // � ENHANCED: 40→60,增加覆盖密度,减少丢失关键点
}

BiasSampler::~BiasSampler() {
}

void BiasSampler::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    
    // Load parameters
    nh.param("bias_sampler/corner_detection_radius", corner_detection_radius_, 3.0);
    nh.param("bias_sampler/sampling_radius", sampling_radius_, 2.0);
    nh.param("bias_sampler/resolution", resolution_, 0.1);
    nh.param("bias_sampler/max_corner_num", max_corner_num_, 60);  // 默认60
    
    ROS_INFO("[BiasSampler] Initialized with:");
    ROS_INFO("  - Corner detection radius: %.2f", corner_detection_radius_);
    ROS_INFO("  - Sampling radius: %.2f", sampling_radius_);
    ROS_INFO("  - Resolution: %.2f", resolution_);
    ROS_INFO("  - Max corner number: %d", max_corner_num_);
}

bool BiasSampler::findSamplingSpaces(const Vector3d& start,
                                     const Vector3d& goal,
                                     vector<SamplingSpace>& spaces) {
    spaces.clear();
    
    // Step 1: Detect obstacle corners between start and goal
    vector<Vector3d> corners = detectObstacleCorners(start, goal);
    
    if (corners.empty()) {
        ROS_INFO("[BiasSampler] No obstacle corners detected - path may be direct");
        return true;  // Not an error - direct path possible
    }
    
    ROS_INFO("[BiasSampler] Detected %zu obstacle corners", corners.size());
    
    // Step 2: Build sampling spaces around each corner
    for (size_t i = 0; i < corners.size(); ++i) {
        int topo_id = calculateTopoClass(corners[i], start, goal);
        SamplingSpace space = buildSamplingSpace(corners[i], topo_id);
        spaces.push_back(space);
    }
    
    ROS_INFO("[BiasSampler] Created %zu sampling spaces", spaces.size());
    return true;
}

vector<Vector3d> BiasSampler::detectObstacleCorners(const Vector3d& start,
                                                     const Vector3d& goal) {
    vector<Vector3d> corners;
    
    // Define search region around direct path
    Vector3d dir = (goal - start).normalized();
    double dist = (goal - start).norm();
    
    // 🚀 MULTI-CORRIDOR SAMPLING: 在多个平行走廊中搜索corner
    // 目标: 生成拓扑多样化的节点,支持K-shortest paths
    // 策略: 在主走廊±侧向偏移的多个走廊中搜索
    
    // 定义搜索走廊: 主走廊 + 左右侧向偏移走廊
    vector<double> corridor_offsets = {
        0.0,      // 主走廊 (原始直线)
        -5.0,     // 左侧5m
        -10.0,    // 左侧10m
        5.0,      // 右侧5m
        10.0      // 右侧10m
    };
    
    // 每个走廊的搜索半径 (主走廊大,侧走廊小)
    double main_search_radius = corner_detection_radius_;      // 主走廊: 3m
    double side_search_radius = corner_detection_radius_ * 0.67;  // 侧走廊: 2m
    
    // 每个走廊的节点配额 (确保多样性)
    int corners_per_corridor = max_corner_num_ / corridor_offsets.size();  // 60/5 = 12
    
    // 计算侧向偏移的参考方向 (垂直于起点-终点连线)
    Vector3d lateral_dir = Vector3d(-dir.y(), dir.x(), 0.0).normalized();  // 2D平面垂直方向
    
    for (double offset : corridor_offsets) {
        int corridor_corners = 0;
        double search_radius = (offset == 0.0) ? main_search_radius : side_search_radius;
        
        // 该走廊的中心线
        Vector3d corridor_start = start + offset * lateral_dir;
        Vector3d corridor_goal = goal + offset * lateral_dir;
        
        int num_samples = static_cast<int>(dist / resolution_);
        
        // Sample points along this corridor
        for (int i = 0; i < num_samples; ++i) {
            double t = static_cast<double>(i) / num_samples;
            Vector3d center_pt = corridor_start + t * dist * dir;
            
            // Sample radially around this center point
            for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 4) {
                for (double r = resolution_; r < search_radius; r += resolution_) {
                    Vector3d offset_vec = r * Vector3d(cos(theta), sin(theta), 0.0);
                    Vector3d sample_pt = center_pt + offset_vec;
                    
                    // Check if this point is a corner
                    if (isCornerPoint(sample_pt)) {
                        // Avoid duplicates
                        bool is_duplicate = false;
                        for (const auto& corner : corners) {
                            if ((sample_pt - corner).norm() < resolution_ * 2.0) {
                                is_duplicate = true;
                                break;
                            }
                        }
                        
                        if (!is_duplicate) {
                            corners.push_back(sample_pt);
                            corridor_corners++;
                            
                            // 达到该走廊配额,跳到下一个走廊
                            if (corridor_corners >= corners_per_corridor) {
                                goto next_corridor;
                            }
                            
                            // 全局上限
                            if (corners.size() >= static_cast<size_t>(max_corner_num_)) {
                                ROS_INFO("[BiasSampler] ✅ Multi-corridor sampling: collected %zu corners from %zu corridors", 
                                         corners.size(), corridor_offsets.size());
                                return corners;
                            }
                        }
                    }
                }
            }
            
            // Also check in vertical direction
            for (double z = -search_radius; z <= search_radius; z += resolution_) {
                Vector3d sample_pt = center_pt + Vector3d(0, 0, z);
                
                if (isCornerPoint(sample_pt)) {
                    bool is_duplicate = false;
                    for (const auto& corner : corners) {
                        if ((sample_pt - corner).norm() < resolution_ * 2.0) {
                            is_duplicate = true;
                            break;
                        }
                    }
                    
                    if (!is_duplicate) {
                        corners.push_back(sample_pt);
                        corridor_corners++;
                        
                        if (corridor_corners >= corners_per_corridor) {
                            goto next_corridor;
                        }
                        
                        if (corners.size() >= static_cast<size_t>(max_corner_num_)) {
                            ROS_INFO("[BiasSampler] ✅ Multi-corridor sampling: collected %zu corners", corners.size());
                            return corners;
                        }
                    }
                }
            }
        }
        
        next_corridor:
        ROS_INFO("[BiasSampler] Corridor offset=%.1fm: collected %d corners", offset, corridor_corners);
    }
    
    ROS_INFO("[BiasSampler] ✅ Multi-corridor sampling complete: %zu total corners from %zu corridors", 
             corners.size(), corridor_offsets.size());
    return corners;
}

bool BiasSampler::isCornerPoint(const Vector3d& pos) {
    // A corner point is at the boundary between free and occupied space
    // AND has high gradient variation (concave/convex geometry)
    
    // 🔧 Phase 4.5.1.5: 添加详细调试日志（诊断为什么TGK 100%失败）
    static int total_attempts = 0;
    static int passed_check1 = 0, passed_check2 = 0, passed_check3 = 0;
    static int check1_failures = 0, check2_failures = 0, check3_failures = 0;
    total_attempts++;
    
    // Check 1: Must be collision-free
    if (!isCollisionFree(pos)) {
        check1_failures++;
        if (total_attempts % 100 == 0) {
            ROS_WARN_THROTTLE(5.0, "[TGK Corner] 统计: 尝试=%d, ✅通过1=%d ✅通过2=%d ✅通过3=%d | ❌失败1=%d ❌失败2=%d ❌失败3=%d",
                          total_attempts, passed_check1, passed_check2, passed_check3,
                          check1_failures, check2_failures, check3_failures);
        }
        return false;
    }
    passed_check1++;
    
    // Check 2: Must be near obstacle boundary (geometric check)
    // 🔧 Phase 4.5.1.7: 移除ESDF依赖，改用纯几何检查（回归TGK原始设计）
    // 原问题：getDistanceWithGrad()在传感器视野外返回10000.0m异常值
    // 新方法：在周围采样检查free/occupied混合 → 更鲁棒、更符合TGK论文
    
    int free_count = 0;
    int occupied_count = 0;
    double check_radius = resolution_ * 3.0;  // 检查半径：3个grid单元
    
    // 在8个方向采样
    for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 4) {
        Vector3d dir(cos(theta), sin(theta), 0.0);
        Vector3d check_pt = pos + check_radius * dir;
        
        if (isCollisionFree(check_pt)) {
            free_count++;
        } else {
            occupied_count++;
        }
    }
    
    // 如果周围既有free又有occupied → 在边界附近
    bool near_boundary = (free_count > 0 && occupied_count > 0);
    
    if (!near_boundary) {
        check2_failures++;
        if (passed_check1 % 100 == 0) {
            ROS_WARN_THROTTLE(5.0, "[TGK Corner] 条件2失败（远离边界）: free=%d, occupied=%d | 统计: 尝试=%d, 通过1=%d, 失败2=%d",
                          free_count, occupied_count, total_attempts, passed_check1, check2_failures);
        }
        return false;  // Not near boundary
    }
    passed_check2++;
    
    // Check 3: Must have corner/concave feature (geometric check)
    // 🎯 STEP 3/5: Relax corner requirements to increase sampling density
    // Original: occupied_count >= 2 (strict corner requirement)
    // Optimized: occupied_count >= 1 (any boundary point is useful)
    // Goal: Increase corner points from 24-29 → 60+ for better path diversity
    
    // 🔧 Phase 4.5.1.7 + Step 3 optimization: 
    // - In sparse environments, any boundary point is a potential topological key point
    // - More corner points = more alternative paths = better MPPI diversity
    
    // Reuse Check 2's sampling results (already have free_count and occupied_count)
    // Only need at least 1 direction with obstacle (relaxed from 2)
    if (occupied_count < 1) {
        check3_failures++;
        if (passed_check2 % 50 == 0) {
            ROS_WARN_THROTTLE(5.0, "[TGK Corner] 条件3失败（障碍物不足）: occupied=%d < 1 | 统计: 尝试=%d, 通过1=%d, 通过2=%d, 失败3=%d",
                          occupied_count, total_attempts, passed_check1, passed_check2, check3_failures);
        }
        return false;
    }
    passed_check3++;
    
    // 🎉 成功找到corner point！
    ROS_INFO("[TGK Corner] ✅ 找到corner point! pos=(%.2f,%.2f,%.2f), free=%d, occupied=%d | 总统计: %d/%d/%d/%d (总/过1/过2/过3)",
             pos.x(), pos.y(), pos.z(), free_count, occupied_count,
             total_attempts, passed_check1, passed_check2, passed_check3);
    
    return true;
}

SamplingSpace BiasSampler::buildSamplingSpace(const Vector3d& corner, int topo_id) {
    SamplingSpace space;
    space.center = corner;
    space.radius = sampling_radius_;
    space.topo_id = topo_id;
    
    // Store corner as a key point
    space.corners.push_back(corner);
    
    return space;
}

bool BiasSampler::sampleFromSpace(const SamplingSpace& space, Vector3d& sample) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_real_distribution<double> dis(-1.0, 1.0);
    
    int max_attempts = 20;
    for (int i = 0; i < max_attempts; ++i) {
        // Sample uniformly in a sphere
        Vector3d offset(dis(gen), dis(gen), dis(gen));
        offset.normalize();
        offset *= space.radius * abs(dis(gen));
        
        sample = space.center + offset;
        
        // Check if sample is collision-free
        if (isCollisionFree(sample)) {
            return true;
        }
    }
    
    ROS_WARN("[BiasSampler] Failed to find collision-free sample after %d attempts", max_attempts);
    return false;
}

bool BiasSampler::isCollisionFree(const Vector3d& pos) {
    return !grid_map_->getInflateOccupancy(pos);
}

bool BiasSampler::isLineFree(const Vector3d& p1, const Vector3d& p2) {
    Vector3d dir = p2 - p1;
    double dist = dir.norm();
    dir.normalize();
    
    int num_checks = static_cast<int>(dist / collision_check_res_);
    for (int i = 0; i <= num_checks; ++i) {
        Vector3d check_pt = p1 + (dist * i / num_checks) * dir;
        if (!isCollisionFree(check_pt)) {
            return false;
        }
    }
    
    return true;
}

int BiasSampler::calculateTopoClass(const Vector3d& corner,
                                    const Vector3d& start,
                                    const Vector3d& goal) {
    // Classify topological relationship based on corner's position
    // relative to the start-goal line
    
    Vector3d sg_dir = (goal - start).normalized();
    Vector3d sc_vec = corner - start;
    
    // Project corner onto start-goal line
    double proj = sc_vec.dot(sg_dir);
    Vector3d proj_pt = start + proj * sg_dir;
    Vector3d perp_vec = corner - proj_pt;
    
    // Classify based on perpendicular distance and direction
    double perp_dist = perp_vec.norm();
    
    if (perp_dist < resolution_) {
        return 0;  // On the line
    }
    
    // Use cross product to determine left/right
    Vector3d cross = sg_dir.cross(perp_vec);
    
    if (cross.z() > 0) {
        return 1;  // Left side
    } else {
        return 2;  // Right side
    }
}

vector<Vector3d> BiasSampler::getTopoKeyPoints(const Vector3d& start,
                                               const Vector3d& goal) {
    // Get all corner points as topological key points
    return detectObstacleCorners(start, goal);
}

} // namespace ego_planner
