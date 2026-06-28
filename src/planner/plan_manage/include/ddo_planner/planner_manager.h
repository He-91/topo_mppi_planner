#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <mutex>
#include <limits>
#include <string>

#include <ddo_bspline_opt/bspline_optimizer.h>
#include <ddo_bspline_opt/uniform_bspline.h>
#include <ddo_planner/DataDisp.h>
#include <ddo_planner/DynamicObstacles.h>
#include <ddo_plan_env/grid_map.h>
#include <ddo_planner/plan_container.hpp>
#include <ros/ros.h>
#include <ddo_traj_utils/planning_visualization.h>
#include <ddo_path_searching/topo_prm.h>
#include <ddo_path_searching/mppi_planner.h>
#include <visualization_msgs/MarkerArray.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /* Topological planning interface */
    bool planWithTopo(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos,
                     std::vector<TopoPath> &topo_paths);
    
    /* MPPI planning interface */
    bool planWithMPPI(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                     const Eigen::Vector3d &goal_pos, const Eigen::Vector3d &goal_vel,
                     MPPITrajectory &optimal_traj);
    
    /*  NEW: Visualization data structures and access */
    struct MPPIPathCandidate {
        std::vector<Eigen::Vector3d> positions;
        double cost;
        double normalized_cost;
        bool is_best;
        bool success;
    };
    
    /* Get all MPPI-optimized paths for visualization */
    const std::vector<MPPIPathCandidate>& getAllMPPIPaths() const { return all_mppi_paths_; }
    bool isLastTrajRecovery() const { return last_traj_is_recovery_; }
    double getDynamicSurfaceDistanceForSafety(const Eigen::Vector3d &point,
                                              double future_t) const;

    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;
    
    /* New topological and MPPI planning modules */
    TopoPRM::Ptr topo_planner_;
    MPPIPlanner::Ptr mppi_planner_;
    
    /* Backup MPPI result for B-spline fallback */
    MPPITrajectory mppi_result_backup_;
    
    /*  V8: Track consecutive B-spline failures to skip B-spline when stuck */
    int bspline_consecutive_failures_{0};
    int bspline_skip_threshold_{1};  // Dynamic-scene MPPI fallback after N consecutive B-spline failures
    
    /* Store all MPPI-optimized paths for visualization */
    std::vector<MPPIPathCandidate> all_mppi_paths_;
    ros::Publisher all_mppi_paths_pub_;
    ros::Publisher topo_paths_smooth_pub_;

    int continous_failures_count_{0};

    /*  Dynamic obstacle data for MPPI planning */
    ros::Subscriber dynamic_obstacle_sub_;
    ddo_planner::DynamicObstacles latest_dynamic_obstacles_;
    mutable std::mutex dynamic_obstacle_mutex_;
    bool has_dynamic_obstacles_{false};
    
    /*  Dynamic obstacle callback */
    void dynamicObstacleCallback(const ddo_planner::DynamicObstaclesConstPtr& msg);
    
    /*  Feed dynamic obstacle data to MPPI planner before planning */
    void feedDynamicObstaclesToMPPI();

    bool isPathWithinZBounds(const std::vector<Eigen::Vector3d> &path,
                             double *min_z_observed = nullptr,
                             double *max_z_observed = nullptr) const;
    bool isBsplineWithinZBounds(UniformBspline position_traj,
                                double *min_z_observed = nullptr,
                                double *max_z_observed = nullptr) const;
    bool isBsplineCollisionFree(UniformBspline position_traj,
                                double min_clearance,
                                double *min_clearance_observed = nullptr,
                                double time_fraction = 1.0,
                                double max_check_time = std::numeric_limits<double>::infinity(),
                                double start_skip_time = 0.0) const;
    bool isBsplineDynamicSafe(UniformBspline position_traj,
                              double safety_radius,
                              double *min_dynamic_distance_observed = nullptr,
                              double start_skip_time = 0.0) const;
    double queryDynamicSurfaceDistance(const Eigen::Vector3d &point,
                                       double future_t) const;
    bool generateRecoveryTraj(const Eigen::Vector3d &start_pt,
                              const Eigen::Vector3d &start_vel,
                              const Eigen::Vector3d &start_acc,
                              const Eigen::Vector3d &local_target_pt,
                              double ts);
    bool generateClearanceRecoveryTraj(const Eigen::Vector3d &start_pt,
                                       const Eigen::Vector3d &start_vel,
                                       const Eigen::Vector3d &start_acc,
                                       const Eigen::Vector3d &local_target_pt,
                                       double ts);
    bool clampBsplineControlPointsZ(Eigen::MatrixXd &control_points,
                                    const Eigen::Vector3d &start_pt,
                                    const Eigen::Vector3d &local_target_pt) const;
    bool densifyPathForBspline(const std::vector<Eigen::Vector3d> &input_path,
                               double max_spacing,
                               std::vector<Eigen::Vector3d> &dense_path) const;
    bool buildSafeSeedBspline(const std::vector<Eigen::Vector3d> &seed_path,
                              const std::vector<Eigen::Vector3d> &start_end_derivative,
                              double ts,
                              UniformBspline &safe_traj,
                              double static_check_time_fraction = 1.0,
                              double static_check_max_time = std::numeric_limits<double>::infinity(),
                              double static_check_start_skip = 0.25,
                              double dynamic_check_start_skip = 0.0,
                              double *min_clearance_observed = nullptr,
                              double *min_dynamic_distance_observed = nullptr) const;
    bool repairSeedPathByClearance(const std::vector<Eigen::Vector3d> &seed_path,
                                   std::vector<Eigen::Vector3d> &repaired_path) const;
    bool rebaseCachedTopoPaths(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &goal_pos,
                               std::vector<TopoPath> &topo_paths) const;
    void commitPublishedTopoCache(const Eigen::Vector3d &goal_pos,
                                  const std::vector<TopoPath> &topo_paths);

    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    /*  Visualize all MPPI candidate paths */
    void visualizeAllMPPIPaths();
    
    /*  NEW: Visualize topological paths with B-spline smoothing */
    void visualizeTopoPathsSmooth(const std::vector<TopoPath> &topo_paths);

    double min_z_{0.3};
    double max_z_{4.5};
    double publish_min_z_{0.65};
    double recovery_min_z_{1.0};
    bool planar_flight_z_lock_{false};
    bool last_traj_is_recovery_{false};
    double mode_score_goal_weight_{90.0};
    double mode_score_clearance_weight_{600.0};
    double mode_score_clearance_margin_{0.55};
    double mode_score_dynamic_weight_{80.0};
    double mode_score_dynamic_margin_{0.80};
    double dynamic_scene_unsafe_clearance_{0.15};
    double dynamic_scene_preferred_clearance_{0.30};
    double dynamic_scene_safe_cost_slack_{40.0};
    bool dynamic_scene_use_mode_score_{false};
    bool static_scene_safety_gate_enabled_{true};
    double static_scene_unsafe_clearance_{0.15};
    double static_scene_preferred_clearance_{0.30};
    double static_scene_safe_score_slack_{40.0};
    bool static_scene_best_effort_gate_enabled_{true};
    double static_scene_min_clearance_improvement_{0.08};
    bool static_scene_preferred_clearance_gate_enabled_{false};
    double static_scene_preferred_clearance_score_slack_{80.0};
    double mode_score_reverse_progress_weight_{350.0};
    double mode_score_weak_progress_weight_{80.0};
    double mode_score_min_progress_{0.25};
    double mode_score_direction_weight_{140.0};
    double mode_score_min_direction_cos_{0.15};
    double mode_score_path_length_weight_{18.0};
    double mode_score_overshoot_weight_{120.0};
    double mode_score_early_progress_weight_{160.0};
    double mode_score_min_early_progress_{0.05};
    int mode_score_safety_skip_points_{2};
    double mppi_seed_preserve_clearance_{0.35};
    double safe_seed_time_stretch_max_{2.6};
    bool static_scene_preserve_safe_seed_{false};
    double static_scene_preserve_min_clearance_{0.15};
    bool dynamic_scene_preserve_safe_seed_{false};
    double dynamic_escape_start_skip_time_{0.35};
    bool use_structured_dynamic_surface_check_{false};
    double final_static_min_clearance_{0.15};
    double final_dynamic_static_check_fraction_{0.35};
    double final_dynamic_static_check_time_{1.2};
    double final_static_check_start_skip_{0.25};
    bool final_fallback_feasibility_repair_{false};
    bool final_fallback_relaxed_derivative_repair_{false};
    double final_fallback_relaxed_velocity_scale_{0.6};
    bool final_publish_feasibility_gate_enabled_{false};
    bool geometric_seed_repair_enabled_{false};
    double geometric_seed_repair_clearance_{0.25};
    double geometric_seed_repair_step_{0.18};
    int geometric_seed_repair_iterations_{3};
    double final_dynamic_min_distance_{0.65};
    double dynamic_publish_preferred_distance_{0.65};
    double dynamic_distance_radius_compensation_{0.0};
    double dynamic_safety_time_buffer_{0.0};
    bool clearance_recovery_enabled_{false};
    int static_escape_check_after_failures_{2};
    double static_escape_check_start_skip_{0.8};
    double static_escape_max_initial_clearance_{0.05};
    double static_escape_min_post_clearance_{0.35};
    int max_mppi_topo_candidates_{4};
    double topo_prefilter_clearance_weight_{220.0};
    double topo_prefilter_dynamic_weight_{80.0};
    double topo_prefilter_clearance_margin_{0.30};
    double topo_safety_start_skip_dist_{0.0};
    double cached_topo_rebase_max_dist_{4.5};
    double cached_topo_goal_reuse_max_dist_{1.0};
    bool cache_only_published_topo_paths_{true};
    bool prefer_cached_topo_paths_{false};
    bool has_cached_topo_paths_{false};
    Eigen::Vector3d cached_topo_goal_{Eigen::Vector3d::Zero()};
    std::vector<TopoPath> cached_topo_paths_;
    bool ablation_disable_topo_guidance_{false};
    bool ablation_disable_mppi_optimization_{false};
    bool planning_timing_log_enabled_{false};
    std::string planning_timing_log_path_;
    bool candidate_quality_log_enabled_{false};
    std::string candidate_quality_log_path_;
    int planning_timing_cycle_{0};

    struct PlanningCycleTiming {
      int cycle{0};
      bool success{false};
      std::string fail_reason{"unknown"};
      double total_ms{0.0};
      double init_ms{0.0};
      double topo_ms{0.0};
      double mppi_ms{0.0};
      double bspline_ms{0.0};
      double refine_ms{0.0};
      double validator_ms{0.0};
      int topo_paths{0};
      int mppi_candidates{0};
      int mppi_successes{0};
      int continuous_failures{0};
      int bspline_failures{0};
      double start_goal_dist{0.0};
      double final_static_clearance{-1.0};
      double final_dynamic_clearance{-1.0};
      bool used_mppi_fallback{false};
      bool used_static_topo_seed{false};
    };
    void appendPlanningCycleTiming(const PlanningCycleTiming &timing) const;

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
