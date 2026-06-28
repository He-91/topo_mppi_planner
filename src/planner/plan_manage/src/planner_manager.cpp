#include <fstream>
#include <ddo_planner/planner_manager.h>
#include <ddo_path_searching/topo_prm.h>
#include <ddo_path_searching/mppi_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <iomanip>
#include <thread>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <sys/stat.h>
#include <omp.h>

#ifdef USE_GPU_MPPI
#include <cuda_runtime.h>
#endif

namespace ego_planner
{
  namespace
  {
    constexpr double kMaxTrustedStaticClearance = 20.0;
    constexpr double kReportedStaticClearanceCap = 5.0;

    double sanitizeStaticClearanceForSafety(double dist)
    {
      if (!std::isfinite(dist)) {
        return 0.0;
      }
      if (dist < 0.0) {
        return dist;
      }
      if (dist > kMaxTrustedStaticClearance) {
        return kReportedStaticClearanceCap;
      }
      return std::min(kReportedStaticClearanceCap, dist);
    }

    double distanceToDynamicCylinderSurface(const Eigen::Vector3d &point,
                                            const Eigen::Vector3d &center,
                                            double radius,
                                            double height)
    {
      const double dx = point.x() - center.x();
      const double dy = point.y() - center.y();
      const double radial_out = std::sqrt(dx * dx + dy * dy) - std::max(0.0, radius);
      const double half_height = std::max(0.0, height) * 0.5;
      const double vertical_out = std::fabs(point.z() - center.z()) - half_height;

      if (radial_out <= 0.0 && vertical_out <= 0.0) {
        return std::max(radial_out, vertical_out);
      }

      const double clamped_radial = std::max(0.0, radial_out);
      const double clamped_vertical = std::max(0.0, vertical_out);
      return std::sqrt(clamped_radial * clamped_radial +
                       clamped_vertical * clamped_vertical);
    }
  }

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::appendPlanningCycleTiming(const PlanningCycleTiming &timing) const
  {
    if (!planning_timing_log_enabled_ || planning_timing_log_path_.empty()) {
      return;
    }

    const bool file_exists = [] (const std::string &path) {
      struct stat buffer;
      return stat(path.c_str(), &buffer) == 0 && buffer.st_size > 0;
    }(planning_timing_log_path_);

    std::ofstream out(planning_timing_log_path_, std::ios::app);
    if (!out.is_open()) {
      ROS_WARN_THROTTLE(2.0, "[PlannerManager] Cannot open planning timing log: %s",
                        planning_timing_log_path_.c_str());
      return;
    }

    if (!file_exists) {
      out << "stamp,cycle,success,fail_reason,total_ms,init_ms,topo_ms,mppi_ms,"
          << "bspline_ms,refine_ms,validator_ms,topo_paths,mppi_candidates,"
          << "mppi_successes,continuous_failures,bspline_failures,start_goal_dist,"
          << "final_static_clearance,final_dynamic_clearance,used_mppi_fallback,"
          << "used_static_topo_seed\n";
    }

    out << std::fixed << std::setprecision(4)
        << ros::Time::now().toSec() << ','
        << timing.cycle << ','
        << (timing.success ? 1 : 0) << ','
        << timing.fail_reason << ','
        << timing.total_ms << ','
        << timing.init_ms << ','
        << timing.topo_ms << ','
        << timing.mppi_ms << ','
        << timing.bspline_ms << ','
        << timing.refine_ms << ','
        << timing.validator_ms << ','
        << timing.topo_paths << ','
        << timing.mppi_candidates << ','
        << timing.mppi_successes << ','
        << timing.continuous_failures << ','
        << timing.bspline_failures << ','
        << timing.start_goal_dist << ','
        << timing.final_static_clearance << ','
        << timing.final_dynamic_clearance << ','
        << (timing.used_mppi_fallback ? 1 : 0) << ','
        << (timing.used_static_topo_seed ? 1 : 0) << '\n';
  }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
#ifdef USE_GPU_MPPI
    //  Initialize CUDA runtime early to prevent heap corruption from later malloc operations
    ROS_INFO("[PlannerManager] Initializing CUDA runtime (cudaFree(0))...");
    cudaError_t err = cudaFree(0);
    if (err != cudaSuccess) {
      ROS_WARN("[PlannerManager] cudaFree(0) failed: %s - GPU may not be available", cudaGetErrorString(err));
    } else {
      ROS_INFO("[PlannerManager] CUDA runtime initialized successfully in PlannerManager");
    }
#endif

    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    // Z-axis independent limits (default to xy limits for backward compatibility)
    nh.param("manager/max_vel_z", pp_.max_vel_z_, pp_.max_vel_);
    nh.param("manager/max_acc_z", pp_.max_acc_z_, pp_.max_acc_);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("fsm/min_z", min_z_, 0.3);
    nh.param("fsm/max_z", max_z_, 4.5);
    double publish_z_margin = 0.35;
    nh.param("fsm/publish_z_margin", publish_z_margin, publish_z_margin);
    publish_min_z_ = std::min(max_z_, min_z_ + std::max(0.0, publish_z_margin));
    nh.param("fsm/recovery_min_z", recovery_min_z_, recovery_min_z_);
    recovery_min_z_ = std::min(max_z_ - 0.1, std::max(publish_min_z_, recovery_min_z_));
    nh.param("manager/planar_flight_z_lock", planar_flight_z_lock_, planar_flight_z_lock_);
    
    // Multi-topology MPPI optimization. Keep the old parameter name as a
    // compatibility alias for existing launch files.
    bool legacy_parallel_mppi = false;
    nh.param("manager/use_parallel_mppi_optimization", legacy_parallel_mppi, false);
    nh.param("manager/use_multi_topo_mppi_optimization",
             pp_.use_multi_topo_mppi_optimization,
             legacy_parallel_mppi);
    nh.param("manager/mode_score/goal_weight", mode_score_goal_weight_, mode_score_goal_weight_);
    nh.param("manager/mode_score/clearance_weight", mode_score_clearance_weight_, mode_score_clearance_weight_);
    nh.param("manager/mode_score/clearance_margin", mode_score_clearance_margin_, mode_score_clearance_margin_);
    nh.param("manager/mode_score/dynamic_weight", mode_score_dynamic_weight_, mode_score_dynamic_weight_);
    nh.param("manager/mode_score/dynamic_margin", mode_score_dynamic_margin_, mode_score_dynamic_margin_);
    nh.param("manager/mode_score/dynamic_scene_unsafe_clearance", dynamic_scene_unsafe_clearance_, dynamic_scene_unsafe_clearance_);
    nh.param("manager/mode_score/dynamic_scene_preferred_clearance", dynamic_scene_preferred_clearance_, dynamic_scene_preferred_clearance_);
    nh.param("manager/mode_score/dynamic_scene_safe_cost_slack", dynamic_scene_safe_cost_slack_, dynamic_scene_safe_cost_slack_);
    nh.param("manager/mode_score/dynamic_scene_use_mode_score", dynamic_scene_use_mode_score_, dynamic_scene_use_mode_score_);
    nh.param("manager/mode_score/static_scene_safety_gate_enabled", static_scene_safety_gate_enabled_, static_scene_safety_gate_enabled_);
    nh.param("manager/mode_score/static_scene_unsafe_clearance", static_scene_unsafe_clearance_, static_scene_unsafe_clearance_);
    nh.param("manager/mode_score/static_scene_preferred_clearance", static_scene_preferred_clearance_, static_scene_preferred_clearance_);
    nh.param("manager/mode_score/static_scene_safe_score_slack", static_scene_safe_score_slack_, static_scene_safe_score_slack_);
    nh.param("manager/mode_score/static_scene_best_effort_gate_enabled", static_scene_best_effort_gate_enabled_, static_scene_best_effort_gate_enabled_);
    nh.param("manager/mode_score/static_scene_min_clearance_improvement", static_scene_min_clearance_improvement_, static_scene_min_clearance_improvement_);
    nh.param("manager/mode_score/static_scene_preferred_clearance_gate_enabled",
             static_scene_preferred_clearance_gate_enabled_,
             static_scene_preferred_clearance_gate_enabled_);
    nh.param("manager/mode_score/static_scene_preferred_clearance_score_slack",
             static_scene_preferred_clearance_score_slack_,
             static_scene_preferred_clearance_score_slack_);
    nh.param("manager/mode_score/reverse_progress_weight", mode_score_reverse_progress_weight_, mode_score_reverse_progress_weight_);
    nh.param("manager/mode_score/weak_progress_weight", mode_score_weak_progress_weight_, mode_score_weak_progress_weight_);
    nh.param("manager/mode_score/min_progress", mode_score_min_progress_, mode_score_min_progress_);
    nh.param("manager/mode_score/direction_weight", mode_score_direction_weight_, mode_score_direction_weight_);
    nh.param("manager/mode_score/min_direction_cos", mode_score_min_direction_cos_, mode_score_min_direction_cos_);
    nh.param("manager/mode_score/path_length_weight", mode_score_path_length_weight_, mode_score_path_length_weight_);
    nh.param("manager/mode_score/overshoot_weight", mode_score_overshoot_weight_, mode_score_overshoot_weight_);
    nh.param("manager/mode_score/early_progress_weight", mode_score_early_progress_weight_, mode_score_early_progress_weight_);
    nh.param("manager/mode_score/min_early_progress", mode_score_min_early_progress_, mode_score_min_early_progress_);
    nh.param("manager/mode_score/safety_skip_points", mode_score_safety_skip_points_, mode_score_safety_skip_points_);
    nh.param("manager/mppi_seed_preserve_clearance", mppi_seed_preserve_clearance_, mppi_seed_preserve_clearance_);
    nh.param("manager/safe_seed_time_stretch_max", safe_seed_time_stretch_max_, safe_seed_time_stretch_max_);
    nh.param("manager/bspline_skip_threshold", bspline_skip_threshold_, bspline_skip_threshold_);
    nh.param("manager/static_scene_preserve_safe_seed", static_scene_preserve_safe_seed_, static_scene_preserve_safe_seed_);
    nh.param("manager/static_scene_preserve_min_clearance", static_scene_preserve_min_clearance_, static_scene_preserve_min_clearance_);
    nh.param("manager/dynamic_scene_preserve_safe_seed", dynamic_scene_preserve_safe_seed_, dynamic_scene_preserve_safe_seed_);
    nh.param("manager/dynamic_escape_start_skip_time", dynamic_escape_start_skip_time_, dynamic_escape_start_skip_time_);
    nh.param("manager/use_structured_dynamic_surface_check", use_structured_dynamic_surface_check_, use_structured_dynamic_surface_check_);
    nh.param("manager/final_static_min_clearance", final_static_min_clearance_, final_static_min_clearance_);
    nh.param("manager/final_dynamic_static_check_fraction", final_dynamic_static_check_fraction_, final_dynamic_static_check_fraction_);
    nh.param("manager/final_dynamic_static_check_time", final_dynamic_static_check_time_, final_dynamic_static_check_time_);
    nh.param("manager/final_static_check_start_skip", final_static_check_start_skip_, final_static_check_start_skip_);
    nh.param("manager/final_fallback_feasibility_repair", final_fallback_feasibility_repair_, final_fallback_feasibility_repair_);
    nh.param("manager/final_fallback_relaxed_derivative_repair",
             final_fallback_relaxed_derivative_repair_,
             final_fallback_relaxed_derivative_repair_);
    nh.param("manager/final_fallback_relaxed_velocity_scale",
             final_fallback_relaxed_velocity_scale_,
             final_fallback_relaxed_velocity_scale_);
    nh.param("manager/final_publish_feasibility_gate_enabled",
             final_publish_feasibility_gate_enabled_,
             final_publish_feasibility_gate_enabled_);
    nh.param("manager/geometric_seed_repair_enabled",
             geometric_seed_repair_enabled_, geometric_seed_repair_enabled_);
    nh.param("manager/geometric_seed_repair_clearance",
             geometric_seed_repair_clearance_, geometric_seed_repair_clearance_);
    nh.param("manager/geometric_seed_repair_step",
             geometric_seed_repair_step_, geometric_seed_repair_step_);
    nh.param("manager/geometric_seed_repair_iterations",
             geometric_seed_repair_iterations_, geometric_seed_repair_iterations_);
    nh.param("manager/final_dynamic_min_distance", final_dynamic_min_distance_, final_dynamic_min_distance_);
    nh.param("manager/dynamic_publish_preferred_distance", dynamic_publish_preferred_distance_, dynamic_publish_preferred_distance_);
    nh.param("manager/dynamic_distance_radius_compensation", dynamic_distance_radius_compensation_, dynamic_distance_radius_compensation_);
    nh.param("manager/dynamic_safety_time_buffer", dynamic_safety_time_buffer_, dynamic_safety_time_buffer_);
    nh.param("manager/clearance_recovery_enabled", clearance_recovery_enabled_, clearance_recovery_enabled_);
    nh.param("manager/static_escape_check_after_failures", static_escape_check_after_failures_, static_escape_check_after_failures_);
    nh.param("manager/static_escape_check_start_skip", static_escape_check_start_skip_, static_escape_check_start_skip_);
    nh.param("manager/static_escape_max_initial_clearance", static_escape_max_initial_clearance_, static_escape_max_initial_clearance_);
    nh.param("manager/static_escape_min_post_clearance", static_escape_min_post_clearance_, static_escape_min_post_clearance_);
    nh.param("manager/max_mppi_topo_candidates", max_mppi_topo_candidates_, max_mppi_topo_candidates_);
    nh.param("manager/topo_prefilter/clearance_weight", topo_prefilter_clearance_weight_, topo_prefilter_clearance_weight_);
    nh.param("manager/topo_prefilter/dynamic_weight", topo_prefilter_dynamic_weight_, topo_prefilter_dynamic_weight_);
    nh.param("manager/topo_prefilter/clearance_margin", topo_prefilter_clearance_margin_, topo_prefilter_clearance_margin_);
    nh.param("manager/topo_prefilter/start_skip_dist", topo_safety_start_skip_dist_, topo_safety_start_skip_dist_);
    nh.param("manager/cached_topo_rebase_max_dist", cached_topo_rebase_max_dist_, cached_topo_rebase_max_dist_);
    nh.param("manager/cached_topo_goal_reuse_max_dist", cached_topo_goal_reuse_max_dist_, cached_topo_goal_reuse_max_dist_);
    nh.param("manager/cache_only_published_topo_paths", cache_only_published_topo_paths_, cache_only_published_topo_paths_);
    nh.param("manager/prefer_cached_topo_paths", prefer_cached_topo_paths_, prefer_cached_topo_paths_);
    nh.param("manager/ablation_disable_topo_guidance", ablation_disable_topo_guidance_, ablation_disable_topo_guidance_);
    nh.param("manager/ablation_disable_mppi_optimization", ablation_disable_mppi_optimization_, ablation_disable_mppi_optimization_);
    nh.param("manager/planning_timing_log_enabled", planning_timing_log_enabled_, planning_timing_log_enabled_);
    nh.param("manager/planning_timing_log_path", planning_timing_log_path_,
             std::string("/tmp/topo_mppi_planning_timing.csv"));
    nh.param("manager/candidate_quality_log_enabled", candidate_quality_log_enabled_,
             candidate_quality_log_enabled_);
    nh.param("manager/candidate_quality_log_path", candidate_quality_log_path_,
             std::string("/tmp/topo_mppi_candidate_quality.csv"));
    mode_score_safety_skip_points_ = std::max(0, mode_score_safety_skip_points_);
    mode_score_path_length_weight_ = std::max(0.0, mode_score_path_length_weight_);
    mode_score_overshoot_weight_ = std::max(0.0, mode_score_overshoot_weight_);
    mode_score_early_progress_weight_ = std::max(0.0, mode_score_early_progress_weight_);
    static_scene_preferred_clearance_score_slack_ =
        std::max(0.0, static_scene_preferred_clearance_score_slack_);
    max_mppi_topo_candidates_ = std::max(1, max_mppi_topo_candidates_);
    bspline_skip_threshold_ = std::max(1, bspline_skip_threshold_);
    safe_seed_time_stretch_max_ = std::max(1.0, safe_seed_time_stretch_max_);
    dynamic_escape_start_skip_time_ =
        std::min(1.0, std::max(0.0, dynamic_escape_start_skip_time_));
    static_scene_preserve_min_clearance_ =
        std::max(final_static_min_clearance_, static_scene_preserve_min_clearance_);
    final_dynamic_static_check_fraction_ = std::min(1.0, std::max(0.1, final_dynamic_static_check_fraction_));
    final_dynamic_static_check_time_ = std::max(0.2, final_dynamic_static_check_time_);
    final_static_check_start_skip_ = std::min(1.2, std::max(0.0, final_static_check_start_skip_));
    final_fallback_relaxed_velocity_scale_ =
        std::min(1.0, std::max(0.1, final_fallback_relaxed_velocity_scale_));
    geometric_seed_repair_clearance_ =
        std::max(final_static_min_clearance_, geometric_seed_repair_clearance_);
    geometric_seed_repair_step_ = std::max(0.02, geometric_seed_repair_step_);
    geometric_seed_repair_iterations_ =
        std::max(1, geometric_seed_repair_iterations_);
    final_dynamic_min_distance_ = std::max(0.1, final_dynamic_min_distance_);
    dynamic_publish_preferred_distance_ =
        std::max(final_dynamic_min_distance_, dynamic_publish_preferred_distance_);
    dynamic_distance_radius_compensation_ = std::max(0.0, dynamic_distance_radius_compensation_);
    dynamic_safety_time_buffer_ = std::max(0.0, dynamic_safety_time_buffer_);
    static_escape_check_after_failures_ = std::max(0, static_escape_check_after_failures_);
    static_escape_check_start_skip_ = std::min(1.5, std::max(0.25, static_escape_check_start_skip_));
    static_escape_max_initial_clearance_ = std::max(0.0, static_escape_max_initial_clearance_);
    static_escape_min_post_clearance_ = std::max(0.0, static_escape_min_post_clearance_);
    topo_safety_start_skip_dist_ = std::max(0.0, topo_safety_start_skip_dist_);
    cached_topo_rebase_max_dist_ = std::max(0.5, cached_topo_rebase_max_dist_);
    cached_topo_goal_reuse_max_dist_ = std::max(0.5, cached_topo_goal_reuse_max_dist_);
    ROS_INFO("[PlannerManager] Multi-topology MPPI optimization: %s", pp_.use_multi_topo_mppi_optimization ? "ENABLED" : "DISABLED");
    if (pp_.use_multi_topo_mppi_optimization && legacy_parallel_mppi) {
      ROS_WARN("[PlannerManager] manager/use_parallel_mppi_optimization is deprecated; use manager/use_multi_topo_mppi_optimization");
    }
    ROS_INFO("[PlannerManager] Z safety bounds: hard=[%.2f, %.2f], publish_min=%.2f, recovery_min=%.2f",
             min_z_, max_z_, publish_min_z_, recovery_min_z_);
    ROS_INFO("[PlannerManager] Planar flight z lock: %s",
             planar_flight_z_lock_ ? "ON" : "OFF");
    ROS_INFO("[PlannerManager] Mode score params: goal=%.1f clearance=%.1f@%.2fm dynamic=%.1f@%.2fm progress=%.1f/%.1f@%.2f early=%.1f@%.2f direction=%.1f@%.2f length=%.1f overshoot=%.1f skip_pts=%d",
             mode_score_goal_weight_, mode_score_clearance_weight_, mode_score_clearance_margin_,
             mode_score_dynamic_weight_, mode_score_dynamic_margin_,
             mode_score_reverse_progress_weight_, mode_score_weak_progress_weight_,
             mode_score_min_progress_, mode_score_early_progress_weight_, mode_score_min_early_progress_,
             mode_score_direction_weight_, mode_score_min_direction_cos_,
             mode_score_path_length_weight_, mode_score_overshoot_weight_, mode_score_safety_skip_points_);
    ROS_INFO("[PlannerManager] Dynamic scene safety gate: unsafe<%.2fm preferred>=%.2fm slack=%.1f",
             dynamic_scene_unsafe_clearance_, dynamic_scene_preferred_clearance_,
             dynamic_scene_safe_cost_slack_);
    ROS_INFO("[PlannerManager] Static scene safety gate: %s unsafe<%.2fm preferred>=%.2fm score_slack=%.1f best_effort=%s improve>=%.2fm preferred_gate=%s slack=%.1f",
             static_scene_safety_gate_enabled_ ? "ON" : "OFF",
             static_scene_unsafe_clearance_, static_scene_preferred_clearance_,
             static_scene_safe_score_slack_,
             static_scene_best_effort_gate_enabled_ ? "ON" : "OFF",
             static_scene_min_clearance_improvement_,
             static_scene_preferred_clearance_gate_enabled_ ? "ON" : "OFF",
             static_scene_preferred_clearance_score_slack_);
    ROS_INFO("[PlannerManager] MPPI seed preserve clearance: %.2fm", mppi_seed_preserve_clearance_);
    ROS_INFO("[PlannerManager] Planning timing CSV: %s (%s)",
             planning_timing_log_path_.c_str(),
             planning_timing_log_enabled_ ? "ENABLED" : "disabled");
    ROS_INFO("[PlannerManager] Candidate quality CSV: %s (%s)",
             candidate_quality_log_path_.c_str(),
             candidate_quality_log_enabled_ ? "ENABLED" : "disabled");
    ROS_INFO("[PlannerManager] Safe seed time stretch max ratio: %.2f",
             safe_seed_time_stretch_max_);
    ROS_INFO("[PlannerManager] Dynamic-scene B-spline skip threshold: %d consecutive failure(s)",
             bspline_skip_threshold_);
    ROS_INFO("[PlannerManager] Static-scene safe seed preserve: %s min_clearance=%.2fm",
             static_scene_preserve_safe_seed_ ? "ON" : "OFF",
             static_scene_preserve_min_clearance_);
    ROS_INFO("[PlannerManager] Dynamic-scene safe seed preserve: %s",
             dynamic_scene_preserve_safe_seed_ ? "ON" : "OFF");
    ROS_INFO("[PlannerManager] Dynamic escape start skip time: %.2fs",
             dynamic_escape_start_skip_time_);
    ROS_INFO("[PlannerManager] Structured dynamic surface check: %s",
             use_structured_dynamic_surface_check_ ? "ON" : "OFF");
    ROS_INFO("[PlannerManager] Final static clearance threshold: %.2fm", final_static_min_clearance_);
    ROS_INFO("[PlannerManager] Final static check start skip: %.2fs", final_static_check_start_skip_);
    ROS_INFO("[PlannerManager] Final fallback feasibility repair: %s",
             final_fallback_feasibility_repair_ ? "ON" : "OFF");
    ROS_INFO("[PlannerManager] Final publish feasibility gate: %s",
             final_publish_feasibility_gate_enabled_ ? "ON" : "OFF");
    ROS_INFO("[PlannerManager] Geometric seed repair: %s target=%.2fm step=%.2fm iter=%d",
             geometric_seed_repair_enabled_ ? "ON" : "OFF",
             geometric_seed_repair_clearance_, geometric_seed_repair_step_,
             geometric_seed_repair_iterations_);
    ROS_INFO("[PlannerManager] Dynamic-scene final gates: static_window=%.2f/%.2fs dynamic_min=%.2fm",
             final_dynamic_static_check_fraction_, final_dynamic_static_check_time_,
             final_dynamic_min_distance_);
    ROS_INFO("[PlannerManager] Dynamic publish preferred distance: %.2fm",
             dynamic_publish_preferred_distance_);
    ROS_INFO("[PlannerManager] Dynamic grid-map radius compensation: %.2fm",
             dynamic_distance_radius_compensation_);
    ROS_INFO("[PlannerManager] Dynamic safety time buffer: %.2fs",
             dynamic_safety_time_buffer_);
    ROS_INFO("[PlannerManager] Clearance recovery: %s",
             clearance_recovery_enabled_ ? "ON" : "OFF");
    ROS_INFO("[PlannerManager] Static escape check: after_failures=%d start_skip=%.2fs initial<=%.2fm post>=%.2fm",
             static_escape_check_after_failures_, static_escape_check_start_skip_,
             static_escape_max_initial_clearance_, static_escape_min_post_clearance_);
    ROS_INFO("[PlannerManager] Topo prefilter: max_candidates=%d clearance_weight=%.1f dynamic_weight=%.1f margin=%.2f start_skip=%.2fm",
             max_mppi_topo_candidates_, topo_prefilter_clearance_weight_,
             topo_prefilter_dynamic_weight_, topo_prefilter_clearance_margin_,
             topo_safety_start_skip_dist_);
    ROS_INFO("[PlannerManager] Cached topo rebase max distance: %.2fm, goal reuse max: %.2fm",
             cached_topo_rebase_max_dist_, cached_topo_goal_reuse_max_dist_);
    ROS_INFO("[PlannerManager] Cached topo commit policy: %s",
             cache_only_published_topo_paths_ ? "published trajectory only" : "topo search success");
    ROS_INFO("[PlannerManager] Prefer cached published topo before fresh search: %s",
             prefer_cached_topo_paths_ ? "ON" : "OFF");
    if (ablation_disable_topo_guidance_ || ablation_disable_mppi_optimization_) {
      ROS_WARN("[PlannerManager] ABLATION MODE: topo_guidance=%s, mppi_optimization=%s",
               ablation_disable_topo_guidance_ ? "DISABLED" : "ENABLED",
               ablation_disable_mppi_optimization_ ? "DISABLED" : "ENABLED");
    }

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->mppi_planner_.reset(new MPPIPlanner);
    bspline_optimizer_rebound_->mppi_planner_->setForceCPU(true);  //  Force this instance to use CPU
    bspline_optimizer_rebound_->mppi_planner_->init(nh, grid_map_);
    bspline_optimizer_rebound_->mppi_planner_->setVehicleLimits(pp_.max_vel_, pp_.max_acc_);  // BUG FIX: Sync dynamics
    bspline_optimizer_rebound_->mppi_planner_->setVehicleLimitsZ(pp_.max_vel_z_, pp_.max_acc_z_);  // Z-axis limits
    ROS_INFO("[PlannerManager] BsplineOptimizer MPPI: Forced CPU-only (avoid multi-GPU instance conflict)");

    /* Initialize new planning modules */
    topo_planner_.reset(new TopoPRM);
    topo_planner_->init(nh, grid_map_);
    topo_planner_->setStepSize(0.2);
    topo_planner_->setSearchRadius(3.0);
    topo_planner_->setMaxSampleNum(1000);

    //  Initialize MPPI planner (uses constructor defaults: 500 samples, 15 horizon, etc.)
    mppi_planner_.reset(new MPPIPlanner);
    mppi_planner_->init(nh, grid_map_);
    //  BUG FIX: Sync MPPI dynamics with planner limits (defaults are 3.0/3.0, actual is 2.0/2.5)
    mppi_planner_->setVehicleLimits(pp_.max_vel_, pp_.max_acc_);
    mppi_planner_->setVehicleLimitsZ(pp_.max_vel_z_, pp_.max_acc_z_);
    ROS_INFO("[PlannerManager] MPPI dynamics synced: max_vel=%.1f, max_acc=%.1f, max_vel_z=%.1f, max_acc_z=%.1f",
             pp_.max_vel_, pp_.max_acc_, pp_.max_vel_z_, pp_.max_acc_z_);

    ROS_INFO("[PlannerManager] Initialized topological and MPPI planners");

    //  Initialize publishers for visualization
    all_mppi_paths_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/all_mppi_candidate_paths", 10);
    topo_paths_smooth_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_paths_smooth", 10);
    ROS_INFO("[PlannerManager] Initialized visualization topics:");
    ROS_INFO("[PlannerManager]   /all_mppi_candidate_paths - MPPI optimized candidate paths");
    ROS_INFO("[PlannerManager]   /topo_paths_smooth - Topo path B-spline smooth visualization");

    //  Subscribe to dynamic obstacle state from dynamic_obstacle_generator
    dynamic_obstacle_sub_ = nh.subscribe("/dynamic_obstacles/state", 1, 
                                         &EGOPlannerManager::dynamicObstacleCallback, this);
    ROS_INFO("[PlannerManager]  Subscribed to /dynamic_obstacles/state for MPPI dynamic obstacle avoidance");

    visualization_ = vis;
  }

  //  Dynamic obstacle callback - stores latest data for MPPI consumption
  void EGOPlannerManager::dynamicObstacleCallback(const ddo_planner::DynamicObstaclesConstPtr& msg) {
    std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
    latest_dynamic_obstacles_ = *msg;
    has_dynamic_obstacles_ = true;
    ROS_INFO_THROTTLE(5.0, "[PlannerManager]  Received %zu dynamic obstacles (horizon=%.1fs, dt=%.2fs, steps=%d)", 
                      msg->obstacles.size(), msg->prediction_horizon, msg->prediction_dt, msg->prediction_steps);
  }

  //  Feed dynamic obstacle data to MPPI planner before each planning call
  void EGOPlannerManager::feedDynamicObstaclesToMPPI() {
    std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
    
    if (!has_dynamic_obstacles_ || latest_dynamic_obstacles_.obstacles.empty()) {
      return;
    }
    
    int num_obstacles = latest_dynamic_obstacles_.obstacles.size();
    int prediction_steps = latest_dynamic_obstacles_.prediction_steps;
    float prediction_dt = (float)latest_dynamic_obstacles_.prediction_dt;
    if (prediction_steps <= 0 || prediction_dt <= 1e-4f) {
      ROS_WARN_THROTTLE(2.0, "[PlannerManager] Invalid dynamic obstacle prediction metadata (steps=%d, dt=%.3f), skipping",
                        prediction_steps, prediction_dt);
      return;
    }

    if (mppi_planner_ != nullptr) {
      const int required_steps = std::max(1, mppi_planner_->getHorizonSteps());
      if (prediction_steps < required_steps) {
        ROS_WARN_THROTTLE(2.0,
                          "[PlannerManager] Dynamic prediction horizon shorter than MPPI horizon (%d < %d); extrapolating",
                          prediction_steps, required_steps);
        prediction_steps = required_steps;
      }
      const float mppi_dt = static_cast<float>(mppi_planner_->getTimeStep());
      if (mppi_dt > 1e-4f && std::abs(mppi_dt - prediction_dt) > 1e-3f) {
        ROS_WARN_THROTTLE(2.0,
                          "[PlannerManager] Dynamic prediction dt %.3f differs from MPPI dt %.3f; using prediction dt for obstacle timeline",
                          prediction_dt, mppi_dt);
      }
    }
    
    // Flatten all predicted positions: [obs0_t0, obs0_t1, ..., obs0_tN, obs1_t0, ...]
    std::vector<Eigen::Vector3d> all_positions;
    std::vector<float> radii;
    std::vector<float> heights;
    
    for (const auto& obs : latest_dynamic_obstacles_.obstacles) {
      radii.push_back((float)obs.radius);
      heights.push_back((float)obs.height);
      
      if (!obs.predicted_positions.empty()) {
        const int num_preds = obs.predicted_positions.size();
        Eigen::Vector3d last_pos(obs.predicted_positions.back().x,
                                 obs.predicted_positions.back().y,
                                 obs.predicted_positions.back().z);
        Eigen::Vector3d extrap_vel(obs.velocity.x, obs.velocity.y, obs.velocity.z);
        if (num_preds >= 2 && prediction_dt > 1e-4f) {
          const auto& p0 = obs.predicted_positions[num_preds - 2];
          const auto& p1 = obs.predicted_positions[num_preds - 1];
          extrap_vel = (Eigen::Vector3d(p1.x, p1.y, p1.z) -
                        Eigen::Vector3d(p0.x, p0.y, p0.z)) / prediction_dt;
        }

        for (int t = 0; t < prediction_steps; ++t) {
          if (t < num_preds) {
            const auto& p = obs.predicted_positions[t];
            all_positions.push_back(Eigen::Vector3d(p.x, p.y, p.z));
          } else {
            all_positions.push_back(last_pos + extrap_vel * ((t - num_preds + 1) * prediction_dt));
          }
        }
      } else {
        // Fallback: constant velocity prediction
        Eigen::Vector3d pos(obs.position.x, obs.position.y, obs.position.z);
        Eigen::Vector3d vel(obs.velocity.x, obs.velocity.y, obs.velocity.z);
        for (int t = 0; t < prediction_steps; ++t) {
          all_positions.push_back(pos + vel * (t * prediction_dt));
        }
      }
    }
    
    // Feed to MPPI planner (handles both GPU and CPU)
    if (mppi_planner_ != nullptr) {
      mppi_planner_->setDynamicObstacles(all_positions, radii, heights, num_obstacles, prediction_steps, prediction_dt);
    }
    
    ROS_INFO_THROTTLE(2.0, "[PlannerManager]  Fed %d dynamic obstacles to MPPI (%zu predicted positions)", 
                      num_obstacles, all_positions.size());
  }

  bool EGOPlannerManager::isPathWithinZBounds(const std::vector<Eigen::Vector3d> &path,
                                              double *min_z_observed,
                                              double *max_z_observed) const {
    if (path.empty()) {
      if (min_z_observed) *min_z_observed = std::numeric_limits<double>::quiet_NaN();
      if (max_z_observed) *max_z_observed = std::numeric_limits<double>::quiet_NaN();
      return false;
    }

    double min_z_path = std::numeric_limits<double>::infinity();
    double max_z_path = -std::numeric_limits<double>::infinity();
    for (const auto &pt : path) {
      min_z_path = std::min(min_z_path, pt.z());
      max_z_path = std::max(max_z_path, pt.z());
    }

    if (min_z_observed) *min_z_observed = min_z_path;
    if (max_z_observed) *max_z_observed = max_z_path;
    return min_z_path >= publish_min_z_ && max_z_path <= max_z_;
  }

  bool EGOPlannerManager::generateRecoveryTraj(const Eigen::Vector3d &start_pt,
                                               const Eigen::Vector3d &start_vel,
                                               const Eigen::Vector3d &start_acc,
                                               const Eigen::Vector3d &local_target_pt,
                                               double ts) {
    Eigen::Vector3d recovery_pt = start_pt;
    recovery_pt.z() = std::min(max_z_ - 0.1,
                               std::max(std::max(recovery_min_z_, local_target_pt.z()), start_pt.z()));

    std::vector<Eigen::Vector3d> recovery_points;
    for (int i = 0; i < 5; ++i) {
      const double alpha = static_cast<double>(i) / 4.0;
      recovery_points.push_back(start_pt * (1.0 - alpha) + recovery_pt * alpha);
    }

    std::vector<Eigen::Vector3d> derivatives;
    derivatives.push_back(start_vel);
    derivatives.push_back(Eigen::Vector3d::Zero());
    derivatives.push_back(start_acc);
    derivatives.push_back(Eigen::Vector3d::Zero());

    Eigen::MatrixXd recovery_ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, recovery_points, derivatives, recovery_ctrl_pts);
    UniformBspline recovery_traj(recovery_ctrl_pts, 3, ts);
    recovery_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double min_z_recovery = 0.0;
    double max_z_recovery = 0.0;
    isBsplineWithinZBounds(recovery_traj, &min_z_recovery, &max_z_recovery);
    if (min_z_recovery < -0.05 || max_z_recovery > max_z_) {
      ROS_WARN("[PlannerManager] Recovery B-spline rejected by z safety (z=[%.2f, %.2f], publish_min=%.2f)",
               min_z_recovery, max_z_recovery, publish_min_z_);
      return false;
    }

    updateTrajInfo(recovery_traj, ros::Time::now());
    last_traj_is_recovery_ = true;
    ROS_WARN("[PlannerManager] Published recovery climb trajectory (z %.2f -> %.2f)",
             start_pt.z(), recovery_pt.z());
    return true;
  }

  bool EGOPlannerManager::isBsplineWithinZBounds(UniformBspline position_traj,
                                                 double *min_z_observed,
                                                 double *max_z_observed) const {
    double t_start = 0.0;
    double t_end = 0.0;
    if (!position_traj.getTimeSpan(t_start, t_end) || t_end < t_start) {
      if (min_z_observed) *min_z_observed = std::numeric_limits<double>::quiet_NaN();
      if (max_z_observed) *max_z_observed = std::numeric_limits<double>::quiet_NaN();
      return false;
    }

    double min_z_path = std::numeric_limits<double>::infinity();
    double max_z_path = -std::numeric_limits<double>::infinity();
    const double sample_dt = std::max(0.02, position_traj.getInterval() * 0.5);
    const double duration = std::max(0.0, t_end - t_start);
    for (double t = 0.0; t <= duration + 1e-6; t += sample_dt) {
      Eigen::VectorXd point = position_traj.evaluateDeBoorT(std::min(t, duration));
      min_z_path = std::min(min_z_path, point.z());
      max_z_path = std::max(max_z_path, point.z());
    }

    if (min_z_observed) *min_z_observed = min_z_path;
    if (max_z_observed) *max_z_observed = max_z_path;
    return min_z_path >= publish_min_z_ && max_z_path <= max_z_;
  }

  bool EGOPlannerManager::isBsplineCollisionFree(UniformBspline position_traj,
                                                 double min_clearance,
                                                 double *min_clearance_observed,
                                                 double time_fraction,
                                                 double max_check_time,
                                                 double start_skip_time) const {
    if (grid_map_ == nullptr) {
      if (min_clearance_observed) *min_clearance_observed = 0.0;
      return false;
    }

    double t_start = 0.0;
    double t_end = 0.0;
    if (!position_traj.getTimeSpan(t_start, t_end) || t_end < t_start) {
      if (min_clearance_observed) *min_clearance_observed = 0.0;
      return false;
    }

    double min_dist = std::numeric_limits<double>::infinity();
    const double sample_dt = std::max(0.02, position_traj.getInterval() * 0.5);
    const double clamped_fraction = std::min(1.0, std::max(0.1, time_fraction));
    const double duration = std::max(0.0, t_end - t_start);
    const double check_t_start = std::min(duration, std::max(0.0, start_skip_time));
    const double fraction_t_end = duration * clamped_fraction;
    const double window_t_end = std::isfinite(max_check_time) ? std::max(0.1, max_check_time) : duration;
    const double check_t_end = std::max(check_t_start, std::min(duration, std::min(fraction_t_end, window_t_end)));
    for (double t = check_t_start; t <= check_t_end + 1e-6; t += sample_dt) {
      const Eigen::Vector3d point = position_traj.evaluateDeBoorT(std::min(t, check_t_end));
      if (!grid_map_->isInMap(point) || grid_map_->getInflateOccupancy(point)) {
        if (min_clearance_observed) *min_clearance_observed = 0.0;
        return false;
      }
      min_dist = std::min(min_dist,
                          sanitizeStaticClearanceForSafety(grid_map_->getDistance(point)));
    }

    if (!std::isfinite(min_dist)) {
      min_dist = 0.0;
    }
    if (min_clearance_observed) *min_clearance_observed = min_dist;
    return min_dist >= min_clearance;
  }

  double EGOPlannerManager::queryDynamicSurfaceDistance(const Eigen::Vector3d &point,
                                                        double future_t) const {
    if (grid_map_ == nullptr) {
      return 0.0;
    }

    ddo_planner::DynamicObstacles dynamic_snapshot;
    bool use_structured_dynamic_state = false;
    {
      std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
      use_structured_dynamic_state =
          use_structured_dynamic_surface_check_ &&
          has_dynamic_obstacles_ && !latest_dynamic_obstacles_.obstacles.empty();
      if (use_structured_dynamic_state) {
        dynamic_snapshot = latest_dynamic_obstacles_;
      }
    }

    double prediction_age = 0.0;
    if (use_structured_dynamic_state && !dynamic_snapshot.header.stamp.isZero()) {
      prediction_age = std::max(0.0, (ros::Time::now() - dynamic_snapshot.header.stamp).toSec());
      if (prediction_age > 0.8) {
        use_structured_dynamic_state = false;
      }
    }

    if (!use_structured_dynamic_state) {
      return grid_map_->getDynamicDistance(point, std::max(0.0, future_t)) -
             dynamic_distance_radius_compensation_;
    }

    double min_surface_dist = std::numeric_limits<double>::infinity();
    const double pred_dt =
        std::max(1e-3, dynamic_snapshot.prediction_dt > 1e-4
                            ? dynamic_snapshot.prediction_dt
                            : 0.1);
    const double query_t = std::max(0.0, future_t + prediction_age);
    for (const auto &obs : dynamic_snapshot.obstacles) {
      Eigen::Vector3d obs_pos(obs.position.x, obs.position.y, obs.position.z);
      if (!obs.predicted_positions.empty()) {
        const double fidx = query_t / pred_dt;
        const int idx0 = std::max(
            0, std::min(static_cast<int>(std::floor(fidx)),
                        static_cast<int>(obs.predicted_positions.size()) - 1));
        const int idx1 = std::max(
            0, std::min(idx0 + 1,
                        static_cast<int>(obs.predicted_positions.size()) - 1));
        const double alpha =
            std::min(1.0, std::max(0.0, fidx - static_cast<double>(idx0)));
        const auto &p0 = obs.predicted_positions[idx0];
        const auto &p1 = obs.predicted_positions[idx1];
        obs_pos = Eigen::Vector3d(p0.x * (1.0 - alpha) + p1.x * alpha,
                                  p0.y * (1.0 - alpha) + p1.y * alpha,
                                  p0.z * (1.0 - alpha) + p1.z * alpha);
      } else {
        obs_pos += Eigen::Vector3d(obs.velocity.x, obs.velocity.y, obs.velocity.z) * query_t;
      }

      const double surface_dist = distanceToDynamicCylinderSurface(
          point, obs_pos, obs.radius, obs.height);
      min_surface_dist = std::min(min_surface_dist, surface_dist);
    }

    if (!std::isfinite(min_surface_dist)) {
      return 10000.0;
    }
    return min_surface_dist;
  }

  bool EGOPlannerManager::isBsplineDynamicSafe(UniformBspline position_traj,
                                               double safety_radius,
                                               double *min_dynamic_distance_observed,
                                               double start_skip_time) const {
    if (grid_map_ == nullptr) {
      if (min_dynamic_distance_observed) *min_dynamic_distance_observed = 0.0;
      return false;
    }

    double t_start = 0.0;
    double t_end = 0.0;
    if (!position_traj.getTimeSpan(t_start, t_end) || t_end < t_start) {
      if (min_dynamic_distance_observed) *min_dynamic_distance_observed = 0.0;
      return false;
    }

    double min_dist = std::numeric_limits<double>::infinity();
    const double sample_dt = std::max(0.05, position_traj.getInterval() * 0.5);
    const double duration = std::max(0.0, t_end - t_start);

    const double check_t_start = std::min(duration, std::max(0.0, start_skip_time));
    for (double t = check_t_start; t <= duration + 1e-6; t += sample_dt) {
      const double traj_t = std::min(t, duration);
      const Eigen::Vector3d point = position_traj.evaluateDeBoorT(traj_t);
      const double future_t = std::max(0.0, traj_t + dynamic_safety_time_buffer_);
      const double dist = queryDynamicSurfaceDistance(point, future_t);
      min_dist = std::min(min_dist, dist);
      if (dist < safety_radius) {
        if (min_dynamic_distance_observed) *min_dynamic_distance_observed = min_dist;
        return false;
      }
    }

    if (min_dynamic_distance_observed) *min_dynamic_distance_observed = min_dist;
    return true;
  }

  double EGOPlannerManager::getDynamicSurfaceDistanceForSafety(
      const Eigen::Vector3d &point, double future_t) const {
    return queryDynamicSurfaceDistance(point, future_t);
  }

  bool EGOPlannerManager::generateClearanceRecoveryTraj(const Eigen::Vector3d &start_pt,
                                                        const Eigen::Vector3d &start_vel,
                                                        const Eigen::Vector3d &start_acc,
                                                        const Eigen::Vector3d &local_target_pt,
                                                        double ts) {
    if (grid_map_ == nullptr || ts <= 1e-4) {
      return false;
    }

    const auto sanitize_clearance = [](double dist) {
      if (!std::isfinite(dist) || dist < 0.0 || dist > 20.0) {
        return 0.0;
      }
      return std::min(5.0, dist);
    };

    const double current_clearance =
        (grid_map_->isInMap(start_pt) && !grid_map_->getInflateOccupancy(start_pt))
            ? sanitize_clearance(grid_map_->getDistance(start_pt))
            : 0.0;
    const Eigen::Vector3d to_goal = local_target_pt - start_pt;
    Eigen::Vector3d goal_dir = to_goal;
    goal_dir.z() = 0.0;
    if (goal_dir.norm() > 1e-3) {
      goal_dir.normalize();
    } else {
      goal_dir = Eigen::Vector3d::UnitX();
    }

    Eigen::Vector3d best_pt = start_pt;
    double best_clearance = -1.0;
    double best_score = -std::numeric_limits<double>::infinity();
    const std::vector<double> radii = {0.6, 0.9, 1.2, 1.6, 2.0};
    constexpr int kDirs = 24;
    for (double radius : radii) {
      for (int i = 0; i < kDirs; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kDirs);
        Eigen::Vector3d dir(std::cos(theta), std::sin(theta), 0.0);
        Eigen::Vector3d candidate = start_pt + radius * dir;
        candidate.z() = std::min(max_z_ - 0.1, std::max(publish_min_z_, start_pt.z()));
        if (!grid_map_->isInMap(candidate) || grid_map_->getInflateOccupancy(candidate)) {
          continue;
        }

        const double clearance = sanitize_clearance(grid_map_->getDistance(candidate));
        if (clearance <= 1e-3) {
          continue;
        }
        const double progress = (candidate - start_pt).dot(goal_dir);
        const double turn_cost =
            start_vel.head<2>().norm() > 0.2
                ? 0.2 * std::max(0.0, -dir.head<2>().dot(start_vel.head<2>().normalized()))
                : 0.0;
        const double score = clearance + 0.15 * progress - turn_cost - 0.05 * radius;
        if (score > best_score) {
          best_score = score;
          best_clearance = clearance;
          best_pt = candidate;
        }
      }
    }

    const double min_required_improvement = 0.08;
    if (best_clearance < std::max(0.10, current_clearance + min_required_improvement)) {
      ROS_WARN("[PlannerManager] Clearance recovery failed to find improving local target (current=%.2fm best=%.2fm)",
               current_clearance, best_clearance);
      return false;
    }

    std::vector<Eigen::Vector3d> recovery_points;
    recovery_points.reserve(7);
    for (int i = 0; i < 7; ++i) {
      const double a = static_cast<double>(i) / 6.0;
      recovery_points.push_back(start_pt * (1.0 - a) + best_pt * a);
    }

    std::vector<Eigen::Vector3d> derivatives(4, Eigen::Vector3d::Zero());
    derivatives[0] = start_vel;
    derivatives[2] = start_acc;

    Eigen::MatrixXd recovery_ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, recovery_points, derivatives, recovery_ctrl_pts);
    if (recovery_ctrl_pts.cols() < 6) {
      return false;
    }

    UniformBspline recovery_traj(recovery_ctrl_pts, 3, ts);
    recovery_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio = 1.0;
    if (!recovery_traj.checkFeasibility(ratio, false) &&
        std::isfinite(ratio) && ratio > 1.0) {
      recovery_traj.lengthenTime(std::min(1.8, ratio * 1.05));
    }

    double min_z_path = 0.0;
    double max_z_path = 0.0;
    if (!isBsplineWithinZBounds(recovery_traj, &min_z_path, &max_z_path)) {
      ROS_WARN("[PlannerManager] Clearance recovery rejected by z bounds (z=[%.2f, %.2f])",
               min_z_path, max_z_path);
      return false;
    }

    double recovery_min_clearance = 0.0;
    if (!isBsplineCollisionFree(recovery_traj, std::min(0.12, final_static_min_clearance_),
                                &recovery_min_clearance, 1.0,
                                std::numeric_limits<double>::infinity(), 0.35)) {
      ROS_WARN("[PlannerManager] Clearance recovery rejected by static check (min_clearance=%.2fm)",
               recovery_min_clearance);
      return false;
    }

    last_traj_is_recovery_ = true;
    updateTrajInfo(recovery_traj, ros::Time::now());
    ROS_WARN("[PlannerManager] Published clearance recovery trajectory to (%.2f, %.2f, %.2f), clearance %.2f -> %.2f",
             best_pt.x(), best_pt.y(), best_pt.z(), current_clearance, best_clearance);
    return true;
  }

  bool EGOPlannerManager::clampBsplineControlPointsZ(Eigen::MatrixXd &control_points,
                                                     const Eigen::Vector3d &start_pt,
                                                     const Eigen::Vector3d &local_target_pt) const {
    if (control_points.rows() < 3 || control_points.cols() == 0) {
      return false;
    }

    if (start_pt.z() < publish_min_z_ - 1e-3 || local_target_pt.z() < publish_min_z_ - 1e-3) {
      return false;
    }

    const double z_floor = std::max(publish_min_z_, std::min(start_pt.z(), local_target_pt.z()));
    bool changed = false;
    for (int i = 0; i < control_points.cols(); ++i) {
      if (control_points(2, i) < z_floor) {
        control_points(2, i) = z_floor;
        changed = true;
      } else if (control_points(2, i) > max_z_) {
        control_points(2, i) = max_z_;
        changed = true;
      }
    }

    return changed;
  }

  bool EGOPlannerManager::densifyPathForBspline(const std::vector<Eigen::Vector3d> &input_path,
                                                double max_spacing,
                                                std::vector<Eigen::Vector3d> &dense_path) const {
    dense_path.clear();
    if (input_path.size() < 2) {
      return false;
    }

    const double spacing = std::max(0.08, max_spacing);
    dense_path.reserve(input_path.size() * 3);
    for (size_t i = 0; i + 1 < input_path.size(); ++i) {
      dense_path.push_back(input_path[i]);
      const Eigen::Vector3d seg = input_path[i + 1] - input_path[i];
      const double seg_len = seg.norm();
      const int segments = std::max(1, static_cast<int>(std::ceil(seg_len / spacing)));
      for (int j = 1; j < segments; ++j) {
        const double a = static_cast<double>(j) / static_cast<double>(segments);
        dense_path.push_back(input_path[i] * (1.0 - a) + input_path[i + 1] * a);
      }
    }
    dense_path.push_back(input_path.back());

    if (dense_path.size() >= 4) {
      return true;
    }

    std::vector<Eigen::Vector3d> expanded;
    const Eigen::Vector3d start = input_path.front();
    const Eigen::Vector3d end = input_path.back();
    for (int i = 0; i < 7; ++i) {
      const double a = static_cast<double>(i) / 6.0;
      expanded.push_back(start * (1.0 - a) + end * a);
    }
    dense_path.swap(expanded);
    return true;
  }

  bool EGOPlannerManager::buildSafeSeedBspline(const std::vector<Eigen::Vector3d> &seed_path,
                                               const std::vector<Eigen::Vector3d> &start_end_derivative,
                                               double ts,
                                               UniformBspline &safe_traj,
                                               double static_check_time_fraction,
                                               double static_check_max_time,
                                               double static_check_start_skip,
                                               double dynamic_check_start_skip,
                                               double *min_clearance_observed,
                                               double *min_dynamic_distance_observed) const {
    if (seed_path.size() < 2 || ts <= 1e-4) {
      return false;
    }

    std::vector<Eigen::Vector3d> derivatives = start_end_derivative;
    if (derivatives.size() != 4) {
      derivatives.assign(4, Eigen::Vector3d::Zero());
    }

    const double base_spacing = std::max(0.10, pp_.ctrl_pt_dist);
    const double spacing_scales[] = {0.75, 0.50, 0.35};
    for (double scale : spacing_scales) {
      std::vector<Eigen::Vector3d> dense_path;
      if (!densifyPathForBspline(seed_path, base_spacing * scale, dense_path)) {
        continue;
      }

      Eigen::MatrixXd seed_ctrl_pts;
      UniformBspline::parameterizeToBspline(ts, dense_path, derivatives, seed_ctrl_pts);
      if (seed_ctrl_pts.cols() < 6) {
        continue;
      }

      UniformBspline candidate(seed_ctrl_pts, 3, ts);
      candidate.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

      double feasibility_ratio = 1.0;
      if (!candidate.checkFeasibility(feasibility_ratio, false) &&
          std::isfinite(feasibility_ratio) && feasibility_ratio > 1.0) {
        candidate.lengthenTime(std::min(2.2, feasibility_ratio * 1.05));
      }

      double min_z_path = 0.0, max_z_path = 0.0;
      if (!isBsplineWithinZBounds(candidate, &min_z_path, &max_z_path)) {
        ROS_DEBUG("[PlannerManager] Direct seed rejected by z bounds at spacing %.2f (z=[%.2f, %.2f])",
                  base_spacing * scale, min_z_path, max_z_path);
        continue;
      }

      double min_clearance = 0.0;
      if (!isBsplineCollisionFree(candidate, final_static_min_clearance_, &min_clearance,
                                  static_check_time_fraction, static_check_max_time,
                                  static_check_start_skip)) {
        ROS_DEBUG("[PlannerManager] Direct seed rejected by static clearance at spacing %.2f (clear=%.2f)",
                  base_spacing * scale, min_clearance);
        if (min_clearance_observed) {
          *min_clearance_observed = min_clearance;
        }
        continue;
      }

      double min_dynamic_dist = 0.0;
      if (!isBsplineDynamicSafe(candidate, final_dynamic_min_distance_,
                                &min_dynamic_dist, dynamic_check_start_skip)) {
        ROS_DEBUG("[PlannerManager] Direct seed rejected by dynamic clearance at spacing %.2f (dyn=%.2f)",
                  base_spacing * scale, min_dynamic_dist);
        if (min_dynamic_distance_observed) {
          *min_dynamic_distance_observed = min_dynamic_dist;
        }
        continue;
      }

      if (min_clearance_observed) {
        *min_clearance_observed = min_clearance;
      }
      if (min_dynamic_distance_observed) {
        *min_dynamic_distance_observed = min_dynamic_dist;
      }
      safe_traj = candidate;
      return true;
    }

    return false;
  }

  bool EGOPlannerManager::repairSeedPathByClearance(
      const std::vector<Eigen::Vector3d> &seed_path,
      std::vector<Eigen::Vector3d> &repaired_path) const {
    repaired_path = seed_path;
    if (!geometric_seed_repair_enabled_ || grid_map_ == nullptr ||
        repaired_path.size() < 4) {
      return false;
    }

    bool changed = false;
    const double target_clearance =
        std::max(final_static_min_clearance_, geometric_seed_repair_clearance_);
    const double max_step = geometric_seed_repair_step_;

    for (int iter = 0; iter < geometric_seed_repair_iterations_; ++iter) {
      bool iter_changed = false;
      for (size_t i = 1; i + 1 < repaired_path.size(); ++i) {
        Eigen::Vector3d point = repaired_path[i];
        if (!grid_map_->isInMap(point)) {
          return false;
        }

        Eigen::Vector3d grad = Eigen::Vector3d::Zero();
        const double dist = grid_map_->getDistanceWithGrad(point, grad);
        if (dist >= target_clearance) {
          continue;
        }

        grad.z() = 0.0;
        if (grad.norm() < 1e-3) {
          const Eigen::Vector3d tangent = repaired_path[i + 1] - repaired_path[i - 1];
          grad = Eigen::Vector3d(-tangent.y(), tangent.x(), 0.0);
        }
        if (grad.norm() < 1e-3) {
          continue;
        }

        const Eigen::Vector3d direction = grad.normalized();
        const double push = std::min(max_step, target_clearance - dist + 0.03);
        Eigen::Vector3d candidate = point + push * direction;
        candidate.z() = std::min(max_z_ - 0.05,
                                 std::max(publish_min_z_, candidate.z()));

        if (!grid_map_->isInMap(candidate) || grid_map_->getInflateOccupancy(candidate)) {
          continue;
        }
        const double candidate_dist = grid_map_->getDistance(candidate);
        if (candidate_dist <= dist + 1e-3) {
          continue;
        }

        repaired_path[i] = candidate;
        iter_changed = true;
        changed = true;
      }

      if (!iter_changed) {
        break;
      }
    }

    return changed;
  }

  // !SECTION

  // SECTION rebond replanning

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    PlanningCycleTiming timing;
    timing.cycle = ++planning_timing_cycle_;
    timing.start_goal_dist = (start_pt - local_target_pt).norm();
    timing.continuous_failures = continous_failures_count_;
    timing.bspline_failures = bspline_consecutive_failures_;
    const ros::Time cycle_start_time = ros::Time::now();
    ros::Time validator_start_time;
    bool validator_started = false;
    struct TimingScope {
      EGOPlannerManager *manager;
      PlanningCycleTiming *timing;
      ros::Time cycle_start;
      ~TimingScope()
      {
        timing->total_ms = (ros::Time::now() - cycle_start).toSec() * 1000.0;
        timing->continuous_failures = manager->continous_failures_count_;
        timing->bspline_failures = manager->bspline_consecutive_failures_;
        manager->appendPlanningCycleTiming(*timing);
      }
    } timing_scope{this, &timing, cycle_start_time};
    auto finish_cycle = [&](bool success, const std::string &reason) {
      if (validator_started) {
        timing.validator_ms =
            (ros::Time::now() - validator_start_time).toSec() * 1000.0;
      }
      timing.success = success;
      timing.fail_reason = reason;
      return success;
    };

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return finish_cycle(false, "close_to_goal");
    }

    if (start_pt.z() < publish_min_z_ + 0.15 && start_vel.z() < -0.05)
    {
      ROS_WARN("[PlannerManager] Low-altitude descent detected before planning (z=%.2f, vz=%.2f); publishing recovery climb",
               start_pt.z(), start_vel.z());
      const bool recovery_ok =
          generateRecoveryTraj(start_pt, start_vel, start_acc, local_target_pt,
                               pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2);
      return finish_cycle(recovery_ok,
                          recovery_ok ? "low_altitude_recovery" : "low_altitude_recovery_fail");
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        if (t_cur + ts >= local_data_.duration_) {
          ROS_WARN("[PlannerManager] Previous trajectory has no usable remainder (t_cur=%.3f, duration=%.3f); regenerate polynomial init",
                   t_cur, local_data_.duration_);
          flag_force_polynomial = true;
          flag_regenerate = true;
          continue;
        }

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_WARN("[PlannerManager] Previous trajectory arc is empty; regenerate polynomial init");
              flag_force_polynomial = true;
              flag_regenerate = true;
              break;
            }
          }
        }

        if (flag_regenerate) {
          continue;
        }

        if (segment_point.size() < 2 || pseudo_arc_length.size() < 2 ||
            pseudo_arc_length.back() < 1e-4) {
          ROS_WARN("[PlannerManager] Previous trajectory arc too short (points=%zu, arc=%.4f); regenerate polynomial init",
                   segment_point.size(), pseudo_arc_length.empty() ? 0.0 : pseudo_arc_length.back());
          flag_force_polynomial = true;
          flag_regenerate = true;
          continue;
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    //  Initialize B-spline optimizer internal structures
    // Note: After architecture refactor (Topo→MPPI→BSpline), initControlPoints() now 
    // initializes with topological/MPPI-optimized points, then BsplineOptimizer refines them
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;
    timing.init_ms = t_init.toSec() * 1000.0;

    //  Visualization: Clear old A* paths (replaced by TOPO+MPPI visualization)
    // static int vis_id = 0;
    // visualization_->displayInitPathList(point_set, 0.2, 0);
    // Note: MPPI paths are now visualized in STEP 1.5 with matching colors

    /*** STEP 1.5: TOPOLOGICAL PLANNING ***/
    //  Architecture: Topo → MPPI → B-spline
    // Topological planning provides global collision-free paths
    
    //  Feed latest dynamic obstacle data to MPPI before planning
    feedDynamicObstaclesToMPPI();
    
    std::vector<TopoPath> topo_paths;
    if (!ablation_disable_topo_guidance_) {
      ROS_INFO("[PlannerManager] Attempting topological planning from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
               start_pt.x(), start_pt.y(), start_pt.z(),
               local_target_pt.x(), local_target_pt.y(), local_target_pt.z());
    } else {
      ROS_WARN("[PlannerManager] ABLATION: Topo guidance disabled, using straight-line MPPI seed");
    }
    
	    bool use_mppi_topo_path = false;
	    bool use_multi_topo_mppi = pp_.use_multi_topo_mppi_optimization;
	    bool using_static_topo_seed = false;
	    std::vector<TopoPath> topo_paths_for_success_cache;
    
    ros::Time topo_t_start = ros::Time::now();
    const bool topo_search_success =
        !ablation_disable_topo_guidance_ && topo_planner_ != nullptr &&
        planWithTopo(start_pt, local_target_pt, topo_paths);
    timing.topo_ms = (ros::Time::now() - topo_t_start).toSec() * 1000.0;
    timing.topo_paths = static_cast<int>(topo_paths.size());

    if (topo_search_success) {
        ROS_INFO("[PlannerManager] STEP 1: Topological Planning");
        ROS_INFO("[PlannerManager]   Found %zu topological paths", topo_paths.size());
        
        // Ensure we have valid paths before processing
        if (!topo_paths.empty()) {
            struct TopoPreScore {
                size_t original_idx;
                double score;
                double length;
                double min_clearance;
                double min_dynamic_dist;
                bool z_safe;
            };

            const auto compute_topo_prescore = [&](const TopoPath& path, size_t idx) {
                TopoPreScore result;
                result.original_idx = idx;
                result.score = std::numeric_limits<double>::max();
                result.length = 0.0;
                result.min_clearance = std::numeric_limits<double>::infinity();
                result.min_dynamic_dist = std::numeric_limits<double>::infinity();
                result.z_safe = isPathWithinZBounds(path.path);

                if (path.path.size() < 2 || !result.z_safe) {
                    return result;
                }

                for (size_t j = 1; j < path.path.size(); ++j) {
                    const Eigen::Vector3d p0 = path.path[j - 1];
                    const Eigen::Vector3d p1 = path.path[j];
                    const Eigen::Vector3d seg = p1 - p0;
                    const double seg_len = seg.norm();
                    const double length_before_segment = result.length;
                    result.length += seg_len;
                    const int samples = std::max(1, static_cast<int>(std::ceil(seg_len / std::max(0.05, pp_.ctrl_pt_dist))));
                    for (int s = 0; s <= samples; ++s) {
                        const double a = static_cast<double>(s) / static_cast<double>(samples);
                        const Eigen::Vector3d p = p0 * (1.0 - a) + p1 * a;
                        const double traveled = length_before_segment + seg_len * a;
                        if (traveled >= topo_safety_start_skip_dist_) {
                            if (grid_map_->isInMap(p) && !grid_map_->getInflateOccupancy(p)) {
                                result.min_clearance = std::min(result.min_clearance,
                                                                std::max(0.0, std::min(5.0, grid_map_->getDistance(p))));
                            } else {
                                result.min_clearance = 0.0;
                            }
                        }
                        const double future_t = std::min(
                            3.0, dynamic_safety_time_buffer_ +
                                     traveled / std::max(0.2, pp_.max_vel_));
                        result.min_dynamic_dist = std::min(
                            result.min_dynamic_dist,
                            std::max(0.0, std::min(5.0, queryDynamicSurfaceDistance(p, future_t))));
                    }
                }

                if (!std::isfinite(result.min_clearance)) result.min_clearance = 5.0;
                if (!std::isfinite(result.min_dynamic_dist)) result.min_dynamic_dist = 5.0;
                const double clearance_penalty =
                    topo_prefilter_clearance_weight_ *
                    std::max(0.0, topo_prefilter_clearance_margin_ - result.min_clearance);
                const double dynamic_penalty =
                    topo_prefilter_dynamic_weight_ *
                    std::max(0.0, dynamic_scene_preferred_clearance_ - result.min_dynamic_dist);
                result.score = path.cost + result.length + clearance_penalty + dynamic_penalty;
                return result;
            };

            std::vector<TopoPreScore> topo_prescores(topo_paths.size());
            #pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < static_cast<int>(topo_paths.size()); ++i) {
                topo_prescores[i] = compute_topo_prescore(topo_paths[i], static_cast<size_t>(i));
            }
            std::sort(topo_prescores.begin(), topo_prescores.end(),
                      [](const TopoPreScore& a, const TopoPreScore& b) {
                          if (a.z_safe != b.z_safe) return a.z_safe > b.z_safe;
                          return a.score < b.score;
                      });

            if (topo_paths.size() > static_cast<size_t>(max_mppi_topo_candidates_)) {
                std::vector<TopoPath> filtered_paths;
                filtered_paths.reserve(max_mppi_topo_candidates_);
                for (int i = 0; i < max_mppi_topo_candidates_ && i < static_cast<int>(topo_prescores.size()); ++i) {
                    filtered_paths.push_back(topo_paths[topo_prescores[i].original_idx]);
                }
                ROS_INFO("[PlannerManager]   Topo prefilter: %zu -> %zu MPPI candidates",
                         topo_paths.size(), filtered_paths.size());
                topo_paths.swap(filtered_paths);
            } else {
                std::vector<TopoPath> sorted_paths;
                sorted_paths.reserve(topo_paths.size());
                for (const auto& score : topo_prescores) {
                    sorted_paths.push_back(topo_paths[score.original_idx]);
                }
                topo_paths.swap(sorted_paths);
            }

            for (size_t i = 0; i < std::min<size_t>(topo_prescores.size(), topo_paths.size()); ++i) {
                ROS_INFO("[PlannerManager]   Topo prefilter rank %zu: orig=%zu score=%.2f len=%.2f clear=%.2f dyn=%.2f z=%d",
                         i + 1, topo_prescores[i].original_idx + 1, topo_prescores[i].score,
                         topo_prescores[i].length, topo_prescores[i].min_clearance,
                         topo_prescores[i].min_dynamic_dist, topo_prescores[i].z_safe);
            }

            TopoPath best_path;
            
            //  CRITICAL OPTIMIZATION: 降低MPPI触发阈值从 >1 到 >=1
            // 即使只有1条路径也运行MPPI优化，确保架构升级生效
            if (!ablation_disable_mppi_optimization_ && use_multi_topo_mppi && mppi_planner_ != nullptr && topo_paths.size() >= 1) {
                ROS_INFO("[PlannerManager] STEP 2: MPPI Optimization ( Triggered for %zu path(s))", topo_paths.size());
                if (topo_paths.size() > 1) {
                    ROS_INFO("[PlannerManager]    Multi-topology optimization mode (batched GPU MPPI, parallel topo prefilter)");
                } else {
                    ROS_INFO("[PlannerManager]    Single-path optimization mode (still beneficial!)");
                }
                ROS_INFO("[PlannerManager]    Optimizing %zu topology-ranked candidate path(s)...", topo_paths.size());
                
                struct MPPICandidate {
                    TopoPath topo_path;
                    MPPITrajectory mppi_result;
                    double normalized_cost;
                    double selection_score;
                    double dynamic_scene_score;
                    double path_length;
                    double topo_length;
                    double final_goal_dist;
                    double progress_ratio;
                    double direction_cos;
                    double max_goal_overshoot;
                    double early_progress_ratio;
                    double min_clearance;
                    double min_dynamic_dist;
                    double topo_min_clearance;
                    double topo_min_dynamic_dist;
                    bool success;
                };
                
                std::vector<MPPICandidate> mppi_candidates(topo_paths.size());
                Eigen::Vector3d current_vel = start_vel;
                Eigen::Vector3d target_vel = local_target_vel;
                
                //  Prepare EDT for GPU ONCE before topo path loop (avoids N×18MB uploads)
                mppi_planner_->beginReplanCycle();

                std::vector<std::vector<Eigen::Vector3d>> dense_paths(topo_paths.size());
                for (size_t i = 0; i < topo_paths.size(); ++i) {
                    dense_paths[i] = topo_paths[i].path;
                    if (dense_paths[i].size() < 7) {
                        std::vector<Eigen::Vector3d> tmp_dense;
                        for (size_t j = 0; j + 1 < dense_paths[i].size(); ++j) {
                            tmp_dense.push_back(dense_paths[i][j]);
                            Eigen::Vector3d seg_vec = dense_paths[i][j + 1] - dense_paths[i][j];
                            double seg_len = seg_vec.norm();
                            int num_inter = std::max(1, (int)(seg_len / (pp_.ctrl_pt_dist * 0.5)));
                            for (int k = 1; k < num_inter; ++k) {
                                double t = (double)k / num_inter;
                                tmp_dense.push_back(dense_paths[i][j] + t * seg_vec);
                            }
                        }
                        tmp_dense.push_back(dense_paths[i].back());
                        dense_paths[i] = tmp_dense;
                    }
                    ROS_INFO("[PlannerManager]   Path %zu/%zu (topo_cost=%.3f, waypoints=%zu): queued for MPPI",
                             i + 1, topo_paths.size(), topo_paths[i].cost, dense_paths[i].size());
                }

                std::vector<MPPITrajectory> batch_results;
                const ros::Time mppi_t_start = ros::Time::now();
                bool batch_mppi_success = mppi_planner_->planTrajectoryBatch(
                    start_pt, current_vel, local_target_pt, target_vel,
                    dense_paths, batch_results);
                timing.mppi_ms += (ros::Time::now() - mppi_t_start).toSec() * 1000.0;
                timing.mppi_candidates = static_cast<int>(dense_paths.size());
                if (batch_mppi_success) {
                    ROS_INFO("[PlannerManager]   Batched MPPI returned %zu candidate trajectory(ies)",
                             batch_results.size());
                } else {
                    ROS_WARN("[PlannerManager]   Batched MPPI failed; candidate loop will mark failed paths");
                }
                
                for (size_t i = 0; i < topo_paths.size(); ++i) {
                    mppi_candidates[i].topo_path = topo_paths[i];
                    mppi_candidates[i].success = false;
                    mppi_candidates[i].normalized_cost = std::numeric_limits<double>::max();
                    mppi_candidates[i].selection_score = std::numeric_limits<double>::max();
                    mppi_candidates[i].dynamic_scene_score = std::numeric_limits<double>::max();
                    mppi_candidates[i].path_length = 0.0;
                    mppi_candidates[i].topo_length = 0.0;
                    mppi_candidates[i].final_goal_dist = std::numeric_limits<double>::max();
                    mppi_candidates[i].progress_ratio = -1.0;
                    mppi_candidates[i].direction_cos = -1.0;
                    mppi_candidates[i].max_goal_overshoot = std::numeric_limits<double>::max();
                    mppi_candidates[i].early_progress_ratio = -1.0;
                    mppi_candidates[i].min_clearance = 0.0;
                    mppi_candidates[i].min_dynamic_dist = 0.0;
                    mppi_candidates[i].topo_min_clearance = 0.0;
                    mppi_candidates[i].topo_min_dynamic_dist = 0.0;

                    double topo_length = 0.0;
                    double topo_min_clearance = std::numeric_limits<double>::infinity();
                    double topo_min_dynamic_dist = std::numeric_limits<double>::infinity();
                    const auto& topo_path_points = topo_paths[i].path;
                    for (size_t j = 1; j < topo_path_points.size(); ++j) {
                        const Eigen::Vector3d p0 = topo_path_points[j - 1];
                        const Eigen::Vector3d p1 = topo_path_points[j];
                        const Eigen::Vector3d seg = p1 - p0;
                        const double seg_len = seg.norm();
                        const double length_before_segment = topo_length;
                        topo_length += seg_len;
                        const int samples = std::max(
                            1, static_cast<int>(std::ceil(seg_len / std::max(0.05, pp_.ctrl_pt_dist))));
                        for (int s = 0; s <= samples; ++s) {
                            const double a = static_cast<double>(s) / static_cast<double>(samples);
                            const Eigen::Vector3d p = p0 * (1.0 - a) + p1 * a;
                            const double traveled = length_before_segment + seg_len * a;
                            if (traveled >= topo_safety_start_skip_dist_) {
                                if (grid_map_->isInMap(p) && !grid_map_->getInflateOccupancy(p)) {
                                    topo_min_clearance = std::min(
                                        topo_min_clearance,
                                        std::max(0.0, std::min(5.0, grid_map_->getDistance(p))));
                                } else {
                                    topo_min_clearance = 0.0;
                                }
                            }
                            const double future_t =
                                std::min(3.0, dynamic_safety_time_buffer_ +
                                                  traveled / std::max(0.2, pp_.max_vel_));
                            topo_min_dynamic_dist = std::min(
                                topo_min_dynamic_dist,
                                std::max(0.0, std::min(5.0, queryDynamicSurfaceDistance(p, future_t))));
                        }
                    }
                    if (!std::isfinite(topo_min_clearance)) topo_min_clearance = 5.0;
                    if (!std::isfinite(topo_min_dynamic_dist)) topo_min_dynamic_dist = 5.0;
                    mppi_candidates[i].topo_length = topo_length;
                    mppi_candidates[i].topo_min_clearance = topo_min_clearance;
                    mppi_candidates[i].topo_min_dynamic_dist = topo_min_dynamic_dist;

                    bool mppi_success =
                        batch_mppi_success && i < batch_results.size() &&
                        !batch_results[i].positions.empty();
                    if (mppi_success) {
                        mppi_candidates[i].mppi_result = batch_results[i];
                    }
                    
                    //  P0改进: GPU现在返回完整30点轨迹 (Phase 2.5A返回6点)
                    double min_z_path = 0.0, max_z_path = 0.0;
                    bool z_safe = mppi_success &&
                                  isPathWithinZBounds(mppi_candidates[i].mppi_result.positions,
                                                      &min_z_path, &max_z_path);

                    if (mppi_success && mppi_candidates[i].mppi_result.positions.size() >= 10 && z_safe) {
                        // Calculate normalized cost (cost per unit length)
                        double path_length = 0.0;
                        double min_clearance = std::numeric_limits<double>::infinity();
                        double min_dynamic_dist = std::numeric_limits<double>::infinity();
                        for (size_t j = 1; j < mppi_candidates[i].mppi_result.positions.size(); ++j) {
                            path_length += (mppi_candidates[i].mppi_result.positions[j] - mppi_candidates[i].mppi_result.positions[j-1]).norm();
                        }
                        const size_t safety_check_start_idx =
                            std::min<size_t>(static_cast<size_t>(mode_score_safety_skip_points_),
                                             mppi_candidates[i].mppi_result.positions.size() - 1);
                        for (size_t j = safety_check_start_idx; j < mppi_candidates[i].mppi_result.positions.size(); ++j) {
                            const Eigen::Vector3d& pos_j = mppi_candidates[i].mppi_result.positions[j];
                            if (grid_map_->isInMap(pos_j) && !grid_map_->getInflateOccupancy(pos_j)) {
                                const double dist = grid_map_->getDistance(pos_j);
                                min_clearance = std::min(min_clearance, std::max(0.0, std::min(5.0, dist)));
                            } else {
                                min_clearance = std::min(min_clearance, 0.0);
                            }
                            const double dyn_dist =
                                queryDynamicSurfaceDistance(
                                    pos_j,
                                    dynamic_safety_time_buffer_ + 0.1 * static_cast<double>(j));
                            min_dynamic_dist = std::min(min_dynamic_dist, std::max(0.0, std::min(5.0, dyn_dist)));
                        }
                        // When close to goal (<5m), path_length is short and MPPI fixed horizon
                        // causes inflated cost/length. Use raw cost for fairer comparison.
                        double dist_to_goal = (local_target_pt - start_pt).norm();
                        if (dist_to_goal < 5.0) {
                            mppi_candidates[i].normalized_cost = mppi_candidates[i].mppi_result.cost;
                        } else {
                            mppi_candidates[i].normalized_cost = (path_length > 0.1) ? (mppi_candidates[i].mppi_result.cost / path_length) : mppi_candidates[i].mppi_result.cost;
                        }
                        const Eigen::Vector3d final_pos = mppi_candidates[i].mppi_result.positions.back();
                        const double final_goal_dist = (final_pos - local_target_pt).norm();
                        const double start_goal_dist = std::max(0.1, dist_to_goal);
                        const double progress_ratio = (dist_to_goal - final_goal_dist) / start_goal_dist;
                        double max_goal_dist = final_goal_dist;
                        for (const auto& path_pos : mppi_candidates[i].mppi_result.positions) {
                            max_goal_dist = std::max(max_goal_dist, (path_pos - local_target_pt).norm());
                        }
                        const size_t early_idx =
                            std::min<size_t>(mppi_candidates[i].mppi_result.positions.size() - 1,
                                             std::max<size_t>(3, mppi_candidates[i].mppi_result.positions.size() / 3));
                        const double early_goal_dist =
                            (mppi_candidates[i].mppi_result.positions[early_idx] - local_target_pt).norm();
                        const double early_progress_ratio =
                            (dist_to_goal - early_goal_dist) / start_goal_dist;
                        const double max_goal_overshoot =
                            std::max(0.0, max_goal_dist - dist_to_goal);
                        const Eigen::Vector3d goal_dir = (local_target_pt - start_pt) / start_goal_dist;
                        const Eigen::Vector3d displacement = final_pos - start_pt;
                        const double displacement_norm = displacement.norm();
                        const double direction_cos = displacement_norm > 1e-3 ? displacement.dot(goal_dir) / displacement_norm : -1.0;
                        const double clearance_penalty =
                            mode_score_clearance_weight_ * std::max(0.0, mode_score_clearance_margin_ - min_clearance);
                        const double dynamic_penalty =
                            mode_score_dynamic_weight_ * std::max(0.0, mode_score_dynamic_margin_ - min_dynamic_dist);
                        const double progress_penalty =
                            mode_score_reverse_progress_weight_ * std::max(0.0, -progress_ratio);
                        const double weak_progress_penalty =
                            mode_score_weak_progress_weight_ * std::max(0.0, mode_score_min_progress_ - progress_ratio);
                        const double direction_penalty =
                            mode_score_direction_weight_ * std::max(0.0, mode_score_min_direction_cos_ - direction_cos);
                        const double path_length_penalty =
                            mode_score_path_length_weight_ *
                            std::max(0.0, path_length - 1.35 * start_goal_dist);
                        const double overshoot_penalty =
                            mode_score_overshoot_weight_ * max_goal_overshoot;
                        const double early_progress_penalty =
                            mode_score_early_progress_weight_ *
                            std::max(0.0, mode_score_min_early_progress_ - early_progress_ratio);
                        mppi_candidates[i].selection_score =
                            mppi_candidates[i].normalized_cost +
                            mode_score_goal_weight_ * final_goal_dist +
                            progress_penalty +
                            weak_progress_penalty +
                            early_progress_penalty +
                            direction_penalty +
                            path_length_penalty +
                            overshoot_penalty +
                            clearance_penalty +
                            dynamic_penalty;
                        const double dynamic_path_discipline_score =
                            mppi_candidates[i].normalized_cost +
                            progress_penalty +
                            0.5 * early_progress_penalty +
                            0.5 * direction_penalty +
                            overshoot_penalty;
                        if (dynamic_scene_use_mode_score_) {
                            mppi_candidates[i].dynamic_scene_score =
                                dynamic_path_discipline_score +
                                clearance_penalty +
                                dynamic_penalty;
                        } else {
                            mppi_candidates[i].dynamic_scene_score = mppi_candidates[i].normalized_cost;
                        }
                        mppi_candidates[i].path_length = path_length;
                        mppi_candidates[i].final_goal_dist = final_goal_dist;
                        mppi_candidates[i].progress_ratio = progress_ratio;
                        mppi_candidates[i].direction_cos = direction_cos;
                        mppi_candidates[i].max_goal_overshoot = max_goal_overshoot;
                        mppi_candidates[i].early_progress_ratio = early_progress_ratio;
                        mppi_candidates[i].min_clearance = min_clearance;
                        mppi_candidates[i].min_dynamic_dist = min_dynamic_dist;
                        mppi_candidates[i].success = true;
                        timing.mppi_successes++;
                        
                        ROS_INFO("[PlannerManager]   Path %zu: MPPI cost=%.3f, norm_cost=%.3f, select_score=%.3f, dyn_score=%.3f, length=%.2fm, final_goal=%.2fm, progress=%.2f, early=%.2f, overshoot=%.2f, dir=%.2f, clear=%.2fm, dyn=%.2fm, topo_clear=%.2fm, topo_dyn=%.2fm, points=%zu", 
                                 i+1, mppi_candidates[i].mppi_result.cost, mppi_candidates[i].normalized_cost,
                                 mppi_candidates[i].selection_score, mppi_candidates[i].dynamic_scene_score,
                                 path_length, final_goal_dist, progress_ratio, early_progress_ratio,
                                 max_goal_overshoot, direction_cos,
                                 min_clearance, min_dynamic_dist,
                                 topo_min_clearance, topo_min_dynamic_dist,
                                 mppi_candidates[i].mppi_result.positions.size());
                    } else {
                        if (mppi_success && !z_safe) {
                            ROS_WARN("[PlannerManager]   Path %zu: MPPI rejected by z safety (z=[%.2f, %.2f], allowed=[%.2f, %.2f])",
                                     i+1, min_z_path, max_z_path, min_z_, max_z_);
                        } else {
                            ROS_WARN("[PlannerManager]   Path %zu: MPPI  failed (points=%zu)", 
                                     i+1, mppi_candidates[i].mppi_result.positions.size());
                        }
                        // Use topo path as fallback for visualization
                        mppi_candidates[i].mppi_result.positions = dense_paths[i];
                        mppi_candidates[i].mppi_result.cost = topo_paths[i].cost;
                    }
                }
                
                
                // Select best MPPI result with topology-mode aware score.
                // Raw MPPI cost remains the base term, but final progress, direction,
                // static clearance and dynamic clearance prevent wrong-mode convergence.
                bool dynamic_scene_active = false;
                {
                    std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
                    dynamic_scene_active = has_dynamic_obstacles_ && !latest_dynamic_obstacles_.obstacles.empty();
                }
                if (dynamic_scene_active) {
                    ROS_INFO("[PlannerManager] Dynamic obstacles active: selecting MPPI candidate by %s with safety gate",
                             dynamic_scene_use_mode_score_ ? "dynamic-aware score" : "normalized cost");
                } else {
                    ROS_INFO("[PlannerManager] Static/maze scene: selecting MPPI candidate by topology-mode score");
                }

                double best_score = std::numeric_limits<double>::max();
                int best_idx = -1;
                if (dynamic_scene_active) {
                    double best_dynamic_score = std::numeric_limits<double>::max();
                    int best_dynamic_idx = -1;
                    for (size_t i = 0; i < mppi_candidates.size(); ++i) {
                        if (mppi_candidates[i].success &&
                            mppi_candidates[i].dynamic_scene_score < best_dynamic_score) {
                            best_dynamic_score = mppi_candidates[i].dynamic_scene_score;
                            best_dynamic_idx = static_cast<int>(i);
                        }
                    }
                    best_idx = best_dynamic_idx;
                    best_score = best_dynamic_score;

	                    const double dynamic_publish_preferred =
	                        dynamic_publish_preferred_distance_;
	                    if (best_dynamic_idx >= 0 &&
	                        (mppi_candidates[best_dynamic_idx].min_clearance < dynamic_scene_unsafe_clearance_ ||
	                         mppi_candidates[best_dynamic_idx].min_dynamic_dist < dynamic_publish_preferred)) {
	                        int safe_idx = -1;
	                        double safe_dynamic_score = std::numeric_limits<double>::max();
	                        int dynamic_better_idx = -1;
	                        double dynamic_better_score = std::numeric_limits<double>::max();
	                        const double allowed_cost =
	                            best_dynamic_score +
	                            std::max(0.0,
	                                     mppi_candidates[best_dynamic_idx].min_dynamic_dist < dynamic_publish_preferred
	                                         ? std::max(dynamic_scene_safe_cost_slack_, 180.0)
	                                         : dynamic_scene_safe_cost_slack_);
	                        for (size_t i = 0; i < mppi_candidates.size(); ++i) {
	                            if (!mppi_candidates[i].success) continue;
	                            if (mppi_candidates[i].dynamic_scene_score > allowed_cost) continue;
	                            if (mppi_candidates[i].min_dynamic_dist >= dynamic_publish_preferred &&
	                                mppi_candidates[i].dynamic_scene_score < dynamic_better_score) {
	                                dynamic_better_score = mppi_candidates[i].dynamic_scene_score;
	                                dynamic_better_idx = static_cast<int>(i);
	                            }
	                            if (mppi_candidates[i].min_clearance < dynamic_scene_preferred_clearance_) continue;
	                            if (mppi_candidates[i].min_dynamic_dist < dynamic_publish_preferred) continue;
	                            if (!dynamic_scene_use_mode_score_) {
	                                if (mppi_candidates[i].dynamic_scene_score < safe_dynamic_score) {
	                                    safe_dynamic_score = mppi_candidates[i].dynamic_scene_score;
	                                    safe_idx = static_cast<int>(i);
	                                }
	                                continue;
	                            }
	                            if (mppi_candidates[i].dynamic_scene_score < safe_dynamic_score) {
	                                safe_dynamic_score = mppi_candidates[i].dynamic_scene_score;
	                                safe_idx = static_cast<int>(i);
	                            }
	                        }
	                        if (safe_idx < 0) {
	                            safe_idx = dynamic_better_idx;
	                            safe_dynamic_score = dynamic_better_score;
	                        }
	                        if (safe_idx >= 0) {
	                            ROS_INFO("[PlannerManager] Dynamic safety gate: replacing Path #%d (dyn_score=%.3f, clear=%.2fm, dyn=%.2fm) with Path #%d (dyn_score=%.3f, clear=%.2fm, dyn=%.2fm)",
                                     best_dynamic_idx + 1, mppi_candidates[best_dynamic_idx].dynamic_scene_score,
                                     mppi_candidates[best_dynamic_idx].min_clearance,
                                     mppi_candidates[best_dynamic_idx].min_dynamic_dist,
                                     safe_idx + 1, mppi_candidates[safe_idx].dynamic_scene_score,
                                     mppi_candidates[safe_idx].min_clearance,
                                     mppi_candidates[safe_idx].min_dynamic_dist);
                            best_idx = safe_idx;
                            best_score = safe_dynamic_score;
                        }
                    }

                } else {
                    for (size_t i = 0; i < mppi_candidates.size(); ++i) {
                        const double candidate_score = mppi_candidates[i].selection_score;
                        if (mppi_candidates[i].success && candidate_score < best_score) {
                            best_score = candidate_score;
                            best_idx = i;
                        }
                    }

                    const double static_hard_clearance =
                        std::max(final_static_min_clearance_, static_scene_unsafe_clearance_);
                    const double static_preferred_clearance =
                        std::max(static_scene_preferred_clearance_, static_hard_clearance);
                    if (static_scene_safety_gate_enabled_ &&
                        best_idx >= 0 &&
                        mppi_candidates[best_idx].min_clearance < static_hard_clearance) {
                        int safe_idx = -1;
                        double safe_score = std::numeric_limits<double>::max();
                        const double allowed_score =
                            best_score + std::max(0.0, static_scene_safe_score_slack_);
                        for (size_t i = 0; i < mppi_candidates.size(); ++i) {
                            if (!mppi_candidates[i].success) continue;
                            if (mppi_candidates[i].min_clearance < static_preferred_clearance) continue;
                            if (mppi_candidates[i].selection_score > allowed_score) continue;
                            if (mppi_candidates[i].progress_ratio + 0.10 <
                                mppi_candidates[best_idx].progress_ratio) continue;
                            if (mppi_candidates[i].final_goal_dist >
                                mppi_candidates[best_idx].final_goal_dist + 0.75) continue;
                            if (mppi_candidates[i].direction_cos + 0.15 <
                                mppi_candidates[best_idx].direction_cos) continue;
                            if (mppi_candidates[i].selection_score < safe_score) {
                                safe_score = mppi_candidates[i].selection_score;
                                safe_idx = static_cast<int>(i);
                            }
                        }
                        if (safe_idx >= 0) {
                            ROS_INFO("[PlannerManager] Static hard safety gate: replacing Path #%d (score=%.3f, clear=%.2fm) with Path #%d (score=%.3f, clear=%.2fm)",
                                     best_idx + 1, mppi_candidates[best_idx].selection_score,
                                     mppi_candidates[best_idx].min_clearance,
                                     safe_idx + 1, mppi_candidates[safe_idx].selection_score,
                                     mppi_candidates[safe_idx].min_clearance);
                            best_idx = safe_idx;
                            best_score = safe_score;
                        } else if (static_scene_best_effort_gate_enabled_) {
                            int improved_idx = -1;
                            double improved_clearance = mppi_candidates[best_idx].min_clearance;
                            double improved_score = std::numeric_limits<double>::max();
                            const double min_required_clearance =
                                mppi_candidates[best_idx].min_clearance +
                                std::max(0.0, static_scene_min_clearance_improvement_);

                            for (size_t i = 0; i < mppi_candidates.size(); ++i) {
                                if (!mppi_candidates[i].success) continue;
                                if (mppi_candidates[i].min_clearance < min_required_clearance) continue;
                                if (mppi_candidates[i].selection_score > allowed_score) continue;
                                if (mppi_candidates[i].progress_ratio + 0.08 <
                                    mppi_candidates[best_idx].progress_ratio) continue;
                                if (mppi_candidates[i].final_goal_dist >
                                    mppi_candidates[best_idx].final_goal_dist + 0.90) continue;
                                if (mppi_candidates[i].direction_cos + 0.12 <
                                    mppi_candidates[best_idx].direction_cos) continue;

                                const bool more_clear =
                                    mppi_candidates[i].min_clearance > improved_clearance + 1e-3;
                                const bool same_clear_better_score =
                                    std::abs(mppi_candidates[i].min_clearance - improved_clearance) <= 1e-3 &&
                                    mppi_candidates[i].selection_score < improved_score;
                                if (more_clear || same_clear_better_score) {
                                    improved_clearance = mppi_candidates[i].min_clearance;
                                    improved_score = mppi_candidates[i].selection_score;
                                    improved_idx = static_cast<int>(i);
                                }
                            }

	                            if (improved_idx >= 0) {
	                                ROS_INFO("[PlannerManager] Static hard best-effort gate: replacing Path #%d (score=%.3f, clear=%.2fm) with Path #%d (score=%.3f, clear=%.2fm)",
	                                         best_idx + 1, mppi_candidates[best_idx].selection_score,
	                                         mppi_candidates[best_idx].min_clearance,
	                                         improved_idx + 1, mppi_candidates[improved_idx].selection_score,
	                                         mppi_candidates[improved_idx].min_clearance);
	                                best_idx = improved_idx;
	                                best_score = improved_score;
	                            }
	                        }
	                    }

	                    if (static_scene_safety_gate_enabled_ &&
	                        static_scene_preferred_clearance_gate_enabled_ &&
	                        best_idx >= 0 &&
	                        mppi_candidates[best_idx].min_clearance >= static_hard_clearance &&
	                        mppi_candidates[best_idx].min_clearance < static_preferred_clearance) {
	                        int preferred_idx = -1;
	                        double preferred_score = std::numeric_limits<double>::max();
	                        const auto& best_candidate = mppi_candidates[best_idx];
	                        const double allowed_score =
	                            best_candidate.selection_score +
	                            std::max(static_scene_safe_score_slack_,
	                                     static_scene_preferred_clearance_score_slack_);
	                        const double min_required_clearance =
	                            std::min(static_preferred_clearance,
	                                     best_candidate.min_clearance +
	                                         std::max(0.0, static_scene_min_clearance_improvement_));

	                        for (size_t i = 0; i < mppi_candidates.size(); ++i) {
	                            const auto& candidate = mppi_candidates[i];
	                            if (!candidate.success) continue;
	                            if (candidate.min_clearance < min_required_clearance) continue;
	                            if (candidate.min_clearance < static_hard_clearance) continue;
	                            if (candidate.selection_score > allowed_score) continue;
	                            if (candidate.progress_ratio + 0.08 < best_candidate.progress_ratio) continue;
	                            if (candidate.early_progress_ratio + 0.08 <
	                                best_candidate.early_progress_ratio) continue;
	                            if (candidate.final_goal_dist > best_candidate.final_goal_dist + 0.70) continue;
	                            if (candidate.direction_cos + 0.10 < best_candidate.direction_cos) continue;
	                            if (candidate.max_goal_overshoot > best_candidate.max_goal_overshoot + 0.50) continue;
	                            if (candidate.path_length > best_candidate.path_length + 4.0) continue;

	                            const double clearance_reward =
	                                80.0 * std::min(static_preferred_clearance,
	                                                candidate.min_clearance);
	                            const double guarded_score =
	                                candidate.selection_score - clearance_reward;
	                            if (guarded_score < preferred_score) {
	                                preferred_score = guarded_score;
	                                preferred_idx = static_cast<int>(i);
	                            }
	                        }

	                        if (preferred_idx >= 0 && preferred_idx != best_idx) {
	                            ROS_INFO("[PlannerManager] Static preferred clearance gate: replacing Path #%d (score=%.3f, clear=%.2fm, progress=%.2f, goal=%.2fm) with Path #%d (score=%.3f, clear=%.2fm, progress=%.2f, goal=%.2fm)",
	                                     best_idx + 1, best_candidate.selection_score,
	                                     best_candidate.min_clearance,
	                                     best_candidate.progress_ratio,
	                                     best_candidate.final_goal_dist,
	                                     preferred_idx + 1,
	                                     mppi_candidates[preferred_idx].selection_score,
	                                     mppi_candidates[preferred_idx].min_clearance,
	                                     mppi_candidates[preferred_idx].progress_ratio,
	                                     mppi_candidates[preferred_idx].final_goal_dist);
	                            best_idx = preferred_idx;
	                            best_score = mppi_candidates[preferred_idx].selection_score;
	                        }
	                    }

	                    if (static_scene_safety_gate_enabled_ &&
	                        best_idx >= 0 &&
	                        mppi_candidates[best_idx].min_clearance < static_hard_clearance &&
	                        (continous_failures_count_ >= 2 ||
	                         bspline_consecutive_failures_ >= 1)) {
	                        int topo_safe_idx = -1;
	                        double topo_safe_score = std::numeric_limits<double>::max();
	                        const double best_progress = mppi_candidates[best_idx].progress_ratio;
	                        const double best_goal_dist = mppi_candidates[best_idx].final_goal_dist;
	                        for (size_t i = 0; i < mppi_candidates.size(); ++i) {
	                            if (!mppi_candidates[i].success) continue;
	                            if (mppi_candidates[i].topo_path.path.size() < 2) continue;
	                            if (mppi_candidates[i].topo_min_clearance < static_hard_clearance) continue;
	                            if (mppi_candidates[i].progress_ratio + 0.20 < best_progress) continue;
	                            if (mppi_candidates[i].final_goal_dist > best_goal_dist + 1.20) continue;

	                            const double topo_score =
	                                mppi_candidates[i].selection_score -
	                                120.0 * std::min(1.0, mppi_candidates[i].topo_min_clearance);
	                            if (topo_score < topo_safe_score) {
	                                topo_safe_score = topo_score;
	                                topo_safe_idx = static_cast<int>(i);
	                            }
	                        }
	                        if (topo_safe_idx >= 0 && topo_safe_idx != best_idx) {
	                            ROS_WARN("[PlannerManager] Static MPPI candidates unsafe; switching executable seed to topo-safe Path #%d (mppi_clear=%.2fm, topo_clear=%.2fm)",
	                                     topo_safe_idx + 1,
	                                     mppi_candidates[topo_safe_idx].min_clearance,
	                                     mppi_candidates[topo_safe_idx].topo_min_clearance);
	                            best_idx = topo_safe_idx;
	                            best_score = mppi_candidates[topo_safe_idx].selection_score;
	                        }
	                    }
	                }

                if (candidate_quality_log_enabled_ && !candidate_quality_log_path_.empty()) {
                    const bool file_exists = [] (const std::string& path) {
                        struct stat buffer;
                        return stat(path.c_str(), &buffer) == 0 && buffer.st_size > 0;
                    }(candidate_quality_log_path_);
                    std::ofstream candidate_log(candidate_quality_log_path_, std::ios::app);
                    if (!candidate_log.is_open()) {
                        ROS_WARN_THROTTLE(2.0,
                                          "[PlannerManager] Cannot open candidate quality log: %s",
                                          candidate_quality_log_path_.c_str());
                    } else {
                        if (!file_exists) {
                            candidate_log
                                << "stamp,cycle,dynamic_scene,candidate_idx,selected,success,"
                                << "reject_reason,raw_cost,normalized_cost,selection_score,"
                                << "dynamic_scene_score,best_score,path_length,topo_length,"
                                << "final_goal_dist,progress_ratio,early_progress_ratio,"
                                << "direction_cos,max_goal_overshoot,min_static_clearance,"
                                << "min_dynamic_clearance,topo_min_static_clearance,"
                                << "topo_min_dynamic_clearance,topo_cost,waypoints\n";
                        }

                        for (size_t i = 0; i < mppi_candidates.size(); ++i) {
                            const auto& candidate = mppi_candidates[i];
                            const bool selected = (best_idx >= 0 && i == static_cast<size_t>(best_idx));
                            std::string reject_reason = "not_selected";
                            if (!candidate.success) {
                                reject_reason = "mppi_failed_or_z_rejected";
                            } else if (selected) {
                                reject_reason = dynamic_scene_active ? "selected_dynamic_score_or_gate"
                                                                     : "selected_selection_score_or_gate";
                            } else if (best_idx < 0) {
                                reject_reason = "no_valid_best_candidate";
                            } else if (dynamic_scene_active) {
                                reject_reason = "not_selected_dynamic_score_or_gate";
                            } else {
                                reject_reason = "not_selected_selection_score_or_gate";
                            }

                            candidate_log << std::fixed << std::setprecision(6)
                                          << ros::Time::now().toSec() << ','
                                          << timing.cycle << ','
                                          << (dynamic_scene_active ? 1 : 0) << ','
                                          << i << ','
                                          << (selected ? 1 : 0) << ','
                                          << (candidate.success ? 1 : 0) << ','
                                          << reject_reason << ','
                                          << candidate.mppi_result.cost << ','
                                          << candidate.normalized_cost << ','
                                          << candidate.selection_score << ','
                                          << candidate.dynamic_scene_score << ','
                                          << best_score << ','
                                          << candidate.path_length << ','
                                          << candidate.topo_length << ','
                                          << candidate.final_goal_dist << ','
                                          << candidate.progress_ratio << ','
                                          << candidate.early_progress_ratio << ','
                                          << candidate.direction_cos << ','
                                          << candidate.max_goal_overshoot << ','
                                          << candidate.min_clearance << ','
                                          << candidate.min_dynamic_dist << ','
                                          << candidate.topo_min_clearance << ','
                                          << candidate.topo_min_dynamic_dist << ','
                                          << candidate.topo_path.cost << ','
                                          << candidate.mppi_result.positions.size() << '\n';
                        }
                    }
                }
                
                //  NEW: Store all MPPI-optimized paths for visualization
                all_mppi_paths_.clear();
                for (size_t i = 0; i < mppi_candidates.size(); ++i) {
                    MPPIPathCandidate path_vis;
                    path_vis.positions = mppi_candidates[i].mppi_result.positions;
                    path_vis.cost = mppi_candidates[i].mppi_result.cost;
                    path_vis.normalized_cost = dynamic_scene_active ? mppi_candidates[i].dynamic_scene_score
                                                                    : mppi_candidates[i].selection_score;
                    path_vis.is_best = (i == static_cast<size_t>(best_idx));  // 修复类型比较警告
                    path_vis.success = mppi_candidates[i].success;
                    all_mppi_paths_.push_back(path_vis);
                }
                ROS_INFO("[PlannerManager]    Stored %zu MPPI-optimized paths for visualization", all_mppi_paths_.size());
                
                //  Publish all MPPI paths for RViz visualization
                visualizeAllMPPIPaths();
                
                if (best_idx >= 0) {
                    topo_paths_for_success_cache.clear();
                    topo_paths_for_success_cache.push_back(mppi_candidates[best_idx].topo_path);
                    ROS_INFO("[PlannerManager]   ");
                    ROS_INFO("[PlannerManager]    Best MPPI: Path #%d with %s=%.3f (norm_cost=%.3f, mode_score=%.3f, dyn_score=%.3f, final_goal=%.2fm, progress=%.2f, clear=%.2fm, dyn=%.2fm)", 
                             best_idx+1, dynamic_scene_active ? "dynamic_score" : "selection_score",
                             best_score, mppi_candidates[best_idx].normalized_cost,
                             mppi_candidates[best_idx].selection_score,
                             mppi_candidates[best_idx].dynamic_scene_score,
                             mppi_candidates[best_idx].final_goal_dist, mppi_candidates[best_idx].progress_ratio,
                             mppi_candidates[best_idx].min_clearance, mppi_candidates[best_idx].min_dynamic_dist);
                    ROS_INFO("[PlannerManager]   ");

                    // MPPI may cut through an obstacle even when its topo guide is safe.
                    // In static scenes, keep the safe topology as the executable seed if
                    // all MPPI modes violate the final clearance gate.
                    const bool mppi_static_unsafe =
                        !dynamic_scene_active &&
                        mppi_candidates[best_idx].min_clearance <
                            std::max(final_static_min_clearance_, static_scene_unsafe_clearance_);
                    const bool topo_seed_static_safe =
                        mppi_candidates[best_idx].topo_min_clearance >=
                            std::max(final_static_min_clearance_, static_scene_unsafe_clearance_);
                    if (mppi_static_unsafe &&
                        topo_seed_static_safe &&
                        mppi_candidates[best_idx].topo_path.path.size() >= 2) {
                        point_set = mppi_candidates[best_idx].topo_path.path;
                        std::vector<Eigen::Vector3d> dense_topo;
                        dense_topo.reserve(point_set.size() * 4);
                        for (size_t j = 0; j + 1 < point_set.size(); ++j) {
                            dense_topo.push_back(point_set[j]);
                            const Eigen::Vector3d seg_vec = point_set[j + 1] - point_set[j];
                            const double seg_len = seg_vec.norm();
                            const int num_inter =
                                std::max(1, static_cast<int>(std::ceil(seg_len / pp_.ctrl_pt_dist)));
                            for (int k = 1; k < num_inter; ++k) {
                                const double a = static_cast<double>(k) / static_cast<double>(num_inter);
                                dense_topo.push_back(point_set[j] * (1.0 - a) + point_set[j + 1] * a);
                            }
                        }
                        dense_topo.push_back(point_set.back());
                        point_set.swap(dense_topo);

	                        mppi_result_backup_.positions = point_set;
	                        mppi_result_backup_.cost = mppi_candidates[best_idx].selection_score;
	                        mppi_result_backup_.velocities.assign(point_set.size(), Eigen::Vector3d::Zero());
	                        mppi_result_backup_.accelerations.assign(point_set.size(), Eigen::Vector3d::Zero());
	                        using_static_topo_seed = true;
                            timing.used_static_topo_seed = true;
                        for (size_t j = 1; j < point_set.size(); ++j) {
                            mppi_result_backup_.velocities[j] =
                                (point_set[j] - point_set[j - 1]) / std::max(1e-3, ts);
                        }

                        ROS_WARN("[PlannerManager]    Best MPPI is static-unsafe (clear=%.2fm); using its topo guide seed with %zu points",
                                 mppi_candidates[best_idx].min_clearance, point_set.size());
                    } else if (mppi_static_unsafe && !topo_seed_static_safe) {
                        ROS_WARN("[PlannerManager]    Best MPPI and topo guide are both static-unsafe (mppi_clear=%.2fm, topo_clear=%.2fm); keeping MPPI for final validator/fallback",
                                 mppi_candidates[best_idx].min_clearance,
                                 mppi_candidates[best_idx].topo_min_clearance);
                        mppi_result_backup_ = mppi_candidates[best_idx].mppi_result;
                        point_set = mppi_result_backup_.positions;
                    } else {
                        //  IMPROVED: Backup MPPI result for potential B-spline fallback
                        mppi_result_backup_ = mppi_candidates[best_idx].mppi_result;
                        point_set = mppi_result_backup_.positions;
                    }

                    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
                    use_mppi_topo_path = true;
                    
                    ROS_INFO("[PlannerManager]   Using %s result with %zu points",
                             mppi_static_unsafe ? "topology-safe" : "MPPI", point_set.size());
	                } else {
	                    ROS_WARN("[PlannerManager]    All MPPI failed, fallback to best topo path");
	                    best_path = topo_planner_->selectBestPath(topo_paths);
	                    topo_paths_for_success_cache.clear();
	                    topo_paths_for_success_cache.push_back(best_path);
	                }
            } else {
                // Original single-path selection
                best_path = topo_planner_->selectBestPath(topo_paths);
                topo_paths_for_success_cache.clear();
                topo_paths_for_success_cache.push_back(best_path);
                if (ablation_disable_mppi_optimization_) {
                    ROS_WARN("[PlannerManager] ABLATION: MPPI optimization disabled, using topological path directly");
                }
                ROS_INFO("[PlannerManager] Using topological path with cost %.3f", best_path.cost);
            }
            
            // If not using parallel MPPI or it failed, use traditional approach
            if (!use_mppi_topo_path && best_path.path.size() >= 2) {
                // Replace control points with topological path
                point_set = best_path.path;
                
                // Ensure sufficient points for B-spline generation
                if (point_set.size() < 7) {
                    //  OPTIMIZED: 稀疏插值，保持Topo路径的引导特性
                    // 旧策略: num = segment_len / (0.4 * 0.5) → 5米段插25个点 → waypoints=55-133
                    // 新策略: num = segment_len / (0.4 * 4.0) → 5米段插3个点 → waypoints=10以内
                    // 目标: Topo路径保持稀疏(3-10个转折点)，MPPI优化局部细节
                    std::vector<Eigen::Vector3d> dense_path;
                    for (size_t i = 0; i < point_set.size() - 1; ++i) {
                        dense_path.push_back(point_set[i]);
                        Eigen::Vector3d segment_vec = point_set[i+1] - point_set[i];
                        double segment_len = segment_vec.norm();
                        
                        // 优化: 4倍稀疏 (0.5 → 4.0)
                        int num_intermediate = std::max(0, (int)(segment_len / (pp_.ctrl_pt_dist * 4.0)));
                        
                        for (int j = 1; j < num_intermediate; ++j) {
                            double t = (double)j / num_intermediate;
                            dense_path.push_back(point_set[i] + t * segment_vec);
                        }
                    }
                    dense_path.push_back(point_set.back());
                    point_set = dense_path;
                    
                    ROS_INFO("[PlannerManager]  Sparse interpolation: %zu → %zu waypoints", 
                             best_path.path.size(), point_set.size());
                }
                
                // Re-parameterize control points using topological path
                UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
                use_mppi_topo_path = true;
                
                ROS_INFO("[PlannerManager] Successfully integrated topological path with %zu waypoints", point_set.size());
            } else if (!use_mppi_topo_path) {
                ROS_WARN("[PlannerManager] Selected topological path has insufficient waypoints (%zu), using original approach", best_path.path.size());
            }
        } else {
            ROS_WARN("[PlannerManager] No valid topological paths found, trying standalone MPPI...");
            // Same standalone MPPI fallback as when planWithTopo fails
            if (!ablation_disable_mppi_optimization_ && mppi_planner_ != nullptr) {
                feedDynamicObstaclesToMPPI();
                std::vector<Eigen::Vector3d> straight_path;
                int num_pts = 30;
                for (int i = 0; i <= num_pts; ++i) {
                    double t_frac = (double)i / num_pts;
                    straight_path.push_back(start_pt + t_frac * (local_target_pt - start_pt));
                }
                MPPITrajectory mppi_result;
                const ros::Time standalone_mppi_start = ros::Time::now();
                bool mppi_success = mppi_planner_->planTrajectory(
                    start_pt, start_vel, local_target_pt, local_target_vel,
                    straight_path, mppi_result);
                timing.mppi_ms += (ros::Time::now() - standalone_mppi_start).toSec() * 1000.0;
                timing.mppi_candidates += 1;
                double min_z_path = 0.0, max_z_path = 0.0;
                bool z_safe = mppi_success && isPathWithinZBounds(mppi_result.positions, &min_z_path, &max_z_path);
                if (mppi_success && mppi_result.positions.size() >= 10 && z_safe) {
                    timing.mppi_successes += 1;
                    ROS_INFO("[PlannerManager] Standalone MPPI (empty topo) succeeded: %zu points",
                             mppi_result.positions.size());
                    mppi_result_backup_ = mppi_result;
                    point_set = mppi_result.positions;
                    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
                    use_mppi_topo_path = true;
                } else if (mppi_success && !z_safe) {
                    ROS_WARN("[PlannerManager] Standalone MPPI (empty topo) rejected by z safety (z=[%.2f, %.2f], allowed=[%.2f, %.2f])",
                             min_z_path, max_z_path, min_z_, max_z_);
                }
            }
        }
    } else {
        if (ablation_disable_topo_guidance_) {
            if (!ablation_disable_mppi_optimization_ && mppi_planner_ != nullptr) {
                feedDynamicObstaclesToMPPI();

                std::vector<Eigen::Vector3d> straight_path;
                int num_pts = 30;
                for (int i = 0; i <= num_pts; ++i) {
                    double t = (double)i / num_pts;
                    straight_path.push_back(start_pt + t * (local_target_pt - start_pt));
                }

                MPPITrajectory mppi_result;
                const ros::Time ablation_mppi_start = ros::Time::now();
                bool mppi_success = mppi_planner_->planTrajectory(
                    start_pt, start_vel, local_target_pt, local_target_vel,
                    straight_path, mppi_result);
                timing.mppi_ms += (ros::Time::now() - ablation_mppi_start).toSec() * 1000.0;
                timing.mppi_candidates += 1;

                double min_z_path = 0.0, max_z_path = 0.0;
                bool z_safe = mppi_success && isPathWithinZBounds(mppi_result.positions, &min_z_path, &max_z_path);
                if (mppi_success && mppi_result.positions.size() >= 10 && z_safe) {
                    timing.mppi_successes += 1;
                    ROS_INFO("[PlannerManager] ABLATION MPPI-only succeeded: %zu points, cost=%.3f",
                             mppi_result.positions.size(), mppi_result.cost);
                    mppi_result_backup_ = mppi_result;
                    point_set = mppi_result.positions;
                    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
                    use_mppi_topo_path = true;
                } else {
                    ROS_WARN("[PlannerManager] ABLATION MPPI-only failed or rejected (success=%d, z_safe=%d, points=%zu)",
                             mppi_success, z_safe, mppi_result.positions.size());
                }
            } else {
                ROS_WARN("[PlannerManager] ABLATION disables both topo and MPPI; using original polynomial seed");
            }
        } else if (topo_planner_ == nullptr) {
            ROS_WARN("[PlannerManager] Topological planner not initialized, using original approach");
        } else {
            ROS_WARN("[PlannerManager] Topological planning failed, trying standalone MPPI...");
            
            // FALLBACK: When TopoPRM fails, run MPPI without topological guidance
            // MPPI can still find good trajectories via sampling-based optimization
            if (!ablation_disable_mppi_optimization_ && mppi_planner_ != nullptr) {
                feedDynamicObstaclesToMPPI();
                
                // Use straight line as initial path for MPPI
                std::vector<Eigen::Vector3d> straight_path;
                int num_pts = 30;
                for (int i = 0; i <= num_pts; ++i) {
                    double t = (double)i / num_pts;
                    straight_path.push_back(start_pt + t * (local_target_pt - start_pt));
                }
                
                MPPITrajectory mppi_result;
                const ros::Time fallback_mppi_start = ros::Time::now();
                bool mppi_success = mppi_planner_->planTrajectory(
                    start_pt, start_vel, local_target_pt, local_target_vel,
                    straight_path, mppi_result);
                timing.mppi_ms += (ros::Time::now() - fallback_mppi_start).toSec() * 1000.0;
                timing.mppi_candidates += 1;
                
                double min_z_path = 0.0, max_z_path = 0.0;
                bool z_safe = mppi_success && isPathWithinZBounds(mppi_result.positions, &min_z_path, &max_z_path);
                if (mppi_success && mppi_result.positions.size() >= 10 && z_safe) {
                    timing.mppi_successes += 1;
                    ROS_INFO("[PlannerManager] Standalone MPPI succeeded: %zu points, cost=%.3f",
                             mppi_result.positions.size(), mppi_result.cost);
                    
                    mppi_result_backup_ = mppi_result;
                    point_set = mppi_result.positions;
                    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
                    use_mppi_topo_path = true;
                } else if (mppi_success && !z_safe) {
                    ROS_WARN("[PlannerManager] Standalone MPPI rejected by z safety (z=[%.2f, %.2f], allowed=[%.2f, %.2f])",
                             min_z_path, max_z_path, min_z_, max_z_);
                } else {
                    ROS_WARN("[PlannerManager] Standalone MPPI also failed, using polynomial fallback");
                }
            }
        }
    }

    t_start = ros::Time::now();

    /*** STEP 2: MPPI DYNAMIC OPTIMIZATION ***/
    //  Apply MPPI optimization to topological path (if available)
    // MPPI considers dynamics, ESDF, and control smoothness
    
    // Note: If parallel MPPI was used in STEP 1.5, this step is skipped
    // as MPPI optimization was already applied to topology-ranked candidate paths
    if (!ablation_disable_mppi_optimization_ && use_mppi_topo_path && !use_multi_topo_mppi && mppi_planner_ != nullptr) {
        ROS_INFO("[PlannerManager] Applying MPPI dynamic optimization with ESDF...");
        
        Eigen::Vector3d current_vel = start_vel;
        Eigen::Vector3d target_vel = local_target_vel;
        
        MPPITrajectory mppi_result;
        const ros::Time step2_mppi_start = ros::Time::now();
        bool mppi_success = planWithMPPI(start_pt, current_vel, local_target_pt, target_vel, mppi_result);
        timing.mppi_ms += (ros::Time::now() - step2_mppi_start).toSec() * 1000.0;
        timing.mppi_candidates += 1;
        double min_z_path = 0.0, max_z_path = 0.0;
        bool z_safe = mppi_success && isPathWithinZBounds(mppi_result.positions, &min_z_path, &max_z_path);
        
        if (mppi_success && mppi_result.positions.size() >= 7 && z_safe) {
            timing.mppi_successes += 1;
            ROS_INFO("[PlannerManager] MPPI optimization succeeded with %zu points, using as control points", 
                     mppi_result.positions.size());
            
            //  IMPROVED: Backup MPPI result for potential B-spline fallback
            mppi_result_backup_ = mppi_result;
            
            // Replace control points with MPPI-optimized trajectory
            point_set = mppi_result.positions;
            
            // Re-parameterize to B-spline with MPPI result
            UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
            
            ROS_INFO("[PlannerManager] MPPI control points integrated successfully");
        } else if (mppi_success && !z_safe) {
            ROS_WARN("[PlannerManager] MPPI optimization rejected by z safety (z=[%.2f, %.2f], allowed=[%.2f, %.2f]), using topo path",
                     min_z_path, max_z_path, min_z_, max_z_);
        } else {
            ROS_WARN("[PlannerManager] MPPI optimization failed or insufficient points (%zu), using topo path", 
                     mppi_result.positions.size());
            // Fallback: keep original topological path control points
        }
        
        ros::Duration mppi_time = ros::Time::now() - t_start;
        ROS_INFO("[PlannerManager] MPPI optimization took %.3f ms", mppi_time.toSec() * 1000.0);
	    } else if (use_multi_topo_mppi) {
	        ROS_INFO("[PlannerManager] Skipping STEP 2: multi-topology MPPI already applied in STEP 1.5");
	    }

	    bool planar_lock_dynamic_scene_active = false;
	    {
	      std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
	      planar_lock_dynamic_scene_active =
	          has_dynamic_obstacles_ && !latest_dynamic_obstacles_.obstacles.empty();
	    }
	    if (planar_flight_z_lock_ &&
	        !planar_lock_dynamic_scene_active &&
	        use_mppi_topo_path &&
	        point_set.size() >= 2 &&
	        start_pt.z() >= publish_min_z_ &&
	        local_target_pt.z() >= publish_min_z_) {
	      const double z_ref =
	          std::min(max_z_ - 0.05,
	                   std::max(publish_min_z_, 0.5 * (start_pt.z() + local_target_pt.z())));
	      double min_before = std::numeric_limits<double>::infinity();
	      double max_before = -std::numeric_limits<double>::infinity();
	      for (auto &pt : point_set) {
	        min_before = std::min(min_before, pt.z());
	        max_before = std::max(max_before, pt.z());
	        pt.z() = z_ref;
	      }
	      if (!mppi_result_backup_.positions.empty()) {
	        for (auto &pt : mppi_result_backup_.positions) {
	          pt.z() = z_ref;
	        }
	        for (auto &vel : mppi_result_backup_.velocities) {
	          vel.z() = 0.0;
	        }
	        for (auto &acc : mppi_result_backup_.accelerations) {
	          acc.z() = 0.0;
	        }
	      }
	      UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
	      ROS_INFO("[PlannerManager] Planar z lock applied before B-spline: z %.2f..%.2f -> %.2f (%zu points)",
	               min_before, max_before, z_ref, point_set.size());
	    }

	    t_start = ros::Time::now();

	    /*** STEP 3: B-SPLINE SMOOTHING ***/
    ROS_INFO("[PlannerManager] STEP 3: B-spline smoothing (ONLY smoothing, NOT planning!)");
    ROS_INFO("[PlannerManager]   Input: %d control points from TOPO/MPPI", (int)ctrl_pts.cols());
    
    bool flag_step_1_success = false;
    bool used_mppi_fallback = false;  // Track if we used MPPI fallback
    bool preserve_safe_mppi_bspline = false;
    bool skipped_rebound_precheck = false;
	    bool step3_dynamic_scene_active = false;
	    {
	      std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
	      step3_dynamic_scene_active =
	          has_dynamic_obstacles_ && !latest_dynamic_obstacles_.obstacles.empty();
	    }
    const double step3_static_check_fraction =
        step3_dynamic_scene_active ? final_dynamic_static_check_fraction_ : 2.0 / 3.0;
    const double step3_static_check_time =
        step3_dynamic_scene_active ? final_dynamic_static_check_time_ : std::numeric_limits<double>::infinity();

    if (dynamic_scene_preserve_safe_seed_ &&
        step3_dynamic_scene_active &&
        use_mppi_topo_path &&
        mppi_result_backup_.positions.size() >= 4) {
      UniformBspline direct_seed_traj;
      double direct_seed_clearance = 0.0;
      double direct_seed_dynamic_dist = 0.0;
      if (buildSafeSeedBspline(mppi_result_backup_.positions, start_end_derivatives, ts,
                               direct_seed_traj,
                               step3_static_check_fraction, step3_static_check_time, 0.25,
                               dynamic_escape_start_skip_time_,
                               &direct_seed_clearance, &direct_seed_dynamic_dist)) {
        ctrl_pts = direct_seed_traj.getControlPoint();
        ts = direct_seed_traj.getInterval();
        flag_step_1_success = true;
        used_mppi_fallback = true;
        bspline_consecutive_failures_ = 0;
        ROS_INFO("[PlannerManager]   Dynamic scene: preserving verified MPPI seed B-spline before rebound (clear=%.2fm, dyn=%.2fm)",
                 direct_seed_clearance, direct_seed_dynamic_dist);
      }
    }

    if (!flag_step_1_success &&
        static_scene_preserve_safe_seed_ &&
        !step3_dynamic_scene_active &&
        use_mppi_topo_path &&
        mppi_result_backup_.positions.size() >= 4) {
      UniformBspline direct_seed_traj;
      double direct_seed_clearance = 0.0;
      double direct_seed_dynamic_dist = 0.0;
      if (buildSafeSeedBspline(mppi_result_backup_.positions, start_end_derivatives, ts,
                               direct_seed_traj,
                               step3_static_check_fraction, step3_static_check_time, 0.25,
                               0.0,
                               &direct_seed_clearance, &direct_seed_dynamic_dist)) {
        if (direct_seed_clearance >= static_scene_preserve_min_clearance_) {
          ctrl_pts = direct_seed_traj.getControlPoint();
          ts = direct_seed_traj.getInterval();
          flag_step_1_success = true;
          used_mppi_fallback = true;
          bspline_consecutive_failures_ = 0;
          ROS_INFO("[PlannerManager]   Static scene: preserving verified MPPI/topo seed B-spline before rebound (clear=%.2fm)",
                   direct_seed_clearance);
        } else {
          ROS_INFO("[PlannerManager]   Static seed preserve skipped: clearance %.2fm < %.2fm",
                   direct_seed_clearance, static_scene_preserve_min_clearance_);
        }
      } else {
        std::vector<Eigen::Vector3d> repaired_seed;
        if (repairSeedPathByClearance(mppi_result_backup_.positions, repaired_seed) &&
            buildSafeSeedBspline(repaired_seed, start_end_derivatives, ts,
                                 direct_seed_traj,
                                 step3_static_check_fraction, step3_static_check_time, 0.25,
                                 0.0,
                                 &direct_seed_clearance, &direct_seed_dynamic_dist) &&
            direct_seed_clearance >= static_scene_preserve_min_clearance_) {
          ctrl_pts = direct_seed_traj.getControlPoint();
          ts = direct_seed_traj.getInterval();
          flag_step_1_success = true;
          used_mppi_fallback = true;
          bspline_consecutive_failures_ = 0;
          ROS_INFO("[PlannerManager]   Static scene: preserving repaired MPPI/topo seed B-spline before rebound (clear=%.2fm)",
                   direct_seed_clearance);
        }
      }
    }

    if (!flag_step_1_success && use_mppi_topo_path && mppi_result_backup_.positions.size() >= 10) {
      UniformBspline mppi_seed(ctrl_pts, 3, ts);
      mppi_seed.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

      double seed_min_z = 0.0, seed_max_z = 0.0, seed_min_clearance = 0.0;
      bool seed_z_safe = isBsplineWithinZBounds(mppi_seed, &seed_min_z, &seed_max_z);
      bool seed_collision_free = isBsplineCollisionFree(mppi_seed, mppi_seed_preserve_clearance_, &seed_min_clearance);
      double feasibility_ratio = 1.0;
      bool seed_dynamic_feasible = mppi_seed.checkFeasibility(feasibility_ratio, false);

      if (seed_z_safe && seed_collision_free && seed_dynamic_feasible) {
        preserve_safe_mppi_bspline = true;
        ROS_INFO("[PlannerManager]   MPPI seed B-spline is already safe (clearance=%.2fm, z=[%.2f, %.2f]); skipping rebound",
                 seed_min_clearance, seed_min_z, seed_max_z);
	      } else if (seed_z_safe && seed_collision_free &&
	                 std::isfinite(feasibility_ratio) &&
	                 feasibility_ratio > 1.0 &&
	                 feasibility_ratio <= (using_static_topo_seed
	                                           ? std::max(safe_seed_time_stretch_max_, 7.5)
	                                           : safe_seed_time_stretch_max_)) {
	        const double seed_stretch_max =
	            using_static_topo_seed ? std::max(safe_seed_time_stretch_max_, 7.5)
	                                   : safe_seed_time_stretch_max_;
	        const double stretch_ratio =
	            std::min(seed_stretch_max, feasibility_ratio * 1.08);
        const double stretched_ts = ts * stretch_ratio;
        Eigen::MatrixXd stretched_ctrl_pts;
        UniformBspline::parameterizeToBspline(stretched_ts,
                                              mppi_result_backup_.positions,
                                              start_end_derivatives,
                                              stretched_ctrl_pts);
        UniformBspline stretched_seed(stretched_ctrl_pts, 3, stretched_ts);
        stretched_seed.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double stretched_min_z = 0.0;
        double stretched_max_z = 0.0;
        double stretched_min_clearance = 0.0;
        double stretched_ratio = 1.0;
        const bool stretched_z_safe =
            isBsplineWithinZBounds(stretched_seed, &stretched_min_z, &stretched_max_z);
        const bool stretched_collision_free =
            isBsplineCollisionFree(stretched_seed, mppi_seed_preserve_clearance_,
                                   &stretched_min_clearance);
        const bool stretched_dynamic_feasible =
            stretched_seed.checkFeasibility(stretched_ratio, false);

        if (stretched_z_safe && stretched_collision_free && stretched_dynamic_feasible) {
          ctrl_pts = stretched_ctrl_pts;
          ts = stretched_ts;
          preserve_safe_mppi_bspline = true;
	          ROS_INFO("[PlannerManager]   %s seed made feasible by time stretch %.2fx (clearance=%.2fm, z=[%.2f, %.2f]); skipping rebound",
	                   using_static_topo_seed ? "Static topo" : "MPPI",
	                   stretch_ratio, stretched_min_clearance, stretched_min_z, stretched_max_z);
        } else {
          ROS_INFO("[PlannerManager]   MPPI seed time stretch %.2fx insufficient (z_safe=%d, collision_free=%d, dyn_feasible=%d, clearance=%.2fm, ratio=%.2f)",
                   stretch_ratio, stretched_z_safe, stretched_collision_free,
                   stretched_dynamic_feasible, stretched_min_clearance, stretched_ratio);
        }
      } else {
        ROS_INFO("[PlannerManager]   MPPI seed needs B-spline processing (z_safe=%d, collision_free=%d, dyn_feasible=%d, clearance=%.2fm, ratio=%.2f)",
                 seed_z_safe, seed_collision_free, seed_dynamic_feasible, seed_min_clearance, feasibility_ratio);
      }
    }
    
	    // Dynamic scenes need a fast MPPI fallback when rebound repeatedly fails,
	    // but the counter is reset after every successful publish below so this
	    // does not lock the planner into raw MPPI mode.
	    const bool allow_bspline_skip = step3_dynamic_scene_active;
	    if (flag_step_1_success) {
	      // Verified direct seed path already produced ctrl_pts.
	    } else if (preserve_safe_mppi_bspline) {
	      flag_step_1_success = true;
	      used_mppi_fallback = true;
	      bspline_consecutive_failures_ = 0;
	    } else if (allow_bspline_skip &&
	               bspline_consecutive_failures_ >= bspline_skip_threshold_ &&
	               mppi_result_backup_.positions.size() >= 3) {
	      ROS_WARN("[PlannerManager]   Skipping B-spline (consecutive failures=%d >= %d), using MPPI directly",
	               bspline_consecutive_failures_, bspline_skip_threshold_);
      point_set = mppi_result_backup_.positions;
      UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
      flag_step_1_success = true;
      used_mppi_fallback = true;
      // Decay the counter so we periodically retry B-spline
      bspline_consecutive_failures_ = std::max(0, bspline_consecutive_failures_ - 1);
    } else {
      bool early_control_points_blocked = false;
      if (grid_map_ != nullptr && ctrl_pts.cols() >= 4) {
        const int check_cols = std::min<int>(4, ctrl_pts.cols() - 1);
        for (int cp_idx = 1; cp_idx <= check_cols; ++cp_idx) {
          const Eigen::Vector3d cp = ctrl_pts.col(cp_idx);
          if (!grid_map_->isInMap(cp) || grid_map_->getInflateOccupancy(cp)) {
            early_control_points_blocked = true;
            break;
          }
        }
      }

      if (early_control_points_blocked) {
        ROS_WARN("[PlannerManager]   B-spline rebound precheck rejected early occupied control points; validating MPPI fallback directly");
        if (mppi_result_backup_.positions.size() >= 3) {
          point_set = mppi_result_backup_.positions;
          UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
          flag_step_1_success = true;
          used_mppi_fallback = true;
          skipped_rebound_precheck = true;
        } else {
          flag_step_1_success = false;
        }
      } else {
        // Normal B-spline optimization.
        flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      }
    }
    
    ROS_INFO("[PlannerManager]   B-spline result: %s", flag_step_1_success ? " SUCCESS" : " FAILED");
    
    if (!flag_step_1_success)
    {
      bspline_consecutive_failures_++;
      ROS_WARN("[PlannerManager]    B-spline failed (consecutive=%d) - using MPPI trajectory as fallback",
               bspline_consecutive_failures_);
      
      //  IMPROVED: Use MPPI trajectory directly instead of failing
      if (mppi_result_backup_.positions.size() >= 3) {
        ROS_INFO("[PlannerManager]    Fallback: Using MPPI trajectory (%zu points)", mppi_result_backup_.positions.size());
        
        // Use MPPI trajectory as-is
        point_set = mppi_result_backup_.positions;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
        
        // Mark as success with fallback
        flag_step_1_success = true;
        used_mppi_fallback = true;  // Set flag
        ROS_INFO("[PlannerManager]    Fallback succeeded, continuing with MPPI path");
      } else {
        ROS_WARN("[PlannerManager]    No MPPI backup available, planning failed");
        continous_failures_count_++;
        return finish_cycle(false, "bspline_fail_no_mppi_backup");
      }
    } else if (skipped_rebound_precheck) {
      ROS_INFO("[PlannerManager]    Rebound precheck fallback prepared; final validators will decide publication");
    } else if (!used_mppi_fallback) {
      // B-spline succeeded normally, reset consecutive failure counter
      bspline_consecutive_failures_ = 0;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = ros::Time::now() - t_start;
    timing.bspline_ms = t_opt.toSec() * 1000.0;
    timing.used_mppi_fallback = used_mppi_fallback;

    t_start = ros::Time::now();

    /*** STEP 4: TIME REALLOCATION FOR FEASIBILITY ***/
    //  Adjust time allocation to satisfy velocity/acceleration constraints
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    
    // MPPI fallback has already been generated with the planner's control
    // limits. Keep it fast here; final z/static/dynamic validators below still
    // decide whether the trajectory can be published.
    if (used_mppi_fallback) {
      ROS_INFO("[PlannerManager]    Skipping refine for MPPI fallback");
      flag_step_2_success = true;
    } else if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success)
    {
      printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
      continous_failures_count_++;
      return finish_cycle(false, "refine_fail");
    }

    t_refine = ros::Time::now() - t_start;
    timing.refine_ms = t_refine.toSec() * 1000.0;
    timing.used_mppi_fallback = used_mppi_fallback;

    // save planned results
    if (final_fallback_feasibility_repair_ && used_mppi_fallback) {
      auto capped_vector = [](const Eigen::Vector3d &v, double max_norm) -> Eigen::Vector3d {
        if (!std::isfinite(max_norm) || max_norm <= 1e-6 || v.norm() <= max_norm) {
          return v;
        }
        return Eigen::Vector3d(v.normalized() * max_norm);
      };
      auto make_relaxed_fallback_derivatives = [&]() {
        std::vector<Eigen::Vector3d> derivatives(4, Eigen::Vector3d::Zero());
        const double max_start_vel =
            std::max(0.1, pp_.max_vel_ * final_fallback_relaxed_velocity_scale_);
        Eigen::Vector3d relaxed_start_vel = capped_vector(start_vel, max_start_vel);
        if (planar_flight_z_lock_) {
          relaxed_start_vel.z() = 0.0;
        }
        derivatives[0] = relaxed_start_vel;
        derivatives[1] = Eigen::Vector3d::Zero();
        derivatives[2] = Eigen::Vector3d::Zero();
        derivatives[3] = Eigen::Vector3d::Zero();
        return derivatives;
      };
      auto try_relaxed_fallback_reparameterization =
          [&](double candidate_ts, UniformBspline &out_pos, double &out_ts,
              Eigen::MatrixXd &out_ctrl_pts) {
            if (!final_fallback_relaxed_derivative_repair_ ||
                mppi_result_backup_.positions.size() < 4) {
              return false;
            }

            Eigen::MatrixXd relaxed_ctrl_pts;
            const std::vector<Eigen::Vector3d> relaxed_derivatives =
                make_relaxed_fallback_derivatives();
            UniformBspline::parameterizeToBspline(candidate_ts,
                                                  mppi_result_backup_.positions,
                                                  relaxed_derivatives,
                                                  relaxed_ctrl_pts);
            UniformBspline relaxed_pos(relaxed_ctrl_pts, 3, candidate_ts);
            relaxed_pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
                                          pp_.feasibility_tolerance_);
            double relaxed_ratio = 1.0;
            if (!relaxed_pos.checkFeasibility(relaxed_ratio, false) &&
                std::isfinite(relaxed_ratio) && relaxed_ratio > 1.0 &&
                relaxed_ratio <= 1.6) {
              relaxed_pos.lengthenTime(relaxed_ratio * 1.05);
              relaxed_pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
                                            pp_.feasibility_tolerance_);
            }

            double relaxed_ratio_check = 1.0;
            if (!relaxed_pos.checkFeasibility(relaxed_ratio_check, false)) {
              ROS_INFO("[PlannerManager] Relaxed fallback derivative repair still infeasible (ratio=%.2f)",
                       relaxed_ratio_check);
              return false;
            }

            out_pos = relaxed_pos;
            out_ts = relaxed_pos.getInterval();
            out_ctrl_pts = relaxed_pos.getControlPoint();
            ROS_INFO("[PlannerManager] Final MPPI fallback feasibility repaired with relaxed endpoint derivatives (ts=%.3f)",
                     out_ts);
            return true;
          };

      double final_feasibility_ratio = 1.0;
      if (!pos.checkFeasibility(final_feasibility_ratio, false)) {
        const double stretch_limit =
            using_static_topo_seed ? std::max(safe_seed_time_stretch_max_, 7.5)
                                   : std::max(safe_seed_time_stretch_max_, 3.0);
        if (std::isfinite(final_feasibility_ratio) &&
            final_feasibility_ratio > 1.0 &&
            final_feasibility_ratio <= stretch_limit) {
          const double stretch_ratio =
              std::min(stretch_limit, final_feasibility_ratio * 1.08);
          UniformBspline stretched_pos = pos;
          stretched_pos.lengthenTime(stretch_ratio);
          stretched_pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
                                          pp_.feasibility_tolerance_);
          double stretched_ratio_check = 1.0;
          if (stretched_pos.checkFeasibility(stretched_ratio_check, false)) {
            pos = stretched_pos;
            ts = pos.getInterval();
            ctrl_pts = pos.getControlPoint();
            ROS_INFO("[PlannerManager] Final MPPI fallback feasibility repaired by B-spline time stretch %.2fx",
                     stretch_ratio);
          } else {
            ROS_INFO("[PlannerManager] Direct B-spline stretch %.2fx insufficient (ratio=%.2f); trying raw seed reparameterization",
                     stretch_ratio, stretched_ratio_check);
            if (mppi_result_backup_.positions.size() >= 4) {
              const double stretched_ts = ts * stretch_ratio;
              Eigen::MatrixXd stretched_ctrl_pts;
              UniformBspline::parameterizeToBspline(stretched_ts,
                                                    mppi_result_backup_.positions,
                                                    start_end_derivatives,
                                                    stretched_ctrl_pts);
              UniformBspline raw_stretched_pos(stretched_ctrl_pts, 3, stretched_ts);
              raw_stretched_pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,
                                                  pp_.feasibility_tolerance_);
              double raw_stretched_ratio = 1.0;
              if (raw_stretched_pos.checkFeasibility(raw_stretched_ratio, false)) {
                ctrl_pts = stretched_ctrl_pts;
                ts = stretched_ts;
                pos = raw_stretched_pos;
                ROS_INFO("[PlannerManager] Final MPPI fallback feasibility repaired by raw seed time stretch %.2fx",
                         stretch_ratio);
              } else if (try_relaxed_fallback_reparameterization(stretched_ts, pos, ts, ctrl_pts)) {
                // Relaxed endpoint derivatives reduce infeasible spikes from
                // repeatedly reparameterizing short MPPI fallback snippets.
              } else {
                ROS_WARN("[PlannerManager] Final MPPI fallback remains dynamically infeasible after stretch (ratio=%.2f)",
                         raw_stretched_ratio);
                continous_failures_count_++;
                return finish_cycle(false, "fallback_final_feasibility_reject");
              }
            } else {
              ROS_WARN("[PlannerManager] Final MPPI fallback infeasible and no raw backup is available for repair");
              continous_failures_count_++;
              return finish_cycle(false, "fallback_final_feasibility_reject");
            }
          }
        } else {
          const double relaxed_ts =
              ts * std::min(stretch_limit,
                            std::max(1.0, std::min(final_feasibility_ratio, 2.2)));
          if (try_relaxed_fallback_reparameterization(relaxed_ts, pos, ts, ctrl_pts)) {
            // Keep final validators below in charge of static/dynamic safety.
          } else {
            ROS_WARN("[PlannerManager] Final MPPI fallback infeasible and cannot be stretched safely (ratio=%.2f limit=%.2f)",
                     final_feasibility_ratio, stretch_limit);
            continous_failures_count_++;
            return finish_cycle(false, "fallback_final_feasibility_reject");
          }
        }
      }
    }

    validator_start_time = ros::Time::now();
    validator_started = true;
    double pre_clamp_min_z = 0.0;
    double pre_clamp_max_z = 0.0;
    if (!isBsplineWithinZBounds(pos, &pre_clamp_min_z, &pre_clamp_max_z) &&
        pre_clamp_min_z < publish_min_z_ && start_pt.z() >= publish_min_z_ &&
        local_target_pt.z() >= publish_min_z_) {
      Eigen::MatrixXd clamped_ctrl_pts = pos.getControlPoint();
      if (clampBsplineControlPointsZ(clamped_ctrl_pts, start_pt, local_target_pt)) {
        UniformBspline clamped_pos(clamped_ctrl_pts, 3, ts);
        clamped_pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
        pos = clamped_pos;
        ROS_WARN("[PlannerManager] Clamped B-spline control-point z floor from %.2f before final check",
                 pre_clamp_min_z);
      }
    }

    double final_min_z = 0.0;
    double final_max_z = 0.0;
    if (!isBsplineWithinZBounds(pos, &final_min_z, &final_max_z)) {
      ROS_WARN("[PlannerManager] Final B-spline rejected by z safety (z=[%.2f, %.2f], publish_min=%.2f, max_z=%.2f)",
               final_min_z, final_max_z, publish_min_z_, max_z_);
      if (generateRecoveryTraj(start_pt, start_vel, start_acc, local_target_pt, ts)) {
        continous_failures_count_ = 0;
        return finish_cycle(true, "z_recovery");
      } else {
        continous_failures_count_++;
        return finish_cycle(false, "z_safety_reject");
      }
    }

	    bool final_dynamic_scene_active = false;
	    {
	      std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
	      final_dynamic_scene_active =
	          has_dynamic_obstacles_ && !latest_dynamic_obstacles_.obstacles.empty();
	    }
    const double final_static_check_fraction =
        final_dynamic_scene_active ? final_dynamic_static_check_fraction_ : 2.0 / 3.0;
	    const double final_static_check_time =
	        final_dynamic_scene_active ? final_dynamic_static_check_time_ : std::numeric_limits<double>::infinity();
	    const double direct_dynamic_publish_min_distance =
	        final_dynamic_scene_active ? dynamic_publish_preferred_distance_
	                                   : final_dynamic_min_distance_;

    auto trajectoryMakesLocalProgress = [&](UniformBspline &traj,
                                            double min_probe_time,
                                            const char *label) -> bool {
      double t_start = 0.0;
      double t_end = 0.0;
      if (!traj.getTimeSpan(t_start, t_end) || t_end < t_start) {
        return false;
      }

      const double duration = std::max(0.0, t_end - t_start);
      const double probe_t =
          std::min(duration, std::max(min_probe_time, 0.45 * duration));
      const Eigen::Vector3d traj_start = traj.evaluateDeBoorT(0.0);
      const Eigen::Vector3d traj_probe = traj.evaluateDeBoorT(probe_t);
      const Eigen::Vector3d traj_end = traj.evaluateDeBoorT(duration);
      const Eigen::Vector3d goal_vec = local_target_pt - traj_start;
      const double start_goal_dist = goal_vec.norm();
      if (start_goal_dist < 0.5) {
        return true;
      }

      const Eigen::Vector3d goal_dir = goal_vec / start_goal_dist;
      const Eigen::Vector3d probe_delta = traj_probe - traj_start;
      const Eigen::Vector3d end_delta = traj_end - traj_start;
      const double probe_forward = probe_delta.dot(goal_dir);
      const double end_forward = end_delta.dot(goal_dir);
      const double probe_lateral =
          (probe_delta - probe_forward * goal_dir).norm();
      const double probe_goal_dist = (local_target_pt - traj_probe).norm();
      const double end_goal_dist = (local_target_pt - traj_end).norm();
      const double lateral_limit =
          std::max(1.2, 1.2 * std::max(0.0, probe_forward) + 0.4);

      const bool makes_progress =
          probe_forward >= 0.15 &&
          end_forward >= -0.10 &&
          probe_goal_dist <= start_goal_dist - 0.05 &&
          end_goal_dist <= start_goal_dist + 0.8 &&
          probe_lateral <= lateral_limit;
      if (!makes_progress) {
        ROS_WARN("[PlannerManager] %s rejected by local progress gate: forward=%.2f/%.2f lateral=%.2f<=%.2f goal=%.2f->%.2f/%.2f",
                 label, probe_forward, end_forward, probe_lateral, lateral_limit,
                 start_goal_dist, probe_goal_dist, end_goal_dist);
      }
      return makes_progress;
    };

    auto trajectoryDynamicallyFeasibleForPublish =
        [&](UniformBspline &traj, const char *label) -> bool {
      if (!final_publish_feasibility_gate_enabled_) {
        return true;
      }
      traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
      double feasibility_ratio = 1.0;
      if (traj.checkFeasibility(feasibility_ratio, false)) {
        return true;
      }
      ROS_WARN("[PlannerManager] %s rejected by final publish feasibility gate (ratio=%.2f)",
               label, feasibility_ratio);
      return false;
    };

    double final_min_clearance = 0.0;
    const bool final_static_ok =
        isBsplineCollisionFree(pos, final_static_min_clearance_, &final_min_clearance,
                               final_static_check_fraction, final_static_check_time,
                               final_static_check_start_skip_);
    timing.final_static_clearance = final_min_clearance;
    if (!final_static_ok) {
      ROS_WARN("[PlannerManager] Final B-spline rejected by static safety check (min_clearance=%.2fm)",
               final_min_clearance);
      if (mppi_result_backup_.positions.size() >= 4) {
        UniformBspline direct_seed_traj;
        double direct_min_clearance = 0.0;
        double direct_min_dynamic_dist = 0.0;
	        if (buildSafeSeedBspline(mppi_result_backup_.positions, start_end_derivatives, ts,
	                                 direct_seed_traj,
	                                 final_static_check_fraction, final_static_check_time,
	                                 final_static_check_start_skip_,
	                                 0.0,
	                                 &direct_min_clearance, &direct_min_dynamic_dist)) {
	          if (final_dynamic_scene_active &&
	              direct_min_dynamic_dist < direct_dynamic_publish_min_distance) {
	            ROS_WARN("[PlannerManager] Direct seed after static rejection rejected by publish dynamic margin (dyn=%.2fm < %.2fm)",
	                     direct_min_dynamic_dist, direct_dynamic_publish_min_distance);
	            continous_failures_count_++;
	            timing.final_dynamic_clearance = direct_min_dynamic_dist;
	            return finish_cycle(false, "direct_seed_dynamic_margin_after_static_reject");
	          }
	          if (!trajectoryMakesLocalProgress(direct_seed_traj, 0.8,
	                                             "Direct seed after static rejection")) {
	            continous_failures_count_++;
	            return finish_cycle(false, "direct_seed_progress_after_static_reject");
          }
	          if (!trajectoryDynamicallyFeasibleForPublish(
	                  direct_seed_traj, "Direct seed after static rejection")) {
	            continous_failures_count_++;
	            return finish_cycle(false, "direct_seed_feasibility_after_static_reject");
          }
	          ROS_WARN("[PlannerManager] Publishing direct MPPI/topo seed B-spline after static rejection (clear=%.2fm, dyn=%.2fm)",
	                   direct_min_clearance, direct_min_dynamic_dist);
	          last_traj_is_recovery_ = false;
	          updateTrajInfo(direct_seed_traj, ros::Time::now());
	          commitPublishedTopoCache(local_target_pt, topo_paths_for_success_cache);
	          continous_failures_count_ = 0;
	          bspline_consecutive_failures_ = 0;
	          timing.final_static_clearance = direct_min_clearance;
	          timing.final_dynamic_clearance = direct_min_dynamic_dist;
	          return finish_cycle(true, "direct_seed_after_static_reject");
	        }
	        std::vector<Eigen::Vector3d> repaired_seed;
	        if (repairSeedPathByClearance(mppi_result_backup_.positions, repaired_seed) &&
	            buildSafeSeedBspline(repaired_seed, start_end_derivatives, ts,
	                                 direct_seed_traj,
	                                 final_static_check_fraction, final_static_check_time,
	                                 final_static_check_start_skip_,
	                                 0.0,
	                                 &direct_min_clearance, &direct_min_dynamic_dist)) {
	          if (final_dynamic_scene_active &&
	              direct_min_dynamic_dist < direct_dynamic_publish_min_distance) {
	            ROS_WARN("[PlannerManager] Repaired direct seed after static rejection rejected by publish dynamic margin (dyn=%.2fm < %.2fm)",
	                     direct_min_dynamic_dist, direct_dynamic_publish_min_distance);
	            continous_failures_count_++;
	            timing.final_dynamic_clearance = direct_min_dynamic_dist;
	            return finish_cycle(false, "repaired_seed_dynamic_margin_after_static_reject");
	          }
	          if (!trajectoryMakesLocalProgress(direct_seed_traj, 0.8,
	                                             "Repaired direct seed after static rejection")) {
	            continous_failures_count_++;
	            return finish_cycle(false, "repaired_seed_progress_after_static_reject");
          }
	          if (!trajectoryDynamicallyFeasibleForPublish(
	                  direct_seed_traj, "Repaired direct seed after static rejection")) {
	            continous_failures_count_++;
	            return finish_cycle(false, "repaired_seed_feasibility_after_static_reject");
          }
	          ROS_WARN("[PlannerManager] Publishing repaired MPPI/topo seed B-spline after static rejection (clear=%.2fm, dyn=%.2fm)",
	                   direct_min_clearance, direct_min_dynamic_dist);
	          last_traj_is_recovery_ = false;
	          updateTrajInfo(direct_seed_traj, ros::Time::now());
	          commitPublishedTopoCache(local_target_pt, topo_paths_for_success_cache);
	          continous_failures_count_ = 0;
	          bspline_consecutive_failures_ = 0;
	          timing.final_static_clearance = direct_min_clearance;
	          timing.final_dynamic_clearance = direct_min_dynamic_dist;
	          return finish_cycle(true, "repaired_seed_after_static_reject");
	        }
      }
      if (static_escape_check_after_failures_ > 0 &&
          final_min_clearance <= static_escape_max_initial_clearance_ &&
          continous_failures_count_ >= static_escape_check_after_failures_) {
        double escape_min_clearance = 0.0;
        double escape_min_dynamic_dist = 0.0;
        if (isBsplineCollisionFree(pos, final_static_min_clearance_,
                                   &escape_min_clearance,
                                   final_static_check_fraction,
                                   final_static_check_time,
                                   static_escape_check_start_skip_) &&
            escape_min_clearance >= static_escape_min_post_clearance_ &&
            trajectoryMakesLocalProgress(pos, static_escape_check_start_skip_ + 0.8,
                                         "Static escape trajectory") &&
            trajectoryDynamicallyFeasibleForPublish(pos,
                                                    "Static escape trajectory") &&
            (!final_dynamic_scene_active ||
             isBsplineDynamicSafe(pos, final_dynamic_min_distance_,
                                  &escape_min_dynamic_dist))) {
	          ROS_WARN("[PlannerManager] Publishing trajectory after static escape-window validation (skip=%.2fs, post_escape_clearance=%.2fm, dyn_clearance=%.2fm)",
	                   static_escape_check_start_skip_, escape_min_clearance,
	                   final_dynamic_scene_active ? escape_min_dynamic_dist : 5.0);
	          last_traj_is_recovery_ = false;
	          updateTrajInfo(pos, ros::Time::now());
	          commitPublishedTopoCache(local_target_pt, topo_paths_for_success_cache);
	          continous_failures_count_ = 0;
	          bspline_consecutive_failures_ = 0;
	          timing.final_static_clearance = escape_min_clearance;
	          timing.final_dynamic_clearance =
	              final_dynamic_scene_active ? escape_min_dynamic_dist : 5.0;
	          return finish_cycle(true, "static_escape_window");
	        }
      }
      if (clearance_recovery_enabled_ &&
          continous_failures_count_ >= 2 &&
          generateClearanceRecoveryTraj(start_pt, start_vel, start_acc, local_target_pt, ts)) {
        continous_failures_count_ = 0;
        bspline_consecutive_failures_ = 0;
        return finish_cycle(true, "clearance_recovery");
      }
      continous_failures_count_++;
      return finish_cycle(false, "final_static_reject");
    }

    double final_min_dynamic_dist = 0.0;
    const bool final_dynamic_ok =
        isBsplineDynamicSafe(pos, final_dynamic_min_distance_, &final_min_dynamic_dist);
    timing.final_dynamic_clearance = final_min_dynamic_dist;
    if (!final_dynamic_ok) {
      ROS_WARN("[PlannerManager] Final B-spline rejected by dynamic obstacle check (min_dynamic_dist=%.2fm)",
               final_min_dynamic_dist);
      if (mppi_result_backup_.positions.size() >= 4) {
        UniformBspline direct_seed_traj;
        double direct_min_clearance = 0.0;
        double direct_min_dynamic_dist = 0.0;
	        if (buildSafeSeedBspline(mppi_result_backup_.positions, start_end_derivatives, ts,
	                                 direct_seed_traj,
	                                 final_static_check_fraction, final_static_check_time,
	                                 final_static_check_start_skip_,
	                                 0.0,
	                                 &direct_min_clearance, &direct_min_dynamic_dist)) {
	          if (direct_min_dynamic_dist < direct_dynamic_publish_min_distance) {
	            ROS_WARN("[PlannerManager] Direct seed after dynamic rejection rejected by publish dynamic margin (dyn=%.2fm < %.2fm)",
	                     direct_min_dynamic_dist, direct_dynamic_publish_min_distance);
	            continous_failures_count_++;
	            timing.final_dynamic_clearance = direct_min_dynamic_dist;
	            return finish_cycle(false, "direct_seed_dynamic_margin_after_dynamic_reject");
	          }
	          if (!trajectoryMakesLocalProgress(direct_seed_traj, 0.8,
	                                             "Direct seed after dynamic rejection")) {
	            continous_failures_count_++;
	            return finish_cycle(false, "direct_seed_progress_after_dynamic_reject");
          }
	          if (!trajectoryDynamicallyFeasibleForPublish(
	                  direct_seed_traj, "Direct seed after dynamic rejection")) {
	            continous_failures_count_++;
	            return finish_cycle(false, "direct_seed_feasibility_after_dynamic_reject");
          }
	          ROS_WARN("[PlannerManager] Publishing direct MPPI/topo seed B-spline after dynamic rejection (clear=%.2fm, dyn=%.2fm)",
	                   direct_min_clearance, direct_min_dynamic_dist);
	          last_traj_is_recovery_ = false;
	          updateTrajInfo(direct_seed_traj, ros::Time::now());
	          commitPublishedTopoCache(local_target_pt, topo_paths_for_success_cache);
	          continous_failures_count_ = 0;
	          bspline_consecutive_failures_ = 0;
	          timing.final_static_clearance = direct_min_clearance;
	          timing.final_dynamic_clearance = direct_min_dynamic_dist;
	          return finish_cycle(true, "direct_seed_after_dynamic_reject");
	        }
      }
      continous_failures_count_++;
      return finish_cycle(false, "final_dynamic_reject");
    }

    if (!trajectoryMakesLocalProgress(pos, 0.8, "Final B-spline")) {
      continous_failures_count_++;
      return finish_cycle(false, "final_progress_reject");
    }
    if (!trajectoryDynamicallyFeasibleForPublish(pos, "Final B-spline")) {
      continous_failures_count_++;
      return finish_cycle(false, "final_feasibility_reject");
    }

    last_traj_is_recovery_ = false;
    updateTrajInfo(pos, ros::Time::now());
    commitPublishedTopoCache(local_target_pt, topo_paths_for_success_cache);

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

	    // success. YoY
	    continous_failures_count_ = 0;
	    bspline_consecutive_failures_ = 0;
	    return finish_cycle(true, "success");
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    if (stop_pos.z() < publish_min_z_ + 0.05)
    {
      Eigen::Vector3d recovery_target = stop_pos;
      recovery_target.z() = std::min(max_z_ - 0.1,
                                     std::max(recovery_min_z_, stop_pos.z() + 0.6));
      const double ts = std::max(0.08, pp_.ctrl_pt_dist / std::max(0.1, pp_.max_vel_) * 1.2);

      ROS_WARN("[PlannerManager] Emergency stop requested below safe z (z=%.2f, publish_min=%.2f); publishing recovery climb",
               stop_pos.z(), publish_min_z_);
      if (generateRecoveryTraj(stop_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                               recovery_target, ts))
      {
        return true;
      }

      ROS_WARN("[PlannerManager] Emergency recovery climb failed; clamping emergency stop z to publish_min_z");
      stop_pos.z() = publish_min_z_;
    }
    else
    {
      last_traj_is_recovery_ = false;
    }

    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());
    last_traj_is_recovery_ = false;

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

  // SECTION new topological and MPPI planning methods

  bool EGOPlannerManager::rebaseCachedTopoPaths(const Eigen::Vector3d &start_pos,
	                                                const Eigen::Vector3d &goal_pos,
	                                                std::vector<TopoPath> &topo_paths) const {
	    topo_paths.clear();
	    if (!has_cached_topo_paths_ || cached_topo_paths_.empty()) {
	      return false;
	    }
	    const double goal_reuse_dist = (cached_topo_goal_ - goal_pos).norm();
	    const bool same_goal_window = goal_reuse_dist <= cached_topo_goal_reuse_max_dist_;
	    bool dynamic_scene_active = false;
	    {
	      std::lock_guard<std::mutex> lock(dynamic_obstacle_mutex_);
	      dynamic_scene_active = has_dynamic_obstacles_ && !latest_dynamic_obstacles_.obstacles.empty();
	    }
	    const double dynamic_goal_reuse_max =
	        std::min(cached_topo_goal_reuse_max_dist_, std::max(0.35, pp_.ctrl_pt_dist));
	    if (!same_goal_window ||
	        (dynamic_scene_active && goal_reuse_dist > dynamic_goal_reuse_max)) {
	      return false;
	    }

    const auto segment_is_free = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
      if (grid_map_ == nullptr) {
        return false;
      }
      const Eigen::Vector3d seg = b - a;
      const double len = seg.norm();
      const int samples = std::max(1, static_cast<int>(std::ceil(len / std::max(0.10, pp_.ctrl_pt_dist * 0.5))));
      for (int i = 0; i <= samples; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(samples);
        const Eigen::Vector3d p = a * (1.0 - alpha) + b * alpha;
        if (!grid_map_->isInMap(p) || grid_map_->getInflateOccupancy(p)) {
          return false;
        }
      }
      return true;
    };

    for (const auto &cached_path : cached_topo_paths_) {
      if (cached_path.path.size() < 2) {
        continue;
      }

      size_t nearest_idx = 0;
      double nearest_dist = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < cached_path.path.size(); ++i) {
        const double dist = (cached_path.path[i] - start_pos).norm();
        if (dist < nearest_dist) {
          nearest_dist = dist;
          nearest_idx = i;
        }
      }

      if (nearest_dist > std::max(cached_topo_rebase_max_dist_, pp_.ctrl_pt_dist * 6.0) ||
          !segment_is_free(start_pos, cached_path.path[nearest_idx])) {
        continue;
      }

      TopoPath rebased = cached_path;
      rebased.path.clear();
      rebased.path.push_back(start_pos);
      for (size_t i = nearest_idx; i < cached_path.path.size(); ++i) {
        if ((cached_path.path[i] - rebased.path.back()).norm() > 0.10) {
          rebased.path.push_back(cached_path.path[i]);
        }
      }
	      if ((rebased.path.back() - goal_pos).norm() > 0.20) {
	        if (!segment_is_free(rebased.path.back(), goal_pos)) {
	          continue;
	        }
	        rebased.path.push_back(goal_pos);
	      }

      bool valid = rebased.path.size() >= 2 && isPathWithinZBounds(rebased.path);
      double length = 0.0;
      for (size_t i = 1; valid && i < rebased.path.size(); ++i) {
        if (!segment_is_free(rebased.path[i - 1], rebased.path[i])) {
          valid = false;
          break;
        }
        length += (rebased.path[i] - rebased.path[i - 1]).norm();
      }

      if (!valid) {
        continue;
      }

	      rebased.cost = length;
	      topo_paths.push_back(rebased);
	      if (topo_paths.size() >= static_cast<size_t>(max_mppi_topo_candidates_)) {
	        break;
	      }
	    }

	    if (!topo_paths.empty()) {
	      ROS_INFO("[PlannerManager] Reused cached topo path(s): goal_shift=%.2fm paths=%zu dynamic=%d",
	               goal_reuse_dist, topo_paths.size(), dynamic_scene_active ? 1 : 0);
	    }
	    return !topo_paths.empty();
	  }

  void EGOPlannerManager::commitPublishedTopoCache(
      const Eigen::Vector3d &goal_pos,
      const std::vector<TopoPath> &topo_paths) {
    if (topo_paths.empty()) {
      return;
    }

    cached_topo_paths_.clear();
    cached_topo_paths_.reserve(std::min(topo_paths.size(),
                                        static_cast<size_t>(max_mppi_topo_candidates_)));

    const auto segment_is_free = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
      if (grid_map_ == nullptr) {
        return false;
      }
      const Eigen::Vector3d seg = b - a;
      const double len = seg.norm();
      const int samples = std::max(
          1, static_cast<int>(std::ceil(len / std::max(0.10, pp_.ctrl_pt_dist * 0.5))));
      for (int i = 0; i <= samples; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(samples);
        const Eigen::Vector3d p = a * (1.0 - alpha) + b * alpha;
        if (!grid_map_->isInMap(p) || grid_map_->getInflateOccupancy(p)) {
          return false;
        }
      }
      return true;
    };

    for (const auto &path : topo_paths) {
      if (path.path.size() < 2 || !isPathWithinZBounds(path.path)) {
        continue;
      }
      bool valid = true;
      for (size_t i = 1; i < path.path.size(); ++i) {
        if (!segment_is_free(path.path[i - 1], path.path[i])) {
          valid = false;
          break;
        }
      }
      if (!valid) {
        continue;
      }
      cached_topo_paths_.push_back(path);
      if (cached_topo_paths_.size() >= static_cast<size_t>(max_mppi_topo_candidates_)) {
        break;
      }
    }

    if (cached_topo_paths_.empty()) {
      return;
    }

    cached_topo_goal_ = goal_pos;
    has_cached_topo_paths_ = true;
    ROS_INFO("[PlannerManager] Committed %zu published topo path(s) to route memory",
             cached_topo_paths_.size());
  }

  bool EGOPlannerManager::planWithTopo(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos,
                                      std::vector<TopoPath> &topo_paths) {
    if (topo_planner_ == nullptr) {
      ROS_ERROR("[PlannerManager] Topological planner not initialized");
      return false;
    }

    if (prefer_cached_topo_paths_ &&
        rebaseCachedTopoPaths(start_pos, goal_pos, topo_paths)) {
      ROS_INFO("[PlannerManager] Using cached published topo path(s) before fresh search: %zu",
               topo_paths.size());
      visualizeTopoPathsSmooth(topo_paths);
      return true;
    }

    bool success = topo_planner_->searchTopoPaths(start_pos, goal_pos, topo_paths);
    
    if (success) {
      ROS_INFO("[PlannerManager] Topological planning succeeded, found %zu paths", topo_paths.size());
      if (!cache_only_published_topo_paths_) {
        commitPublishedTopoCache(goal_pos, topo_paths);
      }
      //  可视化B-spline平滑的拓扑路径
      visualizeTopoPathsSmooth(topo_paths);
    } else {
      if (rebaseCachedTopoPaths(start_pos, goal_pos, topo_paths)) {
        ROS_WARN("[PlannerManager] Topology search missed; using %zu rebased cached topo path(s)",
                 topo_paths.size());
        visualizeTopoPathsSmooth(topo_paths);
        success = true;
      } else {
        ROS_WARN("[PlannerManager] Topological planning failed");
      }
    }

    return success;
  }

  bool EGOPlannerManager::planWithMPPI(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                      const Eigen::Vector3d &goal_pos, const Eigen::Vector3d &goal_vel,
                                      MPPITrajectory &optimal_traj) {
    if (mppi_planner_ == nullptr) {
      ROS_ERROR("[PlannerManager] MPPI planner not initialized");
      return false;
    }

    bool success = mppi_planner_->planTrajectory(start_pos, start_vel, goal_pos, goal_vel, optimal_traj);
    
    if (success) {
      ROS_INFO("[PlannerManager] MPPI planning succeeded, trajectory cost: %f", optimal_traj.cost);
    } else {
      ROS_WARN("[PlannerManager] MPPI planning failed");
    }

    return success;
  }

  //  Visualize all MPPI-optimized candidate paths with B-spline smoothing
  void EGOPlannerManager::visualizeAllMPPIPaths() {
    if (all_mppi_paths_.empty()) {
      return;
    }

    visualization_msgs::MarkerArray marker_array;
    
    //  First, send DELETE markers to clear old paths with different counts
    // This ensures old paths are removed when path count changes
    for (int i = 0; i < 10; ++i) {  // Clear up to 10 possible old paths
      visualization_msgs::Marker delete_line, delete_text;
      delete_line.header.frame_id = "world";
      delete_line.header.stamp = ros::Time::now();
      delete_line.ns = "mppi_bspline";
      delete_line.id = i;
      delete_line.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(delete_line);
      
      delete_text.header = delete_line.header;
      delete_text.ns = "mppi_label";
      delete_text.id = i;
      delete_text.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(delete_text);
    }
    
    // Color palette for different paths (rainbow colors)
    std::vector<std::array<float, 3>> colors = {
      {1.0, 0.0, 0.0},   // Red
      {1.0, 0.5, 0.0},   // Orange
      {0.0, 1.0, 0.0},   // Green
      {0.0, 1.0, 1.0},   // Cyan
      {0.0, 0.0, 1.0},   // Blue
      {0.5, 0.0, 1.0},   // Purple
      {1.0, 0.0, 1.0},   // Magenta
      {1.0, 0.5, 0.5},   // Pink
    };
    
    std::string frame_id = "world";
    double ts = 0.1;  // B-spline time step
    
    for (size_t i = 0; i < all_mppi_paths_.size(); ++i) {
      const auto& path = all_mppi_paths_[i];
      
      if (path.positions.size() < 7 || !path.success) continue;  // Need at least 7 points for B-spline
      
      //  Convert MPPI trajectory to B-spline for smooth visualization
      std::vector<Eigen::Vector3d> mppi_points = path.positions;
      
      // Prepare boundary derivatives: [start_vel, end_vel, start_acc, end_acc]
      std::vector<Eigen::Vector3d> start_end_derivative(4);
      start_end_derivative[0] = Eigen::Vector3d::Zero();  // Start velocity
      start_end_derivative[1] = Eigen::Vector3d::Zero();  // End velocity
      start_end_derivative[2] = Eigen::Vector3d::Zero();  // Start acceleration
      start_end_derivative[3] = Eigen::Vector3d::Zero();  // End acceleration
      
      Eigen::MatrixXd bspline_ctrl_pts;
      try {
        UniformBspline::parameterizeToBspline(ts, mppi_points, start_end_derivative, bspline_ctrl_pts);
        
        // Create B-spline object
        UniformBspline bspline(bspline_ctrl_pts, 3, ts);
        
        // Sample smooth points from B-spline
        double duration = bspline.getTimeSum();
        
        // Create line marker
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = frame_id;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "mppi_bspline";  //  Fixed namespace (not indexed)
        line_marker.id = i;                //  Use index as ID
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.lifetime = ros::Duration(0);  //  Keep until updated (no auto-expire)
        
        if (path.is_best) {
          line_marker.scale.x = 0.08;
          line_marker.color.r = 1.0;
          line_marker.color.g = 0.84;
          line_marker.color.b = 0.0;
          line_marker.color.a = 1.0;
        } else {
          line_marker.scale.x = 0.05;
          auto& color = colors[i % colors.size()];
          line_marker.color.r = color[0];
          line_marker.color.g = color[1];
          line_marker.color.b = color[2];
          line_marker.color.a = 0.7;
        }
        
        // Sample B-spline at high resolution for smooth curve
        for (double t = 0.0; t <= duration; t += 0.02) {  // Every 0.02s = 50Hz
          Eigen::VectorXd pt = bspline.evaluateDeBoorT(t);
          geometry_msgs::Point p;
          p.x = pt(0);
          p.y = pt(1);
          p.z = pt(2);
          line_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
        
        // Add text label
        visualization_msgs::Marker text_marker;
        text_marker.header = line_marker.header;
        text_marker.ns = "mppi_label";  //  Fixed namespace
        text_marker.id = i;              //  Use index as ID
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.lifetime = ros::Duration(0);  //  Keep until updated
        
        text_marker.pose.position.x = mppi_points[0].x();
        text_marker.pose.position.y = mppi_points[0].y();
        text_marker.pose.position.z = mppi_points[0].z() + 0.5;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.25;
        text_marker.color = line_marker.color;
        
        std::stringstream ss;
        if (path.is_best) {
          ss << " #" << i+1 << " \n" << std::fixed << std::setprecision(2) << path.normalized_cost;
        } else {
          ss << "#" << i+1 << "\n" << std::fixed << std::setprecision(2) << path.normalized_cost;
        }
        text_marker.text = ss.str();
        
        marker_array.markers.push_back(text_marker);
        
      } catch (const std::exception& e) {
        ROS_WARN("[PlannerManager] Failed to create B-spline for path #%zu: %s", i, e.what());
        continue;
      }
    }
    
    all_mppi_paths_pub_.publish(marker_array);
    ROS_DEBUG("[PlannerManager] Published %zu MPPI candidate paths for visualization", all_mppi_paths_.size());
  }

  //  NEW: Visualize topological paths with B-spline smoothing
  void EGOPlannerManager::visualizeTopoPathsSmooth(const std::vector<TopoPath> &topo_paths) {
    if (topo_paths.empty()) {
      ROS_WARN("[PlannerManager] No topo paths to visualize smooth");
      return;
    }
    
    ROS_INFO("[PlannerManager]  Visualizing %zu smooth B-spline topo paths", topo_paths.size());
    
    visualization_msgs::MarkerArray marker_array;
    
    // 清除旧的markers
    for (int i = 0; i < 10; ++i) {
      visualization_msgs::Marker delete_line, delete_text;
      delete_line.header.frame_id = "world";
      delete_line.header.stamp = ros::Time::now();
      delete_line.ns = "topo_bspline";
      delete_line.id = i;
      delete_line.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(delete_line);
      
      delete_text.header = delete_line.header;
      delete_text.ns = "topo_label";
      delete_text.id = i;
      delete_text.action = visualization_msgs::Marker::DELETE;
      marker_array.markers.push_back(delete_text);
    }
    
    //  Fast-Planner渐变色 (与/topo_paths一致)
    auto getColor = [](double h) -> std::array<float, 3> {
      if (h < 0.0 || h > 1.0) h = 0.0;
      double lambda;
      std::array<float, 3> color1, color2;
      
      if (h >= -1e-4 && h < 1.0 / 6) {
        lambda = (h - 0.0) * 6;
        color1 = {1.0f, 0.0f, 0.0f}; color2 = {1.0f, 0.0f, 1.0f};
      } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
        lambda = (h - 1.0 / 6) * 6;
        color1 = {1.0f, 0.0f, 1.0f}; color2 = {0.0f, 0.0f, 1.0f};
      } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
        lambda = (h - 2.0 / 6) * 6;
        color1 = {0.0f, 0.0f, 1.0f}; color2 = {0.0f, 1.0f, 1.0f};
      } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
        lambda = (h - 3.0 / 6) * 6;
        color1 = {0.0f, 1.0f, 1.0f}; color2 = {0.0f, 1.0f, 0.0f};
      } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
        lambda = (h - 4.0 / 6) * 6;
        color1 = {0.0f, 1.0f, 0.0f}; color2 = {1.0f, 1.0f, 0.0f};
      } else {
        lambda = (h - 5.0 / 6) * 6;
        color1 = {1.0f, 1.0f, 0.0f}; color2 = {1.0f, 0.0f, 0.0f};
      }
      
      return {
        float((1 - lambda) * color1[0] + lambda * color2[0]),
        float((1 - lambda) * color1[1] + lambda * color2[1]),
        float((1 - lambda) * color1[2] + lambda * color2[2])
      };
    };
    
    double ts = 0.1;  // B-spline时间步长
    
    for (size_t i = 0; i < topo_paths.size() && i < 10; ++i) {
      const auto& path = topo_paths[i].path;
      
      ROS_INFO("[PlannerManager]   Path #%zu: %zu points", i, path.size());
      
      //  如果点数太少,插值增加点数
      std::vector<Eigen::Vector3d> dense_path;
      if (path.size() < 7) {
        // 线性插值增加点数
        for (size_t j = 0; j < path.size() - 1; ++j) {
          dense_path.push_back(path[j]);
          // 在每两点间插入2个点
          Eigen::Vector3d delta = (path[j+1] - path[j]) / 3.0;
          dense_path.push_back(path[j] + delta);
          dense_path.push_back(path[j] + 2.0 * delta);
        }
        dense_path.push_back(path.back());
        ROS_INFO("[PlannerManager]     Densified: %zu → %zu points", path.size(), dense_path.size());
      } else {
        dense_path = path;
      }
      
      // 确保至少有7个点
      if (dense_path.size() < 7) {
        ROS_WARN("[PlannerManager]   Path #%zu still too short (%zu points), drawing line", i, dense_path.size());
        // 直接画折线
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "world";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "topo_bspline";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.lifetime = ros::Duration(0);
        line_marker.scale.x = 0.10;
        
        auto color = getColor(double(i) / topo_paths.size());
        line_marker.color.r = color[0];
        line_marker.color.g = color[1];
        line_marker.color.b = color[2];
        line_marker.color.a = 0.9;
        
        for (const auto& pt : dense_path) {
          geometry_msgs::Point p;
          p.x = pt.x();
          p.y = pt.y();
          p.z = pt.z();
          line_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
        continue;
      }
      
      try {
        // 准备导数 (端点速度和加速度为零)
        std::vector<Eigen::Vector3d> start_end_derivative(4, Eigen::Vector3d::Zero());
        
        // 创建B-spline控制点
        Eigen::MatrixXd bspline_ctrl_pts;
        UniformBspline::parameterizeToBspline(ts, dense_path, start_end_derivative, bspline_ctrl_pts);
        
        // 创建B-spline对象
        UniformBspline bspline(bspline_ctrl_pts, 3, ts);
        double duration = bspline.getTimeSum();
        
        ROS_INFO("[PlannerManager]     B-spline created: duration=%.2fs", duration);
        
        // 创建marker
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "world";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "topo_bspline";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.lifetime = ros::Duration(0);
        
        //  Fast-Planner渐变色 (与/topo_paths一致)
        auto color = getColor(double(i) / topo_paths.size());
        line_marker.scale.x = 0.12;  // 稍粗,便于看清
        line_marker.color.r = color[0];
        line_marker.color.g = color[1];
        line_marker.color.b = color[2];
        line_marker.color.a = 0.6;  // 半透明,与原始路径区分
        
        //  高密度采样B-spline (100Hz = 0.01s间隔)
        int sample_count = 0;
        for (double t = 0.0; t <= duration; t += 0.01) {
          Eigen::VectorXd pt = bspline.evaluateDeBoorT(t);
          geometry_msgs::Point p;
          p.x = pt(0);
          p.y = pt(1);
          p.z = pt(2);
          line_marker.points.push_back(p);
          sample_count++;
        }
        
        ROS_INFO("[PlannerManager]     Sampled %d points from B-spline", sample_count);
        
        marker_array.markers.push_back(line_marker);
        
        //  添加成本标签 (起点上方)
        visualization_msgs::Marker text_marker;
        text_marker.header = line_marker.header;
        text_marker.ns = "topo_label";
        text_marker.id = i;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.lifetime = ros::Duration(0);
        
        text_marker.pose.position.x = dense_path[0].x();
        text_marker.pose.position.y = dense_path[0].y();
        text_marker.pose.position.z = dense_path[0].z() + 0.8;  // 稍高一点
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.35;  // 稍大的文字
        text_marker.color.r = color[0];
        text_marker.color.g = color[1];
        text_marker.color.b = color[2];
        text_marker.color.a = 1.0;
        
        std::stringstream ss;
        if (i == 0) {
          ss << " Path #" << i+1 << " \nCost: " << std::fixed << std::setprecision(1) << topo_paths[i].cost;
        } else {
          ss << "Path #" << i+1 << "\nCost: " << std::fixed << std::setprecision(1) << topo_paths[i].cost;
        }
        text_marker.text = ss.str();
        
        marker_array.markers.push_back(text_marker);
        
      } catch (const std::exception& e) {
        ROS_ERROR("[PlannerManager] Failed to create B-spline for path #%zu: %s", i, e.what());
        continue;
      }
    }
    
    topo_paths_smooth_pub_.publish(marker_array);
  }

} // namespace ego_planner
