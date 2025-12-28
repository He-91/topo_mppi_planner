#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <topo_mppi_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <path_searching/topo_prm.h>
#include <path_searching/mppi_planner.h>
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
    
    /* 🎨 NEW: Visualization data structures and access */
    struct MPPIPathCandidate {
        std::vector<Eigen::Vector3d> positions;
        double cost;
        double normalized_cost;
        bool is_best;
        bool success;
    };
    
    /* Get all MPPI-optimized paths for visualization */
    const std::vector<MPPIPathCandidate>& getAllMPPIPaths() const { return all_mppi_paths_; }
    
    /* 🔥 P2: Trajectory tracking reference structure */
    struct TrajectoryReference {
        std::vector<Eigen::Vector3d> positions;   // 30-point trajectory positions
        std::vector<Eigen::Vector3d> velocities;  // 30-point trajectory velocities
        std::vector<double> timestamps;           // Relative timestamps [0, dt, 2*dt, ...]
        double start_time;                        // Absolute start time (ros::Time)
        bool valid;                               // Is reference valid?
        
        TrajectoryReference() : start_time(0.0), valid(false) {}
        
        void clear() {
            positions.clear();
            velocities.clear();
            timestamps.clear();
            start_time = 0.0;
            valid = false;
        }
    };
    
    /* 🔥 P2: Trajectory tracking error feedback */
    bool computeTrackingError(const Eigen::Vector3d& current_pos, 
                             const Eigen::Vector3d& current_vel,
                             double current_time,
                             Eigen::Vector3d& pos_error,
                             Eigen::Vector3d& vel_error);
    
    /* 🔥 P2: PD feedback correction */
    Eigen::Vector3d feedbackCorrection(const Eigen::Vector3d& pos_error,
                                      const Eigen::Vector3d& vel_error);
    
    /* 🔥 P2: Check if replanning is needed based on tracking error */
    bool shouldReplanFromError(const Eigen::Vector3d& pos_error, double threshold = 0.5);

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
    
    /* 🚀 IMPROVED: Backup MPPI result for B-spline fallback */
    MPPITrajectory mppi_result_backup_;
    
    /* 🎨 NEW: Store all MPPI-optimized paths for visualization */
    std::vector<MPPIPathCandidate> all_mppi_paths_;
    ros::Publisher all_mppi_paths_pub_;  // MPPI候选路径可视化
    ros::Publisher topo_paths_smooth_pub_;  // 🎨 NEW: 拓扑路径B-spline平滑可视化
    
    /* 🔥 P2: Trajectory reference for tracking error feedback */
    TrajectoryReference trajectory_reference_;
    
    /* 🔥 P2: PD feedback parameters */
    double feedback_kp_;  // Position gain
    double feedback_kd_;  // Velocity gain
    double feedback_max_correction_;  // Max correction (m/s²)
    double replan_error_threshold_;   // Replan threshold (m)

    int continous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    /* 🎨 Visualize all MPPI candidate paths */
    void visualizeAllMPPIPaths();
    
    /* 🎨 NEW: Visualize topological paths with B-spline smoothing */
    void visualizeTopoPathsSmooth(const std::vector<TopoPath> &topo_paths);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif