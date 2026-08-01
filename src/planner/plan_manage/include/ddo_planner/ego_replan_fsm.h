#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <ddo_bspline_opt/bspline_optimizer.h>
#include <ddo_plan_env/grid_map.h>
#include <ddo_planner/Bspline.h>
#include <ddo_planner/DataDisp.h>
#include <ddo_planner/planner_manager.h>
#include <ddo_traj_utils/planning_visualization.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    ddo_planner::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double goal_reached_thresh_{0.9};
    double waypoints_[50][3];
    int waypoint_num_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    // Z-axis safety bounds
    double min_z_, max_z_, max_vz_;
    double recovery_grace_time_;
    bool dynamic_exec_safety_enabled_{false};
    double dynamic_exec_safety_distance_{0.30};
    double dynamic_exec_safety_horizon_{1.0};
    double dynamic_exec_safety_step_{0.05};
    double dynamic_exec_emergency_time_{0.45};
    double dynamic_exec_replan_cooldown_{0.25};
    ros::Time last_dynamic_emergency_time_;
    ros::Time last_dynamic_safety_replan_time_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    int current_wp_;

    bool flag_escape_emergency_;
    bool z_recovery_active_{false};
    ros::Time z_recovery_until_;

    /* Stuck detection & retreat recovery */
    double stuck_check_interval_{15.0};   // seconds between stuck checks
    double stuck_disp_threshold_{1.0};    // min displacement (m) to NOT be stuck
    double retreat_distance_{3.0};        // retreat distance (m)
    int max_consecutive_retreats_{3};     // max retreats before giving up

    ros::Time stuck_check_time_;          // last time we recorded a stuck-check position
    Eigen::Vector3d stuck_check_pos_;     // position at last stuck-check time
    int consecutive_retreats_{0};         // consecutive retreat counter
    bool retreat_mode_{false};            // currently retreating to a sub-goal?
    Eigen::Vector3d retreat_original_end_pt_;  // saved original end_pt_ during retreat
    Eigen::Vector3d retreat_original_end_vel_; // saved original end_vel_ during retreat

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

    bool checkCollision();

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
