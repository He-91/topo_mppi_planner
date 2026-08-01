
#include <ddo_planner/ego_replan_fsm.h>

#ifdef USE_GPU_MPPI
#include <cuda_runtime.h>
#endif

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
#ifdef USE_GPU_MPPI
    //  CRITICAL: Initialize CUDA runtime at the VERY BEGINNING before ANY large memory allocations
    ROS_INFO("[FSM] Initializing CUDA runtime early (cudaFree(0))...");
    cudaError_t err = cudaFree(0);
    if (err != cudaSuccess) {
      ROS_WARN("[FSM] cudaFree(0) failed: %s", cudaGetErrorString(err));
    } else {
      ROS_INFO("[FSM] CUDA runtime initialized successfully at FSM init");
    }
#endif

    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /* V15 Stuck detection init */
    stuck_check_time_ = ros::Time::now();
    stuck_check_pos_ = Eigen::Vector3d::Zero();
    consecutive_retreats_ = 0;
    retreat_mode_ = false;
    z_recovery_active_ = false;
    z_recovery_until_ = ros::Time(0);

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/goal_reached_thresh", goal_reached_thresh_, goal_reached_thresh_);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    // Z-axis safety parameters
    nh.param("fsm/min_z", min_z_, 0.3);
    nh.param("fsm/max_z", max_z_, 4.5);
    nh.param("fsm/max_vz", max_vz_, 2.0);
    nh.param("fsm/recovery_grace_time", recovery_grace_time_, 2.0);
    nh.param("fsm/dynamic_exec_safety_enabled", dynamic_exec_safety_enabled_, dynamic_exec_safety_enabled_);
    nh.param("fsm/dynamic_exec_safety_distance", dynamic_exec_safety_distance_, dynamic_exec_safety_distance_);
    nh.param("fsm/dynamic_exec_safety_horizon", dynamic_exec_safety_horizon_, dynamic_exec_safety_horizon_);
    nh.param("fsm/dynamic_exec_safety_step", dynamic_exec_safety_step_, dynamic_exec_safety_step_);
    nh.param("fsm/dynamic_exec_emergency_time", dynamic_exec_emergency_time_, dynamic_exec_emergency_time_);
    nh.param("fsm/dynamic_exec_replan_cooldown", dynamic_exec_replan_cooldown_, dynamic_exec_replan_cooldown_);
    nh.param("fsm/stuck_check_interval", stuck_check_interval_, stuck_check_interval_);
    nh.param("fsm/stuck_disp_threshold", stuck_disp_threshold_, stuck_disp_threshold_);
    nh.param("fsm/retreat_distance", retreat_distance_, retreat_distance_);
    nh.param("fsm/max_consecutive_retreats", max_consecutive_retreats_, max_consecutive_retreats_);
    stuck_check_interval_ = std::max(1.0, stuck_check_interval_);
    stuck_disp_threshold_ = std::max(0.05, stuck_disp_threshold_);
    retreat_distance_ = std::max(0.5, retreat_distance_);
    max_consecutive_retreats_ = std::max(0, max_consecutive_retreats_);
    dynamic_exec_safety_distance_ = std::max(0.05, dynamic_exec_safety_distance_);
    dynamic_exec_safety_horizon_ = std::max(0.1, dynamic_exec_safety_horizon_);
    dynamic_exec_safety_step_ = std::max(0.02, dynamic_exec_safety_step_);
    dynamic_exec_emergency_time_ = std::max(0.05, dynamic_exec_emergency_time_);
    dynamic_exec_replan_cooldown_ = std::max(0.02, dynamic_exec_replan_cooldown_);
    last_dynamic_emergency_time_ = ros::Time(0);
    last_dynamic_safety_replan_time_ = ros::Time(0);
    ROS_INFO("[FSM] Z-axis safety: min_z=%.2f, max_z=%.2f, max_vz=%.2f, recovery_grace=%.2fs",
             min_z_, max_z_, max_vz_, recovery_grace_time_);
    ROS_INFO("[FSM] Dynamic execution safety: %s distance=%.2fm horizon=%.2fs step=%.2fs emergency_time=%.2fs cooldown=%.2fs",
             dynamic_exec_safety_enabled_ ? "ON" : "OFF",
             dynamic_exec_safety_distance_, dynamic_exec_safety_horizon_,
             dynamic_exec_safety_step_, dynamic_exec_emergency_time_,
             dynamic_exec_replan_cooldown_);
    ROS_INFO("[FSM] Stuck recovery: interval=%.1fs disp=%.2fm retreat=%.2fm max_retreats=%d",
             stuck_check_interval_, stuck_disp_threshold_, retreat_distance_, max_consecutive_retreats_);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    // Increased FSM execution frequency from 100Hz to 150Hz for better responsiveness
    exec_timer_ = nh.createTimer(ros::Duration(0.00667), &EGOReplanFSM::execFSMCallback, this);  // 150Hz
    // Increased collision check frequency from 20Hz to 50Hz for better safety
    safety_timer_ = nh.createTimer(ros::Duration(0.02), &EGOReplanFSM::checkCollisionCallback, this);  // 50Hz

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ddo_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ddo_planner::DataDisp>("/planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    const bool recovery_grace_active = z_recovery_active_ && ros::Time::now() < z_recovery_until_;
    if (z_recovery_active_ && !recovery_grace_active) {
      z_recovery_active_ = false;
    }

    // Z-axis safety check
    const bool low_z_violation = odom_pos_(2) < min_z_;
    const bool z_violation = (!recovery_grace_active && low_z_violation) ||
                             odom_pos_(2) > max_z_ ||
                             std::abs(odom_vel_(2)) > max_vz_ * 1.5;
    if (z_violation) {
      ROS_ERROR_THROTTLE(0.5, "[FSM] Z-AXIS VIOLATION: z=%.2f (bounds: [%.2f, %.2f]), vz=%.2f (limit: %.2f)",
                         odom_pos_(2), min_z_, max_z_, odom_vel_(2), max_vz_);
      if (exec_state_ != EMERGENCY_STOP && exec_state_ != INIT) {
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "Z_SAFETY");
      }
    } else if (recovery_grace_active && low_z_violation) {
      ROS_WARN_THROTTLE(0.5, "[FSM] Z recovery grace active: allowing climb from z=%.2f", odom_pos_(2));
    }

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      /* V15: If close enough to goal and planning keeps failing, declare arrival */
      {
        auto consecutive = timesOfConsecutiveStateCalls();
        const double goal_reached_thresh =
            std::min(no_replan_thresh_, goal_reached_thresh_);
        double dist_to_goal = (odom_pos_ - end_pt_).norm();
        if (consecutive.first >= 10 && dist_to_goal < goal_reached_thresh)
        {
          ROS_WARN("[FSM] GEN_NEW_TRAJ failed %d times but is already at goal (dist=%.2fm < %.1fm). Declaring arrival.",
                   consecutive.first, dist_to_goal, goal_reached_thresh);
          have_target_ = false;

          /* If in retreat mode, restore original goal and continue */
          if (retreat_mode_) {
            end_pt_ = retreat_original_end_pt_;
            end_vel_ = retreat_original_end_vel_;
            retreat_mode_ = false;
            have_target_ = true;
            have_new_target_ = true;
            bool success = planner_manager_->planGlobalTraj(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                            end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            (void)success;
            stuck_check_time_ = ros::Time::now();
            stuck_check_pos_ = odom_pos_;
            changeFSMExecState(GEN_NEW_TRAJ, "RETREAT_NEAR_DONE");
            break;
          }

          changeFSMExecState(WAIT_TARGET, "NEAR_GOAL");
          break;
        }
      }

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        if ((odom_pos_ - stuck_check_pos_).norm() > stuck_disp_threshold_) {
          stuck_check_time_ = ros::Time::now();
          stuck_check_pos_ = odom_pos_;
        }
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* --- V15 Stuck detection & retreat recovery --- */
      double dt_stuck = (time_now - stuck_check_time_).toSec();
      if (dt_stuck >= stuck_check_interval_)
      {
        double displacement = (odom_pos_ - stuck_check_pos_).norm();
        if (displacement < stuck_disp_threshold_)
        {
          consecutive_retreats_++;
          ROS_WARN("[FSM] STUCK DETECTED: displacement=%.2fm in %.1fs (threshold=%.1fm). Retreat #%d",
                   displacement, dt_stuck, stuck_disp_threshold_, consecutive_retreats_);

          if (consecutive_retreats_ <= max_consecutive_retreats_)
          {
            // Compute retreat direction: opposite of (current → goal)
            Eigen::Vector3d to_goal = (end_pt_ - odom_pos_);
            to_goal.z() = 0.0; // stay at same altitude
            double to_goal_norm = to_goal.head<2>().norm();

            Eigen::Vector3d retreat_dir;
            if (to_goal_norm > 0.1) {
              retreat_dir = -to_goal.normalized();
            } else {
              // If very close to goal but somehow stuck, retreat towards init point
              retreat_dir = (init_pt_ - odom_pos_).normalized();
            }
            retreat_dir.z() = 0.0;

            // Compute retreat sub-goal
            Eigen::Vector3d retreat_target = odom_pos_ + retreat_dir * retreat_distance_;
            retreat_target.z() = odom_pos_.z(); // maintain altitude

            // Save original goal if not already in retreat mode
            if (!retreat_mode_) {
              retreat_original_end_pt_ = end_pt_;
              retreat_original_end_vel_ = end_vel_;
            }
            retreat_mode_ = true;

            // Override global trajectory to retreat point, then GEN_NEW_TRAJ
            end_pt_ = retreat_target;
            end_vel_ = Eigen::Vector3d::Zero();
            bool success = planner_manager_->planGlobalTraj(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                            end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            if (success) {
              have_target_ = true;
              have_new_target_ = true;
              ROS_WARN("[FSM] Retreat planned: (%.1f,%.1f) -> (%.1f,%.1f), dist=%.1fm",
                       odom_pos_.x(), odom_pos_.y(), retreat_target.x(), retreat_target.y(), retreat_distance_);
            } else {
              ROS_ERROR("[FSM] Retreat global traj failed; trying GEN_NEW_TRAJ anyway.");
            }

            // Reset stuck check for the retreat phase
            stuck_check_time_ = time_now;
            stuck_check_pos_ = odom_pos_;
            changeFSMExecState(GEN_NEW_TRAJ, "STUCK_RETREAT");
            return;
          }
          else
          {
            ROS_ERROR("[FSM] Max retreats (%d) reached. Attempting EMERGENCY_STOP + fresh restart.",
                      max_consecutive_retreats_);
            // Restore original goal before emergency stop
            if (retreat_mode_) {
              end_pt_ = retreat_original_end_pt_;
              end_vel_ = retreat_original_end_vel_;
              retreat_mode_ = false;
            }
            consecutive_retreats_ = 0;
            changeFSMExecState(EMERGENCY_STOP, "STUCK_GIVEUP");
            return;
          }
        }
        else
        {
          // Making progress — reset retreat counter
          if (consecutive_retreats_ > 0) {
            ROS_INFO("[FSM] Progress restored (disp=%.2fm). Reset retreat counter from %d to 0.",
                     displacement, consecutive_retreats_);
          }
          consecutive_retreats_ = 0;
        }

        // Update stuck checkpoint
        stuck_check_time_ = time_now;
        stuck_check_pos_ = odom_pos_;
      }
      /* --- End V15 Stuck detection --- */

      /* Check if retreat sub-goal reached — restore original goal */
      if (retreat_mode_)
      {
        double dist_to_retreat = (odom_pos_ - end_pt_).head<2>().norm();
        if (dist_to_retreat < 1.5 || t_cur > info->duration_ - 1e-2)
        {
          ROS_WARN("[FSM] Retreat sub-goal reached (dist=%.2f). Restoring original goal (%.1f,%.1f).",
                   dist_to_retreat, retreat_original_end_pt_.x(), retreat_original_end_pt_.y());
          end_pt_ = retreat_original_end_pt_;
          end_vel_ = retreat_original_end_vel_;
          retreat_mode_ = false;

          // Re-plan global trajectory to the original goal
          bool success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
                                                          end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
          if (success) {
            have_target_ = true;
            have_new_target_ = true;
          }

          // Reset stuck check from new position
          stuck_check_time_ = ros::Time::now();
          stuck_check_pos_ = odom_pos_;
          changeFSMExecState(GEN_NEW_TRAJ, "RETREAT_DONE");
          return;
        }
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;

        /* If we just completed a retreat, restore original goal instead of WAIT_TARGET */
        if (retreat_mode_)
        {
          ROS_WARN("[FSM] Retreat traj completed. Restoring original goal (%.1f,%.1f).",
                   retreat_original_end_pt_.x(), retreat_original_end_pt_.y());
          end_pt_ = retreat_original_end_pt_;
          end_vel_ = retreat_original_end_vel_;
          retreat_mode_ = false;

          bool success = planner_manager_->planGlobalTraj(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                          end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
          if (success) {
            have_target_ = true;
            have_new_target_ = true;
          }
          stuck_check_time_ = ros::Time::now();
          stuck_check_pos_ = odom_pos_;
          changeFSMExecState(GEN_NEW_TRAJ, "RETREAT_RESTORE");
          return;
        }

        const double dist_to_goal = (odom_pos_ - end_pt_).norm();
        const double goal_reached_thresh =
            std::min(no_replan_thresh_, goal_reached_thresh_);
        if (dist_to_goal < goal_reached_thresh)
        {
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else
        {
          have_target_ = true;
          ROS_INFO("[FSM] Local trajectory finished but goal remains %.2fm away; replanning.", dist_to_goal);
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory (optimized with early exit) ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    double check_horizon = (t_cur < t_2_3) ? t_2_3 : info->duration_;  // Precompute check horizon

    if (dynamic_exec_safety_enabled_) {
      const double dynamic_horizon =
          std::min(info->duration_, t_cur + dynamic_exec_safety_horizon_);
      double min_dynamic_dist = std::numeric_limits<double>::infinity();
      double min_dynamic_dt = 0.0;
      for (double t = t_cur; t < dynamic_horizon; t += dynamic_exec_safety_step_) {
        const double dt = std::max(0.0, t - t_cur);
        const Eigen::Vector3d p = info->position_traj_.evaluateDeBoorT(t);
        const double dist = planner_manager_->getDynamicSurfaceDistanceForSafety(p, dt);
        if (dist < min_dynamic_dist) {
          min_dynamic_dist = dist;
          min_dynamic_dt = dt;
        }
      }

      if (std::isfinite(min_dynamic_dist) &&
          min_dynamic_dist < dynamic_exec_safety_distance_) {
        ROS_WARN_THROTTLE(0.2,
                          "[FSM] Dynamic execution safety risk: min_dyn=%.2fm at %.2fs < %.2fm",
                          min_dynamic_dist, min_dynamic_dt,
                          dynamic_exec_safety_distance_);
        const ros::Time now = ros::Time::now();
        const bool replan_cooldown_ok =
            (now - last_dynamic_safety_replan_time_).toSec() >
            dynamic_exec_replan_cooldown_;
        if (replan_cooldown_ok) {
          last_dynamic_safety_replan_time_ = now;
          if (planFromCurrentTraj()) {
            changeFSMExecState(EXEC_TRAJ, "DYNAMIC_SAFETY_REPLAN");
            return;
          }
        }

        if (min_dynamic_dt < dynamic_exec_emergency_time_) {
          if ((now - last_dynamic_emergency_time_).toSec() > 0.8) {
            last_dynamic_emergency_time_ = now;
            if (callEmergencyStop(odom_pos_)) {
              changeFSMExecState(EMERGENCY_STOP, "DYNAMIC_SAFETY");
            }
          }
        } else {
          changeFSMExecState(REPLAN_TRAJ, "DYNAMIC_SAFETY");
        }
        return;
      }
    }
    
    for (double t = t_cur; t < check_horizon; t += time_step)
    {
      // Early exit optimization: check collision directly
      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;
      if (planner_manager_->isLastTrajRecovery()) {
        z_recovery_active_ = true;
        z_recovery_until_ = ros::Time::now() + ros::Duration(recovery_grace_time_);
        ROS_WARN("[FSM] Recovery trajectory accepted; z safety low-bound grace enabled for %.2fs",
                 recovery_grace_time_);
      } else {
        z_recovery_active_ = false;
      }

      /* publish traj */
      ddo_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    if (!planner_manager_->EmergencyStop(stop_pos))
    {
      ROS_ERROR("[FSM] Emergency stop trajectory generation failed");
      return false;
    }

    auto info = &planner_manager_->local_data_;
    if (planner_manager_->isLastTrajRecovery()) {
      z_recovery_active_ = true;
      z_recovery_until_ = ros::Time::now() + ros::Duration(recovery_grace_time_);
      ROS_WARN("[FSM] Emergency recovery trajectory published; z safety low-bound grace enabled for %.2fs",
               recovery_grace_time_);
    } else {
      z_recovery_active_ = false;
    }

    /* publish traj */
    ddo_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;

    // BUG FIX: When last_progress_time_ desyncs (drone moved far from global path),
    // scan forward to find the closest point on the global trajectory instead of returning error.
    double scan_start = planner_manager_->global_data_.last_progress_time_;
    Eigen::Vector3d pos_start = planner_manager_->global_data_.getPosition(scan_start);
    double dist_start = (pos_start - start_pt_).norm();
    
    if (dist_start > planning_horizen_) {
      // last_progress_time_ is desynchronized — find closest point on global path
      ROS_WARN("[FSM] last_progress_time_ desync detected (dist=%.2f > horizon=%.2f), rescanning...", 
               dist_start, planning_horizen_);
      double best_t = scan_start;
      double best_dist = dist_start;
      for (double tt = 0.0; tt < planner_manager_->global_data_.global_duration_; tt += t_step) {
        double d = (planner_manager_->global_data_.getPosition(tt) - start_pt_).norm();
        if (d < best_dist) {
          best_dist = d;
          best_t = tt;
        }
      }
      planner_manager_->global_data_.last_progress_time_ = best_t;
      ROS_WARN("[FSM] Resynchronized last_progress_time_ to %.3f (dist=%.2f)", best_t, best_dist);
    }

    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      // BUG FIX: Clamp t to valid range before accessing velocity
      double safe_t = std::min(t, planner_manager_->global_data_.global_duration_ - 1e-3);
      safe_t = std::max(safe_t, 0.0);
      local_target_vel_ = planner_manager_->global_data_.getVelocity(safe_t);
    }
  }

} // namespace ego_planner
