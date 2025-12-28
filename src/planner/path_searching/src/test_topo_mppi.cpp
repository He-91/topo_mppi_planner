#include <ros/ros.h>
#include <path_searching/topo_prm.h>
#include <path_searching/mppi_planner.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/Point.h>

using namespace ego_planner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "topo_mppi_test");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting Topological and MPPI planner test...");
    
    // Create a simple grid map for testing
    GridMap::Ptr grid_map;
    grid_map.reset(new GridMap);
    grid_map->initMap(nh);
    
    // Test TopoPRM
    TopoPRM::Ptr topo_planner;
    topo_planner.reset(new TopoPRM);
    topo_planner->init(nh, grid_map);
    
    Eigen::Vector3d start_pos(0, 0, 1);
    Eigen::Vector3d goal_pos(5, 5, 1);
    
    std::vector<TopoPath> topo_paths;
    bool topo_success = topo_planner->searchTopoPaths(start_pos, goal_pos, topo_paths);
    
    if (topo_success) {
        ROS_INFO("Topological planning succeeded! Found %zu paths", topo_paths.size());
        for (size_t i = 0; i < topo_paths.size(); ++i) {
            ROS_INFO("Path %zu: cost = %f, waypoints = %zu", 
                     i, topo_paths[i].cost, topo_paths[i].path.size());
        }
    } else {
        ROS_WARN("Topological planning failed");
    }
    
    // Test MPPI
    MPPIPlanner::Ptr mppi_planner;
    mppi_planner.reset(new MPPIPlanner);
    mppi_planner->init(nh, grid_map);
    
    Eigen::Vector3d start_vel(0, 0, 0);
    Eigen::Vector3d goal_vel(0, 0, 0);
    
    MPPITrajectory optimal_traj;
    bool mppi_success = mppi_planner->planTrajectory(start_pos, start_vel, goal_pos, goal_vel, optimal_traj);
    
    if (mppi_success) {
        ROS_INFO("MPPI planning succeeded! Trajectory cost: %f, steps: %d", 
                 optimal_traj.cost, optimal_traj.size());
    } else {
        ROS_WARN("MPPI planning failed");
    }
    
    ROS_INFO("Test completed successfully!");
    
    return 0;
}