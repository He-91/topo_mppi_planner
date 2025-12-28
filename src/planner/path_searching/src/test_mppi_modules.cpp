/**
 * @file test_mppi_modules.cpp
 * @brief Test new modularized MPPI components
 */

#include <path_searching/mppi_dynamics.h>
#include <path_searching/mppi_cost.h>
#include <path_searching/mppi_sampling.h>
#include <iostream>
#include <Eigen/Eigen>

using namespace ego_planner;
using namespace Eigen;

int main(int argc, char** argv) {
    std::cout << "=== Testing MPPI Modularized Components ===" << std::endl;
    
    // Test 1: SimplifiedQuadrotorDynamics
    std::cout << "\n[Test 1] SimplifiedQuadrotorDynamics" << std::endl;
    SimplifiedQuadrotorDynamics dynamics;
    dynamics.setMaxVelocity(3.0);
    dynamics.setMaxAcceleration(3.0);
    
    VectorXd state(6);
    state << 0, 0, 0, 1, 0, 0;  // pos=[0,0,0], vel=[1,0,0]
    
    Vector3d control(2, 0, 0);  // acc=[2,0,0]
    
    VectorXd state_dot;
    dynamics.computeDynamics(state, control, state_dot);
    
    std::cout << "  State: " << state.transpose() << std::endl;
    std::cout << "  Control: " << control.transpose() << std::endl;
    std::cout << "  State_dot: " << state_dot.transpose() << std::endl;
    
    VectorXd next_state;
    dynamics.step(state, control, 0.1, next_state);
    std::cout << "  Next state (dt=0.1): " << next_state.transpose() << std::endl;
    
    std::cout << "  ✅ Dynamics test passed!" << std::endl;
    
    // Test 2: MPPICost
    std::cout << "\n[Test 2] MPPICost" << std::endl;
    MPPICost cost;
    cost.setObstacleWeight(200.0);
    cost.setSmoothnessWeight(10.0);
    cost.setGoalWeight(50.0);
    
    Vector3d pos(0, 0, 0);
    Vector3d vel(1, 0, 0);
    Vector3d acc(0.5, 0, 0);
    Vector3d goal(5, 0, 0);
    
    double running_cost = cost.computeRunningCost(pos, vel, acc, goal, 0, nullptr);
    std::cout << "  Running cost: " << running_cost << std::endl;
    
    double terminal_cost = cost.computeTerminalCost(pos, vel, goal, Vector3d::Zero());
    std::cout << "  Terminal cost: " << terminal_cost << std::endl;
    
    std::cout << "  Weights: Obs=" << cost.getObstacleWeight() 
              << ", Smooth=" << cost.getSmoothnessWeight()
              << ", Goal=" << cost.getGoalWeight() << std::endl;
    
    std::cout << "  ✅ Cost test passed!" << std::endl;
    
    // Test 3: MPPISampling
    std::cout << "\n[Test 3] MPPISampling" << std::endl;
    MPPISampling sampling;
    sampling.setSigma(1.0);
    sampling.initialize(42);  // Fixed seed for reproducibility
    
    std::vector<Vector3d> gaussian_seq;
    sampling.sampleGaussian(5, gaussian_seq);
    
    std::cout << "  Gaussian samples (5 timesteps):" << std::endl;
    for (int i = 0; i < gaussian_seq.size(); ++i) {
        std::cout << "    t=" << i << ": " << gaussian_seq[i].transpose() << std::endl;
    }
    
    sampling.setUseColoredNoise(true);
    sampling.setTemporalCorrelation(0.7);
    
    std::vector<Vector3d> colored_seq;
    sampling.sampleColoredNoise(5, colored_seq);
    
    std::cout << "  Colored noise samples (α=0.7, 5 timesteps):" << std::endl;
    for (int i = 0; i < colored_seq.size(); ++i) {
        std::cout << "    t=" << i << ": " << colored_seq[i].transpose() << std::endl;
    }
    
    std::cout << "  ✅ Sampling test passed!" << std::endl;
    
    // Test 4: Integration test
    std::cout << "\n[Test 4] Integration Test" << std::endl;
    
    // Simulate one timestep with all modules
    Vector3d sampled_control;
    Vector3d nominal_control(1.0, 0, 0);
    sampling.sampleGuidedControl(nominal_control, sampled_control);
    
    dynamics.enforceConstraints(sampled_control);
    
    VectorXd integrated_state;
    dynamics.step(state, sampled_control, 0.1, integrated_state);
    
    Vector3d int_pos = integrated_state.head<3>();
    Vector3d int_vel = integrated_state.tail<3>();
    
    double int_cost = cost.computeRunningCost(int_pos, int_vel, sampled_control, goal, 1, nullptr);
    
    std::cout << "  Nominal control: " << nominal_control.transpose() << std::endl;
    std::cout << "  Sampled control: " << sampled_control.transpose() << std::endl;
    std::cout << "  Integrated state: " << integrated_state.transpose() << std::endl;
    std::cout << "  Cost: " << int_cost << std::endl;
    
    std::cout << "  ✅ Integration test passed!" << std::endl;
    
    std::cout << "\n=== All Tests Passed! ===" << std::endl;
    
    return 0;
}
