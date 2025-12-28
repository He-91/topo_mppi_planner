#ifndef _MPPI_DYNAMICS_H_
#define _MPPI_DYNAMICS_H_

#include <Eigen/Eigen>
#include <cmath>

namespace ego_planner {

/**
 * @brief Simplified quadrotor dynamics for MPPI
 * State: [px, py, pz, vx, vy, vz] (6D)
 * Control: [ax, ay, az] (3D)
 * 
 * Inspired by MPPI-Generic's dynamics interface
 */
class SimplifiedQuadrotorDynamics {
public:
    static constexpr int STATE_DIM = 6;
    static constexpr int CONTROL_DIM = 3;
    static constexpr int POS_X = 0, POS_Y = 1, POS_Z = 2;
    static constexpr int VEL_X = 3, VEL_Y = 4, VEL_Z = 5;
    static constexpr int ACC_X = 0, ACC_Y = 1, ACC_Z = 2;
    
    SimplifiedQuadrotorDynamics() 
        : max_velocity_(3.0), max_acceleration_(3.0) {}
    
    /**
     * @brief Compute state derivative: ẋ = f(x, u)
     * @param state Current state [pos, vel]
     * @param control Control input [acc]
     * @param state_dot Output: state derivative
     */
    void computeDynamics(const Eigen::VectorXd& state,
                        const Eigen::Vector3d& control,
                        Eigen::VectorXd& state_dot) const {
        state_dot.resize(STATE_DIM);
        
        // Position derivative: ẋ = v
        state_dot(POS_X) = state(VEL_X);
        state_dot(POS_Y) = state(VEL_Y);
        state_dot(POS_Z) = state(VEL_Z);
        
        // Velocity derivative: v̇ = a
        state_dot(VEL_X) = control(ACC_X);
        state_dot(VEL_Y) = control(ACC_Y);
        state_dot(VEL_Z) = control(ACC_Z);
    }
    
    /**
     * @brief Enforce control constraints
     * @param control Control input (will be modified in-place)
     */
    void enforceConstraints(Eigen::Vector3d& control) const {
        // Limit acceleration magnitude
        double acc_mag = control.norm();
        if (acc_mag > max_acceleration_) {
            control *= (max_acceleration_ / acc_mag);
        }
    }
    
    /**
     * @brief Integrate dynamics using Euler method
     * @param state Current state
     * @param control Control input
     * @param dt Time step
     * @param next_state Output: next state
     */
    void step(const Eigen::VectorXd& state,
             const Eigen::Vector3d& control,
             double dt,
             Eigen::VectorXd& next_state) const {
        Eigen::VectorXd state_dot;
        computeDynamics(state, control, state_dot);
        next_state = state + dt * state_dot;
        
        // Enforce velocity constraints on next state
        for (int i = VEL_X; i <= VEL_Z; ++i) {
            if (next_state(i) > max_velocity_) next_state(i) = max_velocity_;
            if (next_state(i) < -max_velocity_) next_state(i) = -max_velocity_;
        }
    }
    
    // Setters for constraints
    void setMaxVelocity(double v) { max_velocity_ = v; }
    void setMaxAcceleration(double a) { max_acceleration_ = a; }
    
    // Getters
    double getMaxVelocity() const { return max_velocity_; }
    double getMaxAcceleration() const { return max_acceleration_; }
    
private:
    double max_velocity_;
    double max_acceleration_;
};

} // namespace ego_planner

#endif // _MPPI_DYNAMICS_H_
