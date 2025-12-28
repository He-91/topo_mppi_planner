#ifndef _MPPI_SAMPLING_H_
#define _MPPI_SAMPLING_H_

#include <Eigen/Eigen>
#include <random>
#include <vector>
#include <memory>
#include <thread>  // ✨ For thread_local and std::this_thread

namespace ego_planner {

/**
 * @brief Sampling distribution for MPPI control noise generation
 * Inspired by MPPI-Generic's sampling_distribution interface
 */
class MPPISampling {
public:
    MPPISampling() 
        : sigma_acc_(1.0), use_colored_noise_(false),
          temporal_correlation_(0.5) {
    }
    
    /**
     * @brief Initialize random number generator
     * @param seed Random seed (0 for random device)
     */
    void initialize(unsigned int seed = 0) {
        if (seed == 0) {
            std::random_device rd;
            generator_.seed(rd());
        } else {
            generator_.seed(seed);
        }
        normal_dist_ = std::normal_distribution<double>(0.0, 1.0);
    }
    
    /**
     * @brief Sample control sequence with Gaussian noise
     * @param horizon_steps Number of timesteps
     * @param control_seq Output: sampled control sequence
     */
    void sampleGaussian(int horizon_steps,
                       std::vector<Eigen::Vector3d>& control_seq) {
        control_seq.resize(horizon_steps);
        for (int t = 0; t < horizon_steps; ++t) {
            control_seq[t] = Eigen::Vector3d(
                sigma_acc_ * normal_dist_(generator_),
                sigma_acc_ * normal_dist_(generator_),
                sigma_acc_ * normal_dist_(generator_)
            );
        }
    }
    
    /**
     * @brief Sample control sequence with colored (temporally correlated) noise
     * Inspired by MPPI-Generic's ColoredNoise distribution
     * @param horizon_steps Number of timesteps
     * @param control_seq Output: sampled control sequence
     */
    void sampleColoredNoise(int horizon_steps,
                           std::vector<Eigen::Vector3d>& control_seq) {
        control_seq.resize(horizon_steps);
        
        if (horizon_steps == 0) return;
        
        // First sample is pure Gaussian
        control_seq[0] = Eigen::Vector3d(
            sigma_acc_ * normal_dist_(generator_),
            sigma_acc_ * normal_dist_(generator_),
            sigma_acc_ * normal_dist_(generator_)
        );
        
        // Subsequent samples are correlated with previous
        // u_t = α * u_{t-1} + √(1 - α²) * ε_t
        double alpha = temporal_correlation_;
        double noise_scale = std::sqrt(1.0 - alpha * alpha);
        
        for (int t = 1; t < horizon_steps; ++t) {
            Eigen::Vector3d white_noise(
                sigma_acc_ * normal_dist_(generator_),
                sigma_acc_ * normal_dist_(generator_),
                sigma_acc_ * normal_dist_(generator_)
            );
            
            control_seq[t] = alpha * control_seq[t-1] + noise_scale * white_noise;
        }
    }
    
    /**
     * @brief Sample control with guidance from nominal control
     * @param nominal_control Nominal control to guide sampling
     * @param control Output: sampled control
     */
    void sampleGuidedControl(const Eigen::Vector3d& nominal_control,
                            Eigen::Vector3d& control) {
        Eigen::Vector3d noise(
            sigma_acc_ * normal_dist_(generator_),
            sigma_acc_ * normal_dist_(generator_),
            sigma_acc_ * normal_dist_(generator_)
        );
        control = nominal_control + noise;
    }
    
    /**
     * @brief Thread-safe version: Sample control with guidance using local generator
     * @param nominal_control Nominal control to guide sampling
     * @param control Output: sampled control
     * @param local_gen Thread-local random generator
     * @param local_dist Thread-local normal distribution
     */
    void sampleGuidedControl(const Eigen::Vector3d& nominal_control,
                            Eigen::Vector3d& control,
                            std::mt19937& local_gen,
                            std::normal_distribution<double>& local_dist) {
        Eigen::Vector3d noise(
            sigma_acc_ * local_dist(local_gen),
            sigma_acc_ * local_dist(local_gen),
            sigma_acc_ * local_dist(local_gen)
        );
        control = nominal_control + noise;
    }
    
    /**
     * @brief Compute likelihood ratio cost (for importance sampling)
     * This is used in the rollout cost calculation
     * @param sampled_control The sampled control
     * @param nominal_control The nominal control
     * @return Likelihood ratio cost
     */
    double computeLikelihoodRatioCost(const Eigen::Vector3d& sampled_control,
                                      const Eigen::Vector3d& nominal_control) const {
        // For Gaussian: φ(u) ∝ ||u - u_nom||²
        Eigen::Vector3d diff = sampled_control - nominal_control;
        return 0.5 * diff.squaredNorm() / (sigma_acc_ * sigma_acc_);
    }
    
    // Setters
    void setSigma(double sigma) { sigma_acc_ = sigma; }
    void setUseColoredNoise(bool use) { use_colored_noise_ = use; }
    void setTemporalCorrelation(double alpha) { 
        temporal_correlation_ = std::max(0.0, std::min(1.0, alpha));
    }
    
    // Getters
    double getSigma() const { return sigma_acc_; }
    bool getUseColoredNoise() const { return use_colored_noise_; }
    double getTemporalCorrelation() const { return temporal_correlation_; }
    std::mt19937& getGenerator() { return generator_; }
    
private:
    double sigma_acc_;              // Control noise standard deviation
    bool use_colored_noise_;        // Use temporally correlated noise
    double temporal_correlation_;   // Temporal correlation coefficient α ∈ [0,1]
    
    std::mt19937 generator_;
    std::normal_distribution<double> normal_dist_;
};

/**
 * @brief Thread-safe sampling for parallel rollouts
 */
class MPPISamplingThreadSafe {
public:
    MPPISamplingThreadSafe(const MPPISampling& base_sampler)
        : base_sampler_(base_sampler) {}
    
    /**
     * @brief Get thread-local sampler
     */
    MPPISampling& getThreadSampler() {
        thread_local MPPISampling sampler = base_sampler_;
        thread_local bool initialized = false;
        
        if (!initialized) {
            // Initialize with thread-specific seed
            sampler.initialize(std::hash<std::thread::id>{}(std::this_thread::get_id()));
            initialized = true;
        }
        
        return sampler;
    }
    
private:
    MPPISampling base_sampler_;
};

} // namespace ego_planner

#endif // _MPPI_SAMPLING_H_
