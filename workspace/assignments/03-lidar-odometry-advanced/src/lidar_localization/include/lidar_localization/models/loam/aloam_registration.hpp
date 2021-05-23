/*
 * @Description: LOAM scan registration, interface
 * @Author: Ge Yao
 * @Date: 2021-05-04 14:53:21
 */

#ifndef LIDAR_LOCALIZATION_MODELS_ALOAM_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_ALOAM_REGISTRATION_HPP_

#include <memory>

#include <string>

#include <vector>
#include <deque>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ceres/ceres.h>

namespace lidar_localization {

class CeresALOAMRegistration {
public:
    class Config {
    public:
        int num_threads{4};
        int max_num_iterations{5};

        double max_solver_time_in_seconds{0.10};

        Config(void) {}

        Config& set_num_threads(const int num_threads) {
            this->num_threads = num_threads;
            return *this;
        }

        Config& set_max_num_iterations(const int max_num_iterations) {
            this->max_num_iterations = max_num_iterations;
            return *this;
        }

        Config& set_max_solver_time_in_seconds(const double& max_solver_time_in_seconds) {
            this->max_solver_time_in_seconds = max_solver_time_in_seconds;
            return *this;
        }
    };

    CeresALOAMRegistration(const Config& config, const Eigen::Quaterniond &dq, const Eigen::Vector3d &dt);
    ~CeresALOAMRegistration();

    /**
     * @brief  add residual block for edge constraint from lidar frontend
     * @param  source, source point
     * @param  target_x, target point x
     * @param  target_y, target point y
     * @param  ratio, interpolation ratio
     * @return void
     */
    bool AddEdgeFactor(
      const Eigen::Vector3d &source,
      const Eigen::Vector3d &target_x, const Eigen::Vector3d &target_y,
      const double &ratio
    );

    /**
     * @brief  add residual block for plane constraint from lidar frontend
     * @param  source, source point
     * @param  target_x, target point x
     * @param  target_y, target point y
     * @param  target_z, target point z
     * @param  ratio, interpolation ratio
     * @return void
     */
    bool AddPlaneFactor(
      const Eigen::Vector3d &source,
      const Eigen::Vector3d &target_x, const Eigen::Vector3d &target_y, const Eigen::Vector3d &target_z,
      const double &ratio
    );

    /**
     * @brief  add residual block for plane constraint from lidar frontend
     * @param  source, source point
     * @param  norm, normal direction of target plane
     * @param  negative_oa_dot_norm
     * @return void
     */
    bool AddPlaneNormFactor(
      const Eigen::Vector3d &source,
      const Eigen::Vector3d &norm, const double &negative_oa_dot_norm
    );

    // do optimization
    bool Optimize();

    /**
     * @brief  get optimized relative pose
     * @return true if success false otherwise
     */
    bool GetOptimizedRelativePose(Eigen::Quaterniond &dq, Eigen::Vector3d &dt);

private:
    // optimizer config:
    struct {
      // 1. quaternion parameterization:
      ceres::LocalParameterization *q_parameterization_ptr{nullptr};
      // 2. loss function:
      ceres::LossFunction *loss_function_ptr{nullptr};
      // 3. solver:
      ceres::Solver::Options options;
    } config_;

    // target variables:
    struct {
      double q[4] = {0.0, 0.0, 0.0, 1.0};
      double t[3] = {0.0, 0.0, 0.0};
    } param_;

    // problem:
    ceres::Problem problem_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_ALOAM_REGISTRATION_HPP_