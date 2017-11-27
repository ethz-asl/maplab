#ifndef MAP_OPTIMIZATION_LEGACY_TEST_6DOF_TEST_TRAJECTORY_GEN_H_
#define MAP_OPTIMIZATION_LEGACY_TEST_6DOF_TEST_TRAJECTORY_GEN_H_

#include <memory>
#include <string>

#include <Eigen/Core>

#include <imu-integrator/imu-integrator.h>
#include <simulation/generic-path-generator.h>

namespace map_optimization_legacy {

class SixDofTestTrajectoryGenerator {
 public:
  // Continuous time sigmas: they will be converted to discrete time
  // counterparts inside path generation routine.
  SixDofTestTrajectoryGenerator(
      const test_trajectory_gen::PathAndLandmarkSettings& settings);

  void generate6DofPath();
  void generateLandmarks();

  const Eigen::Matrix3Xd& getTruePositions() const;
  const Eigen::Matrix4Xd& getTrueRotations() const;
  const Eigen::Matrix3Xd& getTrueVelocities() const;
  const Eigen::Matrix3Xd& getTrueAccBias() const;
  const Eigen::Matrix3Xd& getTrueGyroBias() const;
  const Eigen::Matrix<double, imu_integrator::kImuReadingSize, Eigen::Dynamic>&
  getImuData() const;
  const Eigen::Matrix3Xd& getLandmarks() const;
  const Eigen::VectorXd& getTimestampsInSeconds() const;
  void getGroundTruthTransformations(aslam::TransformationVector* T_G_Bs) const;
  void addNoiseToImuData(
      Eigen::Matrix<double, imu_integrator::kImuReadingSize, Eigen::Dynamic>*
          imu_data);
  void getRandomVector3d(
      std::mt19937* generator, std::normal_distribution<>* distribution,
      Eigen::Vector3d* vector);

  void saveTrajectory(const std::string& filename) const;

 private:
  Eigen::Quaterniond integrateQuaternion(
      const Eigen::Quaterniond& initial_quaternion,
      const Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>&
          debiased_imu_readings) const;

  std::shared_ptr<imu_integrator::ImuIntegratorRK4> integrator_;
  std::shared_ptr<test_trajectory_gen::GenericPathGenerator> generator_;

  Eigen::Matrix3Xd landmarks_;
  Eigen::Matrix3Xd true_p_G_B_;
  Eigen::Matrix4Xd true_q_B_G_;
  Eigen::Matrix3Xd true_velocities_;
  Eigen::Matrix<double, imu_integrator::kImuReadingSize, Eigen::Dynamic>
      imu_data_;
  Eigen::VectorXd imu_timestamps_seconds_;
  Eigen::Matrix3Xd true_acc_bias_;
  Eigen::Matrix3Xd true_gyro_bias_;

  bool is_path_generated_;
  bool are_landmarks_generated_;

  // Path and landmark settings: Sigma in continuous time.
  test_trajectory_gen::PathAndLandmarkSettings settings_;

  static constexpr double kZAccelAmplitude = 5.0;
  static constexpr double kXGyroAmplitude = 0.75;
  static constexpr double kYGyroAmplitude = 0.75;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  //  namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_TEST_6DOF_TEST_TRAJECTORY_GEN_H_
