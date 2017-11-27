#include "map-optimization-legacy/test/6dof-test-trajectory-gen.h"

#include <fstream>  // NOLINT

#include <glog/logging.h>
#include <maplab-common/quaternion-math.h>

#include "map-optimization-legacy/test/vi-optimization-test-helpers.h"

namespace map_optimization_legacy {

SixDofTestTrajectoryGenerator::SixDofTestTrajectoryGenerator(
    const test_trajectory_gen::PathAndLandmarkSettings& settings)
    : is_path_generated_(false),
      are_landmarks_generated_(false),
      settings_(settings) {
  // Create a noise free trajectory generator for integration.
  test_trajectory_gen::PathAndLandmarkSettings settings_without_imu_noise =
      settings;

  vi_map::ImuSigmas& imu_sigmas = settings_without_imu_noise.imu_sigmas;
  setImuSigmasZero(&imu_sigmas);
  generator_ = std::shared_ptr<test_trajectory_gen::GenericPathGenerator>(
      new test_trajectory_gen::GenericPathGenerator(
          settings_without_imu_noise));

  integrator_ = std::shared_ptr<imu_integrator::ImuIntegratorRK4>(
      new imu_integrator::ImuIntegratorRK4(
          0.0, 0.0, 0.0, 0.0, settings_.gravity_meter_by_second2));
}

void SixDofTestTrajectoryGenerator::generate6DofPath() {
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> current_state;
  Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>
      debiased_imu_readings;
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> next_state;
  Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                imu_integrator::kErrorStateSize>
      next_phi;
  Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                imu_integrator::kErrorStateSize>
      next_cov;

  // -----------------------------------------------------------------------
  // Generate 4DOF trajectory.
  LOG(INFO) << "Generating trajectory...";
  // setup.num_path_constraints: 8 points for path constraints.
  // setup.imu_noise_bias_seed: seed does not matter here, we don't want
  //                            noise anyways.
  generator_->generatePath();
  Eigen::Matrix<double, imu_integrator::kImuReadingSize, Eigen::Dynamic>
      imu_data_4dof = generator_->getImuData();
  Eigen::Matrix4Xd rotations_4dof = generator_->getTrueRotations();
  const Eigen::VectorXd& timestamps_4dof = generator_->getTimestampsInSeconds();

  // Set initial state from first position/rotation data.
  current_state.setZero();
  current_state.block<imu_integrator::kStateOrientationBlockSize, 1>(
      imu_integrator::kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);
  current_state.block<imu_integrator::kPositionBlockSize, 1>(
      imu_integrator::kStatePositionOffset, 0) = Eigen::Vector3d(10, 0, 0);

  // -----------------------------------------------------------------------
  // Introduce additional excitation to IMU data.
  for (int i = 0; i < imu_data_4dof.cols(); ++i) {
    // Accelerometer excitation (z axis)
    // Note that we need phase shift s.t. the initial velocity is zero:
    // -cos(t)=0 => sin(t) = 1
    imu_data_4dof(imu_integrator::kAccelReadingOffset + 2, i) +=
        kZAccelAmplitude *
        sin(M_PI * settings_.sampling_time_second * i + M_PI / 2);

    // Additional gyro excitation.
    imu_data_4dof(imu_integrator::kGyroReadingOffset + 0, i) +=
        kXGyroAmplitude * sin(0.5 * M_PI * settings_.sampling_time_second * i);
    imu_data_4dof(imu_integrator::kGyroReadingOffset + 1, i) +=
        kYGyroAmplitude *
        sin(0.5 * M_PI * settings_.sampling_time_second * i + M_PI);
  }

  // -----------------------------------------------------------------------
  // Integrate trajectory to generate ground-truth.
  size_t num_samples = imu_data_4dof.cols() - 1;
  true_p_G_B_.resize(Eigen::NoChange, num_samples);
  true_q_B_G_.resize(Eigen::NoChange, num_samples);
  true_velocities_.resize(Eigen::NoChange, num_samples);
  true_acc_bias_.resize(Eigen::NoChange, num_samples);
  true_gyro_bias_.resize(Eigen::NoChange, num_samples);

  LOG(INFO) << "Integrating trajectory...";
  imu_data_ = imu_data_4dof.leftCols(num_samples);
  imu_timestamps_seconds_ = timestamps_4dof.head(num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    debiased_imu_readings.block<imu_integrator::kImuReadingSize, 1>(0, 0) =
        imu_data_4dof.col(i);
    debiased_imu_readings.block<imu_integrator::kImuReadingSize, 1>(
        imu_integrator::kImuReadingSize, 0) = imu_data_4dof.col(i + 1);

    Eigen::Quaterniond G_q_B_4dof_current(rotations_4dof.col(i).data());
    Eigen::Quaterniond G_q_B_4dof_next(rotations_4dof.col(i + 1).data());

    Eigen::Quaterniond B_q_G_current(
        current_state.block<imu_integrator::kStateOrientationBlockSize, 1>(0, 0)
            .data());
    Eigen::Quaterniond B_q_G_next =
        integrateQuaternion(B_q_G_current, debiased_imu_readings);
    Eigen::Matrix3d B_R_G_current, B_R_G_next;
    Eigen::Matrix3d G_R_B_4dof_current, G_R_B_4dof_next;
    common::toRotationMatrixJPL(B_q_G_current.coeffs(), &B_R_G_current);
    common::toRotationMatrixJPL(B_q_G_next.coeffs(), &B_R_G_next);
    common::toRotationMatrixJPL(
        G_q_B_4dof_current.coeffs(), &G_R_B_4dof_current);
    common::toRotationMatrixJPL(G_q_B_4dof_next.coeffs(), &G_R_B_4dof_next);
    debiased_imu_readings.block<3, 1>(0, 0) =
        B_R_G_current * G_R_B_4dof_current *
        debiased_imu_readings.block<3, 1>(0, 0);
    debiased_imu_readings.block<3, 1>(imu_integrator::kImuReadingSize, 0) =
        B_R_G_next * G_R_B_4dof_next *
        debiased_imu_readings.block<3, 1>(imu_integrator::kImuReadingSize, 0);

    // Save transformed accelerometer measurement.
    imu_data_.block<3, 1>(imu_integrator::kAccelReadingOffset, i) =
        debiased_imu_readings.block<3, 1>(0, 0);

    integrator_->integrate(
        current_state, debiased_imu_readings, settings_.sampling_time_second,
        &next_state, &next_phi, &next_cov);
    double dt = timestamps_4dof(i + 1) - timestamps_4dof(i);
    CHECK_NEAR(dt, settings_.sampling_time_second, 1e-8);

    // Sanity check if next state quaternion is matching the quaternion that
    // was used to transform acceleration measurements (previosuly incremented).
    Eigen::Vector4d next_state_B_q_G =
        next_state.block<imu_integrator::kStateOrientationBlockSize, 1>(
            imu_integrator::kStateOrientationOffset, 0);
    CHECK_EQ(B_q_G_next.coeffs(), next_state_B_q_G);

    // Copy to rotation/position data structures.
    true_p_G_B_.col(i) =
        current_state.block<imu_integrator::kPositionBlockSize, 1>(
            imu_integrator::kStatePositionOffset, 0);
    Eigen::Quaterniond G_q_B(
        current_state
            .block<imu_integrator::kStateOrientationBlockSize, 1>(
                imu_integrator::kStateOrientationOffset, 0)
            .data());

    true_q_B_G_.col(i) = G_q_B.coeffs();
    true_velocities_.col(i) =
        current_state.block<imu_integrator::kVelocityBlockSize, 1>(
            imu_integrator::kStateVelocityOffset, 0);

    current_state = next_state;
  }

  CHECK_EQ(true_p_G_B_.cols(), true_q_B_G_.cols());
  CHECK_EQ(true_p_G_B_.cols(), imu_data_.cols());
  CHECK_EQ(true_p_G_B_.cols(), true_velocities_.cols());
  CHECK_EQ(true_p_G_B_.cols(), imu_timestamps_seconds_.rows());

  // -----------------------------------------------------------------------
  // Add bias and noise to IMU measurements.
  addNoiseToImuData(&imu_data_);

  is_path_generated_ = true;
}

void SixDofTestTrajectoryGenerator::getRandomVector3d(
    std::mt19937* generator, std::normal_distribution<>* distribution,
    Eigen::Vector3d* vector) {
  CHECK_NOTNULL(generator);
  CHECK_NOTNULL(distribution);
  CHECK_NOTNULL(vector);

  (*vector)[0] = (*distribution)(*generator);
  (*vector)[1] = (*distribution)(*generator);
  (*vector)[2] = (*distribution)(*generator);
}

void SixDofTestTrajectoryGenerator::addNoiseToImuData(
    Eigen::Matrix<double, imu_integrator::kImuReadingSize, Eigen::Dynamic>*
        imu_data) {
  CHECK_NOTNULL(imu_data);
  std::mt19937 gen(settings_.imu_noise_bias_seed);
  std::normal_distribution<> dis(0, 1);

  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_noise_temp, acc_noise_temp;
  Eigen::Vector3d gyro_bias_temp, acc_bias_temp;

  size_t num_imu_measurements = imu_data_.cols();
  for (size_t idx = 0; idx < num_imu_measurements; ++idx) {
    getRandomVector3d(&gen, &dis, &gyro_noise_temp);
    getRandomVector3d(&gen, &dis, &acc_noise_temp);
    imu_data->block<3, 1>(0, idx) +=
        acc_bias +
        acc_noise_temp * settings_.imu_sigmas.acc_noise_density /
            sqrt(settings_.sampling_time_second);

    imu_data->block<3, 1>(3, idx) +=
        gyro_bias +
        gyro_noise_temp * settings_.imu_sigmas.gyro_noise_density /
            sqrt(settings_.sampling_time_second);
    true_acc_bias_.col(idx) = acc_bias;
    true_gyro_bias_.col(idx) = gyro_bias;

    getRandomVector3d(&gen, &dis, &gyro_bias_temp);
    getRandomVector3d(&gen, &dis, &acc_bias_temp);
    gyro_bias += gyro_bias_temp *
                 settings_.imu_sigmas.gyro_bias_random_walk_noise_density *
                 sqrt(settings_.sampling_time_second);
    acc_bias += acc_bias_temp *
                settings_.imu_sigmas.acc_bias_random_walk_noise_density *
                sqrt(settings_.sampling_time_second);
  }
}

Eigen::Quaterniond SixDofTestTrajectoryGenerator::integrateQuaternion(
    const Eigen::Quaterniond& initial_quaternion,
    const Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>&
        debiased_imu_readings) const {
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> current_state;
  Eigen::Matrix<double, imu_integrator::kStateSize, 1> next_state;
  Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                imu_integrator::kErrorStateSize>
      next_phi;
  Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                imu_integrator::kErrorStateSize>
      next_cov;

  current_state.setZero();
  current_state.block<imu_integrator::kStateOrientationBlockSize, 1>(
      imu_integrator::kStateOrientationOffset, 0) = initial_quaternion.coeffs();

  integrator_->integrate(
      current_state, debiased_imu_readings, settings_.sampling_time_second,
      &next_state, &next_phi, &next_cov);

  Eigen::Quaterniond integrated_quaternion(
      next_state.block<imu_integrator::kStateOrientationBlockSize, 1>(
          imu_integrator::kStateOrientationOffset, 0));

  return integrated_quaternion;
}

void SixDofTestTrajectoryGenerator::saveTrajectory(
    const std::string& filename) const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  std::ofstream out_file;
  out_file.open(filename);
  for (int i = 0; i < true_p_G_B_.cols(); ++i) {
    out_file << true_p_G_B_(0, i) << ", " << true_p_G_B_(1, i) << ", "
             << true_p_G_B_(2, i) << std::endl;
  }
  out_file.close();
}

void SixDofTestTrajectoryGenerator::generateLandmarks() {
  generator_->generateLandmarks();
  landmarks_ = generator_->getLandmarks();

  are_landmarks_generated_ = true;
}

const Eigen::Matrix3Xd& SixDofTestTrajectoryGenerator::getLandmarks() const {
  CHECK(are_landmarks_generated_) << "Landmarks need to be generated first.";
  return landmarks_;
}

const Eigen::Matrix3Xd& SixDofTestTrajectoryGenerator::getTruePositions()
    const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_p_G_B_;
}

const Eigen::Matrix4Xd& SixDofTestTrajectoryGenerator::getTrueRotations()
    const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_q_B_G_;
}

const Eigen::Matrix3Xd& SixDofTestTrajectoryGenerator::getTrueVelocities()
    const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_velocities_;
}

const Eigen::Matrix<double, imu_integrator::kImuReadingSize, Eigen::Dynamic>&
SixDofTestTrajectoryGenerator::getImuData() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return imu_data_;
}

const Eigen::VectorXd& SixDofTestTrajectoryGenerator::getTimestampsInSeconds()
    const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return imu_timestamps_seconds_;
}

const Eigen::Matrix3Xd& SixDofTestTrajectoryGenerator::getTrueAccBias() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_acc_bias_;
}

const Eigen::Matrix3Xd& SixDofTestTrajectoryGenerator::getTrueGyroBias() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_gyro_bias_;
}

void SixDofTestTrajectoryGenerator::getGroundTruthTransformations(
    aslam::TransformationVector* T_G_Bs) const {
  CHECK_NOTNULL(T_G_Bs);
  CHECK(is_path_generated_) << "Path needs to be generated first.";

  T_G_Bs->clear();
  size_t num_samples = true_p_G_B_.cols();
  CHECK_EQ(static_cast<int>(num_samples), true_q_B_G_.cols());
  T_G_Bs->reserve(num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    Eigen::Vector3d t_G_B = true_p_G_B_.col(i);
    Eigen::Quaterniond q_B_G(true_q_B_G_.col(i));
    T_G_Bs->emplace_back(aslam::Transformation(q_B_G, t_G_B));
  }
  CHECK_EQ(num_samples, T_G_Bs->size());
}

}  // namespace map_optimization_legacy
