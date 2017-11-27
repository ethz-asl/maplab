#include "simulation/visual-inertial-path-generator.h"

#include <glog/logging.h>

namespace test_trajectory_gen {

VisualInertialPathGenerator::VisualInertialPathGenerator(
    const PathAndLandmarkSettings& settings)
    : are_landmarks_generated_(false),
      is_path_generated_(false),
      settings_(settings) {}

const Eigen::Matrix3Xd& VisualInertialPathGenerator::getTruePositions() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_p_G_B_;
}

const Eigen::Matrix4Xd& VisualInertialPathGenerator::getTrueRotations() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_q_B_G_;
}

const Eigen::Matrix3Xd& VisualInertialPathGenerator::getLandmarks() const {
  CHECK(are_landmarks_generated_) << "Landmarks need to be generated first.";
  return G_landmarks_;
}

const Eigen::Matrix3Xd& VisualInertialPathGenerator::getTrueVelocities() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_G_v_B_;
}

const Eigen::Matrix<double, 6, Eigen::Dynamic>&
VisualInertialPathGenerator::getImuData() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return imu_data_;
}

const Eigen::Matrix3Xd& VisualInertialPathGenerator::getTrueAccBias() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_acc_bias_;
}

const Eigen::Matrix3Xd& VisualInertialPathGenerator::getTrueGyroBias() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return true_gyro_bias_;
}

const mav_planning_utils::Motion4D<5, 2>::Vector&
VisualInertialPathGenerator::getPathData() const {
  CHECK(is_path_generated_) << "Path needs to be generated first.";
  return path_data_;
}

void VisualInertialPathGenerator::motionVectorToImuData(uint_fast32_t seed) {
  CHECK_GT(path_data_.size(), 0u) << "Path needs to be generated first.";

  std::mt19937 gen(seed);
  std::normal_distribution<> dis(0, 1);

  // For some reason, last path point is wrong (zero position)
  size_t num_samples = path_data_.size();
  CHECK_GT(num_samples, 0u);
  size_t num_samples_minus_one = num_samples - 1;

  true_p_G_B_.resize(3, num_samples_minus_one);
  true_q_B_G_.resize(4, num_samples_minus_one);
  true_G_v_B_.resize(3, num_samples_minus_one);
  true_acc_bias_.resize(Eigen::NoChange, num_samples_minus_one);
  true_gyro_bias_.resize(Eigen::NoChange, num_samples_minus_one);

  imu_data_.resize(6, num_samples_minus_one);
  Eigen::VectorXd timestamps_resized =
      timestamps_seconds_.head(num_samples_minus_one);
  timestamps_seconds_.resize(num_samples - 1);
  for (size_t i = 0; i < num_samples_minus_one; ++i) {
    timestamps_seconds_(i) = timestamps_resized(i);
  }

  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();

  Eigen::Vector3d gyro_noise_temp, acc_noise_temp;
  Eigen::Vector3d gyro_bias_temp, acc_bias_temp;

  // Copy the data to base class members.
  mav_planning_utils::MavState s;
  for (size_t i = 0; i < num_samples_minus_one; ++i) {
    motionVectorToMavState(path_data_[i], &s);

    true_p_G_B_.col(i) = s.p;
    true_q_B_G_.col(i) = s.q.inverse().coeffs();
    true_G_v_B_.col(i) = s.v;

    getRandomVector3d(&gen, &dis, &gyro_noise_temp);
    getRandomVector3d(&gen, &dis, &acc_noise_temp);

    Eigen::Matrix<double, 6, 1> imu_reading;
    imu_reading.head(3) = s.a_b + acc_bias +
                          acc_noise_temp *
                              settings_.imu_sigmas.acc_noise_density /
                              sqrt(settings_.sampling_time_second);
    imu_reading.tail(3) = s.w_b + gyro_bias +
                          gyro_noise_temp *
                              settings_.imu_sigmas.gyro_noise_density /
                              sqrt(settings_.sampling_time_second);
    imu_data_.col(i) = imu_reading;
    true_acc_bias_.col(i) = acc_bias;
    true_gyro_bias_.col(i) = gyro_bias;

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

void VisualInertialPathGenerator::getRandomVector3d(
    std::mt19937* generator, std::normal_distribution<>* distribution,
    Eigen::Vector3d* vector) {
  CHECK_NOTNULL(generator);
  CHECK_NOTNULL(distribution);
  CHECK_NOTNULL(vector);

  (*vector)[0] = (*distribution)(*generator);
  (*vector)[1] = (*distribution)(*generator);
  (*vector)[2] = (*distribution)(*generator);
}

void VisualInertialPathGenerator::motionVectorToMavState(
    const mav_planning_utils::Motion4D<5, 2>& motion_data,
    mav_planning_utils::MavState* mav_state) const {
  CHECK_NOTNULL(mav_state);

  // Get rotation from yaw angle.
  mav_state->q.coeffs().setZero();
  mav_state->q.w() =
      cos(motion_data.yaw[mav_planning_utils::DerivativesO::o] / 2);
  mav_state->q.z() =
      sin(motion_data.yaw[mav_planning_utils::DerivativesO::o] / 2);
  mav_state->q.normalize();

  // Add gravity to world-frame acceleration and then transform to body frame.
  Eigen::Vector3d acceleration_G =
      motion_data.getStateP(mav_planning_utils::DerivativesP::a) +
      Eigen::Vector3d(0, 0, settings_.gravity_meter_by_second2);
  mav_state->a_b = mav_state->q.toRotationMatrix().transpose() * acceleration_G;

  mav_state->w_b[0] = 0;
  mav_state->w_b[1] = 0;
  mav_state->w_b[2] = motion_data.yaw[mav_planning_utils::DerivativesO::w];

  mav_state->p = motion_data.getStateP(mav_planning_utils::DerivativesP::p);
  mav_state->v = motion_data.getStateP(mav_planning_utils::DerivativesP::v);
  mav_state->a = motion_data.getStateP(mav_planning_utils::DerivativesP::a);
  mav_state->j = motion_data.getStateP(mav_planning_utils::DerivativesP::j);
  mav_state->s = motion_data.getStateP(mav_planning_utils::DerivativesP::s);
}

bool VisualInertialPathGenerator::isPathGenerated() const {
  return is_path_generated_;
}

bool VisualInertialPathGenerator::areLandmarksGenerated() const {
  return are_landmarks_generated_;
}

const Eigen::VectorXd& VisualInertialPathGenerator::getTimestampsInSeconds()
    const {
  return timestamps_seconds_;
}

void VisualInertialPathGenerator::getGroundTruthTransformations(
    aslam::TransformationVector* T_G_Bs) const {
  CHECK_NOTNULL(T_G_Bs);
  CHECK(is_path_generated_) << "Can only get transformations back after a path "
                               "has been generated!";

  T_G_Bs->clear();
  size_t num_samples = true_p_G_B_.cols();
  CHECK_EQ(static_cast<int>(num_samples), true_q_B_G_.cols());
  T_G_Bs->reserve(num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    Eigen::Vector3d t_G_B = true_p_G_B_.col(i);
    Eigen::Quaterniond q_B_G(true_q_B_G_.col(i));
    T_G_Bs->emplace_back(q_B_G.inverse(), t_G_B);
  }
  CHECK_EQ(num_samples, T_G_Bs->size());
}

}  // namespace test_trajectory_gen
