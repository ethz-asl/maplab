#include "visualization/constant-velocity-smoother.h"

#include <chrono>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <maplab-common/conversions.h>

DEFINE_double(
    pose_smoother_cutoff_frequency, 40,
    "Cutoff frequency for the "
    "velocity smoothing [Hz].");

namespace visualization {
ConstantVelocitySmoother::ConstantVelocitySmoother()
    : ConstantVelocitySmoother(FLAGS_pose_smoother_cutoff_frequency) {}

ConstantVelocitySmoother::ConstantVelocitySmoother(double cutoff_frequency)
    : cutoff_frequency_(cutoff_frequency),
      is_initialized_(false),
      n_samples_(0),
      filtered_position_(Eigen::Vector3d::Constant(0.0)),
      filtered_velocity_(Eigen::Vector3d::Constant(0.0)),
      last_sample_(Eigen::Vector3d::Constant(0.0)) {
  CHECK_GT(cutoff_frequency_, 0.0);
  time_constant_ = 1. / (2. * M_PI * cutoff_frequency_);
}
void ConstantVelocitySmoother::addSample(const Eigen::Vector3d& sample) {
  if (!is_initialized_) {
    filtered_position_ = sample;
    last_sample_ = sample;
    is_initialized_ = true;
    last_time_ = steady_clock::now();
    ++n_samples_;
    return;
  }

  using std::chrono::duration_cast;
  steady_clock::time_point current = steady_clock::now();
  double delta_t_seconds =
      kMilliSecondsToSeconds *
      duration_cast<std::chrono::milliseconds>(current - last_time_).count();
  last_time_ = current;

  if (delta_t_seconds == 0.0) {
    return;
  }

  Eigen::Vector3d current_velocity = (sample - last_sample_) / delta_t_seconds;

  double alpha = delta_t_seconds / (time_constant_ + delta_t_seconds);
  filtered_velocity_ =
      alpha * current_velocity + (1. - alpha) * filtered_velocity_;

  Eigen::Vector3d predicted_position =
      filtered_position_ + filtered_velocity_ * delta_t_seconds;
  filtered_position_ = alpha * sample + (1. - alpha) * predicted_position;

  ++n_samples_;
  last_sample_ = sample;
}

const Eigen::Vector3d& ConstantVelocitySmoother::getCurrentPosition() const {
  return filtered_position_;
}
}  // namespace visualization
