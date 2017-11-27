#ifndef VISUALIZATION_CONSTANT_VELOCITY_SMOOTHER_H_
#define VISUALIZATION_CONSTANT_VELOCITY_SMOOTHER_H_
#include <chrono>

#include <Eigen/Dense>
#include <gflags/gflags.h>

// The final version of the standard renamed monotonic_clock to steady_clock.
#define GCC_VERSION (__GNUC__ * 100 + __GNUC_MINOR__)

#if GCC_VERSION >= 407 || defined(__clang__)
using std::chrono::steady_clock;
#else
typedef std::chrono::monotonic_clock steady_clock;
#endif

namespace visualization {
class ConstantVelocitySmoother {
 public:
  ConstantVelocitySmoother();
  explicit ConstantVelocitySmoother(double cutoff_frequency);
  void addSample(const Eigen::Vector3d& sample);
  const Eigen::Vector3d& getCurrentPosition() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  steady_clock::time_point last_time_;
  const double cutoff_frequency_;
  bool is_initialized_;
  int n_samples_;
  double time_constant_;
  Eigen::Vector3d filtered_position_;
  Eigen::Vector3d filtered_velocity_;
  Eigen::Vector3d last_sample_;
};
}  // namespace visualization
#endif  // VISUALIZATION_CONSTANT_VELOCITY_SMOOTHER_H_
