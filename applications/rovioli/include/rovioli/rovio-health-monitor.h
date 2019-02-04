#ifndef ROVIOLI_ROVIO_HEALTH_MONITOR_H_
#define ROVIOLI_ROVIO_HEALTH_MONITOR_H_
#include <memory>

#include <vector>

#include <aslam/common/stl-helpers.h>

#include "rovioli/rovio-estimate.h"
#include "rovioli/rovio-factory.h"

namespace rovioli {

class RovioHealthMonitor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RovioHealthMonitor() : num_subsequent_unhealthy_updates_(0) {}

  // Returns true if healthy; false if unhealthy and reset was triggered.
  bool shouldResetEstimator(const rovio::RovioState& state) {
    CHECK(state.hasFeatureState());
    const rovio::RovioFeatureState& feature_state = state.getFeatureState();

    const int max_num_features = feature_state.get_MaxNumFeatures();
    std::vector<float> distance_covs;
    for (int i = 0; i < max_num_features; ++i) {
      if (feature_state.get_isFeatureValid(i)) {
        distance_covs.push_back(feature_state.get_DistanceCov(i));
      }
    }

    float feature_distance_covariance_median = 0;
    if (!distance_covs.empty()) {
      feature_distance_covariance_median =
          aslam::common::median(distance_covs.begin(), distance_covs.end());
    }
    const float BvB_norm = state.get_BvB().norm();

    if ((BvB_norm > kVelocityToConsiderStatic) &&
        ((BvB_norm > kUnhealthyVelocity) ||
         (feature_distance_covariance_median > kUnhealthyFeatureDistanceCov))) {
      ++num_subsequent_unhealthy_updates_;
      LOG(WARNING) << "Estimator fault counter: "
                   << num_subsequent_unhealthy_updates_ << "/"
                   << kMaxSubsequentUnhealthyUpdates << ". Might reset soon.";

      if (num_subsequent_unhealthy_updates_ > kMaxSubsequentUnhealthyUpdates) {
        LOG(ERROR) << "Will reset ROVIOLI. Velocity norm: " << BvB_norm
                   << " (limit: " << kUnhealthyVelocity
                   << "), median of feature distance covariances: "
                   << feature_distance_covariance_median
                   << " (limit: " << kUnhealthyFeatureDistanceCov << ").";
        return true;
      }
    } else {
      if (feature_distance_covariance_median < kHealthyFeatureDistanceCov) {
        if (std::abs(
                feature_distance_covariance_median -
                last_safe_pose_.feature_distance_covariance_median) <
            kHealthyFeatureDistanceCovIncrement) {
          last_safe_pose_.failsafe_WrWB = state.get_WrWB();
          last_safe_pose_.failsafe_qBW = state.get_qBW();
          last_safe_pose_.feature_distance_covariance_median =
              feature_distance_covariance_median;
        }
      }
      num_subsequent_unhealthy_updates_ = 0;
    }
    return false;
  }

  void resetRovioToLastHealthyPose(rovio::RovioInterface* rovio_interface) {
    CHECK_NOTNULL(rovio_interface);
    rovio_interface->requestResetToPose(
        last_safe_pose_.failsafe_WrWB, last_safe_pose_.failsafe_qBW);
  }

 private:
  struct RovioFailsafePose {
    RovioFailsafePose()
        : failsafe_WrWB(Eigen::Vector3d::Zero()),
          feature_distance_covariance_median(0.0) {
      failsafe_qBW.setIdentity();
    }

    Eigen::Vector3d failsafe_WrWB;
    kindr::RotationQuaternionPD failsafe_qBW;
    float feature_distance_covariance_median;
  };
  RovioFailsafePose last_safe_pose_;

  int num_subsequent_unhealthy_updates_;

  // The landmark covariance is not a good measure for divergence if we are
  // static.
  static constexpr float kVelocityToConsiderStatic = 0.1f;
  static constexpr int kMaxSubsequentUnhealthyUpdates = 3;
  static constexpr float kHealthyFeatureDistanceCov = 0.5f;
  static constexpr float kHealthyFeatureDistanceCovIncrement = 0.3f;
  static constexpr float kUnhealthyFeatureDistanceCov = 10.0f;
  static constexpr float kUnhealthyVelocity = 6.0f;
};

}  // namespace rovioli
#endif  // ROVIOLI_ROVIO_HEALTH_MONITOR_H_
