#ifndef MAPLAB_COMMON_LOCALIZATION_RESULT_H_
#define MAPLAB_COMMON_LOCALIZATION_RESULT_H_

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <aslam/common/unique-id.h>
#include <maplab-common/macros.h>

namespace common {

enum class LocalizationState : uint8_t {
  // No reference map has been set, localization is not performed.
  kUninitialized,
  // Baseframe transformation has not yet been initialized.
  kNotLocalized,
  // Baseframe was initialized and global map matching is performed.
  kLocalized,
  // Map matching is performed using map tracking.
  kMapTracking,
  kInvalid,
};
MAPLAB_DEFINE_ENUM_HASHING(LocalizationState, int);

inline std::string convertLocalizationStateToString(
    const LocalizationState state) {
  switch (state) {
    case LocalizationState::kUninitialized:
      return "Uninitialized";
      break;
    case LocalizationState::kNotLocalized:
      return "Not Localized";
      break;
    case LocalizationState::kLocalized:
      return "Localized";
      break;
    case LocalizationState::kMapTracking:
      return "Map-Tracking";
      break;
    default:
      LOG(FATAL) << "Unknown localization state: " << static_cast<int>(state)
                 << '.';
      break;
  }
  return "";
}

// TODO(LBern): TODO(mfehr): Add lidar/voxblox localization type here.
enum class LocalizationType : uint8_t {
  kVisualFeatureBased,
  kFused,
  kAbsolutePose,
  kUnknown
};

enum class LocalizationMode : uint8_t { kGlobal, kMapTracking, kUnknown };

struct LocalizationResult {
  MAPLAB_POINTER_TYPEDEFS(LocalizationResult);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalizationResult() : LocalizationResult(LocalizationType::kUnknown) {}

  explicit LocalizationResult(const LocalizationType type)
      : timestamp_ns(-1),
        is_T_G_B_set(false),
        is_T_G_M_set(false),
        localization_mode(LocalizationMode::kUnknown),
        localization_type(type) {}

  virtual ~LocalizationResult() = default;

  // What sensor was used to achieve this localization. Can be invalid, if the
  // sensor is irrelevant for the localization.
  aslam::SensorId sensor_id;

  int64_t timestamp_ns;

  // Localization as transformation from the current sensor base frame (B, or I
  // for IMU) to the global frame (G).
  aslam::Transformation T_G_B;
  bool is_T_G_B_set;

  // Covariance matrix representing the localization uncertainty
  Eigen::Matrix<double, 6, 6> T_G_B_covariance;

  // Localization as transformation from the odometry base frame (M) to the
  // global frame (G).
  aslam::Transformation T_G_M;
  bool is_T_G_M_set;

  LocalizationMode localization_mode;
  const LocalizationType localization_type;
};

struct FusedLocalizationResult : public LocalizationResult {
  MAPLAB_POINTER_TYPEDEFS(FusedLocalizationResult);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FusedLocalizationResult() : LocalizationResult(LocalizationType::kFused) {}

  FusedLocalizationResult(const FusedLocalizationResult& other) = default;

  common::FusedLocalizationResult& operator=(
      const FusedLocalizationResult& other) {
    sensor_id = other.sensor_id;
    timestamp_ns = other.timestamp_ns;
    T_G_B = other.T_G_B;
    is_T_G_B_set = other.is_T_G_B_set;
    T_G_M = other.T_G_M;
    is_T_G_M_set = other.is_T_G_M_set;
    localization_mode = other.localization_mode;
    return *this;
  }
};

}  // namespace common

#endif  // MAPLAB_COMMON_LOCALIZATION_RESULT_H_
